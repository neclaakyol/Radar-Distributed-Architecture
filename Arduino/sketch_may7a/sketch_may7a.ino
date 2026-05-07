#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Servo.h>            // ADDED: Hardware servo control
#include <SoftwareSerial.h>   // ADDED: US-100 UART communication

// --- System Definitions ---
#define BAUD_RATE 115200
#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

// --- Data Structures ---
// Raw data passed from Sensor Task to Security Task
struct RawData {
  uint8_t angle;
  uint16_t distance;
};

// The final 12-byte frame sent over UART
struct TelemetryFrame {
  uint8_t bytes[12];
};

// --- FreeRTOS Handles ---
QueueHandle_t rawDataQueue;
QueueHandle_t txQueue;

// Shared state for the mock servo
volatile uint8_t currentAngle = 0;

// --- Cryptography & Security Keys ---
// 128-bit XTEA Key (Hardcoded for prototype)
const uint32_t xtea_key[4] = {0x12345678, 0x9ABCDEF0, 0x11223344, 0x55667788};

// --- Helper Functions: XTEA & CRC-16 ---
void xtea_encrypt(uint32_t v[2], uint32_t const key[4]) {
    unsigned int i;
    uint32_t v0 = v[0], v1 = v[1], sum = 0, delta = 0x9E3779B9;
    for (i = 0; i < 32; i++) {
        v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
        sum += delta;
        v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum >> 11) & 3]);
    }
    v[0] = v0; v[1] = v1;
}

uint16_t crc16_ccitt(const uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF; // Initial value
    for (uint8_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; // CCITT Polynomial
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

Servo radarServo;
SoftwareSerial us100(2, 3); // RX on Pin 2, TX on Pin 3

// --- Task 1: REAL Servo Actuation ---
void TaskServoActuation(void *pvParameters) {
  (void) pvParameters;
  
  radarServo.attach(9); // Attach SG90 to digital pin 9
  bool sweepingForward = true;

  for (;;) {
    if (sweepingForward) {
      currentAngle++;
      if (currentAngle >= 180) sweepingForward = false;
    } else {
      currentAngle--;
      if (currentAngle <= 0) sweepingForward = true;
    }
    
    // Command the physical servo to move
    radarServo.write(currentAngle);
    
    // Yield CPU. 15ms per degree provides a smooth, steady sweep
    vTaskDelay(15 / portTICK_PERIOD_MS);
  }
}

// --- Task 2: REAL Sensor Polling (US-100 UART Mode) ---
void TaskSensorPolling(void *pvParameters) {
  (void) pvParameters;
  RawData data;
  
  us100.begin(9600); // The US-100 operates at 9600 baud in UART mode

  for (;;) {
    // 1. Grab the current angle from the servo task
    data.angle = currentAngle;
    
    // 2. Trigger the US-100 reading by sending 0x55
    us100.write(0x55);
    
    // 3. Wait for the 2-byte response (Non-blocking timeout loop)
    TickType_t startTick = xTaskGetTickCount();
    while (us100.available() < 2) {
        // Timeout if sensor doesn't respond within 50ms
        if ((xTaskGetTickCount() - startTick) > pdMS_TO_TICKS(50)) {
            break; 
        }
        vTaskDelay(1); // Yield CPU while waiting
    }
    
    // 4. Read the data if available
    if (us100.available() >= 2) {
        uint8_t highByte = us100.read();
        uint8_t lowByte = us100.read();
        data.distance = (highByte << 8) | lowByte; // Distance in millimeters
    } else {
        data.distance = 0; // 0 indicates a dropped/timed-out reading
    }
    
    // 5. Push to queue
    xQueueSend(rawDataQueue, &data, (TickType_t)5);

    // Yield CPU before polling again to prevent sensor echo overlap
    vTaskDelay(40 / portTICK_PERIOD_MS);
  }
}

// --- Task 3: Security & Framing (XTEA + CRC16) ---
void TaskSecurity(void *pvParameters) {
  (void) pvParameters;
  RawData incomingData;
  TelemetryFrame outgoingFrame;

  for (;;) {
    // Wait indefinitely for raw data to appear in the queue
    if (xQueueReceive(rawDataQueue, &incomingData, portMAX_DELAY) == pdPASS) {
      
      // 1. Construct the 8-byte plaintext block for XTEA
      uint32_t payloadBlock[2] = {0, 0};
      uint8_t* payloadBytes = (uint8_t*)payloadBlock;
      
      payloadBytes[0] = incomingData.angle;
      payloadBytes[1] = (incomingData.distance >> 8) & 0xFF; // High byte
      payloadBytes[2] = incomingData.distance & 0xFF;        // Low byte
      // Bytes 3-7 are automatically 0 (padding)
      
      // 2. Encrypt the 64-bit block in place
      xtea_encrypt(payloadBlock, xtea_key);
      
      // 3. Assemble the 12-byte frame
      outgoingFrame.bytes[0] = SYNC_BYTE_1;
      outgoingFrame.bytes[1] = SYNC_BYTE_2;
      for(int i = 0; i < 8; i++) {
        outgoingFrame.bytes[i+2] = payloadBytes[i];
      }
      
      // 4. Calculate CRC-16 over the first 10 bytes (Sync + Encrypted Payload)
      uint16_t crc = crc16_ccitt(outgoingFrame.bytes, 10);
      outgoingFrame.bytes[10] = (crc >> 8) & 0xFF; // CRC High
      outgoingFrame.bytes[11] = crc & 0xFF;        // CRC Low
      
      // 5. Push to UART transmission queue
      xQueueSend(txQueue, &outgoingFrame, (TickType_t)5);
    }
  }
}

// --- Task 4: UART Dispatch ---
void TaskUARTDispatch(void *pvParameters) {
  (void) pvParameters;
  TelemetryFrame frameToSend;

  for (;;) {
    // Wait for a secure frame
    if (xQueueReceive(txQueue, &frameToSend, portMAX_DELAY) == pdPASS) {
      // Dispatch 12 bytes over serial
      // Serial.write is buffered and utilizes hardware interrupts under the hood
      Serial.write(frameToSend.bytes, 12);
    }
  }
}

// --- Setup ---
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) { ; } // Wait for serial port to connect

  // Initialize Queues - REDUCE LENGTH FROM 10 to 5
  // We don't need a huge backlog. If we lag behind 5 frames, we have bigger problems.
  rawDataQueue = xQueueCreate(5, sizeof(RawData));
  txQueue = xQueueCreate(5, sizeof(TelemetryFrame));

  if (rawDataQueue != NULL && txQueue != NULL) {
    // Create Tasks - REDUCE STACK SIZES
    // Note: Sizes are in words (1 word = 2 bytes on 8-bit AVR). 
    // 85 is generally the configMINIMAL_STACK_SIZE.
    xTaskCreate(TaskUARTDispatch, "UART", 85, NULL, 3, NULL);
    
    // Crypto needs slightly more stack for the XTEA local variables
    xTaskCreate(TaskSecurity, "Crypto", 120, NULL, 2, NULL); 
    
    // Sensor needs a bit for SoftwareSerial overhead
    xTaskCreate(TaskSensorPolling, "Sensor", 100, NULL, 1, NULL);
    
    // Servo actuation is very simple
    xTaskCreate(TaskServoActuation, "Servo", 85, NULL, 0, NULL);
  }

  // The FreeRTOS scheduler starts automatically after setup() in this library port.
}

void loop() {
  // Empty. Execution is handled entirely by FreeRTOS tasks.
}