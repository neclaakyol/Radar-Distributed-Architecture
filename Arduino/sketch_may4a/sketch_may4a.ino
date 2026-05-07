#include <Arduino_FreeRTOS.h>
#include <queue.h>

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

// --- Task 1: Mock Servo Sweep ---
void TaskServoActuation(void *pvParameters) {
  (void) pvParameters;
  bool sweepingForward = true;

  for (;;) {
    // Simulate servo movement
    if (sweepingForward) {
      currentAngle++;
      if (currentAngle >= 180) sweepingForward = false;
    } else {
      currentAngle--;
      if (currentAngle <= 0) sweepingForward = true;
    }
    // Yield CPU to allow 15ms for the physical servo to reach the new angle
    vTaskDelay(15 / portTICK_PERIOD_MS);
  }
}

// --- Task 2: Mock Sensor Polling ---
void TaskSensorPolling(void *pvParameters) {
  (void) pvParameters;
  RawData data;

  for (;;) {
    // 1. Grab the current angle from the servo task
    data.angle = currentAngle;
    
    // 2. Mock a distance reading (e.g., random target between 10cm and 150cm)
    // In reality, this is where you send 0x55 over Serial and wait for 2 bytes
    data.distance = random(100, 1500); // 10.0cm to 150.0cm in millimeters
    
    // 3. Push to queue. Wait max 5 ticks if queue is full.
    xQueueSend(rawDataQueue, &data, (TickType_t)5);

    // Yield CPU (Simulate ~50ms US-100 sensor read time)
    vTaskDelay(50 / portTICK_PERIOD_MS);
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

  // Initialize Queues
  rawDataQueue = xQueueCreate(10, sizeof(RawData));
  txQueue = xQueueCreate(10, sizeof(TelemetryFrame));

  if (rawDataQueue != NULL && txQueue != NULL) {
    // Create Tasks
    // Priorities: UART (Highest: 3), Security (2), Sensor (1), Servo (Lowest: 0)
    xTaskCreate(TaskUARTDispatch, "UART", 128, NULL, 3, NULL);
    xTaskCreate(TaskSecurity, "Crypto", 200, NULL, 2, NULL); // Needs more RAM for crypto vars
    xTaskCreate(TaskSensorPolling, "Sensor", 128, NULL, 1, NULL);
    xTaskCreate(TaskServoActuation, "Servo", 128, NULL, 0, NULL);
  }

  // The FreeRTOS scheduler starts automatically after setup() in this library port.
}

void loop() {
  // Empty. Execution is handled entirely by FreeRTOS tasks.
}