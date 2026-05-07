#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Servo.h> 

// --- System Definitions ---
#define BAUD_RATE 115200
#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55

// --- Data Structures ---
struct RawData {
  uint8_t angle;
  uint16_t distance;
};

struct TelemetryFrame {
  uint8_t bytes[12];
};

// --- FreeRTOS Handles ---
QueueHandle_t rawDataQueue;
QueueHandle_t txQueue;

// Shared state for the REAL physical servo
volatile uint8_t currentAngle = 0;
Servo radarServo;

// --- Cryptography & Security Keys ---
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
    uint16_t crc = 0xFFFF; 
    for (uint8_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021; 
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// --- Task 1: REAL Servo Actuation ---
void TaskServoActuation(void *pvParameters) {
  (void) pvParameters;
  
  radarServo.attach(9); 
  bool sweepingForward = true;

  for (;;) {
    if (sweepingForward) {
      currentAngle++;
      if (currentAngle >= 180) sweepingForward = false;
    } else {
      currentAngle--;
      if (currentAngle <= 0) sweepingForward = true;
    }
    
    radarServo.write(currentAngle);
    vTaskDelay(1); // 1 tick = ~16ms
  }
}

// --- Task 2: MOCK Sensor Polling (Hybrid Mode) ---
void TaskSensorPolling(void *pvParameters) {
  (void) pvParameters;
  RawData data;

  for (;;) {
    // 1. Grab the REAL physical angle
    data.angle = currentAngle;
    
    // 2. Generate a MOCK distance
    // Let's simulate a flat wall at 40cm (400mm) with a little bit of noise
    data.distance = 400 + random(-10, 10); 
    
    // 3. Push to queue
    xQueueSend(rawDataQueue, &data, (TickType_t)5);

    // Yield CPU (Simulate ~40ms US-100 sensor read time)
    vTaskDelay(3); // 3 ticks = ~48ms
  }
}

// --- Task 3: Security & Framing (XTEA + CRC16) ---
void TaskSecurity(void *pvParameters) {
  (void) pvParameters;
  RawData incomingData;
  TelemetryFrame outgoingFrame;

  for (;;) {
    if (xQueueReceive(rawDataQueue, &incomingData, portMAX_DELAY) == pdPASS) {
      
      uint32_t payloadBlock[2] = {0, 0};
      uint8_t* payloadBytes = (uint8_t*)payloadBlock;
      
      payloadBytes[0] = incomingData.angle;
      payloadBytes[1] = (incomingData.distance >> 8) & 0xFF; 
      payloadBytes[2] = incomingData.distance & 0xFF;        
      
      xtea_encrypt(payloadBlock, xtea_key);
      
      outgoingFrame.bytes[0] = SYNC_BYTE_1;
      outgoingFrame.bytes[1] = SYNC_BYTE_2;
      for(int i = 0; i < 8; i++) {
        outgoingFrame.bytes[i+2] = payloadBytes[i];
      }
      
      uint16_t crc = crc16_ccitt(outgoingFrame.bytes, 10);
      outgoingFrame.bytes[10] = (crc >> 8) & 0xFF; 
      outgoingFrame.bytes[11] = crc & 0xFF;        
      
      xQueueSend(txQueue, &outgoingFrame, (TickType_t)5);
    }
  }
}

// --- Task 4: UART Dispatch ---
void TaskUARTDispatch(void *pvParameters) {
  (void) pvParameters;
  TelemetryFrame frameToSend;

  for (;;) {
    if (xQueueReceive(txQueue, &frameToSend, portMAX_DELAY) == pdPASS) {
      Serial.write(frameToSend.bytes, 12);
    }
  }
}

// --- Setup ---
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) { ; } 

  rawDataQueue = xQueueCreate(5, sizeof(RawData));
  txQueue = xQueueCreate(5, sizeof(TelemetryFrame));

  if (rawDataQueue != NULL && txQueue != NULL) {
    xTaskCreate(TaskUARTDispatch, "UART", 85, NULL, 3, NULL);
    xTaskCreate(TaskSecurity, "Crypto", 120, NULL, 2, NULL); 
    
    // We can drop the Sensor stack size back down since SoftwareSerial is gone!
    xTaskCreate(TaskSensorPolling, "Sensor", 85, NULL, 1, NULL); 
    xTaskCreate(TaskServoActuation, "Servo", 85, NULL, 0, NULL);
  }
}

void loop() {}