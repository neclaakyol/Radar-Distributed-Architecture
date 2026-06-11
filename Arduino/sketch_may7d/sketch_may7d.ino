#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Servo.h> 

// --- System Definitions ---
#define BAUD_RATE 115200
#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55
#define SERVO_PIN 9
#define HCSR04_TRIG_PIN 2
#define HCSR04_ECHO_PIN 3
#define HCSR04_TIMEOUT_US 30000UL
#define SERVO_STEP_DELAY_MS 15
#define MS_TO_TICKS_ROUNDED(ms) ((TickType_t)(((ms) + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS))

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

uint16_t read_hcsr04_distance_mm() {
  digitalWrite(HCSR04_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(HCSR04_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG_PIN, LOW);

  unsigned long duration_us = pulseIn(HCSR04_ECHO_PIN, HIGH, HCSR04_TIMEOUT_US);
  if (duration_us == 0) {
    return 0;
  }

  unsigned long distance_mm = (duration_us * 343UL) / 2000UL;
  if (distance_mm > 65535UL) {
    return 65535;
  }
  return (uint16_t)distance_mm;
}

// --- Task 1: REAL Servo Actuation ---
void TaskServoActuation(void *pvParameters) {
  (void) pvParameters;
  
  radarServo.attach(SERVO_PIN);
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
    vTaskDelay(MS_TO_TICKS_ROUNDED(SERVO_STEP_DELAY_MS));
  }
}

// --- Task 2: REAL Sensor Polling (HC-SR04 Trigger/Echo Mode) ---
void TaskSensorPolling(void *pvParameters) {
  (void) pvParameters;
  RawData data;

  for (;;) {
    data.angle = currentAngle;
    data.distance = read_hcsr04_distance_mm();

    xQueueSend(rawDataQueue, &data, (TickType_t)5);

    vTaskDelay(MS_TO_TICKS_ROUNDED(40));
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

  pinMode(HCSR04_TRIG_PIN, OUTPUT);
  pinMode(HCSR04_ECHO_PIN, INPUT);
  digitalWrite(HCSR04_TRIG_PIN, LOW);

  rawDataQueue = xQueueCreate(5, sizeof(RawData));
  txQueue = xQueueCreate(5, sizeof(TelemetryFrame));

  if (rawDataQueue != NULL && txQueue != NULL) {
    xTaskCreate(TaskUARTDispatch, "UART", 85, NULL, 3, NULL);
    xTaskCreate(TaskSecurity, "Crypto", 120, NULL, 2, NULL); 
    
    xTaskCreate(TaskSensorPolling, "Sensor", 100, NULL, 1, NULL);
    xTaskCreate(TaskServoActuation, "Servo", 85, NULL, 0, NULL);
  }
}

void loop() {}
