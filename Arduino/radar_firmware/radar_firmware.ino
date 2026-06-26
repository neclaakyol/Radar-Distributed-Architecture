// =============================================================================
//  Radar Node — Final Firmware
// =============================================================================
//  3 servo heads, each carrying an ultrasonic ranger. The main head sweeps a
//  0-120 degree arc; the two flank heads each sweep 0-180 degrees. A single
//  FreeRTOS coordinator reads the three sensors in round-robin order
//  (main -> left -> right), batches one reading from each into a 16-byte
//  payload, encrypts it as two XTEA blocks, frames it with sync bytes + CRC-16
//  (20 bytes total), and streams it to the Jetson over UART.
//
//  Sensors: 1x HY-SRF05 (main, sensor_id 0) + 2x HC-SR04 (flanks, sensor_id
//  1/2). The HY-SRF05 mode pin is left unconnected -> trig/echo mode, identical
//  timing to HC-SR04, so one read routine drives all three.
//  Board: Arduino Uno (FreeRTOS ticks off the Watchdog Timer, Servo library
//  drives all three servos off Timer1 — no conflict).
//
//  Wire-compatible with backend/src/protocol.cpp (decode_frame / FrameParser).
// =============================================================================

#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Servo.h>

// --- System Definitions ---
// 38400 (not 115200): the Uno's UART is ~2% off at 115200, which the Jetson
// Tegra UART + level-shifter margin pushes past framing tolerance (garbled
// bytes / 0x00 framing errors). At 38400 the Uno error is ~0.16% and the link
// is reliable; data rate need is well under 200 B/s. Run the backend with
// --baud 38400 to match.
#define BAUD_RATE 38400
#define SYNC_BYTE_1 0xAA
#define SYNC_BYTE_2 0x55
// Start/stop handshake: servos idle (parked, no sweep, no telemetry) until the
// backend sends START_COMMAND_BYTE. On connection close the backend sends
// STOP_COMMAND_BYTE: sweeping halts, all heads return to minAngle, and the
// firmware re-enters the idle wait so the next connect can restart cleanly.
// Both bytes must match the constants in backend/src/serial_source.cpp.
#define START_COMMAND_BYTE 0x53 // 'S'
#define STOP_COMMAND_BYTE 0x58  // 'X'
#define HCSR04_TIMEOUT_US 30000UL
#define SENSOR_SETTLE_DELAY_MS 5
#define SERVO_STEP_DELAY_MS 15
#define MS_TO_TICKS_ROUNDED(ms) ((TickType_t)(((ms) + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS))

// --- Per-channel data model ---
struct ScanChannel
{
  uint8_t id;
  Servo servo;
  uint8_t servoPin;
  uint8_t trigPin;
  uint8_t echoPin;
  uint8_t angle;
  uint8_t minAngle;
  uint8_t maxAngle;
  bool sweepingForward;
  uint16_t lastDistance;
  bool lastTimeout;
};

ScanChannel channels[3];

// --- Queue payload types ---
struct RawData3
{
  uint8_t angles[3];
  uint16_t distances[3];
  bool timeouts[3];
  uint8_t cycleCounter;
};

// 20-byte frame: 2 sync + 16 encrypted + 2 CRC
struct TelemetryFrame
{
  uint8_t bytes[20];
};

// --- FreeRTOS Handles ---
QueueHandle_t rawDataQueue;
QueueHandle_t txQueue;

// --- XTEA key ---
const uint32_t xtea_key[4] = {0x12345678, 0x9ABCDEF0, 0x11223344, 0x55667788};

// --- Crypto & CRC helpers ---
void xtea_encrypt(uint32_t v[2], uint32_t const key[4])
{
  unsigned int i;
  uint32_t v0 = v[0], v1 = v[1], sum = 0, delta = 0x9E3779B9;
  for (i = 0; i < 32; i++)
  {
    v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
    sum += delta;
    v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum >> 11) & 3]);
  }
  v[0] = v0;
  v[1] = v1;
}

uint16_t crc16_ccitt(const uint8_t *data, uint8_t length)
{
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// Shared trig/echo timing routine; works for HC-SR04 and HY-SRF05 in trig/echo mode.
uint16_t read_sensor_distance_mm(uint8_t trigPin, uint8_t echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration_us = pulseIn(echoPin, HIGH, HCSR04_TIMEOUT_US);
  if (duration_us == 0)
    return 0;
  unsigned long distance_mm = (duration_us * 343UL) / 2000UL;
  if (distance_mm > 65535UL)
    return 65535;
  return (uint16_t)distance_mm;
}

// Block until the backend signals start. Servos stay attached and parked at
// their home angle; no readings are taken and no telemetry is emitted until the
// start byte arrives. The UART RX buffer is drained looking for the command, so
// the backend may send it repeatedly (to cover the post-reset bootloader window)
// without ill effect. Returns once the start byte has been seen.
static void waitForStartCommand()
{
  for (;;)
  {
    while (Serial.available() > 0)
    {
      if (Serial.read() == START_COMMAND_BYTE)
        return;
    }
    vTaskDelay(MS_TO_TICKS_ROUNDED(20));
  }
}

// --- Task 1: Scan Coordinator ---
// Round-robin: read main -> left -> right, pack RawData3, advance all 3 servos.
// Outer loop handles the full connection lifecycle: home → idle → sweep → home.
// When the backend disconnects it sends STOP_COMMAND_BYTE; the sweep loop breaks,
// all heads return to minAngle, and the task re-enters waitForStartCommand() so
// the next connect starts cleanly without a firmware reboot.
void TaskScanCoordinator(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  { // connection lifecycle: home -> wait -> sweep -> home -> ...
    // Drive every head to its start angle (minAngle) and give it time to physically
    // travel there. The Servo library keeps pulsing on Timer1, so the heads are
    // actively held at the start position -- not floating -- for the whole wait.
    for (uint8_t i = 0; i < 3; i++)
    {
      channels[i].angle = channels[i].minAngle;
      channels[i].sweepingForward = true;
      channels[i].servo.attach(channels[i].servoPin);
      channels[i].servo.write(channels[i].angle);
    }
    vTaskDelay(MS_TO_TICKS_ROUNDED(500)); // settle at start position before idling

    // Hold at the start position until the backend connects and sends the start
    // command. While we block, the security/UART tasks sit empty (no data is
    // queued), so the whole pipeline stays quiet and the heads stay put at start.
    waitForStartCommand();

    uint8_t cycleCounter = 0;
    RawData3 data;

    for (;;)
    { // sweep loop — exits on STOP_COMMAND_BYTE
      // Drain RX buffer; any STOP_COMMAND_BYTE breaks the sweep.
      bool stopReceived = false;
      while (Serial.available() > 0)
      {
        if (Serial.read() == STOP_COMMAND_BYTE)
        {
          stopReceived = true;
          break;
        }
      }
      if (stopReceived)
        break;

      // Read all 3 channels; brief settle gap between triggers to avoid acoustic crosstalk.
      for (uint8_t i = 0; i < 3; i++)
      {
        channels[i].lastDistance = read_sensor_distance_mm(channels[i].trigPin, channels[i].echoPin);
        channels[i].lastTimeout = (channels[i].lastDistance == 0);
        data.angles[i] = channels[i].angle;
        data.distances[i] = channels[i].lastDistance;
        data.timeouts[i] = channels[i].lastTimeout;
        if (i < 2)
        {
          vTaskDelay(MS_TO_TICKS_ROUNDED(SENSOR_SETTLE_DELAY_MS));
        }
      }

      data.cycleCounter = cycleCounter++;
      xQueueSend(rawDataQueue, &data, (TickType_t)5);

      // Advance each channel's angle independently, then write servos simultaneously.
      for (uint8_t i = 0; i < 3; i++)
      {
        if (channels[i].sweepingForward)
        {
          channels[i].angle++;
          if (channels[i].angle >= channels[i].maxAngle)
            channels[i].sweepingForward = false;
        }
        else
        {
          channels[i].angle--;
          if (channels[i].angle <= channels[i].minAngle)
            channels[i].sweepingForward = true;
        }
        channels[i].servo.write(channels[i].angle);
      }

      vTaskDelay(MS_TO_TICKS_ROUNDED(SERVO_STEP_DELAY_MS));
    }
    // Stop received: return all heads to minAngle before re-entering idle wait.
    for (uint8_t i = 0; i < 3; i++)
    {
      channels[i].angle = channels[i].minAngle;
      channels[i].sweepingForward = true;
      channels[i].servo.write(channels[i].angle);
    }
    vTaskDelay(MS_TO_TICKS_ROUNDED(500)); // settle at home before next wait
  }
}

// --- Task 2: Security & Framing (XTEA × 2 blocks + CRC16, 20-byte frame) ---
// Plaintext payload (16 bytes):
//   [0]   sensor_id 0   [1]  angle 0    [2-3]  dist 0 (hi,lo)
//   [4]   sensor_id 1   [5]  angle 1    [6-7]  dist 1 (hi,lo)
//   [8]   sensor_id 2   [9]  angle 2    [10-11] dist 2 (hi,lo)
//   [12]  cycle counter [13] status flags (bit0=ch0 timeout, bit1=ch1, bit2=ch2)
//   [14-15] reserved
void TaskSecurity(void *pvParameters)
{
  (void)pvParameters;
  RawData3 incomingData;
  TelemetryFrame outgoingFrame;

  for (;;)
  {
    if (xQueueReceive(rawDataQueue, &incomingData, portMAX_DELAY) == pdPASS)
    {
      uint32_t block1[2] = {0, 0};
      uint32_t block2[2] = {0, 0};
      uint8_t *b1 = (uint8_t *)block1;
      uint8_t *b2 = (uint8_t *)block2;

      // Block 1: sensors 0 and 1
      b1[0] = 0; // sensor_id = 0 (main)
      b1[1] = incomingData.angles[0];
      b1[2] = (incomingData.distances[0] >> 8) & 0xFF;
      b1[3] = incomingData.distances[0] & 0xFF;
      b1[4] = 1; // sensor_id = 1 (left)
      b1[5] = incomingData.angles[1];
      b1[6] = (incomingData.distances[1] >> 8) & 0xFF;
      b1[7] = incomingData.distances[1] & 0xFF;

      // Block 2: sensor 2 + metadata
      b2[0] = 2; // sensor_id = 2 (right)
      b2[1] = incomingData.angles[2];
      b2[2] = (incomingData.distances[2] >> 8) & 0xFF;
      b2[3] = incomingData.distances[2] & 0xFF;
      b2[4] = incomingData.cycleCounter;
      uint8_t flags = 0;
      if (incomingData.timeouts[0])
        flags |= 0x01;
      if (incomingData.timeouts[1])
        flags |= 0x02;
      if (incomingData.timeouts[2])
        flags |= 0x04;
      b2[5] = flags;
      b2[6] = 0;
      b2[7] = 0;

      xtea_encrypt(block1, xtea_key);
      xtea_encrypt(block2, xtea_key);

      outgoingFrame.bytes[0] = SYNC_BYTE_1;
      outgoingFrame.bytes[1] = SYNC_BYTE_2;
      for (int i = 0; i < 8; i++)
        outgoingFrame.bytes[i + 2] = b1[i];
      for (int i = 0; i < 8; i++)
        outgoingFrame.bytes[i + 10] = b2[i];

      uint16_t crc = crc16_ccitt(outgoingFrame.bytes, 18);
      outgoingFrame.bytes[18] = (crc >> 8) & 0xFF;
      outgoingFrame.bytes[19] = crc & 0xFF;

      xQueueSend(txQueue, &outgoingFrame, (TickType_t)5);
    }
  }
}

// --- Task 3: UART Dispatch ---
void TaskUARTDispatch(void *pvParameters)
{
  (void)pvParameters;
  TelemetryFrame frameToSend;

  for (;;)
  {
    if (xQueueReceive(txQueue, &frameToSend, portMAX_DELAY) == pdPASS)
    {
      Serial.write(frameToSend.bytes, 20);
    }
  }
}

// --- Setup ---
void setup()
{
  Serial.begin(BAUD_RATE);
  while (!Serial)
  {
    ;
  }

  // Channel 0 — main head, HY-SRF05 (mode pin unconnected → trig/echo mode, same
  // timing as HC-SR04). Origin sensor, 0-120° sweep.
  channels[0].id = 0;
  channels[0].servoPin = 11;
  channels[0].trigPin = 6;
  channels[0].echoPin = 7;
  channels[0].angle = 0;
  channels[0].minAngle = 0;
  channels[0].maxAngle = 120;
  channels[0].sweepingForward = true;
  channels[0].lastDistance = 0;
  channels[0].lastTimeout = false;

  // Channel 1 — left flank head, HC-SR04, 0-180° sweep.
  channels[1].id = 1;
  channels[1].servoPin = 12;
  channels[1].trigPin = 4;
  channels[1].echoPin = 5;
  channels[1].angle = 0;
  channels[1].minAngle = 0;
  channels[1].maxAngle = 180;
  channels[1].sweepingForward = true;
  channels[1].lastDistance = 0;
  channels[1].lastTimeout = false;

  // Channel 2 — right flank head, HC-SR04, 0-180° sweep.
  channels[2].id = 2;
  channels[2].servoPin = 10;
  channels[2].trigPin = 2;
  channels[2].echoPin = 3;
  channels[2].angle = 0;
  channels[2].minAngle = 0;
  channels[2].maxAngle = 180;
  channels[2].sweepingForward = true;
  channels[2].lastDistance = 0;
  channels[2].lastTimeout = false;

  for (uint8_t i = 0; i < 3; i++)
  {
    pinMode(channels[i].trigPin, OUTPUT);
    pinMode(channels[i].echoPin, INPUT);
    digitalWrite(channels[i].trigPin, LOW);
  }

  rawDataQueue = xQueueCreate(5, sizeof(RawData3));
  txQueue = xQueueCreate(5, sizeof(TelemetryFrame));

  if (rawDataQueue != NULL && txQueue != NULL)
  {
    xTaskCreate(TaskUARTDispatch, "UART", 90, NULL, 3, NULL);
    xTaskCreate(TaskSecurity, "Crypto", 160, NULL, 2, NULL);
    xTaskCreate(TaskScanCoordinator, "Scan", 200, NULL, 1, NULL);
  }
}

void loop() {}
