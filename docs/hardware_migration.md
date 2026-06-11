# Hardware Migration Notes

## Final Hardware

- Controller: Arduino Uno
- Sensor: HC-SR04 ultrasonic distance sensor
- Servo: SG90 micro servo
- Edge node: Jetson Nano or another Linux/Windows host running the backend
- Display: HDMI display attached to the edge node

## Wiring

| Device | Signal | Arduino / Supply |
| --- | --- | --- |
| SG90 | Signal | D10 |
| SG90 | VCC | External regulated 5V |
| SG90 | GND | Shared ground with Arduino |
| HC-SR04 | TRIG | D2 |
| HC-SR04 | ECHO | D3 |
| HC-SR04 | VCC | 5V |
| HC-SR04 | GND | Shared ground |

The SG90 should not be powered from the Arduino 5V pin during sweep tests. Use an external regulated 5V supply and tie its ground to the Arduino ground.

## Firmware Behavior

The active sketch is `Arduino/sketch_may7d/sketch_may7d.ino`.

The backend protocol remains unchanged:

- Bytes `0..1`: sync bytes `0xAA 0x55`
- Bytes `2..9`: XTEA-encrypted 8-byte payload
- Bytes `10..11`: CRC-16 CCITT over bytes `0..9`
- Plain payload before encryption:
  - Byte `0`: servo angle in degrees
  - Byte `1`: distance high byte
  - Byte `2`: distance low byte
  - Bytes `3..7`: zero padding

The old US-100 UART request/read flow is replaced by HC-SR04 trigger/echo timing:

1. Drive TRIG low for 2 us.
2. Drive TRIG high for 10 us.
3. Drive TRIG low.
4. Measure ECHO high duration with a 30000 us timeout.
5. Convert the echo duration to millimeters with `duration_us * 343 / 2000`.
6. Emit distance `0` when the echo times out.

Distance `0` is a valid timeout marker. The backend counts it but excludes it from MTI matching.

## Calibration

The default SG90 sweep is `0..180` degrees with `15 ms` per degree. If the physical assembly binds near the endpoints, change the firmware sweep limits to a calibrated range such as `10..170` and record that range here.

Recommended hardware checks:

- Confirm HC-SR04 readings at 100 mm, 200 mm, and 400 mm.
- Confirm timeout readings produce distance `0`.
- Confirm the SG90 sweep is smooth and does not brown out the Arduino.
- Confirm the backend receives valid encrypted frames at 115200 baud.
