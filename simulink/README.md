# Simulink Model: Distributed Radar Simulation

Executable Simulink model of the full dual-node architecture from the
project report, built so the design (especially Stage 5, thermal printing)
can be demonstrated to the advisor **before the printer hardware arrives**.

```
Node 1 (Arduino Uno)            UART              Node 2 (Jetson Nano)         Output
Sweep + HC-SR04 scene --> XTEA/CRC16 framing --> EMI channel --> parse/decrypt
      --> sweep buffers + nearest-neighbor MTI --> live PPI display
      --> ESC/POS encoder --> virtual 58mm thermal printer --> receipt PNG
```

## Requirements

MATLAB R2025b with Simulink (no extra toolboxes).

## Quick start

```matlab
cd simulink
run_demo        % clean link: PPI animates, 2 receipts print
run_demo(1)     % EMI injection on: CRC Drops counter climbs (Metric 1 demo)
```

`run_demo` builds `radar_distributed.slx` on first use (via
`build_radar_model.m` — the model is fully generated from code, so the
`.slx` itself never needs to be committed). Receipts are saved to
`simulink/prints/receipt_cycle_NNN.png`.

Headless check that everything still works:

```matlab
test_radar_helpers   % protocol + printer self-tests
```

## What is byte-accurate vs. simulated

**Byte-accurate (same bytes as the real system):**

- 12-byte frame layout: `AA 55` sync, XTEA-encrypted 8-byte payload,
  CRC-16/CCITT — identical to `Arduino/sketch_may7d/sketch_may7d.ino` and
  `backend/src/protocol.cpp`, including the firmware's little-endian XTEA
  block layout and key.
- The ESC/POS stream sent to the printer block: `ESC @`, `ESC a`, text
  lines, `GS v 0` raster (384 dots wide, bit 7 = leftmost dot), `ESC d`,
  `GS V 0` cut. The virtual printer decodes this stream with no shared
  state — if the receipt renders, a real 58 mm printer will print it.
  This encoder is the porting reference for Stage 5 in the C++ backend.

**Simulated:**

- The HC-SR04 echo comes from `radar_scene.m`: one target drifting
  radially at 8 mm/s (bearing 60°) and one static target (bearing 120°),
  matching the report's Metric 2 setup.
- The EMI channel flips bits with p = 2e-4 per bit when `EMI Injection`
  is set to 1, standing in for the report's Metric 1 copper-wire test.
- Sweep points are clustered to one centroid per physical target before
  MTI matching (the C++ backend currently matches raw points).

## Talking points for the advisor

1. **Security layer works under fault injection** — toggle `EMI Injection`
   to 1: corrupted frames are dropped by CRC (counter on the canvas),
   phantom targets never reach the display.
2. **MTI distinguishes moving from static** — the moving target prints
   with a visible motion arrow; the static target's vector is ~zero.
3. **Stage 5 is de-risked** — the receipt is produced from a real ESC/POS
   byte stream, so the only remaining hardware work is opening the USB
   device and writing the same bytes.

## File map

| File | Role |
| --- | --- |
| `build_radar_model.m` | Generates `radar_distributed.slx` from code |
| `run_demo.m` | Builds (if needed), paces to real time, runs, prints summary |
| `radar_sim_reset.m` | Shared state init (model `InitFcn`) |
| `radar_scene.m` | Tabletop target scene (HC-SR04 stand-in) |
| `radar_xtea.m`, `radar_crc16.m` | Firmware-identical crypto/CRC |
| `radar_build_frame.m`, `radar_parse_frame.m` | Node 1 framing / Node 2 parsing |
| `radar_channel.m` | UART link with optional bit-flip EMI |
| `radar_mti_step.m` | Sweep buffering + nearest-neighbor MTI |
| `radar_ppi_draw.m` | Live half-PPI figure |
| `radar_escpos_encode.m` | PPI raster -> ESC/POS bytes (Stage 5 reference) |
| `radar_virtual_printer.m` | ESC/POS decoder + receipt PNG renderer |
| `test_radar_helpers.m` | Self-tests (CRC KAT, XTEA round trip, printer round trip) |
