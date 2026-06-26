# Multi-Sensor Radar — Implementation Plan

Cross-eyed 3-sensor ultrasonic radar on a cardboard tabletop, fused into one
top-down map, to demonstrate "seeing behind" a frontal object.
Embedded course project. Build **one sensor at a time**, with a verification
gate after each phase so faults are easy to localize.

Layout: **center** sensor sweeps ~120°; **two side** sensors sit forward and
offset left/right, each aimed across toward the center, to peek around the
edges of a frontal object into its acoustic shadow.

---

## Current baseline (already done)
- Arduino sends 20-byte frames carrying 3 sensor slots (`sensor_id` 0/1/2),
  XTEA-encrypted + CRC16. See `Arduino/sketch_may7d/sketch_may7d.ino`.
- Backend decodes correctly: verified 499 valid frames, 0 CRC/sync drops.
- Only **sensor 0** is wired; sensors 1 & 2 send distance 0 and are skipped by
  `main.cpp` (`if (telemetry.distance_mm == 0) continue;`).
- Output: one PBM per cycle in `logs/`.

---

## Phase 1 — Lock in the single sensor on the tabletop
Goal: a clean, fast, correct one-sensor map of the cardboard scene. No backend
code changes yet.

- [ ] Firmware: set `HCSR04_TIMEOUT_US` to match the table depth, not 4 m.
      - 60 cm → `3500`, 1 m → `6000`. Use ~`6000`.
      - This is also the fix for the "rotates slow" issue: empty reads drop
        from 30 ms to ~5 ms.
- [ ] (Optional) reduce `SERVO_STEP_DELAY_MS` toward ~10 ms; if the servo
      visibly lags the reported angle, back off.
- [ ] Place one cardboard object; angle its face roughly toward the sensor
      (flat faces angled away reflect sound off to the side and vanish).
- [ ] Run: `radar_app.exe --serial COMx --baud 115200`
- **Gate:** open the newest `logs/radar_*.pbm`. The object appears at the
  correct angle and distance; sweep is noticeably faster than before.

---

## Phase 2 — Per-sensor routing + pose/fusion plumbing (with ONE sensor)
> ✅ **DONE.** Implemented `pose.hpp`, per-sensor `SweepBuilder` array, fused
> `world_points` + side-sensor render overlay. Verified: unit tests pass,
> single-sensor decode is a byte-for-byte no-op, and a synthetic 3-sensor replay
> renders cleanly (sensor 0 sweep uncorrupted by sensors 1 & 2). Side-sensor
> poses in `default_poses()` are still placeholders — set them in Phase 3.

Goal: build all the multi-sensor machinery **while only sensor 0 is wired**, so
pose 0 = identity and the fused map must look identical to Phase 1. This proves
the plumbing before any second sensor can confuse it.

### 2a. Per-sensor sweep state
Today one `SweepBuilder` is fed by every point — that breaks once >1 sensor
sends real data (direction detection assumes a single sweeper).

- [ ] In `main.cpp`, replace the single `SweepBuilder sweeps;` with one per
      sensor: `std::array<radar::SweepBuilder, radar::kNumSensors> sweeps;`
- [ ] In the ingest loop, route by id: `sweeps[telemetry.sensor_id].ingest(...)`.
- [ ] Keep the existing MTI / sweep-complete logic on **sensor 0 only** for now
      (that's the rotating "radar" head). Side sensors feed the fused point
      cloud (2c) but don't need sweep/MTI for the course demo.

### 2b. Sensor pose table + polar→world transform
- [ ] New header `backend/include/radar/pose.hpp`:
      ```cpp
      struct SensorPose {
        double x_mm = 0;       // mount position in world frame
        double y_mm = 0;
        double mount_deg = 0;  // heading of the sensor's servo-0 direction
      };
      // index by sensor_id
      std::array<SensorPose, kNumSensors> default_poses();
      ```
- [ ] Transform (servo angle α in deg, distance r in mm) → world (x, y):
      ```cpp
      double h = (pose.mount_deg + alpha_deg) * M_PI / 180.0;
      double wx = pose.x_mm + r * std::cos(h);
      double wy = pose.y_mm + r * std::sin(h);
      ```
- [ ] Sensor 0 pose = {0, 0, 0} (identity, origin). Leave 1 & 2 placeholder.

### 2c. Fused Cartesian point cloud → renderer
The existing renderer is polar (PPI) from a single origin. For a fused map we
need Cartesian world points.

- [ ] Add to `RenderState` (render.hpp): `std::vector<WorldPoint> world_points;`
      where `WorldPoint { double x_mm, y_mm; std::uint8_t sensor_id; }`.
- [ ] In `main.cpp`, for every non-zero reading, push the transformed world
      point (tag with `sensor_id` for per-sensor coloring later).
- [ ] In `software_renderer.cpp`, plot `world_points` as a top-down map:
      world (0,0) at a fixed screen anchor, fixed mm-per-pixel scale, y up.
      Color by `sensor_id` (e.g. 0=green, 1=cyan, 2=magenta).
- [ ] Keep writing PBM per cycle (and/or per N frames) to `logs/`.

- **Gate:** with only sensor 0 wired, the fused top-down map shows the same
  single object in the same place as Phase 1. Nothing regressed.

---

## Phase 3 — Add the first side sensor (sensor 1) + calibrate
Goal: confirm a second viewpoint lands in the *same* world frame.

- [ ] Wire ch1: servo D11, TRIG D4, ECHO D5. Mount it forward + to one side,
      aimed across toward center.
- [ ] Firmware: give ch1 a **narrow** sweep aimed at the shadow zone (e.g.
      `minAngle`/`maxAngle` spanning ~40–60° around its look direction). Narrow
      = faster and less crosstalk.
- [ ] Measure its pose with a ruler and enter it in `default_poses()`:
      position (x,y) relative to sensor 0's origin, and `mount_deg` = the world
      heading the servo points when it reads angle 0.
- [ ] Crosstalk check: sensors already fire sequentially with a settle gap;
      if you see ghost points in the overlap region, raise `SENSOR_SETTLE_MS`
      (echo from the far edge of the scene must decay before the next ping).

- **Calibration gate:** put a single object where **both** sensor 0 and sensor 1
  can see it. Their two colored points must land on (nearly) the same world
  spot. If sensor 1's point is rotated/shifted off, fix `mount_deg` / x / y
  until they coincide. This is the make-or-break step — get it tight here.

---

## Phase 4 — Add the second side sensor (sensor 2)
- [ ] Wire ch2: servo D12, TRIG D6, ECHO D7. Mirror sensor 1 on the other side.
- [ ] Narrow sweep aimed across toward center.
- [ ] Measure + enter its pose; repeat the single-object calibration gate so all
      three sensors agree on one object's world position.

---

## Phase 5 — De-occlusion demo + polish
- [ ] Scene: a frontal "occluder" object, plus a smaller object hidden directly
      behind it from the center sensor's viewpoint.
- [ ] Show that the center sensor's points stop at the occluder (shadow behind),
      while a side sensor, peeking around the edge, marks the hidden object.
- [ ] (Optional) flag "de-occluded" points: a world point reported by a side
      sensor that falls in an angular gap / beyond the occluder in the center's
      sweep. Render these in a distinct color for the demo screenshot.
- [ ] (Optional, display) "radar map on the monitor": simplest is a small
      viewer that auto-reloads the newest `logs/radar_*.pbm`. A true live window
      needs the Vulkan path (`RADAR_ENABLE_VULKAN`) — probably more setup than
      the project needs; PBM refresh is enough.

---

## Reference: pins & per-sensor config
| Sensor | id | Servo | TRIG | ECHO | Role | Sweep |
|--------|----|-------|------|------|------|-------|
| Center | 0 | D10 | D2 | D3 | main rotating head | ~120° |
| Right  | 1 | D11 | D4 | D5 | forward-right, looks left | narrow ~40–60° |
| Left   | 2 | D12 | D6 | D7 | forward-left, looks right | narrow ~40–60° |

## Risks / gotchas (ultrasonic physics, not bugs)
- **Specular dropout:** flat faces angled away reflect sound off-axis and
  disappear. Angle object faces toward sensors; irregular objects detect best.
- **Wide beam (~15–30°):** coarse angular resolution; expect blobs, not edges.
- **Crosstalk:** crossing sightlines let side sensors hear each other — keep
  firing sequential; lengthen settle if ghosts appear.
- **Pose accuracy:** the whole fused map is only as good as the measured poses.
  Calibrate with a single shared object (Phase 3/4 gates).
- **Rebuild reminder:** any backend C++ edit needs
  `cmake --build build --target radar_app` before running — a stale binary is
  what caused the original "no data" (the exe predated the protocol).

## Files to touch
- `Arduino/sketch_may7d/sketch_may7d.ino` — timeout, per-channel sweep ranges.
- `backend/include/radar/pose.hpp` *(new)* — pose table + transform.
- `backend/include/radar/render.hpp` — add `world_points` to `RenderState`.
- `backend/src/main.cpp` — per-sensor `SweepBuilder`, transform + push world pts.
- `backend/src/software_renderer.cpp` — Cartesian top-down plot, color by id.
- `backend/CMakeLists.txt` — add `pose.cpp` if you split the impl out.
