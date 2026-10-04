# Joy-Car sonar mapper — project notes

Last updated: 2026-10-02 (end of afternoon session)

## Goal

A Joy-Car (micro:bit v1) scans its surroundings with a sonar on a servo and
sends the readings by radio to a controller (micro:bit v2 with joystick:bit and
Kitronik 128x64 OLED), which draws an obstacle map. Later: track the car's
position from the wheel sensors so scans combine into one map of the room,
an automatic "scan area" mode, and saved maps the car can locate itself in.

## Hardware

| Part | Details |
| --- | --- |
| Car | Joy-IT Joy-Car, micro:bit **v1**, mainboard **older than rev 1.2** (no revision printed on the back) |
| Controller | micro:bit **v2** in ELECFREAKS joystick:bit, Kitronik :VIEW 128x64 OLED in between |
| Sonar | HC-SR04-type on servo 1 (P1). TRIG = P8, ECHO = P12 |
| Wheel speed sensors | Slotted discs, on the I/O expander (I2C 56): bit 0 = left, bit 1 = right. (Rev 1.3 boards use P14/P15 instead.) Polled about every 6 ms |
| Obstacle sensors (IR) | Front left/right, on the I/O expander (I2C address 56), bit 5 = left, bit 6 = right, 0 = obstacle |
| Motors | Via the Joy-Car PWM chip (I2C 112). `JoyCar.drivePwm(ch2, ch3, ch4, ch5)`: ch2 = right reverse, ch3 = right forward, ch4 = left reverse, ch5 = left forward |
| OLED | I2C address 60 |
| Joystick:bit | Stick X = P1, Y = P2; buttons C = P12, D = P13, E = P14, F = P15 |

## Conventions and calibration

- Servo angle: **0° = right, 90° = straight ahead, 180° = left**.
  Map maths: x = d·cos θ (right), y = d·sin θ (ahead).
- Servo trim: **0** (`SERVO_OFFSET_DEG` in car code). The horn was reseated on
  2026-10-02 after a −19° trim; it had been mounted while the servo was not at 90°.
- Joystick X is inverted (`INVERT_X = true` in controller code).
- Distances are whole cm. In scan data: 0 = no echo, −1 = not received
  (controller side), −2 / 65535 = angle out of servo reach.
- Wheel calibration (2026-10-04, two rounds of 6 timed runs + one 720° spin):
  - **0.51 cm per tick, both wheels** (≈ 2 ticks/cm; a tick = one edge of the
    slotted disc). Same at 50/65/80 % power, so no ticks are missed.
    (A first fit gave 0.48 left / 0.511 right and a speed dependence - both
    were artifacts of the car curving: chord shorter than the path.)
  - **Effective wheel base ≈ 15.2 cm**; about 93 ticks per wheel per 360° spin.
  - At equal power the left motor is faster: L/R ticks 1.13–1.18 forward,
    1.19–1.31 backward. Car veers right going forward, left going backward.
  - One motor will not start at 30 % power; 50 % works.
  - Raw data: fwd 50 % 110/97 ticks 51 cm (veer 9.5); back 50 % 111/85 47.5 cm
    (13); fwd 65 % 139/118 61 cm (8); back 65 % 127/100 54 cm (17); fwd 80 %
    162/138 70.5 cm (28); back 80 % 151/127 64 cm (11); spin right 720° 218/−156.

## Radio protocol (group 42)

| Type | Direction | Bytes |
| --- | --- | --- |
| 1 DRIVE | ctrl → car | x:int8, y:int8 (−100..100), 20 per second |
| 4 SWEEP_REQUEST | ctrl → car | stepDeg:uint8 |
| 5 SWEEP_DATA | car → ctrl | sweepId, firstIndex, count, stepDeg, then dist:uint16LE × count (max 7 per packet) |
| 6 SERVO_TRIM | ctrl → car | nudge:int8 (0 = just report) |
| 7 TRIM_VALUE | car → ctrl | offset:int8 |
| 8 STATUS | car → ctrl | ticksLeft:int32LE, ticksRight:int32LE, ir:uint8, flags:uint8 (bit0 scanning, bit1 test drive), 10 per second |
| 9 RESET_ODO | ctrl → car | 0 |
| 10 TEST_DRIVE | ctrl → car | left:int8, right:int8, tenthsOfSecond:uint8 |

Radio payload limit in MakeCode: 19 bytes per packet.

## Current code

- `car/car.ts` — car (step 3b): driving with straight-line speed matching, 5° two-pass scan, servo trim, wheel
  tick counting, obstacle sensors, status messages, test drives.
- `controller/controller.ts` — controller (step 3b): driving, scan request,
  CLEAN / RAW radar views, tables, log scale (default) + linear scales,
  auto-zoom, and the MAP view: position + heading from the wheel ticks
  (dead reckoning), trail, 50 cm grid, zoom 1/2/4/8 cm per px.
  Buttons: C scan, D/E servo trim, F range/zoom, A view
  (CLEAN -> MAP -> RAW -> table 1 -> table 2), B zero position, A+B show trim.
- `controller/controller_calib.ts` — guided wheel calibration tool (step 3a).

All are MakeCode JavaScript. Extensions: car = "Joy-Car"; controller =
"joystickbit" + "128x64Display".

## Where we are / next step

- Done: driving, scanning, radio transfer, OLED radar map (CLEAN view via
  "regions of constant depth", log scale), servo calibration.
- Done (step 3a): wheel calibration, values above.
- Done (step 3b, part 1): straight-line speed matching in the car
  (`drive()`): fixed left-power factor 0.92 forward / 0.85 backward, plus a
  correction of 12 % power per cm one wheel is ahead.
  - Round 2 (target L/R = 1.065): drift down from 8-28 cm to 4-10 cm; ticks
    held exactly at the target, so the target was wrong. Now target = equal
    ticks (0.51 cm/tick both).
  - Round 3 (equal ticks): **drift 0-3 cm**, L/R within 2 ticks. Done.
    Raw: fwd 50 % 98/96 50 cm (veer 2); back 50 % 80/78 39 (1); fwd 65 %
    126/125 65 (0); back 65 % 101/100 52 (3); fwd 80 % 147/146 77 (1.5);
    back 80 % 123/123 64 (3). cm/tick 0.49-0.53, mean ≈ 0.515.
  - Round 2 raw: fwd 50 % 104/97 51 cm (veer 4.5); back 50 % 80/74 41 (4);
    fwd 65 % 128/120 64 (8); back 65 % 110/103 54 (10); fwd 80 % 150/141 75
    (10); back 80 % 128/119 63 (9).
- History of step 3a: First attempts gave 0 ticks:
  the code assumed a rev 1.3 board (sensors on P14/P15), but this board is
  older and the sensors are on the I/O expander. Ticks are now counted by
  polling the expander (bits 0/1) - but the 2026-10-02 runs at 30/50/70 %
  STILL gave 0 ticks.
- **Next diagnostic (open):** car lifted, watch the raw line on the
  calibration screen (`WL:x WR:x X:bbbbbbbb`, X = expander bits 7..0) while
  (1) turning each wheel slowly by hand, (2) covering each obstacle sensor.
  - X changes for obstacle sensors but not wheels -> wheel sensors not giving
    a signal (plugging/socket/adjustment; check for LEDs that blink).
  - X changes for wheels on other bits -> fix the bit numbers.
  - X never changes -> the expander read itself is failing.
  Also check whether the wheel sensors have LEDs that blink as the wheel turns.
- **In progress:** step 3b part 2 - position tracking on the controller (MAP
  view). Written, needs testing: drive a known square or out-and-back and
  compare the end position/heading with reality.
- Then step 3c - place each scan (and obstacle-sensor hits) on that map.
- Pose convention: x = right, y = ahead at the start, heading 0 = start
  direction, + = turned left.

## Roadmap

3. Wheel tracking + obstacle sensors, one combined map while driving manually.
4. Map grid on the controller (5–10 cm cells), pan/zoom with the joystick.
5. Correct position drift by matching scans to the map (maybe in C++).
6. Automatic "scan area" mode.
7. Save/load maps (USB to computer), recognise where the car is (particle filter).

Ideas for later are kept in a separate doc ("Joy-Car mapper – ideas for later").

## Decisions and lessons

- Boards swapped: the v1 (16 KB RAM) is on the car, the v2 holds the map.
- MakeCode JavaScript instead of MicroPython (memory and speed on the v1).
- Stop-and-scan: the car stops while scanning.
- The OLED is drawn into our own 1 KB frame buffer and sent in one I2C write;
  the Kitronik `setPixel` sends 4 I2C messages per pixel.
- Only one loop on the controller writes to the OLED (an earlier freeze was
  probably two loops writing at once).
- Sonar physics: a flat wall is only seen at its closest point (mirror-like
  reflection); the beam is ~20–30° wide. Hence the CLEAN view, and the plan to
  combine scans from several positions.
- Check the mainboard revision before relying on pin maps: no number printed = older than 1.2.
- On rev 1.3 boards, Joy-Car library functions that read P14/P15 would disable pulse events on those pins.
