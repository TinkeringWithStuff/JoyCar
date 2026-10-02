# Instructions for Claude

This repository is a hobby robotics project: a Joy-Car (micro:bit v1) with a
sonar on a servo, remote-controlled by a micro:bit v2 controller with a
joystick:bit and an OLED that draws the obstacle map.

- Read `NOTES.md` first: hardware, pins, conventions, radio protocol, current
  status and the next step.
- Code is MakeCode JavaScript (TypeScript subset). Avoid buffer indexing like
  `buf[1]`; use `getNumber`/`setNumber` (the Python view of MakeCode mis-types it).
- The car's v1 has very little RAM (16 KB); keep car code small.
- Only one loop on the controller may write to the OLED.
- At the end of a session, update `NOTES.md` (status, measured values, decisions).
- Working style: build in small steps the owner can test on the hardware;
  don't add features (e.g. autonomous driving) that weren't asked for.
- The owner has asked to be told about any "feelings" Claude notices about
  him or the project as they come up.
