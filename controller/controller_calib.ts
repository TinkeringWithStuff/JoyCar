// =====================================================================
// CONTROLLER  -  guided wheel calibration (step 3a)
// micro:bit v2 in joystick:bit + Kitronik OLED
// MakeCode JavaScript. Extensions needed: "joystickbit", "128x64Display"
//
// Part 1, distance (6 runs):
//   C  = drive straight 3 s (forward, then backward, alternating).
//        When the car has stopped: measure how far it went, then C again.
// Part 2, turning (2 spins: left, then right):
//   Line the car up with a line on the floor, press C to start.
//   HOLD D to spin, release to stop (tap D to fine-tune).
//   When it has turned exactly 360 deg, press C to record.
//
// The joystick still drives the car between runs (to reposition it).
// Every result is printed over USB, e.g.  run,1,fwd,512,498
// Flash your normal controller program back when you are done.
// =====================================================================

const RADIO_GROUP = 42
const MSG_DRIVE = 1
const MSG_STATUS = 8
const MSG_RESET_ODO = 9
const MSG_TEST_DRIVE = 10

const DIST_RUNS = 6
const RUN_SPEED = 50          // % motor power for the straight runs
const RUN_TENTHS = 30         // 3.0 s
const SPIN_SPEED = 40         // % motor power while D is held
const SETTLE_MS = 700         // wait for the car to roll to a stop

const DEADZONE = 12
const INVERT_X = true
const INVERT_Y = false
const STATUS_TIMEOUT_MS = 1000

// States
const S_DIST_READY = 0        // waiting for C to start a straight run
const S_DIST_RUNNING = 1      // car is driving
const S_DIST_DONE = 2         // run finished, user measures
const S_SPIN_READY = 3        // waiting for C to start a spin (align first)
const S_SPINNING = 4          // user holds/taps D, C records
const S_FINISHED = 5

radio.setGroup(RADIO_GROUP)
radio.setTransmitPower(7)
joystickbit.initJoystickBit()

const centreX = joystickbit.getRockerValue(joystickbit.rockerType.X)
const centreY = joystickbit.getRockerValue(joystickbit.rockerType.Y)

let ticksLeft = 0
let ticksRight = 0
let carFlags = 0
let rawPins = 0          // diagnostic: bit0 = P14, bit1 = P15 on the car
let expanderByte = 0     // diagnostic: the car's I/O expander, all 8 bits
let logLines: string[] = []
let lastStatusMs = -10000

let state = S_DIST_READY
let runNo = 1                 // 1..DIST_RUNS
let spinNo = 1                // 1..2
let runStartMs = 0
let seenTestFlag = false
let dHeld = false

function axis(raw: number, centre: number, invert: boolean): number {
    let v = raw - centre
    let span = v >= 0 ? 1023 - centre : centre
    if (span < 1) span = 1
    v = Math.idiv(v * 100, span)
    if (Math.abs(v) < DEADZONE) v = 0
    if (invert) v = -v
    return Math.constrain(v, -100, 100)
}

function padRight(text: string, width: number): string {
    while (text.length < width) {
        text = text + " "
    }
    return text.substr(0, width)
}

function resetOdo() {
    const b = pins.createBuffer(2)
    b.setNumber(NumberFormat.UInt8LE, 0, MSG_RESET_ODO)
    b.setNumber(NumberFormat.UInt8LE, 1, 0)
    radio.sendBuffer(b)
    ticksLeft = 0
    ticksRight = 0
}

function testDrive(left: number, right: number, tenths: number) {
    const b = pins.createBuffer(4)
    b.setNumber(NumberFormat.UInt8LE, 0, MSG_TEST_DRIVE)
    b.setNumber(NumberFormat.Int8LE, 1, left)
    b.setNumber(NumberFormat.Int8LE, 2, right)
    b.setNumber(NumberFormat.UInt8LE, 3, tenths)
    radio.sendBuffer(b)
}

// Keep every result and re-print the whole log each time, so a console
// that connects late still gets everything
function logResult(line: string) {
    logLines.push(line)
    serial.writeLine("--- log ---")
    for (let i = 0; i < logLines.length; i++) {
        serial.writeLine(logLines[i])
    }
}

function bits8(v: number): string {
    let t = ""
    for (let b = 7; b >= 0; b--) {
        t = t + ((v >> b) & 1)
    }
    return t
}

function runIsForward(): boolean {
    return runNo % 2 == 1
}

// ---- Button C: advances the procedure ----
joystickbit.onButtonEvent(joystickbit.JoystickBitPin.P12, joystickbit.ButtonType.down, function () {
    if (state == S_DIST_READY) {
        resetOdo()
        basic.pause(100)
        const p = runIsForward() ? RUN_SPEED : -RUN_SPEED
        // sent three times in case one packet is lost (only extends the run by a few ms)
        testDrive(p, p, RUN_TENTHS)
        basic.pause(20)
        testDrive(p, p, RUN_TENTHS)
        basic.pause(20)
        testDrive(p, p, RUN_TENTHS)
        runStartMs = input.runningTime()
        seenTestFlag = false
        state = S_DIST_RUNNING
    } else if (state == S_DIST_DONE) {
        if (runNo < DIST_RUNS) {
            runNo++
            state = S_DIST_READY
        } else {
            state = S_SPIN_READY
        }
    } else if (state == S_SPIN_READY) {
        resetOdo()
        state = S_SPINNING
    } else if (state == S_SPINNING) {
        logResult("spin," + spinNo + "," + (spinNo == 1 ? "left" : "right") + "," + ticksLeft + "," + ticksRight)
        if (spinNo < 2) {
            spinNo++
            state = S_SPIN_READY
        } else {
            state = S_FINISHED
        }
    }
})

// ---- Button D: hold to spin ----
joystickbit.onButtonEvent(joystickbit.JoystickBitPin.P13, joystickbit.ButtonType.down, function () {
    dHeld = true
})
joystickbit.onButtonEvent(joystickbit.JoystickBitPin.P13, joystickbit.ButtonType.up, function () {
    dHeld = false
})

radio.onReceivedBuffer(function (buf: Buffer) {
    if (buf.length < 11) return
    if (buf.getNumber(NumberFormat.UInt8LE, 0) != MSG_STATUS) return
    ticksLeft = buf.getNumber(NumberFormat.Int32LE, 1)
    ticksRight = buf.getNumber(NumberFormat.Int32LE, 5)
    carFlags = buf.getNumber(NumberFormat.UInt8LE, 10)
    if (buf.length >= 13) {
        rawPins = buf.getNumber(NumberFormat.UInt8LE, 11)
        expanderByte = buf.getNumber(NumberFormat.UInt8LE, 12)
    }
    lastStatusMs = input.runningTime()
})

kitronik_VIEW128x64.clear()

// ---- Drive loop, 20 per second ----
basic.forever(function () {
    if (state == S_SPINNING && dHeld) {
        // spin on the spot; keep-alive of 0.2 s so it stops soon after release
        if (spinNo == 1) {
            testDrive(-SPIN_SPEED, SPIN_SPEED, 2)   // left wheel back, right forward = turn left
        } else {
            testDrive(SPIN_SPEED, -SPIN_SPEED, 2)   // turn right
        }
    } else {
        const sx = Math.idiv(axis(joystickbit.getRockerValue(joystickbit.rockerType.X), centreX, INVERT_X), 2)
        const sy = Math.idiv(axis(joystickbit.getRockerValue(joystickbit.rockerType.Y), centreY, INVERT_Y), 2)
        const drv = pins.createBuffer(3)
        drv.setNumber(NumberFormat.UInt8LE, 0, MSG_DRIVE)
        drv.setNumber(NumberFormat.Int8LE, 1, sx)
        drv.setNumber(NumberFormat.Int8LE, 2, sy)
        radio.sendBuffer(drv)
    }
    basic.pause(50)
})

// ---- Run finished? (watch the car's "test running" flag) ----
basic.forever(function () {
    if (state == S_DIST_RUNNING) {
        if ((carFlags & 2) != 0) seenTestFlag = true
        const elapsed = input.runningTime() - runStartMs
        const finished = seenTestFlag && (carFlags & 2) == 0
        if (finished || elapsed > RUN_TENTHS * 100 + 2000) {
            basic.pause(SETTLE_MS)                 // let it roll to a stop
            logResult("run," + runNo + "," + (runIsForward() ? "fwd" : "back") + "," + ticksLeft + "," + ticksRight)
            state = S_DIST_DONE
        }
    }
    basic.pause(50)
})

// ---- Screen, 4 times per second (the only place that writes to the OLED) ----
basic.forever(function () {
    const big = kitronik_VIEW128x64.FontSelection.Big
    const leftAlign = kitronik_VIEW128x64.ShowAlign.Left
    let title = ""
    let msg = ""
    let hint = ""
    if (state <= S_DIST_DONE) {
        title = "DISTANCE run " + runNo + "/" + DIST_RUNS + " " + (runIsForward() ? "FWD" : "BACK")
        if (state == S_DIST_READY) {
            msg = runNo == 1 ? "Car at start mark" : "Mark where it is"
            hint = "C = drive 3 s"
        } else if (state == S_DIST_RUNNING) {
            msg = "Driving..."
            hint = ""
        } else {
            msg = "Measure distance now"
            hint = runNo < DIST_RUNS ? "C = next run" : "C = go to turning test"
        }
    } else if (state <= S_SPINNING) {
        title = "TURN " + spinNo + "/2 " + (spinNo == 1 ? "LEFT" : "RIGHT")
        if (state == S_SPIN_READY) {
            msg = "Line car up with a line"
            hint = "C = start"
        } else {
            msg = "Hold D to spin 360 deg"
            hint = "C = record"
        }
    } else {
        title = "CALIBRATION DONE"
        msg = "Send Claude the USB log"
        hint = "+ your distances"
    }

    kitronik_VIEW128x64.show(padRight(title, 25), 1)
    if (input.runningTime() - lastStatusMs > STATUS_TIMEOUT_MS) {
        kitronik_VIEW128x64.show(padRight("NO CAR DATA", 12), 2, leftAlign, big)
        kitronik_VIEW128x64.show(padRight("", 25), 5)
    } else {
        kitronik_VIEW128x64.show(padRight("L" + ticksLeft + " R" + ticksRight, 12), 2, leftAlign, big)
        // raw: wheel pins P14/P15, then the I/O expander bits 7..0
        kitronik_VIEW128x64.show(padRight("P14:" + (rawPins & 1) + " P15:" + ((rawPins >> 1) & 1) + " X:" + bits8(expanderByte), 25), 5)
    }
    kitronik_VIEW128x64.show(padRight(msg, 25), 7)
    kitronik_VIEW128x64.show(padRight(hint, 25), 8)
    basic.pause(250)
})
