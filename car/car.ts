// =====================================================================
// CAR  -  micro:bit v1 in the Joy-Car  -  Step 3b: wheel ticks + straight-line speed matching
// MakeCode JavaScript. Extension needed: "Joy-Car" (joy-it/Joy-Car)
//
// Does:   drives from joystick packets,
//         on request: stops and scans 0..180 deg in the requested step
//         (default 5 deg), first 0 -> 180 (right to left), then back
//         180 -> 0, combines the two passes per angle, stores the result
//         in RAM and sends it to the controller.
//         Servo trim: controller sends +-1 nudges, car replies with offset.
//         NEW: counts wheel-sensor ticks (both edges of every slot) and
//         sends tick counts + IR obstacle sensors 10 times a second.
// =====================================================================

const RADIO_GROUP = 42
const MSG_DRIVE = 1          // controller -> car : [1, x:int8, y:int8]
const MSG_SWEEP_REQUEST = 4  // controller -> car : [4, stepDeg]
const MSG_SWEEP_DATA = 5     // car -> controller : [5, sweepId, firstIndex, count, stepDeg, dist:uint16LE * count]
const MSG_SERVO_TRIM = 6     // controller -> car : [6, nudge:int8]
const MSG_TRIM_VALUE = 7     // car -> controller : [7, offset:int8]
const MSG_STATUS = 8         // car -> controller : [8, left:int32LE, right:int32LE, ir:uint8, flags:uint8, rawPins:uint8, expander:uint8]
                             //   ir bit0 = left obstacle, bit1 = right obstacle
                             //   flags bit0 = scanning, bit1 = test drive running
const MSG_RESET_ODO = 9      // controller -> car : [9, 0]  set both tick counters to 0
const MSG_TEST_DRIVE = 10    // controller -> car : [10, left:int8, right:int8, tenthsOfSecond:uint8]
                             //   drive the motors exactly like this for that long, ignoring the joystick

// Servo calibration: added to every servo command so that 90 points
// straight ahead. Measured with D/E on the controller.
const SERVO_OFFSET_DEG = 0
let servoOffset = SERVO_OFFSET_DEG

const DEFAULT_STEP_DEG = 5
const MAX_COUNT = 91         // 2 deg steps at the finest
const VALUES_PER_PACKET = 7  // 5 header bytes + 7*2 = 19 bytes (radio max)
const SERVO_SETTLE_MS = 100
const PING_GAP_MS = 60
const ECHO_TIMEOUT_US = 30000
const MAX_PINGS = 4
const AGREE_CM = 5           // two pings this close are trusted
const PASS_AGREE_CM = 10     // two passes this close are averaged
const NO_DATA = 65535        // angle the servo cannot reach with this trim

const LINK_TIMEOUT_MS = 300
const STATUS_INTERVAL_MS = 100

// Wheel speed sensors. This mainboard is older than rev 1.2, so they sit on
// the I/O expander (I2C 56): bit 0 = left, bit 1 = right. (On rev 1.3 they
// would be on P14/P15 instead.)
const EXPANDER_ADDR = 56
const TRIG = DigitalPin.P8
const ECHO = DigitalPin.P12

JoyCar.initJoyCar(RevisionMainboard.OnepOne)   // no revision printed on the board = older than 1.2
radio.setGroup(RADIO_GROUP)
radio.setTransmitPower(7)

let driveX = 0
let driveY = 0
let lastDriveMs = -10000
let lastLeft = 999
let lastRight = 999
let sweepRequested = false
let scanning = false

// Tick counters. The sensors cannot tell direction, so each tick is counted
// + or - according to the last direction that wheel was driven.
let ticksLeft = 0
let ticksRight = 0
let dirLeft = 1
let dirRight = 1

// Wheel calibration (measured 2026-10-04, see NOTES.md)
const CM_PER_TICK_LEFT = 0.480
const CM_PER_TICK_RIGHT = 0.511

// Straight-line speed matching: while left and right are commanded equal,
// compare how far each wheel has travelled since the straight stretch began
// and shift power from the wheel that is ahead to the one that is behind.
const STRAIGHT_KP = 12       // % power per cm one wheel is ahead
// Fixed starting correction from the calibration: at equal power the left
// wheel travelled ~9 % further forward and ~18 % further backward.
const LEFT_POWER_FWD = 0.92
const LEFT_POWER_BACK = 0.85
const STRAIGHT_MAX = 30      // max correction, % power
let straightDir = 0          // +1 forward, -1 backward, 0 not driving straight
let straightStartL = 0
let straightStartR = 0

// Test drive (calibration): fixed motor command until testUntilMs
let testUntilMs = 0
let testLeft = 0
let testRight = 0
let requestedStep = DEFAULT_STEP_DEG
let sweepId = 0
let sweepStep = DEFAULT_STEP_DEG
let sweepCount = 0

// The scan in RAM: two passes, then the combined result
let passOut: number[] = []
let passBack: number[] = []
let sweepDist: number[] = []
for (let i = 0; i < MAX_COUNT; i++) {
    passOut.push(0)
    passBack.push(0)
    sweepDist.push(0)
}

servoTo(90)

radio.onReceivedBuffer(function (buf: Buffer) {
    if (buf.length < 2) return
    const msgType = buf.getNumber(NumberFormat.UInt8LE, 0)
    if (msgType == MSG_DRIVE && buf.length >= 3) {
        driveX = buf.getNumber(NumberFormat.Int8LE, 1)
        driveY = buf.getNumber(NumberFormat.Int8LE, 2)
        lastDriveMs = input.runningTime()
    } else if (msgType == MSG_SWEEP_REQUEST) {
        requestedStep = buf.getNumber(NumberFormat.UInt8LE, 1)
        sweepRequested = true
    } else if (msgType == MSG_TEST_DRIVE && buf.length >= 4) {
        testLeft = buf.getNumber(NumberFormat.Int8LE, 1)
        testRight = buf.getNumber(NumberFormat.Int8LE, 2)
        testUntilMs = input.runningTime() + 100 * buf.getNumber(NumberFormat.UInt8LE, 3)
    } else if (msgType == MSG_RESET_ODO) {
        ticksLeft = 0
        ticksRight = 0
    } else if (msgType == MSG_SERVO_TRIM) {
        const nudge = buf.getNumber(NumberFormat.Int8LE, 1)
        servoOffset = Math.constrain(servoOffset + nudge, -30, 30)
        servoTo(90)
        const reply = pins.createBuffer(2)
        reply.setNumber(NumberFormat.UInt8LE, 0, MSG_TRIM_VALUE)
        reply.setNumber(NumberFormat.Int8LE, 1, servoOffset)
        radio.sendBuffer(reply)
    }
})

// With a large trim, the ends of the sweep are physically out of reach
function reachable(angle: number): boolean {
    const cmd = angle + servoOffset
    return cmd >= 0 && cmd <= 180
}

function servoTo(angle: number) {
    JoyCar.servo(1, Math.constrain(angle + servoOffset, 0, 180))
}

// Drive with straight-line correction when left == right
function drive(left: number, right: number) {
    if (left != 0 && left == right) {
        const dir = left > 0 ? 1 : -1
        if (dir != straightDir) {
            straightDir = dir
            straightStartL = ticksLeft
            straightStartR = ticksRight
        }
        // > 0 means the left wheel has travelled further
        const aheadCm = dir * ((ticksLeft - straightStartL) * CM_PER_TICK_LEFT - (ticksRight - straightStartR) * CM_PER_TICK_RIGHT)
        const corr = Math.constrain(aheadCm * STRAIGHT_KP, -STRAIGHT_MAX, STRAIGHT_MAX)
        let l = left * (dir > 0 ? LEFT_POWER_FWD : LEFT_POWER_BACK) - dir * corr
        let r = right + dir * corr
        // never let the correction reverse a wheel
        if (dir > 0) {
            l = Math.max(l, 0)
            r = Math.max(r, 0)
        } else {
            l = Math.min(l, 0)
            r = Math.min(r, 0)
        }
        setMotors(Math.round(l), Math.round(r))
    } else {
        straightDir = 0
        setMotors(left, right)
    }
}

function setMotors(left: number, right: number) {
    left = Math.constrain(left, -100, 100)
    right = Math.constrain(right, -100, 100)
    if (left > 0) dirLeft = 1
    if (left < 0) dirLeft = -1
    if (right > 0) dirRight = 1
    if (right < 0) dirRight = -1
    if (left == lastLeft && right == lastRight) return
    lastLeft = left
    lastRight = right
    JoyCar.drivePwm(
        right < 0 ? Math.idiv(-right * 255, 100) : 0,
        right > 0 ? Math.idiv(right * 255, 100) : 0,
        left < 0 ? Math.idiv(-left * 255, 100) : 0,
        left > 0 ? Math.idiv(left * 255, 100) : 0
    )
}

// One ping in cm, 0 = no echo
function sonarCm(): number {
    const waitStart = input.runningTime()
    while (pins.digitalReadPin(ECHO) == 1 && input.runningTime() - waitStart < 100) {
        basic.pause(1)
    }
    pins.digitalWritePin(TRIG, 0)
    control.waitMicros(2)
    pins.digitalWritePin(TRIG, 1)
    control.waitMicros(10)
    pins.digitalWritePin(TRIG, 0)
    const echoUs = pins.pulseIn(ECHO, PulseValue.High, ECHO_TIMEOUT_US)
    return Math.idiv(echoUs, 58)
}

// Pings until two agree; otherwise nearest valid; 0 if all missed
function sonarRobustCm(): number {
    let valid: number[] = []
    for (let attempt = 0; attempt < MAX_PINGS; attempt++) {
        if (attempt > 0) basic.pause(PING_GAP_MS)
        const cm = sonarCm()
        if (cm > 0) {
            for (let j = 0; j < valid.length; j++) {
                if (Math.abs(cm - valid[j]) <= AGREE_CM) {
                    return Math.idiv(cm + valid[j], 2)
                }
            }
            valid.push(cm)
        }
    }
    if (valid.length == 0) return 0
    let nearest = valid[0]
    for (let j = 1; j < valid.length; j++) {
        nearest = Math.min(nearest, valid[j])
    }
    return nearest
}

// Only steps that divide 180 evenly, 2..30
function cleanStep(step: number): number {
    const allowed = [2, 3, 4, 5, 6, 9, 10, 12, 15, 18, 20, 30]
    for (let k = 0; k < allowed.length; k++) {
        if (allowed[k] >= step) return allowed[k]
    }
    return DEFAULT_STEP_DEG
}

function runSweep(step: number) {
    scanning = true
    sweepStep = cleanStep(step)
    sweepCount = Math.idiv(180, sweepStep) + 1
    setMotors(0, 0)
    basic.clearScreen()

    // Pass 1: 0 -> 180
    servoTo(0)
    basic.pause(500)
    for (let i = 0; i < sweepCount; i++) {
        servoTo(i * sweepStep)
        basic.pause(SERVO_SETTLE_MS)
        passOut[i] = reachable(i * sweepStep) ? sonarRobustCm() : NO_DATA
        led.plot(Math.idiv(i * 5, sweepCount), 0)
    }
    // Pass 2: 180 -> 0
    for (let i = sweepCount - 1; i >= 0; i--) {
        servoTo(i * sweepStep)
        basic.pause(SERVO_SETTLE_MS)
        passBack[i] = reachable(i * sweepStep) ? sonarRobustCm() : NO_DATA
        led.plot(Math.idiv(i * 5, sweepCount), 1)
    }
    servoTo(90)

    // Combine: agreeing passes are averaged, a miss is ignored,
    // a disagreement keeps the nearer reading
    for (let i = 0; i < sweepCount; i++) {
        const a = passOut[i]
        const b = passBack[i]
        if (a == NO_DATA || b == NO_DATA) {
            sweepDist[i] = NO_DATA
        } else if (a > 0 && b > 0) {
            if (Math.abs(a - b) <= PASS_AGREE_CM) {
                sweepDist[i] = Math.idiv(a + b, 2)
            } else {
                sweepDist[i] = Math.min(a, b)
            }
        } else {
            sweepDist[i] = Math.max(a, b)
        }
    }

    sweepId = (sweepId + 1) % 256
    sendSweep()
    basic.clearScreen()
    scanning = false
}

function sendSweep() {
    for (let first = 0; first < sweepCount; first += VALUES_PER_PACKET) {
        const count = Math.min(VALUES_PER_PACKET, sweepCount - first)
        const pkt = pins.createBuffer(5 + 2 * count)
        pkt.setNumber(NumberFormat.UInt8LE, 0, MSG_SWEEP_DATA)
        pkt.setNumber(NumberFormat.UInt8LE, 1, sweepId)
        pkt.setNumber(NumberFormat.UInt8LE, 2, first)
        pkt.setNumber(NumberFormat.UInt8LE, 3, count)
        pkt.setNumber(NumberFormat.UInt8LE, 4, sweepStep)
        for (let k = 0; k < count; k++) {
            pkt.setNumber(NumberFormat.UInt16LE, 5 + 2 * k, sweepDist[first + k])
        }
        radio.sendBuffer(pkt)
        basic.pause(20)
    }
}

basic.forever(function () {
    if (sweepRequested) {
        sweepRequested = false
        runSweep(requestedStep)
    }
    const linkOk = input.runningTime() - lastDriveMs < LINK_TIMEOUT_MS
    if (input.runningTime() < testUntilMs) {
        drive(testLeft, testRight)
        led.plot(2, 2)
    } else if (linkOk) {
        drive(driveY + driveX, driveY - driveX)
        led.plot(2, 2)
    } else {
        drive(0, 0)
        led.unplot(2, 2)
    }
    basic.pause(20)
})

// ---- Wheel ticks: poll the I/O expander and count every change of bit 0
// (left) and bit 1 (right). pause(1) is about 6 ms on the v1, so very fast
// driving may miss a few ticks (cm/tick drops ~6 % from 50 % to 80 % power).
let expander = pins.i2cReadNumber(EXPANDER_ADDR, NumberFormat.UInt8LE, false)
let rawLeft = expander & 1
let rawRight = (expander >> 1) & 1
control.inBackground(function () {
    while (true) {
        expander = pins.i2cReadNumber(EXPANDER_ADDR, NumberFormat.UInt8LE, false)
        const l = expander & 1
        if (l != rawLeft) {
            rawLeft = l
            ticksLeft += dirLeft
        }
        const r = (expander >> 1) & 1
        if (r != rawRight) {
            rawRight = r
            ticksRight += dirRight
        }
        basic.pause(1)
    }
})

// ---- Status to the controller, 10 times a second (also while scanning) ----
basic.forever(function () {
    // Obstacle sensors from the expander byte the tick loop just read:
    // bit 5 = left, bit 6 = right, 0 = obstacle (active low).
    let irBits = 0
    if ((expander & 0x20) == 0) irBits += 1
    if ((expander & 0x40) == 0) irBits += 2
    const st = pins.createBuffer(13)
    st.setNumber(NumberFormat.UInt8LE, 0, MSG_STATUS)
    st.setNumber(NumberFormat.Int32LE, 1, ticksLeft)
    st.setNumber(NumberFormat.Int32LE, 5, ticksRight)
    st.setNumber(NumberFormat.UInt8LE, 9, irBits)
    let flags = 0
    if (scanning) flags += 1
    if (input.runningTime() < testUntilMs) flags += 2
    st.setNumber(NumberFormat.UInt8LE, 10, flags)
    // diagnostic: raw wheel-sensor bits (bit0 = left, bit1 = right) and the
    // whole I/O expander byte
    st.setNumber(NumberFormat.UInt8LE, 11, rawLeft + 2 * rawRight)
    st.setNumber(NumberFormat.UInt8LE, 12, expander)
    radio.sendBuffer(st)
    basic.pause(STATUS_INTERVAL_MS)
})
