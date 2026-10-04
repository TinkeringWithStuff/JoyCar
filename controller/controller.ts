// =====================================================================
// CONTROLLER  -  micro:bit v2 in joystick:bit + Kitronik OLED  -  Step 3b
// MakeCode JavaScript. Extensions needed: "joystickbit", "128x64Display"
//
//   C  =  scan (5 deg steps, two passes - takes ~15-20 s)
//   F  =  change scale: LOG (default) -> 50 -> 100 -> 200 -> 400 cm -> LOG
//         LOG shows 0..4 m with rings at 25 cm, 50 cm, 1 m, 2 m, 4 m.
//         In the linear scales, auto-zoom picks one after each scan.
//   D/E=  servo trim -1 / +1 deg
//   A  =  (micro:bit) cycle view: CLEAN radar -> MAP -> RAW dots -> table 1 -> table 2
//   B  =  (micro:bit) set the car's position to zero (start point) and clear the trail
//
// MAP view (step 3b): the car's position and heading, worked out from the
// wheel ticks the car reports 10 times a second, on a fixed map of the room.
// Arrow = car, dots = its trail, grid dots every 50 cm. F zooms 1/2/4/8 cm per pixel.
//   A+B=  (micro:bit) show the car's current servo trim
//
// CLEAN view: each run of near-equal readings ("region of constant depth")
// is narrowed by the beam width and drawn as a point or a short wall
// facing the car. RAW view: every reading as a dot, neighbours joined.
//
// Heartbeat LEDs: top-left = drive loop, top-right = display loop.
// Leave the stick centred while powering on.
// =====================================================================

const RADIO_GROUP = 42
const MSG_DRIVE = 1
const MSG_SWEEP_REQUEST = 4
const MSG_SWEEP_DATA = 5
const MSG_SERVO_TRIM = 6
const MSG_TRIM_VALUE = 7
const MSG_STATUS = 8          // car: ticksLeft:int32, ticksRight:int32, ir, flags, ...

const SCAN_STEP_DEG = 5
const MAX_COUNT = 91
const INCOMPLETE_TIMEOUT_MS = 800

const DEADZONE = 12
const INVERT_X = true
const INVERT_Y = false

// ---- Map drawing ----
const OLED_ADDR = 60
const ORIGIN_X = 64
const ORIGIN_Y = 62
const RADIUS_PX = 54
const RANGES_CM = [50, 100, 200, 400]
const LOG_INDEX = 4          // rangeIndex 4 = logarithmic scale
const LOG_K_CM = 10          // roughly linear below this, logarithmic above
const LOG_MAX_CM = 400       // edge of the log map
const LOG_RINGS_CM = [25, 50, 100, 200, 400]
const BEAM_DEG = 20          // effective sonar beam width
const RCD_TOL_CM = 4         // readings this close belong to the same run
const JOIN_CM = 25           // RAW view: join neighbours closer than this
const AUTO_ZOOM_SHARE = 80   // auto-zoom: show at least this % of echoes

const VIEW_CLEAN = 0
const VIEW_RAW = 1
const VIEW_TABLE1 = 2
const VIEW_TABLE2 = 3
const VIEW_WORLD = 4
// Order of views when pressing A
const VIEW_ORDER = [VIEW_CLEAN, VIEW_WORLD, VIEW_RAW, VIEW_TABLE1, VIEW_TABLE2]

// ---- Odometry (wheel calibration 2026-10-04, see NOTES.md) ----
const CM_PER_TICK = 0.51
const WHEEL_BASE_CM = 15.2
const MAX_TICKS_PER_STATUS = 50   // bigger jumps = car restarted, ignored
const TRAIL_STEP_CM = 3           // trail point every 3 cm
const TRAIL_MAX = 400
const WORLD_CM_PER_PX = [1, 2, 4, 8]
const WORLD_SCREEN_CX = 64        // screen pixel of the view centre
const WORLD_SCREEN_CY = 36

radio.setGroup(RADIO_GROUP)
radio.setTransmitPower(7)
joystickbit.initJoystickBit()

const centreX = joystickbit.getRockerValue(joystickbit.rockerType.X)
const centreY = joystickbit.getRockerValue(joystickbit.rockerType.Y)

// ---- Scan data in RAM. -1 = not received, -2 = angle out of servo reach,
// 0 = no echo ----
const NO_DATA = 65535
let sweepDist: number[] = []
for (let i = 0; i < MAX_COUNT; i++) {
    sweepDist.push(-1)
}
let sweepCount = 0
let sweepStep = SCAN_STEP_DEG
let currentSweepId = -1
let receivedCount = 0
let lastSweepPacketMs = 0
let sweepShown = true
let sweepsReceived = 0

// ---- Display state (only the display loop touches the OLED) ----
let rangeIndex = LOG_INDEX
let viewMode = VIEW_CLEAN
let viewIndex = 0                 // position in VIEW_ORDER

// Car pose in the room: cm from the start point, heading in radians.
// x = right, y = ahead (as seen at the start); heading PI/2 = start direction.
let poseX = 0
let poseY = 0
let poseTh = Math.PI / 2
let haveTicks = false
let prevTicksL = 0
let prevTicksR = 0
let carIr = 0
let trailX: number[] = []
let trailY: number[] = []
let worldZoom = 1                 // index into WORLD_CM_PER_PX
let viewCx = 0                    // world cm shown at the view centre
let viewCy = 0
let testPattern = false
let needRedraw = true
let pendingStatus = "C scan A view B zero pos"

// ---- Frame buffer ----
const frame = pins.createBuffer(1025)
const windowCmd = pins.createBuffer(7)
windowCmd.setNumber(NumberFormat.UInt8LE, 0, 0x00)
windowCmd.setNumber(NumberFormat.UInt8LE, 1, 0x21)
windowCmd.setNumber(NumberFormat.UInt8LE, 2, 0)
windowCmd.setNumber(NumberFormat.UInt8LE, 3, 127)
windowCmd.setNumber(NumberFormat.UInt8LE, 4, 0x22)
windowCmd.setNumber(NumberFormat.UInt8LE, 5, 0)
windowCmd.setNumber(NumberFormat.UInt8LE, 6, 7)

function frameClear() {
    frame.fill(0)
    frame.setNumber(NumberFormat.UInt8LE, 0, 0x40)
}

function setPx(x: number, y: number) {
    if (x < 0 || x > 127 || y < 8 || y > 63) return
    const idx = 1 + x + (y >> 3) * 128
    frame.setNumber(NumberFormat.UInt8LE, idx, frame.getNumber(NumberFormat.UInt8LE, idx) | (1 << (y & 7)))
}

function dot(x: number, y: number) {
    setPx(x, y)
    setPx(x + 1, y)
    setPx(x, y + 1)
    setPx(x + 1, y + 1)
}

function framePush() {
    pins.i2cWriteBuffer(OLED_ADDR, windowCmd)
    pins.i2cWriteBuffer(OLED_ADDR, frame)
}

function drawLineXY(x0: number, y0: number, x1: number, y1: number) {
    x0 = Math.round(x0)
    y0 = Math.round(y0)
    x1 = Math.round(x1)
    y1 = Math.round(y1)
    const dx = Math.abs(x1 - x0)
    const sx = x0 < x1 ? 1 : -1
    const dy = -Math.abs(y1 - y0)
    const sy = y0 < y1 ? 1 : -1
    let err = dx + dy
    for (let guard = 0; guard < 300; guard++) {
        setPx(x0, y0)
        if (x0 == x1 && y0 == y1) break
        const e2 = 2 * err
        if (e2 >= dy) {
            err += dy
            x0 += sx
        }
        if (e2 <= dx) {
            err += dx
            y0 += sy
        }
    }
}

// ---- Scale: linear (cm * pxPerCm) or logarithmic ----
let scaleLog = true
let scaleMaxCm = LOG_MAX_CM
let pxPerCm = RADIUS_PX / LOG_MAX_CM
const logDenominator = Math.log(1 + LOG_MAX_CM / LOG_K_CM)

function setScale() {
    scaleLog = rangeIndex == LOG_INDEX
    scaleMaxCm = scaleLog ? LOG_MAX_CM : RANGES_CM[rangeIndex]
    pxPerCm = RADIUS_PX / scaleMaxCm
}

function radiusPx(cm: number): number {
    if (scaleLog) return RADIUS_PX * Math.log(1 + cm / LOG_K_CM) / logDenominator
    return cm * pxPerCm
}

// Polar (cm, degrees; 0 = right, 90 = ahead, 180 = left) -> screen
function screenX(cm: number, deg: number): number {
    return Math.round(ORIGIN_X + radiusPx(cm) * Math.cos(deg * Math.PI / 180))
}
function screenY(cm: number, deg: number): number {
    return Math.round(ORIGIN_Y - radiusPx(cm) * Math.sin(deg * Math.PI / 180))
}

// Straight line in the real world (cm, x right, y ahead), drawn in pieces
// so it bends correctly on the log scale
function drawWorldLine(x0: number, y0: number, x1: number, y1: number) {
    const pieces = 8
    let lastX = 0
    let lastY = 0
    for (let p = 0; p <= pieces; p++) {
        const wx = x0 + (x1 - x0) * p / pieces
        const wy = y0 + (y1 - y0) * p / pieces
        const cm = Math.sqrt(wx * wx + wy * wy)
        const deg = Math.atan2(wy, wx) * 180 / Math.PI
        const sxp = screenX(cm, deg)
        const syp = screenY(cm, deg)
        if (p > 0) drawLineXY(lastX, lastY, sxp, syp)
        lastX = sxp
        lastY = syp
    }
}

function status(text: string) {
    while (text.length < 25) {
        text = text + " "
    }
    kitronik_VIEW128x64.show(text.substr(0, 25), 1)
}

function scanLabel(): string {
    return testPattern ? "TEST" : "Scan " + sweepsReceived
}

function drawBackground() {
    if (scaleLog) {
        for (let r = 0; r < LOG_RINGS_CM.length; r++) {
            const spacing = r == LOG_RINGS_CM.length - 1 ? 6 : 18
            for (let a = 0; a <= 180; a += spacing) {
                setPx(screenX(LOG_RINGS_CM[r], a), screenY(LOG_RINGS_CM[r], a))
            }
        }
    } else {
        for (let a = 0; a <= 180; a += 6) {
            setPx(screenX(scaleMaxCm, a), screenY(scaleMaxCm, a))
            if (a % 12 == 0) setPx(screenX(scaleMaxCm / 2, a), screenY(scaleMaxCm / 2, a))
        }
    }
    for (let dx = -2; dx <= 2; dx++) setPx(ORIGIN_X + dx, 63)
    for (let dx = -1; dx <= 1; dx++) setPx(ORIGIN_X + dx, 62)
    setPx(ORIGIN_X, 61)
}

// RAW: every echo as a dot, neighbours that are close in the world joined
function drawRaw() {
    const stepRad = sweepStep * Math.PI / 180
    let prevCm = -1
    let prevX = 0
    let prevY = 0
    for (let i = 0; i < sweepCount; i++) {
        const cm = sweepDist[i]
        if (cm > 0 && cm <= scaleMaxCm) {
            const hx = screenX(cm, i * sweepStep)
            const hy = screenY(cm, i * sweepStep)
            dot(hx, hy)
            if (prevCm > 0) {
                const gapCm = Math.sqrt(cm * cm + prevCm * prevCm - 2 * cm * prevCm * Math.cos(stepRad))
                if (gapCm <= JOIN_CM) drawLineXY(prevX, prevY, hx, hy)
            }
            prevCm = cm
            prevX = hx
            prevY = hy
        } else {
            prevCm = -1
        }
    }
}

// CLEAN: regions of constant depth, narrowed by the beam width
function drawClean() {
    let i = 0
    while (i < sweepCount) {
        const startCm = sweepDist[i]
        if (startCm <= 0) {
            i++
            continue
        }
        const tol = Math.max(RCD_TOL_CM, Math.idiv(startCm * 4, 100))
        let j = i
        let sum = startCm
        while (j + 1 < sweepCount && sweepDist[j + 1] > 0 && Math.abs(sweepDist[j + 1] - startCm) <= tol) {
            j++
            sum += sweepDist[j]
        }
        const n = j - i + 1
        const cm = sum / n
        if (cm <= scaleMaxCm) {
            const centreDeg = (i + j) / 2 * sweepStep
            const widthDeg = n * sweepStep - BEAM_DEG
            if (widthDeg <= 0) {
                dot(screenX(cm, centreDeg), screenY(cm, centreDeg))
            } else {
                // short straight wall perpendicular to the centre bearing (world cm)
                const t = centreDeg * Math.PI / 180
                const px0 = cm * Math.cos(t)
                const py0 = cm * Math.sin(t)
                const half = cm * Math.tan(widthDeg / 2 * Math.PI / 180)
                const tx = -Math.sin(t) * half
                const ty = Math.cos(t) * half
                drawWorldLine(px0 - tx, py0 - ty, px0 + tx, py0 + ty)
            }
        }
        i = j + 1
    }
}

// ---- MAP view ----
function worldX(x: number): number {
    return Math.round(WORLD_SCREEN_CX + (x - viewCx) / WORLD_CM_PER_PX[worldZoom])
}
function worldY(y: number): number {
    return Math.round(WORLD_SCREEN_CY - (y - viewCy) / WORLD_CM_PER_PX[worldZoom])
}

function headingDeg(): number {
    // 0 = the direction the car faced at the start, + = turned left
    let d = Math.round((poseTh - Math.PI / 2) * 180 / Math.PI) % 360
    if (d > 180) d -= 360
    if (d < -180) d += 360
    return d
}

function drawWorld() {
    const cpp = WORLD_CM_PER_PX[worldZoom]
    // keep the car on screen: jump the view when it nears an edge
    let cx = worldX(poseX)
    let cy = worldY(poseY)
    if (cx < 8 || cx > 119 || cy < 16 || cy > 56) {
        viewCx = poseX
        viewCy = poseY
        cx = worldX(poseX)
        cy = worldY(poseY)
    }
    frameClear()

    // grid dots, fixed to the room
    const grid = cpp >= 4 ? 100 : 50
    const halfW = 64 * cpp
    const halfH = 28 * cpp
    let gx = Math.floor((viewCx - halfW) / grid) * grid
    while (gx <= viewCx + halfW) {
        let gy = Math.floor((viewCy - halfH) / grid) * grid
        while (gy <= viewCy + halfH) {
            setPx(worldX(gx), worldY(gy))
            gy += grid
        }
        gx += grid
    }

    // start point: a small cross
    const ox = worldX(0)
    const oy = worldY(0)
    setPx(ox - 1, oy)
    setPx(ox + 1, oy)
    setPx(ox, oy - 1)
    setPx(ox, oy + 1)

    // trail
    for (let i = 0; i < trailX.length; i++) {
        setPx(worldX(trailX[i]), worldY(trailY[i]))
    }

    // the car: an arrow pointing along its heading
    const noseX = cx + 5 * Math.cos(poseTh)
    const noseY = cy - 5 * Math.sin(poseTh)
    const leftX = cx + 4 * Math.cos(poseTh + 2.5)
    const leftY = cy - 4 * Math.sin(poseTh + 2.5)
    const rightX = cx + 4 * Math.cos(poseTh - 2.5)
    const rightY = cy - 4 * Math.sin(poseTh - 2.5)
    drawLineXY(noseX, noseY, leftX, leftY)
    drawLineXY(noseX, noseY, rightX, rightY)
    drawLineXY(leftX, leftY, cx, cy)
    drawLineXY(rightX, rightY, cx, cy)

    framePush()
    status("x" + Math.round(poseX) + " y" + Math.round(poseY) + " h" + headingDeg() + " " + cpp + "cm/px")
}

function resetPose() {
    poseX = 0
    poseY = 0
    poseTh = Math.PI / 2
    trailX = []
    trailY = []
    viewCx = 0
    viewCy = 0
}

// Dead reckoning from the change in each wheel's tick count
function updatePose(ticksL: number, ticksR: number) {
    if (!haveTicks) {
        prevTicksL = ticksL
        prevTicksR = ticksR
        haveTicks = true
        return
    }
    const dl = ticksL - prevTicksL
    const dr = ticksR - prevTicksR
    prevTicksL = ticksL
    prevTicksR = ticksR
    if (dl == 0 && dr == 0) return
    if (Math.abs(dl) > MAX_TICKS_PER_STATUS || Math.abs(dr) > MAX_TICKS_PER_STATUS) return
    const sL = dl * CM_PER_TICK
    const sR = dr * CM_PER_TICK
    const ds = (sL + sR) / 2
    const dth = (sR - sL) / WHEEL_BASE_CM
    const mid = poseTh + dth / 2
    poseX += ds * Math.cos(mid)
    poseY += ds * Math.sin(mid)
    poseTh += dth
    // trail point every TRAIL_STEP_CM
    const n = trailX.length
    if (n == 0 || Math.abs(poseX - trailX[n - 1]) + Math.abs(poseY - trailY[n - 1]) >= TRAIL_STEP_CM) {
        trailX.push(Math.round(poseX))
        trailY.push(Math.round(poseY))
        if (trailX.length > TRAIL_MAX) {
            trailX.shift()
            trailY.shift()
        }
    }
    if (viewMode == VIEW_WORLD) needRedraw = true
}

function drawMap() {
    setScale()
    frameClear()
    drawBackground()
    if (viewMode == VIEW_RAW) {
        drawRaw()
    } else {
        drawClean()
    }
    framePush()
    const scaleText = scaleLog ? "LOG" : scaleMaxCm + "cm"
    let header = scanLabel() + " " + scaleText + " " + (viewMode == VIEW_RAW ? "RAW" : "CLEAN")
    if (!testPattern && sweepsReceived > 0 && receivedCount < sweepCount) {
        header = header + " -" + (sweepCount - receivedCount)
    }
    status(header)
}

// Smallest range that still shows AUTO_ZOOM_SHARE % of the echoes
function autoZoom() {
    if (rangeIndex == LOG_INDEX) return      // log scale shows everything
    let validCount = 0
    for (let i = 0; i < sweepCount; i++) {
        if (sweepDist[i] > 0) validCount++
    }
    if (validCount == 0) return
    for (let r = 0; r < RANGES_CM.length; r++) {
        let inside = 0
        for (let i = 0; i < sweepCount; i++) {
            if (sweepDist[i] > 0 && sweepDist[i] <= RANGES_CM[r]) inside++
        }
        if (inside * 100 >= validCount * AUTO_ZOOM_SHARE) {
            rangeIndex = r
            return
        }
    }
    rangeIndex = RANGES_CM.length - 1
}

function padLeft(text: string, width: number): string {
    while (text.length < width) {
        text = " " + text
    }
    return text
}

function tableCell(index: number): string {
    const cm = sweepDist[index]
    let distText = ""
    if (cm == -2) {
        distText = "n/a"
    } else if (cm < 0) {
        distText = "--"
    } else if (cm == 0) {
        distText = "far"
    } else {
        distText = "" + Math.min(cm, 999)
    }
    return padLeft("" + index * sweepStep, 3) + ":" + padLeft(distText, 3) + " "
}

// 21 values per page (7 lines x 3)
function drawTable(page: number) {
    kitronik_VIEW128x64.clear()
    status(scanLabel() + " table " + (page + 1))
    const first = page * 21
    const last = Math.min(first + 21, sweepCount) - 1
    let lineText = ""
    let lineNo = 2
    for (let i = first; i <= last; i++) {
        lineText = lineText + tableCell(i)
        if ((i - first) % 3 == 2 || i == last) {
            kitronik_VIEW128x64.show(lineText, lineNo)
            lineNo++
            lineText = ""
        }
    }
    if (last < first) status(scanLabel() + " table " + (page + 1) + " empty")
}

function axis(raw: number, centre: number, invert: boolean): number {
    let v = raw - centre
    let span = v >= 0 ? 1023 - centre : centre
    if (span < 1) span = 1
    v = Math.idiv(v * 100, span)
    if (Math.abs(v) < DEADZONE) v = 0
    if (invert) v = -v
    return Math.constrain(v, -100, 100)
}

function sendTrimNudge(nudge: number) {
    const trimBuf = pins.createBuffer(2)
    trimBuf.setNumber(NumberFormat.UInt8LE, 0, MSG_SERVO_TRIM)
    trimBuf.setNumber(NumberFormat.Int8LE, 1, nudge)
    radio.sendBuffer(trimBuf)
}

// ---- Buttons (flags only, no OLED access) ----
joystickbit.onButtonEvent(joystickbit.JoystickBitPin.P12, joystickbit.ButtonType.down, function () {
    const req = pins.createBuffer(2)
    req.setNumber(NumberFormat.UInt8LE, 0, MSG_SWEEP_REQUEST)
    req.setNumber(NumberFormat.UInt8LE, 1, SCAN_STEP_DEG)
    radio.sendBuffer(req)
    pendingStatus = "Scanning... (~20 s)"
})

joystickbit.onButtonEvent(joystickbit.JoystickBitPin.P13, joystickbit.ButtonType.down, function () {
    sendTrimNudge(-1)
})

joystickbit.onButtonEvent(joystickbit.JoystickBitPin.P14, joystickbit.ButtonType.down, function () {
    sendTrimNudge(1)
})

joystickbit.onButtonEvent(joystickbit.JoystickBitPin.P15, joystickbit.ButtonType.down, function () {
    if (viewMode == VIEW_WORLD) {
        worldZoom = (worldZoom + 1) % WORLD_CM_PER_PX.length
        viewCx = poseX
        viewCy = poseY
    } else {
        rangeIndex = (rangeIndex + 1) % (LOG_INDEX + 1)
        if (viewMode == VIEW_TABLE1 || viewMode == VIEW_TABLE2) {
            viewMode = VIEW_CLEAN
            viewIndex = 0
        }
    }
    needRedraw = true
})

// A nudge of 0 just makes the car report its trim
input.onButtonPressed(Button.AB, function () {
    sendTrimNudge(0)
})

input.onButtonPressed(Button.A, function () {
    viewIndex = (viewIndex + 1) % VIEW_ORDER.length
    viewMode = VIEW_ORDER[viewIndex]
    needRedraw = true
})

// Set the car's position to zero and clear the trail
input.onButtonPressed(Button.B, function () {
    resetPose()
    pendingStatus = "Position set to zero"
    needRedraw = true
})

// ---- Radio (no OLED access) ----
radio.onReceivedBuffer(function (buf: Buffer) {
    if (buf.length < 2) return
    const msgType = buf.getNumber(NumberFormat.UInt8LE, 0)

    if (msgType == MSG_STATUS && buf.length >= 11) {
        carIr = buf.getNumber(NumberFormat.UInt8LE, 9)
        updatePose(buf.getNumber(NumberFormat.Int32LE, 1), buf.getNumber(NumberFormat.Int32LE, 5))
        return
    }
    if (msgType == MSG_TRIM_VALUE) {
        pendingStatus = "Servo trim " + buf.getNumber(NumberFormat.Int8LE, 1) + " deg"
        return
    }
    if (msgType != MSG_SWEEP_DATA || buf.length < 5) return

    const packetSweepId = buf.getNumber(NumberFormat.UInt8LE, 1)
    const firstIndex = buf.getNumber(NumberFormat.UInt8LE, 2)
    const valueCount = buf.getNumber(NumberFormat.UInt8LE, 3)
    const packetStep = buf.getNumber(NumberFormat.UInt8LE, 4)
    if (packetStep < 1) return

    if (packetSweepId != currentSweepId) {
        currentSweepId = packetSweepId
        sweepStep = packetStep
        sweepCount = Math.min(Math.idiv(180, sweepStep) + 1, MAX_COUNT)
        for (let i = 0; i < MAX_COUNT; i++) {
            sweepDist[i] = -1
        }
        receivedCount = 0
        sweepShown = false
        testPattern = false
        sweepsReceived++
    }
    for (let k = 0; k < valueCount; k++) {
        const idx = firstIndex + k
        if (idx < sweepCount && sweepDist[idx] < 0 && buf.length >= 7 + 2 * k) {
            const value = buf.getNumber(NumberFormat.UInt16LE, 5 + 2 * k)
            sweepDist[idx] = value == NO_DATA ? -2 : value
            receivedCount++
        }
    }
    lastSweepPacketMs = input.runningTime()
})

kitronik_VIEW128x64.clear()

// Drive packets, 20 per second
basic.forever(function () {
    const sx = axis(joystickbit.getRockerValue(joystickbit.rockerType.X), centreX, INVERT_X)
    const sy = axis(joystickbit.getRockerValue(joystickbit.rockerType.Y), centreY, INVERT_Y)
    const drv = pins.createBuffer(3)
    drv.setNumber(NumberFormat.UInt8LE, 0, MSG_DRIVE)
    drv.setNumber(NumberFormat.Int8LE, 1, sx)
    drv.setNumber(NumberFormat.Int8LE, 2, sy)
    radio.sendBuffer(drv)
    led.toggle(0, 0)
    basic.pause(50)
})

// The ONLY place that writes to the OLED
let lastBeatMs = 0
basic.forever(function () {
    if (!sweepShown) {
        const complete = receivedCount >= sweepCount
        const timedOut = input.runningTime() - lastSweepPacketMs > INCOMPLETE_TIMEOUT_MS
        if (complete || timedOut) {
            sweepShown = true
            autoZoom()
            needRedraw = true
            serial.writeLine("scan " + sweepsReceived + " step " + sweepStep)
            for (let i = 0; i < sweepCount; i++) {
                serial.writeLine("" + i * sweepStep + "," + sweepDist[i])
            }
        }
    }
    if (needRedraw) {
        needRedraw = false
        if (viewMode == VIEW_TABLE1) {
            drawTable(0)
        } else if (viewMode == VIEW_TABLE2) {
            drawTable(1)
        } else if (viewMode == VIEW_WORLD) {
            drawWorld()
        } else {
            drawMap()
        }
    }
    if (pendingStatus.length > 0) {
        status(pendingStatus)
        pendingStatus = ""
    }
    if (input.runningTime() - lastBeatMs > 500) {
        lastBeatMs = input.runningTime()
        led.toggle(4, 0)
    }
    basic.pause(50)
})
