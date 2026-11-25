// =====================================================================
//  DeliveryRobot - AIROST Project 1, basic task (Figure 1 map)
//
//  Run:
//   1. Start on the line leaving the START box, press button 2 (AUTO).
//   2. PID line following along the top line (dashed section included).
//   3. TCS34725 reads the colour card beside the line:
//        RED -> ball 1   BLUE -> ball 2   GREEN -> ball 3
//   4. After the two right-hand corners the robot drives along the bottom
//      line and counts the ball branches on its LEFT side
//      (ball 1 is reached first, then ball 2, then ball 3).
//   5. At the chosen branch it turns left, follows the stub, and stops on
//      the end bar (all sensors black).
//   6. On arrival it switches itself to MANUAL so you can grip the ball
//      (button 3) and drive it back to "unload ball".
//
//  Phone (Adafruit Bluefruit Connect -> Controller -> Control Pad):
//   Arrows : drive (manual mode, hold)
//   1      : MANUAL (also aborts an auto run)
//   2      : start AUTO run
//   3      : gripper open / close
//   4      : target override: AUTO(colour) -> B1 -> B2 -> B3 -> AUTO
//            (lets you test branch counting without the colour sensor)
//  Status messages are also sent to the app's UART screen.
//
//  Everything marked "TUNE" depends on your chassis and will need
//  adjusting on the real track.
// =====================================================================
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "ColorSensor.h"

// ===================== Pins =====================
// OLED on I2C bus Wire (GPIO 11/12). TCS34725 is on its own bus Wire1
// (GPIO 1/2) - pins are set in include/ColorSensor.h
#define OLED_SDA 11
#define OLED_SCL 12
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// L298N
const int ENA = 4, IN1 = 5, IN2 = 6;     // left motor
const int IN3 = 7, IN4 = 15, ENB = 16;   // right motor

// 5-channel IR array, left -> right (1 = black)
const int IR1 = 17, IR2 = 18, IR3 = 8, IR4 = 9, IR5 = 10;

// Gripper servo
const int SERVO_PIN = 13;

// ===================== PWM (LEDC) =====================
// Channels 0/1 share LEDC timer 0 (motors, 20 kHz).
// Channel 4 uses LEDC timer 2 (servo, 50 Hz) - separate timer, no clash.
const int MOTOR_PWM_FREQ = 20000;
const int MOTOR_PWM_RES = 8;
const int PWM_CH_LEFT = 0;
const int PWM_CH_RIGHT = 1;

const int SERVO_PWM_CH = 4;
const int SERVO_PWM_FREQ = 50;
const int SERVO_PWM_RES = 14;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;
const int GRIPPER_OPEN_ANGLE = 0;        // TUNE
const int GRIPPER_CLOSED_ANGLE = 90;     // TUNE

// ===================== Tuning =====================
// These are variables (not const) so they can be changed live over BLE:
//   "set kp 45"   "get"   - see the PARAMS table below for names.
// Values set live are lost on reset: copy good ones back here.
int MANUAL_SPEED = 200;

// PID line following (same values as LineFollowing / DualMode)
int   AUTO_BASE_SPEED = 155;             // TUNE
int   CORNER_BRAKE = 35;                 // speed drop when line is on outer sensors
int   LOST_LINE_SPIN = 125;              // pivot speed when the line is lost on a corner
float Kp = 40.0, Ki = 0.0, Kd = 40.0;    // TUNE

// Start: ignore the START box outline for this long after pressing AUTO
uint32_t START_GRACE_MS = 700;           // TUNE

// Junction (ball branch) detection on the left side
int      JUNCTION_CONFIRM_READS = 2;     // consecutive loops the pattern must be seen
uint32_t JUNCTION_MIN_GAP_MS = 350;      // TUNE: min time between two branches

// Turning into the branch
int      TURN_FORWARD_SPEED = 130;
uint32_t TURN_FORWARD_MS = 180;          // TUNE: drive on so the wheels sit over the junction
int      TURN_SPIN_SPEED = 130;          // TUNE
uint32_t TURN_MIN_SPIN_MS = 150;         // ignore sensors for the first part of the spin
uint32_t TURN_TIMEOUT_MS = 2500;         // give up and go manual if the branch isn't found

// Colour card
int COLOR_CONFIRM_READS = 3;             // same colour this many reads in a row = locked

// Live-tunable parameter table (name used in "set <name> <value>")
struct Param { const char* name; char type; void* ptr; };   // type: f=float i=int u=uint32
const Param PARAMS[] = {
  {"speed",   'i', &AUTO_BASE_SPEED},
  {"kp",      'f', &Kp},
  {"ki",      'f', &Ki},
  {"kd",      'f', &Kd},
  {"brake",   'i', &CORNER_BRAKE},
  {"lostspin",'i', &LOST_LINE_SPIN},
  {"grace",   'u', &START_GRACE_MS},
  {"jreads",  'i', &JUNCTION_CONFIRM_READS},
  {"jgap",    'u', &JUNCTION_MIN_GAP_MS},
  {"fwdspd",  'i', &TURN_FORWARD_SPEED},
  {"fwdms",   'u', &TURN_FORWARD_MS},
  {"turnspd", 'i', &TURN_SPIN_SPEED},
  {"turnmin", 'u', &TURN_MIN_SPIN_MS},
  {"turnto",  'u', &TURN_TIMEOUT_MS},
  {"creads",  'i', &COLOR_CONFIRM_READS},
  {"manual",  'i', &MANUAL_SPEED},
};
const int NUM_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

// Telemetry (off by default; "tel on", "tel 20" = 20 Hz, "tel off")
bool     telemetryOn = false;
uint32_t telemetryPeriodMs = 50;

// Behaviour switches
const bool AUTO_CLOSE_GRIPPER_ON_ARRIVAL = false;
const bool ABORT_AUTO_ON_DISCONNECT = true;

// ===================== State =====================
enum RobotMode { MODE_MANUAL, MODE_AUTO };
enum AutoPhase { PH_IDLE, PH_TO_BRANCH, PH_TURN_FORWARD, PH_TURN_SPIN, PH_TO_BALL };

RobotMode mode = MODE_MANUAL;
AutoPhase phase = PH_IDLE;

// Written by BLE callbacks (core 0), consumed by loop() (core 1)
volatile int  manualLeft = 0, manualRight = 0;
volatile bool reqManual = false, reqAuto = false;
volatile bool reqGripperToggle = false, reqCycleTarget = false;
volatile bool deviceConnected = false;    // true while at least one device is connected
volatile int  connectedCount = 0;         // phone + PC logger can both be connected
volatile bool reqDisconnectStop = false;

// Text command from BLE (e.g. "set kp 45"), handed to loop() to execute
char          cmdBuf[64];
volatile bool cmdPending = false;

// Last values, for telemetry
int   lastLeftCmd = 0, lastRightCmd = 0;
float lastError = 0, lastPid = 0;
int   lastIrBits = 0;
uint16_t  lastColorC = 0;
CardColor lastColorClass = COLOR_NONE;
uint32_t  lastTelemetryMs = 0;

// Auto-run state (loop() only)
float previousError = 0, integral = 0;
uint32_t autoStartMs = 0;
uint32_t phaseStartMs = 0;
bool spinLeftMainLine = false;

int  junctionsPassed = 0;
int  junctionSeenReads = 0;
bool junctionLatched = false;
uint32_t lastJunctionMs = 0;

bool      colorSensorOk = false;
CardColor detectedColor = COLOR_NONE;
CardColor colorCandidate = COLOR_NONE;
int       colorCandidateReads = 0;

// Live readout in manual mode (for checking the mounted sensor)
CardColor liveColor = COLOR_NONE;
CardColor liveCandidate = COLOR_NONE;
int       liveCandidateReads = 0;

int  targetOverride = 0;                 // 0 = use colour, 1..3 = forced ball
bool gripperClosed = false;

char statusMsg[24] = "Ready";
bool displayDirty = true;
bool lastShownConnected = false;

// BLE
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;

void sendParams();   // defined in the "Text commands & telemetry" section

// ===================== Helpers =====================
int ballForColor(CardColor c) {
  switch (c) {
    case COLOR_RED:   return 1;
    case COLOR_BLUE:  return 2;
    case COLOR_GREEN: return 3;
    default:          return 0;
  }
}

int currentTarget() {
  return targetOverride > 0 ? targetOverride : ballForColor(detectedColor);
}

// Send one line to every connected BLE device (and USB serial).
// Keep lines short: phones with the default MTU only get the first 20 bytes.
void bleSend(const char* line) {
  Serial.println(line);
  if (deviceConnected && pTxCharacteristic) {
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "%s\n", line);
    pTxCharacteristic->setValue((uint8_t*)buf, min(n, (int)sizeof(buf) - 1));
    pTxCharacteristic->notify();
  }
}

// Event line: shown on the OLED and sent as "E,<msg>"
void setStatus(const char* msg) {
  strncpy(statusMsg, msg, sizeof(statusMsg) - 1);
  statusMsg[sizeof(statusMsg) - 1] = '\0';
  displayDirty = true;
  char line[48];
  snprintf(line, sizeof(line), "E,%lu,%s", (unsigned long)millis(), msg);
  bleSend(line);
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  lastLeftCmd = leftSpeed;
  lastRightCmd = rightSpeed;

  if (leftSpeed >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); leftSpeed = -leftSpeed; }

  if (rightSpeed >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); rightSpeed = -rightSpeed; }

  ledcWrite(PWM_CH_LEFT, leftSpeed);
  ledcWrite(PWM_CH_RIGHT, rightSpeed);
}

void servoWriteAngle(int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulseUs = SERVO_MIN_US + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
  uint32_t duty = pulseUs * ((1UL << SERVO_PWM_RES) - 1) / (1000000UL / SERVO_PWM_FREQ);
  ledcWrite(SERVO_PWM_CH, duty);
}

void applyGripper() {
  servoWriteAngle(gripperClosed ? GRIPPER_CLOSED_ANGLE : GRIPPER_OPEN_ANGLE);
  displayDirty = true;
}

// ===================== OLED =====================
void updateOLED() {
  char buf[24];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(deviceConnected ? "BLE: CONNECTED" : "BLE: WAITING...");
  if (!colorSensorOk) { display.setCursor(110, 0); display.print("!C"); }
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 13);
  display.print(mode == MODE_AUTO ? "AUTO" : "MANUAL");

  display.setTextSize(1);
  display.setCursor(0, 32);
  int tgt = currentTarget();
  if (targetOverride > 0) snprintf(buf, sizeof(buf), "Target: B%d (forced)", tgt);
  else if (mode == MODE_MANUAL && colorSensorOk)
                          snprintf(buf, sizeof(buf), "Sees: %s", liveColor == COLOR_NONE ? "--" : colorName(liveColor));
  else if (tgt > 0)       snprintf(buf, sizeof(buf), "Colour %s -> B%d", colorName(detectedColor), tgt);
  else                    snprintf(buf, sizeof(buf), "Colour: --");
  display.print(buf);

  display.setCursor(0, 42);
  if (mode == MODE_AUTO) snprintf(buf, sizeof(buf), "Branch %d/%d", junctionsPassed, tgt);
  else                   snprintf(buf, sizeof(buf), "%s", statusMsg);
  display.print(buf);

  display.setCursor(0, 54);
  display.print(gripperClosed ? "Gripper: CLOSED" : "Gripper: OPEN");

  display.display();
}

// ===================== Sensors =====================
struct IRState { int s1, s2, s3, s4, s5, active; };

IRState readIR() {
  IRState ir;
  ir.s1 = !digitalRead(IR1);
  ir.s2 = !digitalRead(IR2);
  ir.s3 = !digitalRead(IR3);
  ir.s4 = !digitalRead(IR4);
  ir.s5 = !digitalRead(IR5);
  ir.active = ir.s1 + ir.s2 + ir.s3 + ir.s4 + ir.s5;
  lastIrBits = (ir.s1 << 4) | (ir.s2 << 3) | (ir.s3 << 2) | (ir.s4 << 1) | ir.s5;
  return ir;
}

void pollColor() {
  if (!colorSensorOk || detectedColor != COLOR_NONE) return;
  ColorReading rd;
  if (!colorSensorRead(rd)) return;
  lastColorC = rd.c;
  lastColorClass = rd.color;

  if (rd.color != COLOR_NONE && rd.color == colorCandidate) {
    colorCandidateReads++;
  } else {
    colorCandidate = rd.color;
    colorCandidateReads = (rd.color != COLOR_NONE) ? 1 : 0;
  }

  if (colorCandidateReads >= COLOR_CONFIRM_READS) {
    detectedColor = colorCandidate;
    char msg[24];
    snprintf(msg, sizeof(msg), "Colour %s -> B%d", colorName(detectedColor), ballForColor(detectedColor));
    setStatus(msg);
  }
}

// Manual mode: show what the sensor sees right now (2 matching reads to change)
void pollLiveColor() {
  if (!colorSensorOk) return;
  ColorReading rd;
  if (!colorSensorRead(rd)) return;
  lastColorC = rd.c;
  lastColorClass = rd.color;

  if (rd.color == liveCandidate) liveCandidateReads++;
  else { liveCandidate = rd.color; liveCandidateReads = 1; }

  if (liveCandidateReads >= 2 && liveCandidate != liveColor) {
    liveColor = liveCandidate;
    displayDirty = true;
  }
}

// ===================== Line following =====================
// straightThrough = true while crossing a branch we don't want: ignore the
// branch so PID doesn't pull the robot into it.
void followLine(const IRState& ir, bool straightThrough) {
  float error;

  if (straightThrough) {
    error = 0;
  } else if (ir.active == 0) {
    // Lost the line: pivot toward where it was, or coast through dashed gaps
    if (previousError <= -1.0) { setMotors(-LOST_LINE_SPIN, LOST_LINE_SPIN); return; }
    if (previousError >= 1.0)  { setMotors(LOST_LINE_SPIN, -LOST_LINE_SPIN); return; }
    error = previousError;
  } else {
    float sum = ir.s1 * -2.0f + ir.s2 * -1.0f + ir.s4 * 1.0f + ir.s5 * 2.0f;
    error = sum / ir.active;
  }

  integral = constrain(integral + error, -50.0f, 50.0f);
  float D = error - previousError;
  float pid = Kp * error + Ki * integral + Kd * D;
  previousError = error;
  lastError = error;
  lastPid = pid;

  int base = AUTO_BASE_SPEED;
  if (error <= -1.0 || error >= 1.0) base -= CORNER_BRAKE;

  setMotors(base + (int)pid, base - (int)pid);
}

// ===================== Auto run =====================
void enterPhase(AutoPhase p) {
  phase = p;
  phaseStartMs = millis();
  displayDirty = true;
}

void startAutoRun() {
  mode = MODE_AUTO;
  previousError = 0;
  integral = 0;
  junctionsPassed = 0;
  junctionSeenReads = 0;
  junctionLatched = false;
  lastJunctionMs = 0;
  detectedColor = COLOR_NONE;
  colorCandidate = COLOR_NONE;
  colorCandidateReads = 0;
  autoStartMs = millis();
  enterPhase(PH_TO_BRANCH);
  setStatus(targetOverride ? "AUTO (forced target)" : "AUTO: reading colour");
  sendParams();                        // log the settings used for this run
}

// Stop and hand control back to the user
void endAutoRun(const char* msg) {
  setMotors(0, 0);
  mode = MODE_MANUAL;
  manualLeft = 0;
  manualRight = 0;
  enterPhase(PH_IDLE);
  setStatus(msg);
}

void autoStep() {
  IRState ir = readIR();
  uint32_t now = millis();

  if (phase == PH_TO_BRANCH) pollColor();

  switch (phase) {
    case PH_TO_BRANCH: {
      // Leaving the START box: its outline looks like an all-black crossing
      if (now - autoStartMs < START_GRACE_MS) {
        if (ir.active >= 4 || ir.active == 0) setMotors(AUTO_BASE_SPEED, AUTO_BASE_SPEED);
        else followLine(ir, false);
        return;
      }

      if (ir.active >= 4) { endAutoRun("Stopped: all black"); return; }

      // Ball branch: line under the left sensors AND the centre, nothing far right.
      // Corners on this route are right turns, so they never match.
      bool leftBranch = ir.s1 && ir.s2 && (ir.s3 || ir.s4) && !ir.s5;
      junctionSeenReads = leftBranch ? junctionSeenReads + 1 : 0;

      if (leftBranch) {
        if (!junctionLatched && junctionSeenReads >= JUNCTION_CONFIRM_READS &&
            now - lastJunctionMs > JUNCTION_MIN_GAP_MS) {
          junctionLatched = true;
          lastJunctionMs = now;
          junctionsPassed++;
          displayDirty = true;

          int tgt = currentTarget();
          if (tgt == 0) { endAutoRun("No colour read!"); return; }

          char msg[24];
          snprintf(msg, sizeof(msg), "Branch %d of %d", junctionsPassed, tgt);
          setStatus(msg);

          if (junctionsPassed >= tgt) {
            setMotors(TURN_FORWARD_SPEED, TURN_FORWARD_SPEED);
            enterPhase(PH_TURN_FORWARD);
            return;
          }
        }
        followLine(ir, true);           // cross the unwanted branch straight
      } else {
        junctionLatched = false;
        followLine(ir, false);
      }
      break;
    }

    case PH_TURN_FORWARD:
      setMotors(TURN_FORWARD_SPEED, TURN_FORWARD_SPEED);
      if (now - phaseStartMs >= TURN_FORWARD_MS) {
        spinLeftMainLine = false;
        enterPhase(PH_TURN_SPIN);
      }
      break;

    case PH_TURN_SPIN: {
      setMotors(-TURN_SPIN_SPEED, TURN_SPIN_SPEED);   // pivot left
      uint32_t t = now - phaseStartMs;
      if (!ir.s3) spinLeftMainLine = true;
      if (t >= TURN_MIN_SPIN_MS && spinLeftMainLine && ir.s3) {
        previousError = 0;
        integral = 0;
        setMotors(0, 0);
        enterPhase(PH_TO_BALL);
        setStatus("On branch");
      } else if (t > TURN_TIMEOUT_MS) {
        endAutoRun("Turn timeout");
      }
      break;
    }

    case PH_TO_BALL:
      if (ir.active >= 4) {
        if (AUTO_CLOSE_GRIPPER_ON_ARRIVAL) { gripperClosed = true; applyGripper(); }
        char msg[24];
        snprintf(msg, sizeof(msg), "ARRIVED ball %d", currentTarget());
        endAutoRun(msg);
      } else {
        followLine(ir, false);
      }
      break;

    default:
      setMotors(0, 0);
      break;
  }
}

// ===================== BLE callbacks (core 0) =====================
// These only set flags; loop() does the actual work so the two cores
// never touch the motors / I2C at the same time.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    connectedCount = connectedCount + 1;
    deviceConnected = true;
    if (connectedCount < 3) s->startAdvertising();   // let a 2nd device (PC logger) join
  }
  void onDisconnect(BLEServer* s) override {
    connectedCount = max(0, connectedCount - 1);
    deviceConnected = connectedCount > 0;
    if (connectedCount == 0) {         // last link gone: stop for safety
      manualLeft = 0;
      manualRight = 0;
      reqDisconnectStop = true;
    }
    s->startAdvertising();
  }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    const uint8_t* d = c->getData();
    size_t len = c->getLength();

    // Bluefruit Connect control pad: "!B" <button> <'1' pressed | '0' released> <crc>
    if (len >= 4 && d[0] == '!' && d[1] == 'B') {
      char button = (char)d[2];
      char action = (char)d[3];

      if (action == '1') {
        switch (button) {
          case '1': reqManual = true; return;
          case '2': reqAuto = true; return;
          case '3': reqGripperToggle = true; return;
          case '4': reqCycleTarget = true; return;
        }
      }

      if (action == '0') { manualLeft = 0; manualRight = 0; return; }
      switch (button) {
        case '5': manualLeft =  MANUAL_SPEED; manualRight =  MANUAL_SPEED; break;  // up
        case '6': manualLeft = -MANUAL_SPEED; manualRight = -MANUAL_SPEED; break;  // down
        case '7': manualLeft = -MANUAL_SPEED; manualRight =  MANUAL_SPEED; break;  // left
        case '8': manualLeft =  MANUAL_SPEED; manualRight = -MANUAL_SPEED; break;  // right
      }
    }
    // Text line from a UART terminal or the PC logger - executed in loop()
    else if (len > 0 && !cmdPending) {
      size_t n = min(len, sizeof(cmdBuf) - 1);
      memcpy(cmdBuf, d, n);
      while (n > 0 && (cmdBuf[n - 1] == '\n' || cmdBuf[n - 1] == '\r' || cmdBuf[n - 1] == ' ')) n--;
      cmdBuf[n] = '\0';
      cmdPending = true;
    }
  }
};

// ===================== Text commands & telemetry =====================
// Commands (UART screen of the app, or tools/robot_link.py on a PC):
//   F B L R S        drive forward/back/left/right/stop (manual)
//   A / M / G        start auto run / manual / toggle gripper
//   force <0-3>      target override (0 = use colour)
//   set <name> <v>   change a tuning value, e.g. "set kp 45"
//   get              list all tuning values  ("P,name=value,...")
//   tel on|off|<hz>  telemetry stream, default 20 Hz
//   ping             replies "E,...,pong"
//
// Telemetry line (one per period):
//   T,ms,phase,ir,err,pid,L,R,branches,target,see,locked,C
//   phase: 0 manual/idle 1 to-branch 2 turn-fwd 3 turn-spin 4 to-ball
//   ir: 5 bits left->right (1 = black)   see/locked: N R G B   C: clear count
char colorChar(CardColor c) {
  switch (c) { case COLOR_RED: return 'R'; case COLOR_GREEN: return 'G'; case COLOR_BLUE: return 'B'; default: return 'N'; }
}

void sendParams() {
  char line[110];
  int n = snprintf(line, sizeof(line), "P");
  for (int i = 0; i < NUM_PARAMS; i++) {
    char item[24];
    const Param& p = PARAMS[i];
    if (p.type == 'f')      snprintf(item, sizeof(item), ",%s=%.2f", p.name, *(float*)p.ptr);
    else if (p.type == 'i') snprintf(item, sizeof(item), ",%s=%d", p.name, *(int*)p.ptr);
    else                    snprintf(item, sizeof(item), ",%s=%lu", p.name, (unsigned long)*(uint32_t*)p.ptr);
    if (n + strlen(item) >= sizeof(line) - 1) { bleSend(line); n = snprintf(line, sizeof(line), "P"); }
    n += snprintf(line + n, sizeof(line) - n, "%s", item);
  }
  if (n > 1) bleSend(line);
}

bool setParam(const char* name, const char* value) {
  for (int i = 0; i < NUM_PARAMS; i++) {
    const Param& p = PARAMS[i];
    if (strcasecmp(p.name, name) != 0) continue;
    if (p.type == 'f')      *(float*)p.ptr = atof(value);
    else if (p.type == 'i') *(int*)p.ptr = atoi(value);
    else                    *(uint32_t*)p.ptr = (uint32_t)strtoul(value, nullptr, 10);
    return true;
  }
  return false;
}

void sendTelemetry() {
  char line[96];
  char ir[6];
  for (int i = 0; i < 5; i++) ir[i] = (lastIrBits & (1 << (4 - i))) ? '1' : '0';
  ir[5] = '\0';
  snprintf(line, sizeof(line), "T,%lu,%d,%s,%.2f,%.0f,%d,%d,%d,%d,%c,%c,%u",
           (unsigned long)millis(), mode == MODE_AUTO ? (int)phase : 0, ir,
           lastError, lastPid, lastLeftCmd, lastRightCmd,
           junctionsPassed, currentTarget(),
           colorChar(lastColorClass), colorChar(detectedColor), lastColorC);
  bleSend(line);
}

void handleTextCommand(char* cmd) {
  char msg[40];

  // Single-letter drive commands (case-sensitive, kept from ManualControl)
  if (strlen(cmd) == 1) {
    switch (cmd[0]) {
      case 'F': manualLeft =  MANUAL_SPEED; manualRight =  MANUAL_SPEED; return;
      case 'B': manualLeft = -MANUAL_SPEED; manualRight = -MANUAL_SPEED; return;
      case 'L': manualLeft = -MANUAL_SPEED; manualRight =  MANUAL_SPEED; return;
      case 'R': manualLeft =  MANUAL_SPEED; manualRight = -MANUAL_SPEED; return;
      case 'S': manualLeft = 0; manualRight = 0; return;
      case 'A': reqAuto = true; return;
      case 'M': reqManual = true; return;
      case 'G': reqGripperToggle = true; return;
    }
  }

  char* word = strtok(cmd, " ");
  if (!word) return;
  char* arg1 = strtok(nullptr, " ");
  char* arg2 = strtok(nullptr, " ");

  if (!strcasecmp(word, "ping")) {
    setStatus("pong");
  } else if (!strcasecmp(word, "stop")) {
    if (mode == MODE_AUTO) endAutoRun("Stopped by command");
    manualLeft = 0; manualRight = 0;
  } else if (!strcasecmp(word, "get")) {
    sendParams();
  } else if (!strcasecmp(word, "set") && arg1 && arg2) {
    if (setParam(arg1, arg2)) { snprintf(msg, sizeof(msg), "set %s=%s", arg1, arg2); setStatus(msg); }
    else { snprintf(msg, sizeof(msg), "unknown param %s", arg1); setStatus(msg); }
  } else if (!strcasecmp(word, "force") && arg1) {
    targetOverride = constrain(atoi(arg1), 0, 3);
    snprintf(msg, sizeof(msg), "force target %d", targetOverride);
    setStatus(msg);
  } else if (!strcasecmp(word, "tel")) {
    if (!arg1 || !strcasecmp(arg1, "on")) telemetryOn = true;
    else if (!strcasecmp(arg1, "off")) telemetryOn = false;
    else {
      int hz = constrain(atoi(arg1), 1, 50);
      telemetryPeriodMs = 1000 / hz;
      telemetryOn = true;
    }
    snprintf(msg, sizeof(msg), "telemetry %s %luHz", telemetryOn ? "on" : "off",
             (unsigned long)(1000 / telemetryPeriodMs));
    setStatus(msg);
  } else {
    snprintf(msg, sizeof(msg), "unknown cmd %.20s", word);
    setStatus(msg);
  }
}

// ===================== Setup / loop =====================
void handleRequests() {
  if (cmdPending) {
    char cmd[sizeof(cmdBuf)];
    memcpy(cmd, cmdBuf, sizeof(cmd));
    cmdPending = false;
    handleTextCommand(cmd);
  }
  if (reqDisconnectStop) {
    reqDisconnectStop = false;
    if (mode == MODE_AUTO && ABORT_AUTO_ON_DISCONNECT) endAutoRun("BLE lost - stopped");
    displayDirty = true;
  }
  if (reqManual) {
    reqManual = false;
    if (mode == MODE_AUTO) endAutoRun("Manual (aborted)");
    else setStatus("Manual");
  }
  if (reqAuto) {
    reqAuto = false;
    startAutoRun();
  }
  if (reqGripperToggle) {
    reqGripperToggle = false;
    gripperClosed = !gripperClosed;
    applyGripper();
  }
  if (reqCycleTarget) {
    reqCycleTarget = false;
    if (mode == MODE_MANUAL) {
      targetOverride = (targetOverride + 1) % 4;
      setStatus(targetOverride ? "Target forced" : "Target from colour");
    }
  }
  if (deviceConnected != lastShownConnected) {
    lastShownConnected = deviceConnected;
    displayDirty = true;
  }
}

void setup() {
  // Motor pins LOW immediately so the wheels don't twitch on boot
  for (int pin : {IN1, IN2, IN3, IN4, ENA, ENB}) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }

  Serial.begin(115200);

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println("System Booting...");
    display.display();
  }

  colorSensorOk = colorSensorBegin() != 0;
  Serial.println(colorSensorOk ? "TCS34725 OK" : "TCS34725 NOT FOUND (use button 4 to force a target)");

  ledcSetup(PWM_CH_LEFT, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(ENA, PWM_CH_LEFT);
  ledcSetup(PWM_CH_RIGHT, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(ENB, PWM_CH_RIGHT);

  ledcSetup(SERVO_PWM_CH, SERVO_PWM_FREQ, SERVO_PWM_RES);
  ledcAttachPin(SERVO_PIN, SERVO_PWM_CH);
  applyGripper();

  for (int pin : {IR1, IR2, IR3, IR4, IR5}) pinMode(pin, INPUT);
  setMotors(0, 0);

  BLEDevice::init("ESP32-S3-Robot");
  BLEDevice::setMTU(247);              // allow long telemetry lines (peer decides final size)
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* svc = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = svc->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic* rx = svc->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  rx->setCallbacks(new RxCallbacks());
  svc->start();
  pServer->getAdvertising()->start();

  setStatus(colorSensorOk ? "Ready" : "Ready (no colour!)");
}

void loop() {
  handleRequests();

  if (mode == MODE_AUTO) {
    autoStep();
  } else {
    setMotors(manualLeft, manualRight);
    pollLiveColor();
  }

  if (telemetryOn && millis() - lastTelemetryMs >= telemetryPeriodMs) {
    lastTelemetryMs = millis();
    sendTelemetry();
  }

  if (displayDirty) {
    displayDirty = false;
    updateOLED();
  }

  delay(mode == MODE_AUTO ? 5 : 20);
}
