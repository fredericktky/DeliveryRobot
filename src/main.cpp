// =====================================================================
//  DeliveryRobot - AIROST Project 1 (basic and advanced task)
//
//  Run modes (phone button 4 cycles them in manual mode, shown on the OLED):
//   0 BASIC on the basic map (Figure 1)
//   1 BASIC on the advanced map (Figure 2): station branches are skipped
//   2 ADVANCED task on the advanced map
//
//  Basic task: read one colour card (RED -> ball 1, BLUE -> 2, GREEN -> 3),
//  take both right corners, count the ball branches on the LEFT, turn in,
//  stop on the end bar, close the gripper and switch to MANUAL.
//
//  Advanced task: read 3 cards on the way out
//     card 1 = ball (R1 B2 G3)   card 2 = station (R1 B2 G3)   card 3 = base (R1 B2)
//  ball: turn in, grab, reverse back to the T, turn left (east), cross the
//  other ball branches, LEFT corner, count stations from the bottom (on the
//  RIGHT), turn in, release, reverse back to the T, then
//     base 1: turn left (north), left corner, stop on the base 1 box edge
//     base 2: turn right (south), right corner, stop on the base 2 box edge
//  Testing without cards: "adv <ball> <station> <base>" forces all three.
//
//  Phone (Adafruit Bluefruit Connect -> Controller -> Control Pad):
//   Arrows : drive (manual mode, hold)
//   1      : MANUAL (also aborts an auto run)
//   2      : start AUTO run
//   3      : gripper open / close
//   4      : run mode: BASIC (basic map) -> BASIC (adv map) -> ADVANCED
//  Status messages are also sent to the app's UART screen.
//  Testing / tuning workflow: see TESTING.md.
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

// 5-channel IR array, left -> right (1 = black), sensors 23 mm apart.
// The board is shifted to the robot's LEFT so the 19 mm line fits in the gap
// between IR3 and IR4 (both white when centred). As built the gap is still
// 5-10 mm right of the robot's centre, so the robot rides that far left of the
// line - fine within the map's +-2 cm. Nominal positions from the gap centre:
//   IR1 -57.5 mm  far left: only sees a ball branch (not used for steering)
//   IR2 -34.5 mm  outer left     IR3 -11.5 mm  inner left
//   IR4 +11.5 mm  inner right    IR5 +34.5 mm  outer right
// Map patterns: centred 00000, right corner 00011, left branch 11100, crossbar 11111
// Geometry (measured): wheel track 13 cm (centre to centre); the IR bar is 8 cm
// AHEAD of the wheel axle. So a feature under the sensors reaches the wheels
// 8 cm later (cfwdms / fwdms cover that), and an in-place pivot swings the bar
// on an 8 cm radius.
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
int GRIPPER_OPEN_ANGLE = 0;              // TUNE live: "set gopen <deg>" / "grip <deg>"
int GRIPPER_CLOSED_ANGLE = 90;           // TUNE live: "set gclose <deg>"
int AUTO_CLOSE_GRIPPER_ON_ARRIVAL = 1;    // close the gripper on ARRIVED ("set gauto 0" to disable)

// ===================== Tuning =====================
// These are variables (not const) so they can be changed live over BLE:
//   "set kp 45"   "get"   - see the PARAMS table below for names.
// Values set live are lost on reset: copy good ones back here.
int MANUAL_SPEED = 200;

// Battery scale: every motor command is multiplied by this % at the motor driver,
// so all the speeds below (tuned on a 3-cell pack) give the same motor voltage on
// a 4-cell pack at 75 (= 3/4). 100 = no scaling. Telemetry shows the unscaled value.
// Left motor trim: + boosts the left motor by this %, - boosts the right one.
// For a robot that pulls to one side when both get the same command.
int MOTOR_TRIM_PCT = 0;                  // TUNE ("ltrim")
int MOTOR_SCALE_PCT = 95;                // TUNE: 4-cell pack (75/85 too weak to start, 100 too fast)

// PID line following (same values as LineFollowing / DualMode)
int   AUTO_BASE_SPEED = 145;             // TUNE (4-cell: 155 too fast; needs the start kick)
int   CORNER_BRAKE = 0;                  // speed drop when line is on outer sensors (35 left the inside wheel too weak after corners)
int   LOST_LINE_SPIN = 158;              // pivot speed (4-cell: 145 too weak, 170 overshoots corner 1)
uint32_t OUTER_MEMORY_MS = 120;          // all-white this soon after an outer sensor alone = drifted out, pivot back (300 spun repeatedly)
uint32_t LOST_STOP_MS = 2000;            // no sensor sees black this long = off the track, stop
uint32_t PIVOT_BRAKE_MS = 80;            // TUNE: counter-spin when a pivot finds the line (0 = off); 40 overturned corner 1 on the real map
uint32_t PIVOT_SETTLE_MS = 300;          // TUNE: no branch detection this long after a pivot
uint32_t CORNER_FWD_MS = 150;            // TUNE: drive on (at fwdspd) before a corner pivot
int      CORNER_CONFIRM_READS = 1;       // IR4+IR5 black this many loops (~5 ms each); 2 missed angled corners on the square
float Kp = 30.0, Ki = 0.0, Kd = 40.0;    // TUNE (error in sensor pitches: inner 0.5, outer 1.5). Square/straight
                                         // tests 2026-11-27: 50/70 bounced +-25, 25/35 slower; ki (even 0.2) grows the swings

// Start: ignore the START box outline for this long after pressing AUTO
uint32_t START_GRACE_MS = 1000;          // TUNE (700: the box outline set off drift pivots)
// Start kick: the motors need more PWM to start from rest than to keep rolling,
// so leave the START box at KICK_SPEED for KICK_MS, then drop to "speed".
int      KICK_SPEED = 160;               // TUNE
uint32_t KICK_MS = 300;                  // TUNE
// "Stopped: all black" on the way out: ignored for this long after AUTO (the
// start box and the first straight never have a full crossbar), and then only
// when all five stay black for ALL_BLACK_HOLD_MS (one bad read can't end a run).
uint32_t ALL_BLACK_IGNORE_MS = 2000;     // TUNE
uint32_t ALL_BLACK_HOLD_MS = 100;        // TUNE

// Junction (ball branch) detection on the left side
int      JUNCTION_CONFIRM_READS = 1;     // consecutive loops the pattern must be seen (2 missed ball branch 2 at 4-cell speed)
uint32_t JUNCTION_MIN_GAP_MS = 350;      // TUNE: min time between two branches

// Run mode ("set mode <0-2>" or phone button 4):
//   0 = BASIC task, basic map: no stations, every left branch is a ball branch.
//   1 = BASIC task, advanced map: the 3 station branches (on the left, heading
//       down) come first, so branches only count after CORNERS_BEFORE_BALLS corners.
//   2 = ADVANCED task (advanced map).
int      RUN_MODE = 1;
int      CORNERS_BEFORE_BALLS = 2;       // right corners before the ball branches
// Corner = STOP, CREEP, PIVOT (cstopms > 0): on the corner strip stop for
// cstopms, creep forward ccreepms at ccreepspd so the wheels (8 cm behind the
// IR bar) reach the corner, then pivot until IR3/IR4 finds the new line.
// Doesn't wait for the strip to clear, so an angled approach can't run it off.
// cstopms 0 = old behaviour: hold straight until the strip clears, cfwdms, pivot.
uint32_t CORNER_STOP_MS = 120;           // TUNE
uint32_t CORNER_CREEP_MS = 250;          // TUNE: longer if it cuts inside the corner, shorter if it swings wide
int      CORNER_CREEP_SPEED = 150;       // TUNE: must start from rest (130 may stall)
uint32_t PIVOT_MIN_MS = 120;             // TUNE: a corner pivot turns at least this long before the line can end it
                                         // (the creep can leave IR3/IR4 on the corner tape; 0 = off)
uint32_t WHITE_STEER_MS = 150;           // TUNE: all white this long = steer gently (+-0.5) toward the side the
                                         // line was LAST seen on, instead of straight (0 = off)
uint32_t GAP_MS = 0;                     // TUNE: all-white this long after IR3/IR4 = line hidden in an outer gap (0 = off;
                                         // 80 helped on the straight but a wrong guess could steer it off the square)
uint32_t CORNER_MAX_MS = 250;            // old behaviour only: strip never clears this long = pivot anyway
int      LOOP_CORNERS = 8;               // "auto loop" stops after this many corners (4 = one lap)
uint32_t CORNER_MIN_GAP_MS = 1000;       // corners are >= 40 cm apart; ignore re-pivots

// Turning into the branch
int      TURN_FORWARD_SPEED = 130;
uint32_t TURN_FORWARD_MS = 180;          // TUNE: drive on so the wheels sit over the junction
int      TURN_SPIN_SPEED = 158;          // TUNE (130 likely stalls, like lostspin 125)
uint32_t TURN_MIN_SPIN_MS = 150;         // ignore sensors for the first part of the spin
uint32_t TURN_TIMEOUT_MS = 2500;         // give up and go manual if the branch isn't found

// Colour card
int COLOR_CONFIRM_READS = 3;             // same colour this many reads in a row = locked
int CARD_MAX_C = 3000;                   // clear count below this = a card (white floor ~4000, cards 1200-2200); 0 = off
int CARD_WHITE_READS = 2;                // white reads between two cards (advanced: 3 cards in a row)

// Advanced task
uint32_t GRAB_MS = 400;                  // wait after the gripper has finished moving
// Servo speed: the gripper moves 1 degree every SERVO_STEP_MS instead of jumping,
// which limits the current spike (0 = jump straight to the angle).
uint32_t SERVO_STEP_MS = 10;             // TUNE ("gstep"): 90 deg = 0.9 s at 10
int      BACK_SPEED = 135;               // reverse speed back along a branch to the T
uint32_t BALL_BACK_MS = 250;             // TUNE: ignore ball crossbar for this long while reversing
uint32_t STATION_BACK_MS = 250;          // TUNE: ignore station crossbar for this long while reversing
uint32_t T_FWD_MS = 220;                 // TUNE: drive forward after backing onto the T before 90 deg turn
uint32_t BACK_TIMEOUT_MS = 3000;         // give up if T isn't reached while reversing
// Steering while reversing. The IR bar trails the wheels when reversing, which
// is unstable with strong steering, so: gentle correction on the inner sensors,
// and stop-and-pivot (no travel, so no snaking) when an edge sensor sees the line.
int      REVERSE_KP = 25;                // TUNE: PWM difference when IR3/IR4 sees the line (0 = off)
uint32_t REALIGN_TIMEOUT_MS = 1000;      // realign pivot gives up after this
int      TJUNC_READS = 2;                // T junction (line on both sides) confirm loops

// Live-tunable parameter table (name used in "set <name> <value>")
struct Param { const char* name; char type; void* ptr; };   // type: f=float i=int u=uint32
const Param PARAMS[] = {
  {"vscale",  'i', &MOTOR_SCALE_PCT},
  {"ltrim",   'i', &MOTOR_TRIM_PCT},
  {"speed",   'i', &AUTO_BASE_SPEED},
  {"kp",      'f', &Kp},
  {"ki",      'f', &Ki},
  {"kd",      'f', &Kd},
  {"brake",   'i', &CORNER_BRAKE},
  {"lostspin",'i', &LOST_LINE_SPIN},
  {"outerms", 'u', &OUTER_MEMORY_MS},
  {"loststop",'u', &LOST_STOP_MS},
  {"pbrake",  'u', &PIVOT_BRAKE_MS},
  {"psettle", 'u', &PIVOT_SETTLE_MS},
  {"cfwdms",  'u', &CORNER_FWD_MS},
  {"cornreads",'i', &CORNER_CONFIRM_READS},
  {"grace",   'u', &START_GRACE_MS},
  {"kick",    'i', &KICK_SPEED},
  {"kickms",  'u', &KICK_MS},
  {"bignore", 'u', &ALL_BLACK_IGNORE_MS},
  {"bhold",   'u', &ALL_BLACK_HOLD_MS},
  {"jreads",  'i', &JUNCTION_CONFIRM_READS},
  {"jgap",    'u', &JUNCTION_MIN_GAP_MS},
  {"mode",    'i', &RUN_MODE},
  {"corners", 'i', &CORNERS_BEFORE_BALLS},
  {"cgap",    'u', &CORNER_MIN_GAP_MS},
  {"loopc",   'i', &LOOP_CORNERS},
  {"cmaxms",  'u', &CORNER_MAX_MS},
  {"gapms",   'u', &GAP_MS},
  {"wsteer",  'u', &WHITE_STEER_MS},
  {"pivmin",  'u', &PIVOT_MIN_MS},
  {"cstopms", 'u', &CORNER_STOP_MS},
  {"ccreepms",'u', &CORNER_CREEP_MS},
  {"ccreepspd",'i', &CORNER_CREEP_SPEED},
  {"fwdspd",  'i', &TURN_FORWARD_SPEED},
  {"fwdms",   'u', &TURN_FORWARD_MS},
  {"turnspd", 'i', &TURN_SPIN_SPEED},
  {"turnmin", 'u', &TURN_MIN_SPIN_MS},
  {"turnto",  'u', &TURN_TIMEOUT_MS},
  {"creads",  'i', &COLOR_CONFIRM_READS},
  {"cardmax", 'i', &CARD_MAX_C},
  {"cwhite",  'i', &CARD_WHITE_READS},
  {"cdist",   'f', &colorMaxDistance},
  {"grabms",  'u', &GRAB_MS},
  {"gstep",   'u', &SERVO_STEP_MS},
  {"backspd", 'i', &BACK_SPEED},
  {"bback",   'u', &BALL_BACK_MS},
  {"sback",   'u', &STATION_BACK_MS},
  {"tfwdms",  'u', &T_FWD_MS},
  {"backto",  'u', &BACK_TIMEOUT_MS},
  {"rkp",     'i', &REVERSE_KP},
  {"realto",  'u', &REALIGN_TIMEOUT_MS},
  {"treads",  'i', &TJUNC_READS},
  {"manual",  'i', &MANUAL_SPEED},
  {"gopen",   'i', &GRIPPER_OPEN_ANGLE},
  {"gclose",  'i', &GRIPPER_CLOSED_ANGLE},
  {"gauto",   'i', &AUTO_CLOSE_GRIPPER_ON_ARRIVAL},
};
const int NUM_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

// Telemetry (off by default; "tel on", "tel 20" = 20 Hz, "tel off")
bool     telemetryOn = false;
bool     colorRawOn = false;             // "craw on": stream every colour reading as K lines
uint32_t telemetryPeriodMs = 50;

// Behaviour switches
const bool ABORT_AUTO_ON_DISCONNECT = true;

// ===================== State =====================
enum RobotMode { MODE_MANUAL, MODE_AUTO };
// Telemetry "phase" is this number.
enum AutoPhase {
  PH_IDLE,            // 0
  PH_TO_BRANCH,       // 1 out: start -> ball branch (reads cards)
  PH_TURN_FORWARD,    // 2 turn-in: drive on over the junction
  PH_TURN_SPIN,       // 3 turn-in: pivot into the branch
  PH_TO_BALL,         // 4 follow the ball branch to its end bar
  PH_GRAB,            // 5 advanced: gripper closed / opened, wait
  PH_BACKUP,          // 6 advanced: reverse straight back to the T, then 90 deg turn (phases 2-3)
  PH_TO_STATION,      // 7 advanced: bottom line east, left corner, count stations
  PH_TO_STATION_END,  // 8 advanced: follow the station branch to its end bar
  PH_TO_BASE,         // 9 advanced: to base 1 or 2, stop on the box edge
  PH_PID_TEST,        // 10 "auto pid": PID only (no corners/branches/cards), for a test track
  PH_LOOP_TEST        // 11 "auto loop": square test loop, corners + branches, no cards
};

RobotMode mode = MODE_MANUAL;
AutoPhase phase = PH_IDLE;

// Written by BLE callbacks (core 0), consumed by loop() (core 1)
volatile int  manualLeft = 0, manualRight = 0;
volatile bool reqManual = false, reqAuto = false;
volatile bool reqGripperToggle = false, reqCycleMode = false;
volatile bool deviceConnected = false;    // true while at least one device is connected
volatile int  connectedCount = 0;         // phone + PC logger can both be connected
volatile bool reqDisconnectStop = false;

// Text command from BLE (e.g. "set kp 45"), handed to loop() to execute
char          cmdBuf[64];
volatile bool cmdPending = false;

// Last values, for telemetry
int   lastLeftCmd = 0, lastRightCmd = 0;
float lastError = 0, lastPid = 0;
// Line position estimate (sensor pitches). The 19 mm line fits in EVERY 23 mm
// gap, so all-white is either centred (IR3-IR4 gap) or hidden in the IR2-IR3 /
// IR4-IR5 gap. Which one follows from the direction the line was moving when it
// left IR3 / IR4: centre -> IR3 -> white = IR2-IR3 gap (-1.0); IR2 -> IR3 ->
// white = centred again (0).
// Only after the white has lasted GAP_MS: a quick swing back through the centre
// takes ~30 ms and must not count. GAP_MS 0 = estimate off (all-white = centred).
float lineEst = 0, linePrevEst = 0;
float    lastSeenErr = 0;                // error the last time a steering sensor saw the line
uint32_t lineWhiteSinceMs = 0;           // all-white started (0 = a sensor sees the line)
int   lastIrBits = 0;
uint16_t  lastColorC = 0;
CardColor lastColorClass = COLOR_NONE;
uint32_t  lastTelemetryMs = 0;

// Auto-run state (loop() only)
float previousError = 0, integral = 0;
int      lastOuterSide = 0;              // -1 / +1: an outer sensor last saw the line on that side
uint32_t lastOuterMs = 0;
uint32_t lastBranchMs = 0;               // a branch (not-corner side) was last under the sensors
bool     branchFeature = false;          // a branch is under the branch-side sensors (cross it straight)
int      cornerSide = 1;                 // +1: corners turn right, branches on the left; -1: mirrored; 0: neither
uint32_t lastLineMs = 0;                 // any sensor last saw black
int      pivotDir = 0;                   // -1 / +1 while pivoting to find a corner line
uint32_t pivotStartMs = 0;               // corner move started (forward, then pivot)
uint32_t pivotEndMs = 0;                 // last pivot found its line (settle window starts)
uint32_t cornerLatchMs = 0;              // when the corner strip was latched
uint32_t pivotStopMs = 0;                // stop part of the current move (before the forward part)
int      pivotFwdSpeed = 0;              // forward speed of the current move
bool     pivotIsCorner = false;          // the current pivot is a counted corner (pivmin applies)
bool     cornerArmed = true;             // corner-side outer sensor read white since the last corner
uint32_t brakeUntilMs = 0;               // counter-spin brake after a pivot runs until then
int      brakeDir = 0;
uint32_t pivotFwdMs = 0;                 // forward part of the current move
bool     cornerFeature = false;          // a corner strip is under the corner-side sensors
int      cornerHitReads = 0, cornerClearReads = 0;
bool     noBranchesYet = false;          // first leg before corner 1: no branch guard
int      legCorners = 0;                 // corners taken in the current leg of the route
uint32_t lastCornerMs = 0;
uint32_t autoStartMs = 0;
uint32_t phaseStartMs = 0;
bool spinSawBranch = false;             // the outer sensor saw the target line during the 90 deg pivot
int       turnDir = -1;                  // 90 deg turn direction: -1 left, +1 right
uint32_t  turnFwdMs = 180;               // PH_TURN_FORWARD duration for the current turn
int       turnNextSide = 0;              // cornerSide after the turn
AutoPhase turnNext = PH_TO_BALL;         // phase after the turn
uint32_t  backClearMs = 250;             // PH_BACKUP: ignore crossbar this long before looking for the T
int       realignDir = 0;                // PH_BACKUP: -1/+1 while pivoting back onto the line
uint32_t  realignStartMs = 0;
uint32_t  backPausedMs = 0;              // time spent realigning (doesn't count toward backto)
int       tTurnDir = -1;                 // 90 deg turn at the T after reversing: -1 left, +1 right
int       tNextSide = -1;                // cornerSide after that T
AutoPhase tNext = PH_TO_STATION;         // phase after that T
int       tReads = 0;
int       stationsPassed = 0;
bool      haveBall = false;              // advanced: true between grab and release

int  junctionsPassed = 0;
int  junctionSeenReads = 0;
bool junctionLatched = false;
uint32_t lastJunctionMs = 0;

bool      colorSensorOk = false;
CardColor detectedColor = COLOR_NONE;    // card 1 (basic: the only card)
CardColor colorCandidate = COLOR_NONE;
int       colorCandidateReads = 0;
CardColor cards[3] = {COLOR_NONE, COLOR_NONE, COLOR_NONE};   // advanced: ball, station, base
int       cardCount = 0;
bool      cardLocked = false;            // the card under the sensor has been recorded
int       cardWhiteReads = 0;

// Live readout in manual mode (for checking the mounted sensor)
CardColor liveColor = COLOR_NONE;
CardColor liveCandidate = COLOR_NONE;
int       liveCandidateReads = 0;

int  targetOverride = 0;                 // 0 = use colour, 1..3 = forced ball
int  stationOverride = 0;                // advanced: forced station (0 = card 2)
int  baseOverride = 0;                   // advanced: forced base (0 = card 3)
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
struct ColorReading;
void sendColorRaw(const ColorReading& rd);

// ===================== Helpers =====================
int ballForColor(CardColor c) {
  switch (c) {
    case COLOR_RED:   return 1;
    case COLOR_BLUE:  return 2;
    case COLOR_GREEN: return 3;
    default:          return 0;
  }
}

bool advMap()  { return RUN_MODE >= 1; }
bool advTask() { return RUN_MODE == 2; }

const char* modeName() {
  switch (RUN_MODE) { case 0: return "BASIC/basic map"; case 1: return "BASIC/adv map"; default: return "ADVANCED"; }
}

int currentTarget() {
  return targetOverride > 0 ? targetOverride : ballForColor(detectedColor);
}
int stationTarget() {
  return stationOverride > 0 ? stationOverride : ballForColor(cards[1]);   // R1 B2 G3
}
int baseTarget() {
  if (baseOverride > 0) return baseOverride;
  if (cards[2] == COLOR_RED) return 1;
  if (cards[2] == COLOR_BLUE) return 2;
  return 0;                                                 // green / none: not defined by the rules
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
  int trim = constrain(MOTOR_TRIM_PCT, -50, 50);
  if (trim > 0) leftSpeed = constrain(leftSpeed * (100 + trim) / 100, -255, 255);
  else if (trim < 0) rightSpeed = constrain(rightSpeed * (100 - trim) / 100, -255, 255);
  int scale = constrain(MOTOR_SCALE_PCT, 0, 100);
  leftSpeed = leftSpeed * scale / 100;
  rightSpeed = rightSpeed * scale / 100;

  if (leftSpeed >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); leftSpeed = -leftSpeed; }

  if (rightSpeed >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); rightSpeed = -rightSpeed; }

  ledcWrite(PWM_CH_LEFT, leftSpeed);
  ledcWrite(PWM_CH_RIGHT, rightSpeed);
}

void servoOutput(int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t pulseUs = SERVO_MIN_US + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle / 180;
  uint32_t duty = pulseUs * ((1UL << SERVO_PWM_RES) - 1) / (1000000UL / SERVO_PWM_FREQ);
  ledcWrite(SERVO_PWM_CH, duty);
}

// Slow servo: servoWriteAngle sets the target, servoUpdate (every loop) steps to it.
int      servoPos = -1;                  // angle sent to the servo (-1 = not yet)
int      servoTarget = 0;
uint32_t servoStepMs = 0;

void servoWriteAngle(int angle) {
  servoTarget = constrain(angle, 0, 180);
  if (servoPos < 0 || SERVO_STEP_MS == 0) { servoPos = servoTarget; servoOutput(servoPos); }
}

bool servoDone() { return servoPos == servoTarget; }

void servoUpdate() {
  if (servoDone()) return;
  uint32_t now = millis();
  if (now - servoStepMs < SERVO_STEP_MS) return;
  servoStepMs = now;
  servoPos += servoTarget > servoPos ? 1 : -1;
  servoOutput(servoPos);
}

void applyGripper() {
  servoWriteAngle(gripperClosed ? GRIPPER_CLOSED_ANGLE : GRIPPER_OPEN_ANGLE);
  displayDirty = true;
}

// ===================== OLED =====================
char colorChar(CardColor c);   // defined with telemetry

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
  display.print(mode == MODE_AUTO ? "AUTO" : "MAN");

  // Run mode, two short lines on the right
  display.setTextSize(1);
  display.setCursor(56, 13);
  display.print(advTask() ? "ADVANCED" : "BASIC");
  display.setCursor(56, 22);
  display.print(advMap() ? "adv map" : "basic map");

  display.setCursor(0, 32);
  int tgt = currentTarget();
  if (advTask()) {
    snprintf(buf, sizeof(buf), "%c%c%c B%d S%d H%d", colorChar(cards[0]), colorChar(cards[1]),
             colorChar(cards[2]), tgt, stationTarget(), baseTarget());
  } else if (targetOverride > 0) snprintf(buf, sizeof(buf), "Target: B%d (forced)", tgt);
  else if (mode == MODE_MANUAL && colorSensorOk)
                          snprintf(buf, sizeof(buf), "Sees: %s", liveColor == COLOR_NONE ? "--" : colorName(liveColor));
  else if (tgt > 0)       snprintf(buf, sizeof(buf), "Colour %s -> B%d", colorName(detectedColor), tgt);
  else                    snprintf(buf, sizeof(buf), "Colour: --");
  display.print(buf);

  display.setCursor(0, 42);
  if (mode == MODE_AUTO && phase == PH_TO_BRANCH) snprintf(buf, sizeof(buf), "Branch %d/%d", junctionsPassed, tgt);
  else snprintf(buf, sizeof(buf), "%s", statusMsg);
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

// True when the reading looks like a card: darker than the white floor but not
// the black line. Stops white being read as GREEN (they are close in r/g/b).
bool cardUnderSensor(const ColorReading& rd) {
  if (rd.c <= COLOR_MIN_CLEAR) return false;
  return CARD_MAX_C <= 0 || rd.c < (uint16_t)CARD_MAX_C;
}

// Basic: lock the first card seen COLOR_CONFIRM_READS times in a row.
// Advanced: record up to 3 cards, each separated by CARD_WHITE_READS white reads.
void pollColor() {
  if (!colorSensorOk) return;
  if (advTask() ? cardCount >= 3 : detectedColor != COLOR_NONE) return;
  ColorReading rd;
  if (!colorSensorRead(rd)) return;
  lastColorC = rd.c;
  lastColorClass = rd.color;
  sendColorRaw(rd);

  CardColor c = cardUnderSensor(rd) ? rd.color : COLOR_NONE;
  if (c == COLOR_NONE) {
    colorCandidate = COLOR_NONE;
    colorCandidateReads = 0;
    if (++cardWhiteReads >= CARD_WHITE_READS) cardLocked = false;   // gap: ready for the next card
    return;
  }
  cardWhiteReads = 0;
  if (c == colorCandidate) colorCandidateReads++;
  else { colorCandidate = c; colorCandidateReads = 1; }
  if (cardLocked || colorCandidateReads < COLOR_CONFIRM_READS) return;

  cardLocked = true;
  char msg[24];
  if (!advTask()) {
    detectedColor = c;
    snprintf(msg, sizeof(msg), "Colour %s -> B%d", colorName(c), ballForColor(c));
  } else {
    cards[cardCount++] = c;
    if (cardCount == 1) detectedColor = c;                    // card 1 = the ball
    snprintf(msg, sizeof(msg), "Card %d: %s", cardCount, colorName(c));
  }
  setStatus(msg);
}

// Manual mode: show what the sensor sees right now (2 matching reads to change)
void pollLiveColor() {
  if (!colorSensorOk) return;
  ColorReading rd;
  if (!colorSensorRead(rd)) return;
  lastColorC = rd.c;
  lastColorClass = rd.color;
  sendColorRaw(rd);

  if (rd.color == liveCandidate) liveCandidateReads++;
  else { liveCandidate = rd.color; liveCandidateReads = 1; }

  if (liveCandidateReads >= 2 && liveCandidate != liveColor) {
    liveColor = liveCandidate;
    displayDirty = true;
  }
}

// ===================== Line following =====================
// The 19 mm line can never cover two neighbouring sensors (23 mm apart), so a
// drifting line only ever lights one sensor. Two neighbours together = a line
// lying ACROSS the sensors: a corner strip or a branch.
//   left side : IR2 with IR1 or IR3        right side: IR4 and IR5
// cornerSide says which side is a corner on this leg (turn that way) and which
// is a branch (cross it straight). Outbound: corners right, branches left.
// straightThrough = true forces straight driving (e.g. crossing a ball branch).
void resetLineMemory() {
  lastSeenErr = 0;
  lineWhiteSinceMs = 0;
  cornerArmed = true;
  lineEst = 0;
  linePrevEst = 0;
  lastOuterSide = 0;
  pivotDir = 0;
  branchFeature = false;
  cornerFeature = false;
  cornerHitReads = 0;
  cornerClearReads = 0;
}

// Start a turn: stop stopMs, drive forward fwdMs at fwdSpeed (0 = pivot at
// once), then pivot toward dir until an inner sensor finds the line.
// count = a map corner (logged, counted).
void startCorner(int dir, uint32_t fwdMs, uint32_t now, bool count,
                 uint32_t stopMs = 0, int fwdSpeed = -1) {
  pivotDir = dir;
  pivotFwdMs = fwdMs;
  pivotStopMs = stopMs;
  pivotIsCorner = count;
  pivotFwdSpeed = fwdSpeed < 0 ? TURN_FORWARD_SPEED : fwdSpeed;
  pivotStartMs = now;
  lastOuterSide = 0;
  if (count) {
    cornerArmed = false;               // re-armed once the corner-side outer sensor reads white
    lastCornerMs = now;                // only a real corner blocks the next one (cgap);
                                       // a drift-out pivot must not, or the next corner is missed
    legCorners++;
    char msg[24];
    snprintf(msg, sizeof(msg), "Corner %d %s", legCorners, dir > 0 ? "R" : "L");
    setStatus(msg);
  }
  if (stopMs > 0) setMotors(0, 0);
  else setMotors(pivotFwdSpeed, pivotFwdSpeed);
}

void followLine(const IRState& ir, bool straightThrough) {
  uint32_t now = millis();
  int n = ir.s2 + ir.s3 + ir.s4 + ir.s5;          // steering sensors (IR1 = branch detector)
  float error;

  // ---- Turn in progress (see startCorner) ----
  // Drive forward pivotFwdMs so the wheels sit over the corner, then pivot
  // until an INNER sensor finds the new line. The outer sensors sweep across
  // the corner line early in the pivot and must not end it.
  if (now < brakeUntilMs) {             // stop the pivot's momentum (else it overshoots)
    setMotors(-brakeDir * LOST_LINE_SPIN, brakeDir * LOST_LINE_SPIN);
    return;
  }
  if (pivotDir != 0) {
    if (now - pivotStartMs < pivotStopMs) {
      setMotors(0, 0);
      return;
    }
    if (now - pivotStartMs < pivotStopMs + pivotFwdMs) {
      setMotors(pivotFwdSpeed, pivotFwdSpeed);
      return;
    }
    bool tooEarly = pivotIsCorner &&
                    now - pivotStartMs < pivotStopMs + pivotFwdMs + PIVOT_MIN_MS;
    if (tooEarly || (!ir.s3 && !ir.s4)) {
      setMotors(pivotDir * LOST_LINE_SPIN, -pivotDir * LOST_LINE_SPIN);
      return;
    }
    brakeDir = pivotDir;
    brakeUntilMs = now + PIVOT_BRAKE_MS;
    pivotEndMs = now;
    pivotDir = 0;                      // found it: brake, then back to PID below
    lastOuterSide = 0;
    lineEst = 0;                       // fresh estimate on the new line
    lastSeenErr = 0;
    linePrevEst = 0;
    cornerFeature = false;
    cornerHitReads = 0;
  }

  if (cornerSide != 0) {
    bool leftHit  = ir.s2 && (ir.s1 || ir.s3);
    bool rightHit = ir.s4 && ir.s5;
    // A corner strip never reaches the branch side: both sides black = a branch / crossbar
    bool cornerHit   = cornerSide > 0 ? (rightHit && !ir.s1 && !ir.s2) : (leftHit && !ir.s4 && !ir.s5);
    bool branchHit   = cornerSide > 0 ? (leftHit || ir.s1) : rightHit;   // IR1 only ever sees a branch
    bool cornerClear = cornerSide > 0 ? (!ir.s4 && !ir.s5) : (!ir.s1 && !ir.s2);
    bool branchClear = cornerSide > 0 ? (!ir.s1 && !ir.s2) : (!ir.s4 && !ir.s5);

    // ---- Branch side: cross it straight until it clears ----
    bool settling = now - pivotEndMs < PIVOT_SETTLE_MS;    // overshoot after a pivot is not a branch
    if (branchHit && !noBranchesYet && !settling) branchFeature = true;
    if (branchClear) branchFeature = false;
    if (branchFeature) {
      lastBranchMs = now;
      straightThrough = true;
    }

    // ---- Corner side: confirm (noise filter), latch, turn when it clears ----
    // Once latched nothing cancels it until the corner-side sensors read white
    // again, whichever leaves the strip first (the robot may be slightly angled).
    // Arming: after a corner the new line may still lie across IR4+IR5 (a slightly
    // short pivot). That must not count as the next corner, so a corner only
    // counts once the corner-side outer sensor has read white again.
    if (cornerSide > 0 ? !ir.s5 : !ir.s2) cornerArmed = true;
    // To the ball branches there is no corner after the last one before the balls:
    // IR4+IR5 there (leaving a ball branch) is not a corner.
    bool noMoreCorners = phase == PH_TO_BRANCH && advMap() && legCorners >= CORNERS_BEFORE_BALLS;
    cornerHitReads = (cornerHit && cornerArmed && !noMoreCorners && now - lastCornerMs > CORNER_MIN_GAP_MS) ? cornerHitReads + 1 : 0;
    if (!cornerFeature && cornerHitReads >= CORNER_CONFIRM_READS) {
      if (CORNER_STOP_MS > 0) {        // stop, creep onto the corner, pivot
        cornerHitReads = 0;
        startCorner(cornerSide, CORNER_CREEP_MS, now, true, CORNER_STOP_MS, CORNER_CREEP_SPEED);
        return;
      }
      cornerFeature = true;
      cornerLatchMs = now;
    }
    if (cornerFeature) {
      cornerClearReads = cornerClear ? cornerClearReads + 1 : 0;
      if (CORNER_MAX_MS > 0 && now - cornerLatchMs > CORNER_MAX_MS) {
        // Strip never cleared (angled approach, or off the paper): pivot now,
        // no forward drive - the robot has already rolled on past the strip.
        cornerFeature = false;
        cornerClearReads = 0;
        startCorner(cornerSide, 0, now, true);
        return;
      }
      if (cornerClearReads >= 2) {     // strip passed: turn
        cornerFeature = false;
        cornerClearReads = 0;
        startCorner(cornerSide, CORNER_FWD_MS, now, true);
        return;
      }
      straightThrough = true;          // hold the heading while crossing the strip
    }
  }

  // ---- Drift memory (for the rare case the line drifts out past an edge) ----
  if (ir.s2 && !ir.s5)      { lastOuterSide = -1; lastOuterMs = now; }
  else if (ir.s5 && !ir.s2) { lastOuterSide = 1;  lastOuterMs = now; }
  else if (lastOuterSide == -1 && ir.s3 && !ir.s2) lastOuterSide = 0;
  else if (lastOuterSide == 1 && ir.s4 && !ir.s5)  lastOuterSide = 0;

  // ---- Position estimate (see lineEst) ----
  if (straightThrough) {
    lineEst = 0;
    linePrevEst = 0;
    lineWhiteSinceMs = 0;
  } else if (n > 0) {
    lineWhiteSinceMs = 0;
    float measured = (ir.s2 * -1.5f + ir.s3 * -0.5f + ir.s4 * 0.5f + ir.s5 * 1.5f) / n;
    lastSeenErr = measured;
    if (measured != lineEst) { linePrevEst = lineEst; lineEst = measured; }
  } else if (lineWhiteSinceMs == 0) {
    lineWhiteSinceMs = now;
  } else if (GAP_MS > 0 && now - lineWhiteSinceMs >= GAP_MS &&
             (lineEst == -0.5f || lineEst == 0.5f)) {
    float moved = lineEst - linePrevEst;           // direction it reached IR3 / IR4 from
    float next = lineEst + (moved < 0 ? -0.5f : 0.5f);
    linePrevEst = lineEst;
    lineEst = next;                                // -1.0 / +1.0 = hidden in an outer gap, 0 = centred
  }

  if (straightThrough) {
    lastOuterSide = 0;                 // a branch / corner strip is not a drift
    error = 0;
  } else if (n == 0) {
    // All white: centred between IR3 and IR4, a dashed gap, or lost.
    // Pivot in place only if an outer sensor just lost the line (drifted out).
    if (lastOuterSide != 0 && now - lastOuterMs <= OUTER_MEMORY_MS &&
        now - lastBranchMs > OUTER_MEMORY_MS) {
      startCorner(lastOuterSide, 0, now, false);
      return;
    }
    if (lineEst == -1.0f || lineEst == 1.0f) error = lineEst;       // hidden in an outer gap: keep steering
    else if (WHITE_STEER_MS > 0 && lineWhiteSinceMs != 0 && now - lineWhiteSinceMs >= WHITE_STEER_MS)
      error = lastSeenErr > 0 ? 0.5f : (lastSeenErr < 0 ? -0.5f : 0);  // drift back toward where it was last seen
    else error = 0;
  } else {
    float sum = ir.s2 * -1.5f + ir.s3 * -0.5f + ir.s4 * 0.5f + ir.s5 * 1.5f;
    error = sum / n;
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

void resetPid() {
  previousError = 0;
  integral = 0;
}

// New leg of the route: corners/branches counted afresh
void startLeg(AutoPhase p, int side) {
  cornerSide = side;
  legCorners = 0;
  resetLineMemory();
  resetPid();
  enterPhase(p);
}

void startAutoRun() {
  mode = MODE_AUTO;
  lastCornerMs = 0;
  lastLineMs = millis();
  junctionsPassed = 0;
  junctionSeenReads = 0;
  junctionLatched = false;
  lastJunctionMs = 0;
  stationsPassed = 0;
  tReads = 0;
  haveBall = false;
  detectedColor = COLOR_NONE;
  colorCandidate = COLOR_NONE;
  colorCandidateReads = 0;
  for (int i = 0; i < 3; i++) cards[i] = COLOR_NONE;
  cardCount = 0;
  cardLocked = false;
  cardWhiteReads = 0;
  if (advTask()) { gripperClosed = false; applyGripper(); }   // open, ready for the ball
  turnDir = -1;
  turnFwdMs = TURN_FORWARD_MS;
  turnNext = PH_TO_BALL;
  turnNextSide = 0;
  autoStartMs = millis();
  startLeg(PH_TO_BRANCH, 1);
  char msg[24];
  snprintf(msg, sizeof(msg), "AUTO %s", modeName());
  setStatus(msg);
  if (targetOverride || stationOverride || baseOverride) setStatus("forced targets");
  sendParams();                        // log the settings used for this run
}

// Advanced task, starting part-way so one leg can be tested on its own.
// Targets come from "adv <ball> <station> <base>".
bool startAutoAt(const char* where) {
  if (!advTask()) { setStatus("auto <leg>: mode 2 only"); return false; }
  if (currentTarget() == 0 || stationTarget() == 0 || baseTarget() == 0) {
    setStatus("set adv b s h first");
    return false;
  }
  AutoPhase p;
  int side = 0;
  bool ball;
  if (!strcasecmp(where, "ball"))         { p = PH_TO_BALL;        ball = false; }
  else if (!strcasecmp(where, "station")) { p = PH_TO_STATION;     ball = true; side = -1; }
  else if (!strcasecmp(where, "stend"))   { p = PH_TO_STATION_END; ball = true; }
  else { setStatus("auto ball|station|stend"); return false; }

  startAutoRun();                      // reset everything, then jump to the leg
  haveBall = ball;
  gripperClosed = ball;
  applyGripper();
  startLeg(p, side);
  if (p == PH_TO_STATION) legCorners = 1;   // as if corner 2 was just taken
  char msg[24];
  snprintf(msg, sizeof(msg), "AUTO from %s", where);
  setStatus(msg);
  return true;
}

// PID test ("auto pid"): plain line following on a straight / S-curve test
// track. No corner pivots, branch handling, colour reads or crossbar logic -
// only the PID (plus the drift-out pivot). Start kick as usual. Ends when the
// line is lost for loststop ms or all five sensors stay black (an end bar).
void startPidTest() {
  startAutoRun();
  startLeg(PH_PID_TEST, 0);            // cornerSide 0: no corners, no branches
  setStatus("AUTO pid test");
}

// Square test loop ("auto loop" / "auto loopl"): normal corner pivots and
// branch crossing, but no cards, no ball-branch turn-in, no crossbar stop.
//   loop : clockwise, corners RIGHT, branches on the LEFT (like the real outbound leg)
//   loopl: anticlockwise, corners LEFT, branches on the RIGHT (like the station leg)
// Branches crossed are counted in the telemetry "branches" field.
// Ends after loopc corners, or when the line is lost for loststop ms.
void startLoopTest(int side) {
  startAutoRun();
  startLeg(PH_LOOP_TEST, side);
  setStatus(side > 0 ? "AUTO loop R" : "AUTO loop L");
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

// Gripper action done: reverse straight back to the T (ignoring the crossbar
// for clearMs), drive forward T_FWD_MS, then pivot 90 deg (turnAtT: -1 left /
// +1 right) and continue in phase next with cornerSide = side.
void leaveBranch(uint32_t clearMs, int turnAtT, int side, AutoPhase next) {
  backClearMs = clearMs;
  realignDir = 0;
  backPausedMs = 0;
  tTurnDir = turnAtT;
  tNextSide = side;
  tNext = next;
  tReads = 0;
  enterPhase(PH_BACKUP);
}

void autoStep() {
  IRState ir = readIR();
  uint32_t now = millis();

  // Safety: centred on a straight is also all-white, but never for this long
  if (ir.active > 0) lastLineMs = now;
  bool following = phase == PH_TO_BRANCH || phase == PH_TO_BALL ||
                   phase == PH_TO_STATION || phase == PH_TO_STATION_END || phase == PH_TO_BASE ||
                   phase == PH_PID_TEST || phase == PH_LOOP_TEST;
  if (following && now - lastLineMs > LOST_STOP_MS) {
    endAutoRun("Line lost");
    return;
  }

  // No branch handling before corner 1, nor on the square test loop (branches removed)
  noBranchesYet = (phase == PH_TO_BRANCH && legCorners == 0) || phase == PH_LOOP_TEST;
  switch (phase) {
    case PH_TO_BRANCH: {
      // Leaving the START box: its outline looks like an all-black crossing
      if (now - autoStartMs < START_GRACE_MS) {
        // No pivots here: the box edge crossing the outer sensors at an angle
        // looks like a corner. Steer on the inner sensors only, else go straight.
        // No colour reads either: the box edge can read as a card.
        int n = ir.s2 + ir.s3 + ir.s4 + ir.s5;
        if (now - autoStartMs < KICK_MS) {
          int k = max(KICK_SPEED, AUTO_BASE_SPEED);
          setMotors(k, k);
        } else if (ir.active >= 4 || n == 0) setMotors(AUTO_BASE_SPEED, AUTO_BASE_SPEED);
        else followLine(ir, false);
        resetLineMemory();
        return;
      }
      if (legCorners == 0) pollColor();      // the cards are all before corner 1

      // Advanced: all three targets must be known by the first corner
      if (advTask() && legCorners >= 1) {
        if (currentTarget() == 0 || stationTarget() == 0 || baseTarget() == 0) {
          char msg[24];
          snprintf(msg, sizeof(msg), "Cards: %d read, need 3", cardCount);
          endAutoRun(msg);
          return;
        }
      }

      // Only the T crossbar / a box edge covers all five; corners and branches never do
      // Not while a corner move (stop, creep, pivot) or its settle time is running:
      // the robot stops ON the corner and its overlap can read all black.
      static uint32_t allBlackSinceMs = 0;
      bool cornerMove = pivotDir != 0 || now < brakeUntilMs || now - pivotEndMs < PIVOT_SETTLE_MS;
      if (ir.active == 5 && !cornerMove) {
        if (allBlackSinceMs == 0) allBlackSinceMs = now;
        if (now - autoStartMs >= ALL_BLACK_IGNORE_MS &&
            now - allBlackSinceMs >= ALL_BLACK_HOLD_MS) {
          allBlackSinceMs = 0;
          endAutoRun("Stopped: all black");
          return;
        }
      } else {
        allBlackSinceMs = 0;
      }

      // Ball branch: far-left IR1 and outer-left IR2 black, outer-right IR5 white.
      // The 19 mm line can't cover IR1 and IR2 (23 mm apart) except on a branch,
      // and corners on this route are right turns (00011), so they never match.
      // No branches before corner 1: a wobble onto IR1+IR2 there must be steered, not crossed.
      // IR1+IR2, or IR2+IR3+IR4 (3 wide = a crossing line; the 19 mm line covers
      // at most 2) for when the robot is right of the line and IR1 misses the branch.
      bool leftBranch = legCorners >= 1 && now - pivotEndMs >= PIVOT_SETTLE_MS && !ir.s5 &&
                        ((ir.s1 && ir.s2) || (ir.s2 && ir.s3 && ir.s4));
      junctionSeenReads = leftBranch ? junctionSeenReads + 1 : 0;
      // Before the ball section (station branches): cross straight, don't count
      bool countBranches = !advMap() || legCorners >= CORNERS_BEFORE_BALLS;

      if (leftBranch) {
        if (countBranches && !junctionLatched && junctionSeenReads >= JUNCTION_CONFIRM_READS &&
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
            turnDir = -1;
            turnFwdMs = TURN_FORWARD_MS;
            turnNext = PH_TO_BALL;
            turnNextSide = 0;
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
      if (now - phaseStartMs >= turnFwdMs) {
        spinSawBranch = false;
        enterPhase(PH_TURN_SPIN);
      }
      break;

    case PH_TURN_SPIN: {
      setMotors(turnDir * TURN_SPIN_SPEED, -turnDir * TURN_SPIN_SPEED);
      uint32_t t = now - phaseStartMs;
      // The target line sweeps in from the turn side (left: IR1/IR2 then IR3;
      // right: IR5 then IR4). The old line moves the other way in the pivot,
      // so the inner sensor alone (no outer before it) is still the old line.
      bool outer = turnDir < 0 ? (ir.s1 || ir.s2) : ir.s5;
      bool inner = turnDir < 0 ? ir.s3 : ir.s4;
      if (outer) spinSawBranch = true;
      if (t >= TURN_MIN_SPIN_MS && spinSawBranch && inner) {
        setMotors(0, 0);
        startLeg(turnNext, turnNextSide);
        if (turnNext == PH_TO_BALL)             setStatus("On branch");
        else if (turnNext == PH_TO_STATION_END) setStatus("On station branch");
        else if (turnNext == PH_TO_STATION)     setStatus("To station");
        else                                    setStatus("To base");
      } else if (t > TURN_TIMEOUT_MS) {
        endAutoRun("Turn timeout");
      }
      break;
    }

    case PH_TO_BALL:
      if (ir.active >= 4) {
        char msg[24];
        if (advTask()) {
          setMotors(0, 0);
          gripperClosed = true;
          applyGripper();
          haveBall = true;
          snprintf(msg, sizeof(msg), "Got ball %d", currentTarget());
          setStatus(msg);
          enterPhase(PH_GRAB);
        } else {
          if (AUTO_CLOSE_GRIPPER_ON_ARRIVAL) { gripperClosed = true; applyGripper(); }
          snprintf(msg, sizeof(msg), "ARRIVED ball %d", currentTarget());
          endAutoRun(msg);
        }
      } else {
        followLine(ir, false);
      }
      break;

    case PH_GRAB:                       // gripper moving
      setMotors(0, 0);
      if (!servoDone()) phaseStartMs = now;          // GRAB_MS counts from the end of the move
      if (now - phaseStartMs >= GRAB_MS) {
        if (haveBall) {
          // Reverse straight back up the ball branch to the T (still facing into
          // the branch), then turn LEFT (east): corners now turn LEFT and the
          // other ball branches are on the RIGHT.
          leaveBranch(BALL_BACK_MS, -1, -1, PH_TO_STATION);
        } else {
          // Released at the station: reverse back to the vertical line, then to the base
          if (baseTarget() == 1) leaveBranch(STATION_BACK_MS, -1, -1, PH_TO_BASE);  // north, left corner
          else                   leaveBranch(STATION_BACK_MS, 1, 1, PH_TO_BASE);    // south, right corner
        }
      }
      break;

    case PH_BACKUP: {
      // Reverse back to the T on the main line (line on BOTH sides), then drive
      // forward T_FWD_MS and pivot 90 deg onto the main line. The IR bar keeps
      // the reverse on the branch (see REVERSE_KP).
      uint32_t t = now - phaseStartMs;
      bool tHit = (ir.s1 || ir.s2) && (ir.s4 || ir.s5);
      bool leftEdge  = (ir.s1 || ir.s2) && !ir.s4 && !ir.s5;
      bool rightEdge = ir.s5 && !ir.s1 && !ir.s2;

      if (t < backClearMs) {             // still on the end bar: straight back, no checks
        setMotors(-BACK_SPEED, -BACK_SPEED);
        break;
      }

      if (realignDir != 0) {
        // Pivot in place until an inner sensor has the line again (or we reach the T)
        if (ir.s3 || ir.s4 || tHit) {
          backPausedMs += now - realignStartMs;
          realignDir = 0;
        } else if (now - realignStartMs > REALIGN_TIMEOUT_MS) {
          endAutoRun("Reverse lost line");
          return;
        } else {
          setMotors(realignDir * LOST_LINE_SPIN, -realignDir * LOST_LINE_SPIN);
          break;
        }
      }

      if (!tHit && (leftEdge || rightEdge) && tReads == 0) {
        // Edge sensor: the bar is well off the line. Stop and pivot toward it:
        // line on the right -> pivot right (moves the bar right), and vice versa.
        realignDir = rightEdge ? 1 : -1;
        realignStartMs = now;
        setMotors(realignDir * LOST_LINE_SPIN, -realignDir * LOST_LINE_SPIN);
        setStatus(rightEdge ? "Reverse: realign R" : "Reverse: realign L");
        break;
      }

      // Gentle steering on the inner sensors: same turning sense as forward
      // (line on the right -> turn right), kept small because the bar trails.
      int e = (ir.s4 && !ir.s3) ? 1 : (ir.s3 && !ir.s4) ? -1 : 0;
      if (tHit) e = 0;
      setMotors(-BACK_SPEED + e * REVERSE_KP, -BACK_SPEED - e * REVERSE_KP);

      {
        tReads = tHit ? tReads + 1 : 0;
        if (tReads >= TJUNC_READS) {
          turnDir = tTurnDir;
          turnFwdMs = T_FWD_MS;
          turnNext = tNext;
          turnNextSide = tNextSide;
          setStatus(tTurnDir > 0 ? "T: turn right" : "T: turn left");
          setMotors(TURN_FORWARD_SPEED, TURN_FORWARD_SPEED);
          enterPhase(PH_TURN_FORWARD);
          return;
        }
      }
      if (t - backPausedMs > BACK_TIMEOUT_MS) endAutoRun("Back timeout");
      break;
    }

    case PH_TO_STATION: {
      // East along the bottom line (ball branches on the right: cross them),
      // LEFT corner, then north: stations on the RIGHT, station 3 comes first.
      // (pivotDir: the T-turn that starts this leg still has the T under the sensors)
      if (ir.active == 5 && pivotDir == 0) { endAutoRun("Stopped: all black"); return; }
      if (legCorners >= 1 && pivotDir == 0) {
        bool rightBranch = ir.s4 && ir.s5 && !ir.s1 && !ir.s2;
        junctionSeenReads = rightBranch ? junctionSeenReads + 1 : 0;
        if (rightBranch && !junctionLatched && junctionSeenReads >= JUNCTION_CONFIRM_READS &&
            now - lastJunctionMs > JUNCTION_MIN_GAP_MS) {
          junctionLatched = true;
          lastJunctionMs = now;
          stationsPassed++;
          int want = 4 - stationTarget();               // station 3 -> 1st, station 1 -> 3rd
          char msg[24];
          snprintf(msg, sizeof(msg), "Station pass %d of %d", stationsPassed, want);
          setStatus(msg);
          if (stationsPassed >= want) {
            turnDir = 1;
            turnFwdMs = TURN_FORWARD_MS;
            turnNext = PH_TO_STATION_END;
            turnNextSide = 0;
            setMotors(TURN_FORWARD_SPEED, TURN_FORWARD_SPEED);
            enterPhase(PH_TURN_FORWARD);
            return;
          }
        }
        if (!rightBranch) junctionLatched = false;
      }
      followLine(ir, false);
      break;
    }

    case PH_TO_STATION_END:
      if (ir.active >= 4) {
        setMotors(0, 0);
        gripperClosed = false;
        applyGripper();
        haveBall = false;
        char msg[24];
        snprintf(msg, sizeof(msg), "Placed at station %d", stationTarget());
        setStatus(msg);
        enterPhase(PH_GRAB);
      } else {
        followLine(ir, false);
      }
      break;

    case PH_TO_BASE:
      // After one corner the next all-black is the base box edge
      if (ir.active == 5 && pivotDir == 0) {
        char msg[24];
        if (legCorners >= 1) { snprintf(msg, sizeof(msg), "DONE base %d", baseTarget()); endAutoRun(msg); }
        else endAutoRun("Stopped: all black");
        return;
      }
      followLine(ir, false);
      break;

    case PH_PID_TEST: {
      static uint32_t pidBlackSinceMs = 0;
      if (ir.active == 5 && now - autoStartMs >= START_GRACE_MS) {
        if (pidBlackSinceMs == 0) pidBlackSinceMs = now;
        if (now - pidBlackSinceMs >= ALL_BLACK_HOLD_MS) {
          pidBlackSinceMs = 0;
          endAutoRun("PID test: end bar");
          return;
        }
      } else {
        pidBlackSinceMs = 0;
      }
      if (now - autoStartMs < KICK_MS) {
        int k = max(KICK_SPEED, AUTO_BASE_SPEED);
        setMotors(k, k);
      } else {
        followLine(ir, false);
      }
      break;
    }

    case PH_LOOP_TEST: {
      // Stuck (wheels spinning on the paper, caught on tape): the IR pattern
      // stays the same for 1.5 s while the line is in view. Stop instead of
      // counting the same corner again and again.
      static uint8_t stuckBits = 0xFF;
      static uint32_t stuckSinceMs = 0;
      uint8_t steerBits = lastIrBits & 0x0F;   // IR2..IR5 (IR1 is branch-only and may flicker)
      if (steerBits != stuckBits || steerBits == 0 || now - autoStartMs < 1000) {
        stuckBits = steerBits;
        stuckSinceMs = now;
      } else if (now - stuckSinceMs > 1500) {
        stuckBits = 0xFF;
        char msg[24];
        snprintf(msg, sizeof(msg), "Stuck %dc %db", legCorners, junctionsPassed);
        endAutoRun(msg);
        return;
      }
      if (legCorners >= LOOP_CORNERS && pivotDir == 0 && now >= brakeUntilMs) {
        char msg[24];
        snprintf(msg, sizeof(msg), "Loop done %dc %db", legCorners, junctionsPassed);
        endAutoRun(msg);
        return;
      }
      if (now - autoStartMs < KICK_MS) {
        int k = max(KICK_SPEED, AUTO_BASE_SPEED);
        setMotors(k, k);
        return;
      }
      bool wasBranch = branchFeature;
      followLine(ir, false);
      if (branchFeature && !wasBranch) {
        junctionsPassed++;
        char msg[24];
        snprintf(msg, sizeof(msg), "Branch %d", junctionsPassed);
        setStatus(msg);
      }
      break;
    }

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
          case '4': reqCycleMode = true; return;
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
//   adv <b> <s> <h>  advanced: force ball, station, base (0 0 0 = use cards)
//   set <name> <v>   change a tuning value, e.g. "set kp 45"
//   get              list all tuning values  ("P,name=value,...")
//   tel on|off|<hz>  telemetry stream, default 20 Hz
//   ping             replies "E,...,pong"
//
// Telemetry line (one per period):
//   T,ms,phase,ir,err,pid,L,R,branches,target,see,locked,C
//   phase: see enum AutoPhase (0 idle, 1 to-branch ... 9 to-base, 10 pid test, 11 loop test)
//
// Colour calibration (tools/calibrate_color.py):
//   craw on|off        stream raw colour readings: K,ms,c,r%,g%,b%,class,dist
//   cref               list the colour references: R,label,r,g,b
//   cref <label> r g b change one reference live (RED GREEN BLUE WHITE BLACK)
// Start an advanced run part-way (targets from "adv b s h"):
//   auto ball          robot on a ball branch facing the end bar
//   auto station       robot on the vertical line heading north, below station 3, ball held
//   auto stend         robot on a station branch facing the end bar, ball held
// PID tuning on a test track (any run mode):
//   auto pid           PID line following only; stops on line lost / end bar
//   auto loop          square loop clockwise: corners right, branches left (loopc corners)
//   auto loopl         square loop anticlockwise: corners left, branches right
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

void sendColorRaw(const ColorReading& rd) {
  if (!colorRawOn) return;
  char line[64];
  const char* label = rd.refIndex >= 0 ? COLOR_REFS[rd.refIndex].label : "-";
  snprintf(line, sizeof(line), "K,%lu,%u,%.1f,%.1f,%.1f,%s,%.1f",
           (unsigned long)millis(), rd.c, rd.rp, rd.gp, rd.bp, label, rd.distance);
  bleSend(line);
}

void sendColorRefs() {
  char line[48];
  for (int i = 0; i < NUM_COLOR_REFS; i++) {
    snprintf(line, sizeof(line), "R,%s,%.1f,%.1f,%.1f", COLOR_REFS[i].label,
             COLOR_REFS[i].r, COLOR_REFS[i].g, COLOR_REFS[i].b);
    bleSend(line);
  }
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
  char* arg3 = strtok(nullptr, " ");
  char* arg4 = strtok(nullptr, " ");

  if (!strcasecmp(word, "ping")) {
    setStatus("pong");
  } else if (!strcasecmp(word, "stop")) {
    if (mode == MODE_AUTO) endAutoRun("Stopped by command");
    manualLeft = 0; manualRight = 0;
  } else if (!strcasecmp(word, "get")) {
    sendParams();
  } else if (!strcasecmp(word, "set") && arg1 && arg2) {
    if (setParam(arg1, arg2)) {
      snprintf(msg, sizeof(msg), "set %s=%s", arg1, arg2);
      setStatus(msg);
      if (!strncasecmp(arg1, "gopen", 5) || !strncasecmp(arg1, "gclose", 6)) applyGripper();  // show it
    }
    else { snprintf(msg, sizeof(msg), "unknown param %s", arg1); setStatus(msg); }
  } else if (!strcasecmp(word, "grip") && arg1) {
    // grip open | grip close | grip <deg> (raw angle, for finding gopen/gclose)
    if (!strcasecmp(arg1, "open"))       { gripperClosed = false; applyGripper(); }
    else if (!strcasecmp(arg1, "close")) { gripperClosed = true;  applyGripper(); }
    else { servoWriteAngle(atoi(arg1)); displayDirty = true; }
    snprintf(msg, sizeof(msg), "grip %s", arg1);
    setStatus(msg);
  } else if (!strcasecmp(word, "adv") && arg1) {
    targetOverride = constrain(atoi(arg1), 0, 3);
    stationOverride = arg2 ? constrain(atoi(arg2), 0, 3) : 0;
    baseOverride = arg3 ? constrain(atoi(arg3), 0, 2) : 0;
    snprintf(msg, sizeof(msg), "adv ball %d stn %d base %d", targetOverride, stationOverride, baseOverride);
    setStatus(msg);
    displayDirty = true;
  } else if (!strcasecmp(word, "force") && arg1) {
    targetOverride = constrain(atoi(arg1), 0, 3);
    snprintf(msg, sizeof(msg), "force target %d", targetOverride);
    setStatus(msg);
  } else if (!strcasecmp(word, "craw")) {
    colorRawOn = !arg1 || !strcasecmp(arg1, "on");
    setStatus(colorRawOn ? "colour raw on" : "colour raw off");
  } else if (!strcasecmp(word, "cref")) {
    if (arg1 && arg2 && arg3 && arg4) {
      bool found = false;
      for (int i = 0; i < NUM_COLOR_REFS; i++) {
        if (strcasecmp(COLOR_REFS[i].label, arg1) != 0) continue;
        COLOR_REFS[i].r = atof(arg2);
        COLOR_REFS[i].g = atof(arg3);
        COLOR_REFS[i].b = atof(arg4);
        found = true;
      }
      snprintf(msg, sizeof(msg), found ? "cref %s set" : "cref: no %s", arg1);
      setStatus(msg);
    }
    sendColorRefs();
  } else if (!strcasecmp(word, "auto") && arg1) {
    if (!strcasecmp(arg1, "pid")) startPidTest();
    else if (!strcasecmp(arg1, "loop")) startLoopTest(1);
    else if (!strcasecmp(arg1, "loopl")) startLoopTest(-1);
    else startAutoAt(arg1);
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
  if (reqCycleMode) {
    reqCycleMode = false;
    if (mode == MODE_MANUAL) {
      RUN_MODE = (RUN_MODE + 1) % 3;
      char msg[24];
      snprintf(msg, sizeof(msg), "Mode %d %s", RUN_MODE, modeName());
      setStatus(msg);
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
  Serial.println(colorSensorOk ? "TCS34725 OK" : "TCS34725 NOT FOUND (use force / adv to set targets)");

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
  servoUpdate();

  if (mode == MODE_AUTO) {
    autoStep();
  } else {
    setMotors(manualLeft, manualRight);
    readIR();                 // keep telemetry's IR bits live while idle
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
