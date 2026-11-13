// ============================================================
//  RC Car — Firmware v3  (4WD, dual TB6612FNG)
//  ESP32 + 2× TB6612FNG + MG90S + 4× N20-1000 + 8BitDo 2C
//  Board: ESP32 Dev Module (Bluepad32 board package)
//  Fixes: timer 3 allocation before Bluepad32, GPIO 0 removed,
//         servo attach order corrected
// ============================================================

#include <Bluepad32.h>
#include <ESP32Servo.h>

// ── Servo & peripheral pins ───────────────────────────────────
#define PIN_SERVO      21
#define PIN_LED_LEFT   32
#define PIN_LED_RIGHT  33
#define PIN_LED_BRAKE  19
#define PIN_BUZZER     23

// ── TB6612FNG #1 — FRONT axle ─────────────────────────────────
#define F_AIN1   25   // Front-Left direction A
#define F_AIN2   26   // Front-Left direction B
#define F_PWMA   27   // Front-Left speed
#define F_BIN1   14   // Front-Right direction A
#define F_BIN2   12   // Front-Right direction B
#define F_PWMB   13   // Front-Right speed

// ── TB6612FNG #2 — REAR axle ──────────────────────────────────
#define R_AIN1   16   // Rear-Left direction A
#define R_AIN2   17   // Rear-Left direction B
#define R_PWMA    4   // Rear-Left speed
#define R_BIN1    2   // Rear-Right direction A
#define R_BIN2   15   // Rear-Right direction B
#define R_PWMB   22   // Rear-Right speed (GPIO 22 — safe pin)

// Both STBY pins wired together to this single ESP32 pin
#define PIN_STBY  5

// ── Tuning constants — adjust these to fit your build ─────────
#define SERVO_CENTER      90   // degrees — tweak if wheels don't center
#define SERVO_LEFT_MAX    45   // degrees — full left lock
#define SERVO_RIGHT_MAX  135   // degrees — full right lock
#define DEADBAND          20   // joystick deadband (out of ±512)
#define TRIGGER_DEADBAND  15   // trigger noise floor (out of 0–1023)
#define MAX_SPEED        200   // governed speed 0–255
#define BOOST_SPEED      255   // hold A for full power
#define AXIS_MAX         512   // Bluepad32 axis range

// ── Blinker timing ────────────────────────────────────────────
#define BLINK_ON_MS   300
#define BLINK_OFF_MS  300

// ── Reverse reading method ────────────────────────────────────
// If L2 trigger doesn't reverse, change BRAKE_METHOD to AXISRX_METHOD
#define BRAKE_METHOD   0
#define AXISRX_METHOD  1
#define READ_REVERSE   BRAKE_METHOD

// ── Globals ───────────────────────────────────────────────────
Servo steeringServo;
ControllerPtr myController = nullptr;

bool leftSignalOn  = false;
bool rightSignalOn = false;
bool hazardOn      = false;
unsigned long lastBlinkTime = 0;
bool blinkState = false;

bool diagMode = false;
unsigned long diagStart = 0;

// ── Bluepad32 callbacks ───────────────────────────────────────
void onConnectedController(ControllerPtr ctl) {
  Serial.println("=== Controller connected ===");
  Serial.println("Running 5s diagnostic — move triggers and sticks...");
  myController = ctl;
  diagMode  = true;
  diagStart = millis();
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("Controller disconnected — stopping.");
  myController = nullptr;
  stopAll();
}

// ── Motor helpers ─────────────────────────────────────────────
void setMotorChannel(uint8_t in1, uint8_t in2, uint8_t pwm, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, speed);
}

void driveAll(int speed) {
  setMotorChannel(F_AIN1, F_AIN2, F_PWMA, speed);  // Front-Left
  setMotorChannel(F_BIN1, F_BIN2, F_PWMB, speed);  // Front-Right
  setMotorChannel(R_AIN1, R_AIN2, R_PWMA, speed);  // Rear-Left
  setMotorChannel(R_BIN1, R_BIN2, R_PWMB, speed);  // Rear-Right
}

void stopMotors() {
  driveAll(0);
}

void stopAll() {
  stopMotors();
  steeringServo.write(SERVO_CENTER);
  digitalWrite(PIN_LED_LEFT,  LOW);
  digitalWrite(PIN_LED_RIGHT, LOW);
  digitalWrite(PIN_LED_BRAKE, LOW);
  noTone(PIN_BUZZER);
}

// ── Trigger reading ───────────────────────────────────────────
int getRawForward() {
  return myController->throttle();  // R2, range 0–1023
}

int getRawReverse() {
#if READ_REVERSE == BRAKE_METHOD
  return myController->brake();     // L2, range 0–1023
#else
  int v = myController->axisRX();
  return (v > 0) ? map(v, 0, 512, 0, 1023) : 0;
#endif
}

// ── Blinker update ────────────────────────────────────────────
void updateBlinkers(bool l, bool r, bool haz) {
  unsigned long now = millis();
  if (now - lastBlinkTime >= (blinkState ? BLINK_ON_MS : BLINK_OFF_MS)) {
    blinkState    = !blinkState;
    lastBlinkTime = now;
  }
  if (haz) {
    digitalWrite(PIN_LED_LEFT,  blinkState ? HIGH : LOW);
    digitalWrite(PIN_LED_RIGHT, blinkState ? HIGH : LOW);
  } else {
    digitalWrite(PIN_LED_LEFT,  l && blinkState ? HIGH : LOW);
    digitalWrite(PIN_LED_RIGHT, r && blinkState ? HIGH : LOW);
  }
}

// ── Diagnostic print ──────────────────────────────────────────
void runDiagnostic() {
  static unsigned long lastDiag = 0;
  if (millis() - lastDiag < 150) return;
  lastDiag = millis();
  Serial.printf(
    "[DIAG] throttle:%4d  brake:%4d  axisY:%4d  axisRY:%4d  axisRX:%4d  btns:0x%04X\n",
    myController->throttle(),
    myController->brake(),
    myController->axisY(),
    myController->axisRY(),
    myController->axisRX(),
    myController->buttons()
  );
  if (millis() - diagStart > 5000) {
    diagMode = false;
    Serial.println("[DIAG] Done — ready to drive!");
    Serial.println("If reverse didn't work, change READ_REVERSE to AXISRX_METHOD.");
  }
}

// ── Main gamepad processing ───────────────────────────────────
void processGamepad() {
  if (!myController || !myController->isConnected()) {
    stopAll();
    return;
  }

  // Diagnostic runs for first 5 seconds after connect
  if (diagMode) {
    runDiagnostic();
    stopMotors();
    steeringServo.write(SERVO_CENTER);
    return;
  }

  // ── Read buttons ──────────────────────────────────────────
  bool btnA      = myController->a();
  bool btnB      = myController->b();
  bool bumperL   = myController->l1();
  bool bumperR   = myController->r1();
  bool btnSelect = myController->miscSelect();
  bool btnStart  = myController->miscStart();

  // ── Throttle via triggers ─────────────────────────────────
  int rawFwd = getRawForward();
  int rawRev = getRawReverse();
  int throttle = 0;

  if (rawFwd > TRIGGER_DEADBAND) {
    throttle = map(rawFwd, 0, 1023, 0, btnA ? BOOST_SPEED : MAX_SPEED);
  } else if (rawRev > TRIGGER_DEADBAND) {
    throttle = -map(rawRev, 0, 1023, 0, MAX_SPEED);
  }

  // B button = emergency stop
  if (btnB) throttle = 0;

  // ── Steering — left stick X ───────────────────────────────
  int rawSteer   = myController->axisX();
  int servoAngle = (abs(rawSteer) < DEADBAND)
    ? SERVO_CENTER
    : map(rawSteer, -AXIS_MAX, AXIS_MAX, SERVO_LEFT_MAX, SERVO_RIGHT_MAX);
  servoAngle = constrain(servoAngle, SERVO_LEFT_MAX, SERVO_RIGHT_MAX);

  // ── Drive ─────────────────────────────────────────────────
  driveAll(throttle);
  steeringServo.write(servoAngle);

  // ── Brake light ───────────────────────────────────────────
  digitalWrite(PIN_LED_BRAKE, (throttle < 0 || btnB) ? HIGH : LOW);

  // ── Turn signals & hazards (debounced toggles) ────────────
  static bool lastBumperL = false;
  static bool lastBumperR = false;
  static bool lastSelect  = false;

  if (bumperL && !lastBumperL) {
    leftSignalOn  = !leftSignalOn;
    hazardOn      = false;
  }
  if (bumperR && !lastBumperR) {
    rightSignalOn = !rightSignalOn;
    hazardOn      = false;
  }
  if (btnSelect && !lastSelect) {
    hazardOn      = !hazardOn;
    leftSignalOn  = false;
    rightSignalOn = false;
  }
  lastBumperL = bumperL;
  lastBumperR = bumperR;
  lastSelect  = btnSelect;

  updateBlinkers(leftSignalOn, rightSignalOn, hazardOn);

  // ── Horn ──────────────────────────────────────────────────
  if (btnStart) {
    tone(PIN_BUZZER, 2637); delay(300);
    noTone(PIN_BUZZER); delay(40);
    tone(PIN_BUZZER, 2093); delay(300);
    noTone(PIN_BUZZER);
  }

  // ── Serial debug every 200ms ──────────────────────────────
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    Serial.printf(
      "Fwd:%4d  Rev:%4d  Thr:%4d  Servo:%3d | L:%d R:%d Haz:%d Brake:%d Boost:%d\n",
      rawFwd, rawRev, throttle, servoAngle,
      leftSignalOn, rightSignalOn, hazardOn,
      (throttle < 0 || btnB), btnA
    );
  }
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("RC Car v3 booting...");

  // ── Motor pins ──────────────────────────────────────────
  uint8_t motorPins[] = {
    F_AIN1, F_AIN2, F_PWMA,
    F_BIN1, F_BIN2, F_PWMB,
    R_AIN1, R_AIN2, R_PWMA,
    R_BIN1, R_BIN2, R_PWMB,
    PIN_STBY
  };
  for (uint8_t p : motorPins) pinMode(p, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  stopMotors();

  // ── Peripheral pins ─────────────────────────────────────
  pinMode(PIN_LED_LEFT,  OUTPUT);
  pinMode(PIN_LED_RIGHT, OUTPUT);
  pinMode(PIN_LED_BRAKE, OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);

  // ── LED + buzzer self-test ───────────────────────────────
  Serial.println("Self-test...");
  digitalWrite(PIN_LED_LEFT,  HIGH); delay(200);
  digitalWrite(PIN_LED_RIGHT, HIGH); delay(200);
  digitalWrite(PIN_LED_BRAKE, HIGH); delay(200);
  tone(PIN_BUZZER, 1000); delay(150); noTone(PIN_BUZZER);
  digitalWrite(PIN_LED_LEFT,  LOW);
  digitalWrite(PIN_LED_RIGHT, LOW);
  digitalWrite(PIN_LED_BRAKE, LOW);

  // ── Servo — MUST come before BP32.setup() ───────────────
  // Allocate timer 3 first — Bluepad32 uses timers 0-2
  ESP32PWM::allocateTimer(3);
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(PIN_SERVO, 500, 2400);
  steeringServo.write(SERVO_CENTER);
  delay(500);

  // ── Bluepad32 — LAST, after servo is claimed ────────────
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();  // comment out after first successful pair

  Serial.println("Ready — hold Start on 8BitDo 2C to pair.");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  if (BP32.update()) processGamepad();
  delay(10);
}