/*
 * Minesweeper Rover - SIMPLE ROS 2 differential-drive test firmware
 * Board: Arduino Mega 2560
 * Protocol: 115200 baud, comma-separated line from roverInterface:
 *   rp1.00,lp1.00,gp0.00,sp0.00,\n
 * Also accepts signed tokens such as rn0.30 and ln0.30.
 *
 * This is intentionally drive-only to isolate ROS -> serial -> motor control.
 * It controls the two sides (two motors wired per side) using encoder speed
 * feedback and PID_v1. Gripper, ultrasonic, proximity, metal detector, siren,
 * and camera are NOT controlled by this test firmware.
 */

#include <Arduino.h>
#include <PID_v1.h>
#include <util/atomic.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// -------------------- Drive motor pins (from the rover sketch) --------------------
constexpr uint8_t RIGHT_PWM_1 = 44;
constexpr uint8_t RIGHT_DIR_1 = 42;
constexpr uint8_t RIGHT_PWM_2 = 9;
constexpr uint8_t RIGHT_DIR_2 = 8;

constexpr uint8_t LEFT_PWM_1 = 46;
constexpr uint8_t LEFT_DIR_1 = 40;
constexpr uint8_t LEFT_PWM_2 = 11;
constexpr uint8_t LEFT_DIR_2 = 10;

// Encoders (one rising edge on phase A; phase B gives direction)
constexpr uint8_t RIGHT_ENC_A = 19;
constexpr uint8_t RIGHT_ENC_B = 18;
constexpr uint8_t LEFT_ENC_A  = 21;
constexpr uint8_t LEFT_ENC_B  = 20;

// Preserve the motor/encoder direction conventions in the supplied rover code.
constexpr bool RIGHT_MOTOR_INVERTED = false;
constexpr bool LEFT_MOTOR_INVERTED  = true;
constexpr bool RIGHT_ENCODER_INVERTED = false;
constexpr bool LEFT_ENCODER_INVERTED  = true;

// Supplied rover firmware assumes 2200 counted A-channel pulses per wheel revolution.
// Verify this against the actual encoder/gear ratio during odometry calibration.
constexpr double ENCODER_PPR = 2200.0;
constexpr double RAD_PER_PULSE = (2.0 * PI) / ENCODER_PPR;

constexpr uint32_t CONTROL_PERIOD_MS = 20;       // PID update: 50 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 10;     // ROS feedback: 100 Hz
constexpr uint32_t COMMAND_TIMEOUT_MS = 400;     // stop if ROS stream disappears
constexpr double MAX_WHEEL_RAD_S = 6.0;          // safety clamp
constexpr double MAX_ACCEL_RAD_S2 = 6.0;         // smoother starts/stops
constexpr double MAX_DECEL_RAD_S2 = 10.0;
constexpr uint8_t MIN_PWM = 22;                   // overcome driver/motor deadband

// PID tuning values from the supplied working example.
double rightInput = 0.0, rightOutput = 0.0, rightSetpoint = 0.0;
double leftInput  = 0.0, leftOutput  = 0.0, leftSetpoint  = 0.0;
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, 11.5, 7.5, 0.1, DIRECT);
PID leftPID (&leftInput,  &leftOutput,  &leftSetpoint,  12.8, 8.3, 0.1, DIRECT);

volatile int32_t rightDeltaPulses = 0;
volatile int32_t leftDeltaPulses  = 0;
volatile int32_t rightTotalPulses = 0;
volatile int32_t leftTotalPulses  = 0;

float rightTargetRadS = 0.0f;
float leftTargetRadS  = 0.0f;
float rightProfileRadS = 0.0f;
float leftProfileRadS  = 0.0f;
float rightMeasuredRadS = 0.0f;
float leftMeasuredRadS  = 0.0f;
uint32_t lastCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;

char tokenBuffer[48];
uint8_t tokenIndex = 0;

static float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float approach(float current, float target, float dt) {
  const float error = target - current;
  const float rate = (fabsf(target) > fabsf(current) && current * target >= 0.0f)
                       ? MAX_ACCEL_RAD_S2 : MAX_DECEL_RAD_S2;
  const float step = rate * dt;
  if (error > step) return current + step;
  if (error < -step) return current - step;
  return target;
}

static int32_t takeRightDelta() {
  int32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    value = rightDeltaPulses;
    rightDeltaPulses = 0;
  }
  return value;
}

static int32_t takeLeftDelta() {
  int32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    value = leftDeltaPulses;
    leftDeltaPulses = 0;
  }
  return value;
}

static int32_t readRightTotal() {
  int32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = rightTotalPulses; }
  return value;
}

static int32_t readLeftTotal() {
  int32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = leftTotalPulses; }
  return value;
}

void rightEncoderISR() {
  const int8_t direction = digitalRead(RIGHT_ENC_B) ? 1 : -1;
  rightDeltaPulses += direction;
  rightTotalPulses += direction;
}

void leftEncoderISR() {
  const int8_t direction = digitalRead(LEFT_ENC_B) ? 1 : -1;
  leftDeltaPulses += direction;
  leftTotalPulses += direction;
}

static void setOneMotor(uint8_t pwmPin, uint8_t dirPin, uint8_t pwm,
                        bool logicalForward, bool inverted) {
  const bool dirHigh = logicalForward ^ inverted;
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

static void applyDriveOutput() {
  uint8_t rightPWM = (uint8_t)clampFloat(rightOutput, 0.0, 255.0);
  uint8_t leftPWM  = (uint8_t)clampFloat(leftOutput, 0.0, 255.0);

  if (fabsf(rightProfileRadS) < 0.03f) rightPWM = 0;
  else if (rightPWM < MIN_PWM) rightPWM = MIN_PWM;
  if (fabsf(leftProfileRadS) < 0.03f) leftPWM = 0;
  else if (leftPWM < MIN_PWM) leftPWM = MIN_PWM;

  const bool rightForward = rightProfileRadS >= 0.0f;
  const bool leftForward  = leftProfileRadS >= 0.0f;
  setOneMotor(RIGHT_PWM_1, RIGHT_DIR_1, rightPWM, rightForward, RIGHT_MOTOR_INVERTED);
  setOneMotor(RIGHT_PWM_2, RIGHT_DIR_2, rightPWM, rightForward, RIGHT_MOTOR_INVERTED);
  setOneMotor(LEFT_PWM_1, LEFT_DIR_1, leftPWM, leftForward, LEFT_MOTOR_INVERTED);
  setOneMotor(LEFT_PWM_2, LEFT_DIR_2, leftPWM, leftForward, LEFT_MOTOR_INVERTED);
}

static void setTarget(char wheel, char sign, double magnitude) {
  if (!isfinite(magnitude) || magnitude < 0.0) return;
  magnitude = clampFloat((float)magnitude, 0.0f, (float)MAX_WHEEL_RAD_S);
  const float signedValue = (sign == 'n') ? -(float)magnitude : (float)magnitude;
  if (wheel == 'r') rightTargetRadS = signedValue;
  else if (wheel == 'l') leftTargetRadS = signedValue;
  lastCommandMs = millis();
}

static void processToken(char *token) {
  if (!token || !token[0]) return;
  for (char *p = token; *p; ++p) *p = (char)tolower((unsigned char)*p);
  const size_t len = strlen(token);
  if (len < 3) return;

  // Wheel tokens accepted from roverInterface: rp1.20 / rn1.20 / lp1.20 / ln1.20
  if ((token[0] == 'r' || token[0] == 'l') &&
      (token[1] == 'p' || token[1] == 'n')) {
    char *end = nullptr;
    const double value = strtod(token + 2, &end);
    if (end != token + 2 && *end == '\0') setTarget(token[0], token[1], value);
    return;
  }

  // gp... and sp... are intentionally ignored in this DRIVE-ONLY diagnostic sketch.
}

static void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == ',' || c == '\n' || c == '\r') {
      if (tokenIndex > 0) {
        tokenBuffer[tokenIndex] = '\0';
        processToken(tokenBuffer);
        tokenIndex = 0;
      }
      continue;
    }
    if (tokenIndex < sizeof(tokenBuffer) - 1) tokenBuffer[tokenIndex++] = c;
    else tokenIndex = 0;  // discard an overlong/malformed token
  }
}

static void updateControl() {
  const uint32_t now = millis();
  float dt = (now - lastControlMs) / 1000.0f;
  if (dt <= 0.0f || dt > 0.2f) dt = CONTROL_PERIOD_MS / 1000.0f;
  lastControlMs = now;

  if (now - lastCommandMs > COMMAND_TIMEOUT_MS) {
    rightTargetRadS = 0.0f;
    leftTargetRadS = 0.0f;
  }

  int32_t rCounts = takeRightDelta();
  int32_t lCounts = takeLeftDelta();
  rightMeasuredRadS = ((float)rCounts * (float)RAD_PER_PULSE) / dt;
  leftMeasuredRadS  = ((float)lCounts * (float)RAD_PER_PULSE) / dt;
  if (RIGHT_ENCODER_INVERTED) rightMeasuredRadS = -rightMeasuredRadS;
  if (LEFT_ENCODER_INVERTED)  leftMeasuredRadS  = -leftMeasuredRadS;

  rightProfileRadS = approach(rightProfileRadS, rightTargetRadS, dt);
  leftProfileRadS  = approach(leftProfileRadS, leftTargetRadS, dt);

  // PID regulates speed magnitude; the target sign independently sets motor direction.
  rightInput = fabs(rightMeasuredRadS);
  leftInput  = fabs(leftMeasuredRadS);
  rightSetpoint = fabs(rightProfileRadS);
  leftSetpoint  = fabs(leftProfileRadS);

  // Keep PID state updating even at zero setpoint so the integral term unwinds.
  rightPID.Compute();
  leftPID.Compute();
  if (fabs(rightSetpoint) < 0.03) rightOutput = 0.0;
  if (fabs(leftSetpoint) < 0.03) leftOutput = 0.0;

  applyDriveOutput();
}

static void printSignedToken(char prefix, float velocity) {
  Serial.print(prefix);
  Serial.print(velocity < 0.0f ? 'n' : 'p');
  Serial.print(fabsf(velocity), 2);
}

static void sendTelemetry() {
  // Exact wheel token names consumed by roverInterface::read().
  printSignedToken('r', rightMeasuredRadS);
  Serial.print(',');
  printSignedToken('l', leftMeasuredRadS);
  Serial.print(",p0,m0,er:");
  Serial.print(readRightTotal());
  Serial.print(",el:");
  Serial.print(readLeftTotal());
  Serial.println(',');
}

void setup() {
  Serial.begin(115200);

  pinMode(RIGHT_PWM_1, OUTPUT); pinMode(RIGHT_DIR_1, OUTPUT);
  pinMode(RIGHT_PWM_2, OUTPUT); pinMode(RIGHT_DIR_2, OUTPUT);
  pinMode(LEFT_PWM_1, OUTPUT);  pinMode(LEFT_DIR_1, OUTPUT);
  pinMode(LEFT_PWM_2, OUTPUT);  pinMode(LEFT_DIR_2, OUTPUT);
  analogWrite(RIGHT_PWM_1, 0); analogWrite(RIGHT_PWM_2, 0);
  analogWrite(LEFT_PWM_1, 0);  analogWrite(LEFT_PWM_2, 0);

  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  pinMode(LEFT_ENC_A, INPUT_PULLUP);  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);

  rightPID.SetMode(AUTOMATIC);
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(CONTROL_PERIOD_MS);
  leftPID.SetSampleTime(CONTROL_PERIOD_MS);
  rightPID.SetOutputLimits(0.0, 255.0);
  leftPID.SetOutputLimits(0.0, 255.0);

  lastCommandMs = millis();
  lastControlMs = millis();
  lastTelemetryMs = millis();
}

void loop() {
  readSerialCommands();
  const uint32_t now = millis();

  if (now - lastControlMs >= CONTROL_PERIOD_MS) updateControl();
  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
