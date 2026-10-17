/*
 * Minesweeper Rover - ROS 2 compatible Arduino Mega 2560 firmware (METAL ONLY)
 *
 * Built by combining the known-good single-file motor/encoder test sketch
 * with the sensor/siren pin information from the FreeRTOS reference firmware.
 *
 * ROS2 roverInterface RX (115200 baud):
 *   rp1.00,lp1.00,gp0.00,sp0.00,

 *   rn0.50,ln0.50,gp0.00,sp0.00,

 *   CSERVO:90

 *
 * Telemetry TX (one line, repeatedly):
 *   rp1.82,lp1.79,p0,m0,go,er:1234,el:-1234,
 *   p0 is fixed for protocol compatibility; this firmware never reads proximity.
 *   The roverInterface consumes rp/rn, lp/ln, p0/p1, m0/m1 and go/gc/gm.
 *   Encoder totals are included for diagnostics.
 *
 * IMPORTANT WIRING:
 *   Drive/encoder/camera pins are preserved from the working base sketch.
 *   Proximity sensors are intentionally NOT read. Metal D27, siren D47,
 *   magnet D38, lift D7/D49, top limit D51, bottom limit D53, ultrasonic D43/D45.
 *   Limit switches must have the required pull-downs and read HIGH when active.
 *   A separate buzzer output is not used.
 *
 * Logic: metal-detector rising edge -> siren for 2 s and 10 s auto window;
 * ultrasonic near confirmation (<20 cm for 3 confirmed rolling samples) requests lift.
 * Non-zero G rising edge or cycle:1 requests a manual lift without ultrasonic gating.
 * Magnet stays ON at the lower position, turns OFF for 5 s at the top, then ON again
 * to power limit switches before the lift descends. Proximity is never read.
 * Safety: test with drive wheels lifted off the floor and a reachable battery
 * disconnect. Do not power a siren/buzzer directly from an Arduino GPIO; use
 * the required transistor/relay/driver and a suitable external supply.
 */

#include <Arduino.h>
#include <util/atomic.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// -------------------- Pins: from the GitHub firmware --------------------
constexpr uint8_t MOTOR_R_PWM   = 44;
constexpr uint8_t MOTOR_R_DIR   = 42;
constexpr uint8_t MOTOR_L_PWM   = 46;
constexpr uint8_t MOTOR_L_DIR   = 40;
constexpr uint8_t MOTOR_R_PWM_2 = 9;
constexpr uint8_t MOTOR_R_DIR_2 = 8;
constexpr uint8_t MOTOR_L_PWM_2 = 11;
constexpr uint8_t MOTOR_L_DIR_2 = 10;

constexpr uint8_t ENCODER_R_A = 19;
constexpr uint8_t ENCODER_R_B = 18;
constexpr uint8_t ENCODER_L_A = 21;
constexpr uint8_t ENCODER_L_B = 20;

constexpr uint8_t CAMERA_SERVO_PIN = 4;

// Same motor/encoder inversion used by the referenced firmware.
constexpr bool RIGHT_MOTOR_INVERTED = false;
constexpr bool LEFT_MOTOR_INVERTED  = true;
constexpr bool RIGHT_ENCODER_INVERTED = false;
constexpr bool LEFT_ENCODER_INVERTED  = true;

// -------------------- Control --------------------
constexpr float ENCODER_PPR = 2200.0f;
constexpr float RAD_PER_PULSE = (2.0f * PI) / ENCODER_PPR;
constexpr uint32_t CONTROL_PERIOD_MS = 20;       // 50 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 10;     // 100 Hz, match ros2_control read loop
constexpr uint32_t SENSOR_PERIOD_MS = 20;        // 50 Hz
constexpr uint32_t COMMAND_TIMEOUT_MS = 400;
constexpr float MAX_WHEEL_VELOCITY = 6.0f;

constexpr float MAX_ACCELERATION = 30.0f;
constexpr float MAX_DECELERATION = 20.0f;

// Starting PID values from the referenced firmware.
constexpr float KP_RIGHT = 11.5f;
constexpr float KI_RIGHT = 7.5f;
constexpr float KD_RIGHT = 0.1f;
constexpr float FF_RIGHT = 0.5f;
constexpr float KP_LEFT  = 12.8f;
constexpr float KI_LEFT  = 8.3f;
constexpr float KD_LEFT  = 0.1f;
constexpr float FF_LEFT  = 0.5f;

constexpr float PID_OUTPUT_MIN = -255.0f;
constexpr float PID_OUTPUT_MAX = 255.0f;
constexpr float INTEGRAL_LIMIT = 200.0f;
constexpr float MIN_PWM_DEADBAND = 20.0f;
constexpr float PWM_SLEW_RATE = 1000.0f;

// -------------------- Sensors / siren / lift-gripper --------------------
// PROXIMITY INPUTS ARE NOT CONFIGURED OR READ IN THIS FIRMWARE.
constexpr uint8_t METAL_DETECTOR_PIN = 27;
constexpr uint8_t METAL_ACTIVE_STATE = HIGH;
constexpr uint32_t METAL_DEBOUNCE_MS = 50;
constexpr uint32_t METAL_TRIGGER_WINDOW_MS = 10000;
constexpr uint32_t SIREN_ON_MS = 2000;

constexpr uint8_t SIREN_PIN = 47;
constexpr bool SIREN_ACTIVE_HIGH = true;
constexpr uint8_t MAGNET_PIN = 38;
constexpr bool MAGNET_ALWAYS_ON_WHEN_DOWN = true;

constexpr uint8_t LIFT_PWM = 7;
constexpr uint8_t LIFT_DIR = 49;
constexpr uint8_t LIMIT_SW_TOP = 51;
constexpr uint8_t LIMIT_SW_BOTTOM = 53;
// This pin map follows the working base sketch: HIGH means a limit is active.
constexpr uint8_t LIMIT_SWITCH_ACTIVE_STATE = HIGH;
constexpr bool TOP_LIMIT_MEANS_CLOSED = true;
constexpr bool LIFT_CLOSE_DIR_HIGH = true;   // HIGH moves the lift UP in this wiring.
constexpr uint8_t LIFT_PWM_PERCENT = 35;     // Slow lift to avoid damaging the gripper.
constexpr uint32_t LIFT_TIMEOUT_MS = 10000;
constexpr uint32_t MAGNET_SETTLE_MS = 200;
constexpr uint32_t RELEASE_DWELL_MS = 5000;

// Ultrasonic sensor mounted on the gripper. A near reading confirms a mine is
// being held after a metal-detector edge. Missing echoes are NOT treated as a mine.
constexpr uint8_t US_TRIG_PIN = 43;
constexpr uint8_t US_ECHO_PIN = 45;
constexpr uint8_t US_AVG_N = 5;
constexpr uint32_t US_SAMPLE_INTERVAL_MS = 60;
constexpr uint32_t US_ECHO_TIMEOUT_US = 3500;
constexpr float US_MAX_DISTANCE_CM = 25.0f;
constexpr float US_MINE_NEAR_CM = 20.0f;
constexpr uint8_t US_CONFIRM_N = 3;

// -------------------- Camera --------------------
constexpr int CAMERA_MIN_ANGLE = 10;
constexpr int CAMERA_MAX_ANGLE = 170;
constexpr int CAMERA_CENTER_ANGLE = 90;
constexpr int CAMERA_STEP_DEG = 2;
constexpr uint32_t CAMERA_COMMAND_TIMEOUT_MS = 300;
constexpr uint32_t SERVO_FRAME_US = 20000;
constexpr uint32_t SERVO_MIN_PULSE_US = 1000;
constexpr uint32_t SERVO_MAX_PULSE_US = 2000;

struct WheelState {
  float target_velocity = 0.0f;
  float profiled_velocity = 0.0f;
  float measured_velocity = 0.0f;
  float integral_term = 0.0f;
  float last_error = 0.0f;
  float last_d_term = 0.0f;
  float target_pwm = 0.0f;
  float current_pwm = 0.0f;
};

WheelState right_wheel, left_wheel;

volatile int32_t right_pulse_count = 0;  // delta pulses used for velocity
volatile int32_t left_pulse_count = 0;
volatile int32_t right_total_pulses = 0;  // cumulative encoder counts for diagnostics
volatile int32_t left_total_pulses = 0;

uint32_t last_control_ms = 0;
uint32_t last_telemetry_ms = 0;
uint32_t last_sensor_ms = 0;
uint32_t last_command_ms = 0;

bool metal_detected = false;
bool metal_last_raw = false;
uint32_t metal_last_change_ms = 0;
uint32_t metal_trigger_until_ms = 0;
uint32_t siren_until_ms = 0;
bool siren_state = false;
bool magnet_state = false;

char gripper_status = 'm';  // 'o' open/down, 'c' closed/up, 'm' moving/fault
bool gripper_is_moving = false;
bool g_command_was_active = false;
bool manual_grip_request = false;
uint32_t grip_state_started_ms = 0;
uint32_t last_us_sample_ms = 0;
uint32_t us_sample_seq = 0;
uint32_t us_last_valid_ms = 0;
bool us_latest_valid = false;
float us_last_distance_cm = 0.0f;
float us_filtered_distance_cm = 0.0f;
float us_readings[US_AVG_N] = {0};
uint8_t us_reading_count = 0;
uint8_t us_reading_next = 0;
uint8_t us_near_streak = 0;

// All lift states run non-blocking, so serial parsing and wheel PID stay alive.
enum GripperState : uint8_t {
  GRIP_BOOT = 0,
  GRIP_DOWN_READY,
  GRIP_MAGNET_SETTLE,
  GRIP_MOVING_UP,
  GRIP_TOP_RELEASE,
  GRIP_MAGNET_POWER_WAIT,
  GRIP_MOVING_DOWN,
  GRIP_RECOVERY_POWER_WAIT,
  GRIP_RECOVERY_DOWN,
  GRIP_FAULT
};
GripperState grip_state = GRIP_BOOT;
char grip_fault_reason[24] = "NONE";
char cycle_outcome[24] = "NONE";

int camera_angle = CAMERA_CENTER_ANGLE;
int camera_direction = 0;
uint32_t camera_last_command_ms = 0;

bool servo_pulse_high = false;
uint32_t servo_frame_start_us = 0;
uint32_t servo_pulse_width_us = 1500;
uint32_t last_camera_step_ms = 0;

// -------------------- Helpers --------------------
float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// -------------------- Motors --------------------
void setMotorOutput(uint8_t pwm_pin, uint8_t dir_pin, float pwm, bool inverted) {
  pwm = clampFloat(pwm, -255.0f, 255.0f);
  bool forward = (pwm >= 0.0f);
  if (inverted) forward = !forward;
  digitalWrite(dir_pin, forward ? HIGH : LOW);
  analogWrite(pwm_pin, static_cast<uint8_t>(fabs(pwm)));
}

void stopAllMotors() {
  analogWrite(MOTOR_R_PWM, 0);
  analogWrite(MOTOR_L_PWM, 0);
  analogWrite(MOTOR_R_PWM_2, 0);
  analogWrite(MOTOR_L_PWM_2, 0);
  right_wheel.current_pwm = 0.0f;
  left_wheel.current_pwm = 0.0f;
}

void applyDrivePWM() {
  setMotorOutput(MOTOR_R_PWM,   MOTOR_R_DIR,   right_wheel.current_pwm, RIGHT_MOTOR_INVERTED);
  setMotorOutput(MOTOR_R_PWM_2, MOTOR_R_DIR_2, right_wheel.current_pwm, RIGHT_MOTOR_INVERTED);
  setMotorOutput(MOTOR_L_PWM,   MOTOR_L_DIR,   left_wheel.current_pwm, LEFT_MOTOR_INVERTED);
  setMotorOutput(MOTOR_L_PWM_2, MOTOR_L_DIR_2, left_wheel.current_pwm, LEFT_MOTOR_INVERTED);
}

// -------------------- Encoders --------------------
void rightEncoderISR() {
  const int8_t delta = digitalRead(ENCODER_R_B) ? 1 : -1;
  right_pulse_count += delta;
  right_total_pulses += delta;
}

void leftEncoderISR() {
  const int8_t delta = digitalRead(ENCODER_L_B) ? 1 : -1;
  left_pulse_count += delta;
  left_total_pulses += delta;
}

int32_t takeRightPulses() {
  int32_t p;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { p = right_pulse_count; right_pulse_count = 0; }
  return p;
}

int32_t takeLeftPulses() {
  int32_t p;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { p = left_pulse_count; left_pulse_count = 0; }
  return p;
}

float calculateWheelVelocity(int32_t delta_pulses, float dt, bool inverted) {
  if (dt <= 0.0f) return 0.0f;
  float v = (static_cast<float>(delta_pulses) * RAD_PER_PULSE) / dt;
  return inverted ? -v : v;
}

// -------------------- Motion + PID --------------------
float applyVelocityRamp(float target, float current, float dt) {
  float error = target - current;
  float accel_step = MAX_ACCELERATION * dt;
  float decel_step = MAX_DECELERATION * dt;
  float max_step = ((error > 0.0f && target > current) || (error < 0.0f && target < current))
                     ? accel_step : decel_step;
  if (error > max_step) return current + max_step;
  if (error < -max_step) return current - max_step;
  return target;
}

float computePID(WheelState &w, float kp, float ki, float kd, float ff, float dt) {
  const float error = w.profiled_velocity - w.measured_velocity;
  const float p_term = kp * error;

  w.integral_term += ki * error * dt;
  w.integral_term = clampFloat(w.integral_term, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float d_term = 0.0f;
  if (dt > 0.0f) {
    const float raw_d = kd * (error - w.last_error) / dt;
    constexpr float alpha = 0.1f;
    d_term = alpha * raw_d + (1.0f - alpha) * w.last_d_term;
  }
  w.last_error = error;
  w.last_d_term = d_term;

  const float ff_term = ff * w.profiled_velocity;
  float output = p_term + w.integral_term + d_term + ff_term;

  if (output > PID_OUTPUT_MAX) {
    output = PID_OUTPUT_MAX;
    w.integral_term = output - p_term - d_term - ff_term;
  } else if (output < PID_OUTPUT_MIN) {
    output = PID_OUTPUT_MIN;
    w.integral_term = output - p_term - d_term - ff_term;
  }
  return output;
}

float applyPWMRamp(float target_pwm, float current_pwm, float dt) {
  const float max_step = PWM_SLEW_RATE * dt;
  const float error = target_pwm - current_pwm;
  if (error > max_step) return current_pwm + max_step;
  if (error < -max_step) return current_pwm - max_step;
  return target_pwm;
}

void updateControl() {
  const float dt = CONTROL_PERIOD_MS / 1000.0f;

  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    right_wheel.target_velocity = 0.0f;
    left_wheel.target_velocity = 0.0f;
  }

  right_wheel.measured_velocity = calculateWheelVelocity(takeRightPulses(), dt, RIGHT_ENCODER_INVERTED);
  left_wheel.measured_velocity = calculateWheelVelocity(takeLeftPulses(), dt, LEFT_ENCODER_INVERTED);

  right_wheel.profiled_velocity = applyVelocityRamp(right_wheel.target_velocity, right_wheel.profiled_velocity, dt);
  left_wheel.profiled_velocity = applyVelocityRamp(left_wheel.target_velocity, left_wheel.profiled_velocity, dt);

  right_wheel.target_pwm = computePID(right_wheel, KP_RIGHT, KI_RIGHT, KD_RIGHT, FF_RIGHT, dt);
  left_wheel.target_pwm = computePID(left_wheel, KP_LEFT, KI_LEFT, KD_LEFT, FF_LEFT, dt);

  if (fabs(right_wheel.profiled_velocity) < 0.01f) right_wheel.target_pwm = 0.0f;
  else if (fabs(right_wheel.target_pwm) > 0.0f && fabs(right_wheel.target_pwm) < MIN_PWM_DEADBAND)
    right_wheel.target_pwm = (right_wheel.target_pwm >= 0.0f) ? MIN_PWM_DEADBAND : -MIN_PWM_DEADBAND;

  if (fabs(left_wheel.profiled_velocity) < 0.01f) left_wheel.target_pwm = 0.0f;
  else if (fabs(left_wheel.target_pwm) > 0.0f && fabs(left_wheel.target_pwm) < MIN_PWM_DEADBAND)
    left_wheel.target_pwm = (left_wheel.target_pwm >= 0.0f) ? MIN_PWM_DEADBAND : -MIN_PWM_DEADBAND;

  right_wheel.current_pwm = applyPWMRamp(right_wheel.target_pwm, right_wheel.current_pwm, dt);
  left_wheel.current_pwm = applyPWMRamp(left_wheel.target_pwm, left_wheel.current_pwm, dt);

  applyDrivePWM();
}

// -------------------- Metal-only sensing and siren --------------------
void setMagnet(bool enabled) {
  magnet_state = enabled;
  digitalWrite(MAGNET_PIN, enabled ? HIGH : LOW);
}

bool topLimitActive() {
  return digitalRead(LIMIT_SW_TOP) == LIMIT_SWITCH_ACTIVE_STATE;
}

bool bottomLimitActive() {
  return digitalRead(LIMIT_SW_BOTTOM) == LIMIT_SWITCH_ACTIVE_STATE;
}

char statusForTopLimit() {
  return TOP_LIMIT_MEANS_CLOSED ? 'c' : 'o';
}

char statusForBottomLimit() {
  return TOP_LIMIT_MEANS_CLOSED ? 'o' : 'c';
}

void setGripFault(const char *reason) {
  analogWrite(LIFT_PWM, 0);
  gripper_is_moving = false;
  grip_state = GRIP_FAULT;
  gripper_status = 'm';
  strncpy(grip_fault_reason, reason, sizeof(grip_fault_reason) - 1);
  grip_fault_reason[sizeof(grip_fault_reason) - 1] = '\0';
  strncpy(cycle_outcome, reason, sizeof(cycle_outcome) - 1);
  cycle_outcome[sizeof(cycle_outcome) - 1] = '\0';
}

void beginGripCycle(const char *source) {
  if (grip_state != GRIP_DOWN_READY || !bottomLimitActive() || topLimitActive()) {
    strncpy(cycle_outcome, "REQUEST_REJECTED", sizeof(cycle_outcome) - 1);
    cycle_outcome[sizeof(cycle_outcome) - 1] = '\0';
    return;
  }
  (void)source;
  manual_grip_request = false;
  setMagnet(true);
  grip_state = GRIP_MAGNET_SETTLE;
  grip_state_started_ms = millis();
  strncpy(cycle_outcome, "IN_PROGRESS", sizeof(cycle_outcome) - 1);
  cycle_outcome[sizeof(cycle_outcome) - 1] = '\0';
}

void startRecoveryDown(const char *reason) {
  analogWrite(LIFT_PWM, 0);
  gripper_is_moving = false;
  setMagnet(true);  // The reference wiring powers the limit switches from the magnet supply.
  grip_state = GRIP_RECOVERY_POWER_WAIT;
  grip_state_started_ms = millis();
  strncpy(grip_fault_reason, reason, sizeof(grip_fault_reason) - 1);
  grip_fault_reason[sizeof(grip_fault_reason) - 1] = '\0';
  strncpy(cycle_outcome, "RECOVERY_DOWN", sizeof(cycle_outcome) - 1);
  cycle_outcome[sizeof(cycle_outcome) - 1] = '\0';
}

void updateSensors() {
  // Deliberately read ONLY the metal detector. No proximity pinMode/digitalRead,
  // no proximity-based siren, magnet, gripper, or serial event is used.
  const bool raw_metal = (digitalRead(METAL_DETECTOR_PIN) == METAL_ACTIVE_STATE);
  if (raw_metal != metal_last_raw) {
    metal_last_raw = raw_metal;
    metal_last_change_ms = millis();
  }

  if (millis() - metal_last_change_ms >= METAL_DEBOUNCE_MS) {
    const bool old_state = metal_detected;
    metal_detected = metal_last_raw;
    if (metal_detected && !old_state) {
      const uint32_t now = millis();
      metal_trigger_until_ms = now + METAL_TRIGGER_WINDOW_MS;
      siren_until_ms = now + SIREN_ON_MS;
      // Magnet is already ON while down; explicitly ensure it remains on after detection.
      if (grip_state == GRIP_DOWN_READY || grip_state == GRIP_BOOT) setMagnet(true);
    }
  }
}

void updateAlarmOutputs() {
  const bool active = static_cast<int32_t>(siren_until_ms - millis()) > 0;
  siren_state = active;
  const bool level = SIREN_ACTIVE_HIGH ? active : !active;
  digitalWrite(SIREN_PIN, level ? HIGH : LOW);
}

// -------------------- Non-blocking ultrasonic sampler --------------------
enum UltrasonicStage : uint8_t { US_IDLE = 0, US_TRIGGER_HIGH, US_WAIT_RISE, US_WAIT_FALL };
UltrasonicStage us_stage = US_IDLE;
uint32_t us_stage_started_us = 0;
uint32_t us_echo_rise_us = 0;

void recordUltrasonicSample(bool valid, float distance_cm) {
  ++us_sample_seq;
  us_latest_valid = valid;
  if (!valid) {
    us_near_streak = 0;
    return;
  }

  us_last_valid_ms = millis();
  if (distance_cm > US_MAX_DISTANCE_CM) distance_cm = US_MAX_DISTANCE_CM;
  if (distance_cm < 2.0f) {
    us_near_streak = 0;
    return;
  }
  us_last_distance_cm = distance_cm;
  us_readings[us_reading_next] = distance_cm;
  us_reading_next = (us_reading_next + 1) % US_AVG_N;
  if (us_reading_count < US_AVG_N) ++us_reading_count;

  float total = 0.0f;
  for (uint8_t i = 0; i < us_reading_count; ++i) total += us_readings[i];
  us_filtered_distance_cm = total / us_reading_count;

  if (us_reading_count >= US_AVG_N && us_filtered_distance_cm < US_MINE_NEAR_CM) {
    if (us_near_streak < 255) ++us_near_streak;
  } else {
    us_near_streak = 0;
  }
}

void updateUltrasonic() {
  const uint32_t now_us = micros();
  const uint32_t now_ms = millis();
  switch (us_stage) {
    case US_IDLE:
      if (now_ms - last_us_sample_ms >= US_SAMPLE_INTERVAL_MS) {
        last_us_sample_ms = now_ms;
        digitalWrite(US_TRIG_PIN, HIGH);
        us_stage_started_us = now_us;
        us_stage = US_TRIGGER_HIGH;
      }
      break;
    case US_TRIGGER_HIGH:
      if (static_cast<uint32_t>(now_us - us_stage_started_us) >= 10UL) {
        digitalWrite(US_TRIG_PIN, LOW);
        us_stage_started_us = now_us;
        us_stage = US_WAIT_RISE;
      }
      break;
    case US_WAIT_RISE:
      if (digitalRead(US_ECHO_PIN) == HIGH) {
        us_echo_rise_us = now_us;
        us_stage = US_WAIT_FALL;
      } else if (static_cast<uint32_t>(now_us - us_stage_started_us) >= US_ECHO_TIMEOUT_US) {
        recordUltrasonicSample(false, 0.0f);
        us_stage = US_IDLE;
      }
      break;
    case US_WAIT_FALL:
      if (digitalRead(US_ECHO_PIN) == LOW) {
        const uint32_t width = static_cast<uint32_t>(now_us - us_echo_rise_us);
        if (width == 0 || width > US_ECHO_TIMEOUT_US) {
          recordUltrasonicSample(false, 0.0f);
        } else {
          const float distance_cm = (static_cast<float>(width) * 0.0343f) / 2.0f;
          recordUltrasonicSample(true, distance_cm);
        }
        us_stage = US_IDLE;
      } else if (static_cast<uint32_t>(now_us - us_echo_rise_us) >= US_ECHO_TIMEOUT_US) {
        recordUltrasonicSample(false, 0.0f);
        us_stage = US_IDLE;
      }
      break;
  }
}

void initializeGripperState() {
  // The reference firmware's limit switches need the magnet supply powered.
  setMagnet(true);
  grip_state_started_ms = millis();
  grip_state = GRIP_BOOT;
  analogWrite(LIFT_PWM, 0);
  gripper_is_moving = false;
}

void updateGripperMotor() {
  const uint32_t now = millis();
  const bool top = topLimitActive();
  const bool bottom = bottomLimitActive();

  if (top && bottom) {
    setMagnet(false);
    setGripFault("BOTH_LIMITS_ACTIVE");
    return;
  }

  switch (grip_state) {
    case GRIP_BOOT:
      analogWrite(LIFT_PWM, 0);
      setMagnet(true);
      if (now - grip_state_started_ms >= MAGNET_SETTLE_MS) {
        if (bottom && !top) {
          grip_state = GRIP_DOWN_READY;
          gripper_status = statusForBottomLimit();
          strncpy(grip_fault_reason, "NONE", sizeof(grip_fault_reason) - 1);
          strncpy(cycle_outcome, "READY", sizeof(cycle_outcome) - 1);
        } else {
          setGripFault(top ? "BOOT_AT_TOP" : "BOOT_NOT_DOWN");
        }
      }
      return;

    case GRIP_DOWN_READY:
      analogWrite(LIFT_PWM, 0);
      gripper_is_moving = false;
      gripper_status = statusForBottomLimit();
      if (MAGNET_ALWAYS_ON_WHEN_DOWN) setMagnet(true);

      // A rising G command is the manual override and requests a lift directly.
      if (manual_grip_request) {
        beginGripCycle("manual");
        return;
      }
      // Automatic pickup: metal detector edge arms a 10 s window, and the
      // ultrasonic must confirm a nearby object for several fresh readings.
      if (static_cast<int32_t>(metal_trigger_until_ms - now) > 0 &&
          us_latest_valid && (now - us_last_valid_ms) <= 250UL &&
          us_near_streak >= US_CONFIRM_N) {
        beginGripCycle("metal+ultrasonic");
      }
      return;

    case GRIP_MAGNET_SETTLE:
      analogWrite(LIFT_PWM, 0);
      setMagnet(true);
      if (!bottom || top) {
        setGripFault("BOTTOM_LOST_BEFORE_UP");
        return;
      }
      if (now - grip_state_started_ms >= MAGNET_SETTLE_MS) {
        grip_state = GRIP_MOVING_UP;
        grip_state_started_ms = now;
      }
      return;

    case GRIP_MOVING_UP:
      if (top) {
        analogWrite(LIFT_PWM, 0);
        gripper_is_moving = false;
        gripper_status = statusForTopLimit();
        setMagnet(false); // release the mine into the box
        grip_state = GRIP_TOP_RELEASE;
        grip_state_started_ms = now;
        strncpy(cycle_outcome, "RELEASING", sizeof(cycle_outcome) - 1);
        return;
      }
      if (now - grip_state_started_ms >= LIFT_TIMEOUT_MS) {
        startRecoveryDown("UP_TIMEOUT_RECOVERY");
        return;
      }
      digitalWrite(LIFT_DIR, LIFT_CLOSE_DIR_HIGH ? HIGH : LOW);
      analogWrite(LIFT_PWM, static_cast<uint8_t>((255UL * LIFT_PWM_PERCENT) / 100UL));
      gripper_is_moving = true;
      gripper_status = 'm';
      return;

    case GRIP_TOP_RELEASE:
      analogWrite(LIFT_PWM, 0);
      gripper_is_moving = false;
      setMagnet(false);
      // Let the mine fall before re-powering the limit switches and descending.
      if (now - grip_state_started_ms >= RELEASE_DWELL_MS) {
        setMagnet(true);
        grip_state = GRIP_MAGNET_POWER_WAIT;
        grip_state_started_ms = now;
      }
      return;

    case GRIP_MAGNET_POWER_WAIT:
      analogWrite(LIFT_PWM, 0);
      setMagnet(true);
      if (now - grip_state_started_ms >= MAGNET_SETTLE_MS) {
        if (top) {
          grip_state = GRIP_MOVING_DOWN;
          grip_state_started_ms = now;
        } else {
          startRecoveryDown("TOP_LIMIT_LOST_AFTER_RELEASE");
        }
      }
      return;

    case GRIP_MOVING_DOWN:
      setMagnet(true); // reference wiring powers the limit switches from this supply
      if (bottom) {
        analogWrite(LIFT_PWM, 0);
        gripper_is_moving = false;
        grip_state = GRIP_DOWN_READY;
        gripper_status = statusForBottomLimit();
        grip_state_started_ms = now;
        metal_trigger_until_ms = 0;
        us_near_streak = 0;
        strncpy(grip_fault_reason, "NONE", sizeof(grip_fault_reason) - 1);
        strncpy(cycle_outcome, "DONE", sizeof(cycle_outcome) - 1);
        return;
      }
      if (now - grip_state_started_ms >= LIFT_TIMEOUT_MS) {
        setGripFault("DOWN_TIMEOUT");
        return;
      }
      digitalWrite(LIFT_DIR, LIFT_CLOSE_DIR_HIGH ? LOW : HIGH);
      analogWrite(LIFT_PWM, static_cast<uint8_t>((255UL * LIFT_PWM_PERCENT) / 100UL));
      gripper_is_moving = true;
      gripper_status = 'm';
      return;

    case GRIP_RECOVERY_POWER_WAIT:
      setMagnet(true);
      analogWrite(LIFT_PWM, 0);
      if (bottom) {
        grip_state = GRIP_DOWN_READY;
        gripper_status = statusForBottomLimit();
        strncpy(grip_fault_reason, "NONE", sizeof(grip_fault_reason) - 1);
        strncpy(cycle_outcome, "RECOVERED_AT_BOTTOM", sizeof(cycle_outcome) - 1);
      } else if (now - grip_state_started_ms >= MAGNET_SETTLE_MS) {
        grip_state = GRIP_RECOVERY_DOWN;
        grip_state_started_ms = now;
      }
      return;

    case GRIP_RECOVERY_DOWN:
      setMagnet(true);
      if (bottom) {
        analogWrite(LIFT_PWM, 0);
        gripper_is_moving = false;
        grip_state = GRIP_DOWN_READY;
        gripper_status = statusForBottomLimit();
        strncpy(grip_fault_reason, "NONE", sizeof(grip_fault_reason) - 1);
        strncpy(cycle_outcome, "RECOVERED_AT_BOTTOM", sizeof(cycle_outcome) - 1);
        return;
      }
      if (now - grip_state_started_ms >= LIFT_TIMEOUT_MS) {
        setGripFault("RECOVERY_DOWN_TIMEOUT");
        return;
      }
      digitalWrite(LIFT_DIR, LIFT_CLOSE_DIR_HIGH ? LOW : HIGH);
      analogWrite(LIFT_PWM, static_cast<uint8_t>((255UL * LIFT_PWM_PERCENT) / 100UL));
      gripper_is_moving = true;
      gripper_status = 'm';
      return;

    case GRIP_FAULT:
    default:
      analogWrite(LIFT_PWM, 0);
      gripper_is_moving = false;
      gripper_status = 'm';
      return;
  }
}

int32_t getRightTotalPulses() {
  int32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = right_total_pulses; }
  return value;
}

int32_t getLeftTotalPulses() {
  int32_t value;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { value = left_total_pulses; }
  return value;
}

// -------------------- Camera servo, software PWM --------------------
uint32_t angleToPulse(int angle) {
  angle = clampInt(angle, CAMERA_MIN_ANGLE, CAMERA_MAX_ANGLE);
  return static_cast<uint32_t>(map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US));
}

void setCameraAngle(int angle) {
  camera_angle = clampInt(angle, CAMERA_MIN_ANGLE, CAMERA_MAX_ANGLE);
  servo_pulse_width_us = angleToPulse(camera_angle);
}

void updateCameraServo() {
  uint32_t now_us = micros();

  if (!servo_pulse_high && static_cast<uint32_t>(now_us - servo_frame_start_us) >= SERVO_FRAME_US) {
    digitalWrite(CAMERA_SERVO_PIN, HIGH);
    servo_pulse_high = true;
    servo_frame_start_us = now_us;
  }

  if (servo_pulse_high && static_cast<uint32_t>(now_us - servo_frame_start_us) >= servo_pulse_width_us) {
    digitalWrite(CAMERA_SERVO_PIN, LOW);
    servo_pulse_high = false;
  }

  if (millis() - camera_last_command_ms > CAMERA_COMMAND_TIMEOUT_MS) camera_direction = 0;

  if (camera_direction != 0 && millis() - camera_last_command_ms <= CAMERA_COMMAND_TIMEOUT_MS) {
    if (millis() - last_camera_step_ms >= 30) {
      last_camera_step_ms = millis();
      setCameraAngle(camera_angle + camera_direction * CAMERA_STEP_DEG);
    }
  }
}

// -------------------- Serial --------------------
char token_buffer[64];
uint8_t token_index = 0;

void setWheelTarget(char wheel, char sign, float value) {
  if (sign == 'n') value = -value;
  value = clampFloat(value, -MAX_WHEEL_VELOCITY, MAX_WHEEL_VELOCITY);
  if (wheel == 'r') right_wheel.target_velocity = value;
  else if (wheel == 'l') left_wheel.target_velocity = value;
  last_command_ms = millis();
}

void processVelocityToken(const char *token) {
  const size_t len = strlen(token);
  if (len < 3) return;

  if ((token[0] == 'r' || token[0] == 'l') && (token[1] == 'p' || token[1] == 'n')) {
    setWheelTarget(token[0], token[1], atof(token + 2));
    return;
  }

  // Camera command from the current ROS-side interface.
  // sp1.00 = one direction, sn1.00 = opposite direction.
  if (token[0] == 's' && (token[1] == 'p' || token[1] == 'n')) {
    float v = atof(token + 2);
    if (v < 0.01f) {
      camera_direction = 0;
      return;
    }
    camera_direction = (token[1] == 'p') ? +1 : -1;
    camera_last_command_ms = millis();
    return;
  }

  // Any transition from g=0 to a non-zero ROS gripper command is a manual
  // one-shot lift request. It is edge-triggered so the repeated ROS frames do
  // not restart the cycle. A new request requires the command to return to zero.
  if (token[0] == 'g' && (token[1] == 'p' || token[1] == 'n')) {
    float value = atof(token + 2);
    const bool active = fabs(value) > 0.01f;
    if (active && !g_command_was_active) {
      if (grip_state == GRIP_DOWN_READY) manual_grip_request = true;
      else {
        strncpy(cycle_outcome, "REQUEST_REJECTED", sizeof(cycle_outcome) - 1);
        cycle_outcome[sizeof(cycle_outcome) - 1] = '\0';
      }
    }
    g_command_was_active = active;
    return;
  }
}

void processExtendedToken(const char *token) {
  if (strncmp(token, "CSERVO:", 7) == 0) {
    setCameraAngle(atoi(token + 7));
    return;
  }

  if (strncmp(token, "SERVO:", 6) == 0) {
    setCameraAngle(atoi(token + 6));
    return;
  }

  // Direct serial test. Each command changes the target to a safe test velocity.
  if (strlen(token) == 1) {
    const float v = 2.0f;
    switch (token[0]) {
      case 'w': case 'W':
        right_wheel.target_velocity = +v;
        left_wheel.target_velocity = +v;
        last_command_ms = millis();
        break;
      case 's': case 'S':
        right_wheel.target_velocity = -v;
        left_wheel.target_velocity = -v;
        last_command_ms = millis();
        break;
      case 'a': case 'A':
        right_wheel.target_velocity = +v;
        left_wheel.target_velocity = -v;
        last_command_ms = millis();
        break;
      case 'd': case 'D':
        right_wheel.target_velocity = -v;
        left_wheel.target_velocity = +v;
        last_command_ms = millis();
        break;
      case 'x': case 'X':
        right_wheel.target_velocity = 0.0f;
        left_wheel.target_velocity = 0.0f;
        right_wheel.profiled_velocity = 0.0f;
        left_wheel.profiled_velocity = 0.0f;
        right_wheel.integral_term = 0.0f;
        left_wheel.integral_term = 0.0f;
        right_wheel.current_pwm = 0.0f;
        left_wheel.current_pwm = 0.0f;
        stopAllMotors();
        last_command_ms = millis();
        break;
      default:
        break;
    }
  }
}

void processToken(const char *token) {
  if (token[0] == '\0') return;

  if (strcmp(token, "cycle:1") == 0 || strcmp(token, "CYCLE:1") == 0) {
    if (grip_state == GRIP_DOWN_READY) manual_grip_request = true;
    else {
      strncpy(cycle_outcome, "REQUEST_REJECTED", sizeof(cycle_outcome) - 1);
      cycle_outcome[sizeof(cycle_outcome) - 1] = '\0';
    }
    return;
  }
  if (strcmp(token, "grip:recover_down") == 0 || strcmp(token, "GRIP:RECOVER_DOWN") == 0) {
    if (grip_state == GRIP_FAULT && !(topLimitActive() && bottomLimitActive())) {
      startRecoveryDown("MANUAL_RECOVERY");
    }
    return;
  }
  if (strcmp(token, "grip:reset") == 0 || strcmp(token, "GRIP:RESET") == 0) {
    if (grip_state == GRIP_FAULT && bottomLimitActive() && !topLimitActive()) {
      grip_state = GRIP_DOWN_READY;
      gripper_status = statusForBottomLimit();
      manual_grip_request = false;
      setMagnet(true);
      strncpy(grip_fault_reason, "NONE", sizeof(grip_fault_reason) - 1);
      strncpy(cycle_outcome, "RESET_AT_BOTTOM", sizeof(cycle_outcome) - 1);
    }
    return;
  }

  if (token[0] == 'C' || strncmp(token, "SERVO:", 6) == 0) {
    processExtendedToken(token);
    return;
  }

  if (token[0] == 'r' || token[0] == 'l' || token[0] == 'g' || token[0] == 's') {
    processVelocityToken(token);
    return;
  }

  processExtendedToken(token);
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());

    if (c == ',' || c == '\n' || c == '\r') {
      if (token_index > 0) {
        token_buffer[token_index] = '\0';
        processToken(token_buffer);
        token_index = 0;
      }
      continue;
    }

    if (token_index < sizeof(token_buffer) - 1) token_buffer[token_index++] = c;
    else token_index = 0;
  }
}

// -------------------- Telemetry --------------------
void printSignedToken(char prefix, float value) {
  Serial.print(prefix);
  Serial.print(value >= 0.0f ? 'p' : 'n');
  Serial.print(fabs(value), 2);
}

void sendTelemetry() {
  // Keep the exact token names understood by roverInterface on EVERY frame.
  printSignedToken('r', right_wheel.measured_velocity);
  Serial.print(',');
  printSignedToken('l', left_wheel.measured_velocity);
  Serial.print(',');
  // Keep p0 for roverInterface protocol compatibility. Proximity is ignored.
  Serial.print("p0,");
  Serial.print('m');
  Serial.print(metal_detected ? '1' : '0');
  Serial.print(',');
  Serial.print('g');
  Serial.print(gripper_status);
  Serial.print(',');

  // Encoder totals are sent on every frame so roverInterface can update the
  // encoder joint positions and estimate their velocities at the 100 Hz loop.
  Serial.print("er:");
  Serial.print(getRightTotalPulses());
  Serial.print(",el:");
  Serial.print(getLeftTotalPulses());
  Serial.print(',');

  Serial.println();
}

// Proximity diagnostics are intentionally omitted: this firmware does not read proximity pins.

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_R_PWM, OUTPUT);   pinMode(MOTOR_R_DIR, OUTPUT);
  pinMode(MOTOR_L_PWM, OUTPUT);   pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_PWM_2, OUTPUT); pinMode(MOTOR_R_DIR_2, OUTPUT);
  pinMode(MOTOR_L_PWM_2, OUTPUT); pinMode(MOTOR_L_DIR_2, OUTPUT);
  stopAllMotors();

  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), leftEncoderISR, RISING);

  pinMode(METAL_DETECTOR_PIN, INPUT);

  pinMode(SIREN_PIN, OUTPUT);
  digitalWrite(SIREN_PIN, SIREN_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(MAGNET_PIN, OUTPUT);
  digitalWrite(MAGNET_PIN, HIGH);  // Energize supply so the limit switches can be read.
  magnet_state = true;

  pinMode(US_TRIG_PIN, OUTPUT);
  digitalWrite(US_TRIG_PIN, LOW);
  pinMode(US_ECHO_PIN, INPUT);

  pinMode(LIFT_PWM, OUTPUT);
  pinMode(LIFT_DIR, OUTPUT);
  analogWrite(LIFT_PWM, 0);
  pinMode(LIMIT_SW_TOP, INPUT);     // Requires external pull-downs as in reference firmware.
  pinMode(LIMIT_SW_BOTTOM, INPUT);

  // Software servo output on D4.
  pinMode(CAMERA_SERVO_PIN, OUTPUT);
  digitalWrite(CAMERA_SERVO_PIN, LOW);
  setCameraAngle(CAMERA_CENTER_ANGLE);
  camera_last_command_ms = millis();
  servo_frame_start_us = micros();
  last_camera_step_ms = millis();

  last_control_ms = millis();
  last_telemetry_ms = millis();
  last_sensor_ms = millis();
  last_command_ms = millis();
  grip_state_started_ms = millis();
  last_us_sample_ms = millis();
  metal_last_raw = (digitalRead(METAL_DETECTOR_PIN) == METAL_ACTIVE_STATE);
  metal_detected = metal_last_raw;
  metal_last_change_ms = millis();
  siren_until_ms = 0;
  updateSensors();
  updateAlarmOutputs();
  initializeGripperState();

  // This one startup line is harmless: roverInterface ignores unknown tokens.
  Serial.println("ROVER_ARDUINO_READY");
}

void loop() {
  readSerial();
  updateCameraServo();
  updateUltrasonic();
  updateGripperMotor();

  uint32_t now = millis();

  if (now - last_control_ms >= CONTROL_PERIOD_MS) {
    last_control_ms += CONTROL_PERIOD_MS;
    updateControl();
  }

  if (now - last_sensor_ms >= SENSOR_PERIOD_MS) {
    last_sensor_ms = now;
    updateSensors();
    updateAlarmOutputs();
  }

  if (now - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms += TELEMETRY_PERIOD_MS;
    sendTelemetry();
  }
}