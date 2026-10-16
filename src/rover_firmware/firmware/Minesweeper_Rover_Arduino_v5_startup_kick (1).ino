/*
 * Minesweeper Rover - ROS 2 compatible Arduino Mega 2560 firmware (V4)
 *
 * Built by combining the known-good single-file motor/encoder test sketch
 * with the sensor/siren pin information from the FreeRTOS reference firmware.
 *
 * ROS2 roverInterface RX (115200 baud):
 *   rp1.00,lp1.00,gn2.97,sp0.00,
 *   rn0.50,ln0.50,gp2.97,sp0.00,

 *   CSERVO:90

 *
 * Telemetry TX (one line, repeatedly):
 *   rp1.82,lp1.79,p0,m0,q1:1,q2:0,q3:1,q4:0,er:1234,el:-1234,

 *   The current roverInterface consumes rp/rn, lp/ln, p0/p1, m0/m1 and gripper status go/gc/gm.
 *   Extra q1..q4, er and el tokens are diagnostic and are ignored by that interface.
 *   Wheel position/odometry is integrated by roverInterface from wheel velocity.
 *
 * IMPORTANT WIRING:
 *   Motor/encoder/camera pins follow the tested single-file sketch.
 *   Proximity D25/D23/D12/D13, metal D27, siren D47, lift D7/D49 and
 *   limit switches D51/D53 follow the supplied original FreeRTOS firmware.
 *   The separate buzzer pin was NOT specified in either source, so that output
 *   stays disabled. Set BUZZER_PIN only after confirming the physical wiring.
 *
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
constexpr float ENCODER_PPR = 385.0f;
constexpr float RAD_PER_PULSE = (2.0f * PI) / ENCODER_PPR;
constexpr uint32_t CONTROL_PERIOD_MS = 20;       // 50 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 10;     // 100 Hz, match ros2_control read loop
constexpr uint32_t SENSOR_PERIOD_MS = 20;        // 50 Hz
constexpr uint32_t COMMAND_TIMEOUT_MS = 400;
constexpr float MAX_WHEEL_VELOCITY = 6.0f;

constexpr float MAX_ACCELERATION = 15.0f;
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

// Startup compensation for geared DC motors that do not break away at low PWM.
// This is separate from the PID gains; tune only after a wheels-up test.
constexpr float MOTOR_STARTUP_PWM = 65.0f;       // about 25% duty
constexpr uint32_t MOTOR_STARTUP_KICK_MS = 250;  // kick duration after starting/reversing
constexpr float STARTUP_TARGET_EPSILON = 0.05f;
constexpr float STARTUP_MEASURED_SPEED_EPSILON = 0.15f;

// -------------------- Sensors / siren / lift-gripper --------------------
// Sensor pins below follow the supplied original FreeRTOS firmware.
// Proximity sensors are DIGITAL outputs on D25, D23, D12 and D13.
constexpr uint8_t PROXIMITY_COUNT = 4;
constexpr uint8_t PROXIMITY_PINS[PROXIMITY_COUNT] = {25, 23, 12, 13};
constexpr uint8_t PROXIMITY_ACTIVE_STATE = HIGH;  // Change to LOW if the sensors are active-low.
constexpr uint8_t METAL_DETECTOR_PIN = 27;
constexpr uint8_t METAL_ACTIVE_STATE = HIGH;      // Confirmed in original FreeRTOS firmware.
constexpr uint32_t METAL_DEBOUNCE_MS = 50;

constexpr uint8_t SIREN_PIN = 47;
constexpr bool SIREN_ACTIVE_HIGH = true;
constexpr uint8_t MAGNET_PIN = 38;
constexpr bool MAGNET_ACTIVE_HIGH = true;

// The physical lift/gripper pins come from the original FreeRTOS firmware.
constexpr uint8_t LIFT_PWM = 7;
constexpr uint8_t LIFT_DIR = 49;
constexpr uint8_t LIMIT_SW_TOP = 51;
constexpr uint8_t LIMIT_SW_BOTTOM = 53;
// Reference firmware used INPUT with external pull-downs, so HIGH means pressed.
// Use COM -> +5V, NO -> D51/D53, plus a 10k pull-down from input to GND.
constexpr uint8_t LIMIT_SWITCH_ACTIVE_STATE = HIGH;
// Direction assumption inherited from the prior test firmware. If a no-load test
// shows D7/D49 moves UP when it should move DOWN, change this to false.
constexpr bool LIFT_DOWN_DIR_HIGH = true;
constexpr uint8_t LIFT_PWM_PERCENT = 60;
constexpr uint32_t LIFT_DOWN_TIMEOUT_MS = 12000;
constexpr uint32_t LIFT_UP_TIMEOUT_MS = 12000;
constexpr uint32_t LIMIT_DEPART_TIMEOUT_MS = 800;
constexpr uint32_t PROXIMITY_RAISE_DELAY_MS = 4000;
constexpr float MAX_GRIPPER_COMMAND = 3.0f;
constexpr float GRIPPER_COMMAND_EPSILON = 0.01f;

// The uploaded roverInterface's gripper_joy_teleop logs use OPEN=0 and CLOSE=-2.967.
// Therefore negative g velocity moves toward CLOSED, positive moves toward OPEN.
// Keep BUZZER_PIN disabled until its actual wired pin is known; D47 is the siren.
constexpr int16_t BUZZER_PIN = -1;
constexpr bool BUZZER_ACTIVE_HIGH = true;
constexpr uint32_t BUZZER_TOGGLE_MS = 200;

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
  uint32_t startup_kick_until_ms = 0;
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

uint8_t proximity_values[PROXIMITY_COUNT] = {0};
bool proximity_triggered = false;
bool metal_detected = false;
bool metal_last_raw = false;
uint32_t metal_last_change_ms = 0;

enum LiftCycleState : uint8_t {
  LIFT_STATE_IDLE,
  LIFT_STATE_MOVING_DOWN,
  LIFT_STATE_BOTTOM_WAIT_PROX,
  LIFT_STATE_MOVING_UP,
  LIFT_STATE_FAULT
};

LiftCycleState lift_cycle_state = LIFT_STATE_IDLE;
uint32_t lift_state_started_ms = 0;
bool proximity_raise_timer_started = false;
uint32_t proximity_raise_timer_started_ms = 0;
// Require a release/open command before another close command can start a new cycle.
bool close_cycle_armed = true;
char gripper_status = 'm';  // go=open/top, gc=closed/bottom, gm=moving or fault
uint32_t last_buzzer_toggle_ms = 0;
bool buzzer_output_on = false;

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

void assignWheelTarget(WheelState &wheel, float value) {
  const bool starting = fabs(value) > STARTUP_TARGET_EPSILON &&
    (fabs(wheel.target_velocity) <= STARTUP_TARGET_EPSILON ||
     wheel.target_velocity * value < 0.0f);
  if (starting) {
    wheel.startup_kick_until_ms = millis() + MOTOR_STARTUP_KICK_MS;
  }
  wheel.target_velocity = value;
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

  // Temporarily provide enough torque to start a stationary wheel or reverse it.
  // Once the kick window expires, the original PID output takes over unchanged.
  const uint32_t now_ms = millis();
  if (fabs(right_wheel.profiled_velocity) > STARTUP_TARGET_EPSILON &&
      fabs(right_wheel.measured_velocity) < STARTUP_MEASURED_SPEED_EPSILON &&
      now_ms < right_wheel.startup_kick_until_ms) {
    right_wheel.target_pwm = (right_wheel.profiled_velocity >= 0.0f)
      ? MOTOR_STARTUP_PWM : -MOTOR_STARTUP_PWM;
  }
  if (fabs(left_wheel.profiled_velocity) > STARTUP_TARGET_EPSILON &&
      fabs(left_wheel.measured_velocity) < STARTUP_MEASURED_SPEED_EPSILON &&
      now_ms < left_wheel.startup_kick_until_ms) {
    left_wheel.target_pwm = (left_wheel.profiled_velocity >= 0.0f)
      ? MOTOR_STARTUP_PWM : -MOTOR_STARTUP_PWM;
  }

  right_wheel.current_pwm = applyPWMRamp(right_wheel.target_pwm, right_wheel.current_pwm, dt);
  left_wheel.current_pwm = applyPWMRamp(left_wheel.target_pwm, left_wheel.current_pwm, dt);

  applyDrivePWM();
}

// -------------------- Sensors --------------------
void updateSensors() {
  proximity_triggered = false;
  for (uint8_t i = 0; i < PROXIMITY_COUNT; ++i) {
    const uint8_t raw = digitalRead(PROXIMITY_PINS[i]);
    proximity_values[i] = raw;
    if (raw == PROXIMITY_ACTIVE_STATE) proximity_triggered = true;
  }

  const bool raw_metal = (digitalRead(METAL_DETECTOR_PIN) == METAL_ACTIVE_STATE);
  if (raw_metal != metal_last_raw) {
    metal_last_raw = raw_metal;
    metal_last_change_ms = millis();
  }
  if (millis() - metal_last_change_ms >= METAL_DEBOUNCE_MS) {
    metal_detected = metal_last_raw;
  }
}

void updateAlarmOutputs() {
  const bool alarm_active = proximity_triggered || metal_detected;
  const bool siren_level = SIREN_ACTIVE_HIGH ? alarm_active : !alarm_active;
  digitalWrite(SIREN_PIN, siren_level ? HIGH : LOW);

  if (BUZZER_PIN < 0) return;  // No physical buzzer pin was specified in either source.

  const uint32_t now = millis();
  if (!alarm_active) {
    buzzer_output_on = false;
    digitalWrite(static_cast<uint8_t>(BUZZER_PIN), BUZZER_ACTIVE_HIGH ? LOW : HIGH);
    return;
  }

  if (now - last_buzzer_toggle_ms >= BUZZER_TOGGLE_MS) {
    last_buzzer_toggle_ms = now;
    buzzer_output_on = !buzzer_output_on;
    const bool level = BUZZER_ACTIVE_HIGH ? buzzer_output_on : !buzzer_output_on;
    digitalWrite(static_cast<uint8_t>(BUZZER_PIN), level ? HIGH : LOW);
  }
}

bool topLimitActive() {
  return digitalRead(LIMIT_SW_TOP) == LIMIT_SWITCH_ACTIVE_STATE;
}

bool bottomLimitActive() {
  return digitalRead(LIMIT_SW_BOTTOM) == LIMIT_SWITCH_ACTIVE_STATE;
}

void setMagnet(bool enabled) {
  const bool level = MAGNET_ACTIVE_HIGH ? enabled : !enabled;
  digitalWrite(MAGNET_PIN, level ? HIGH : LOW);
}

void stopLiftMotor() {
  analogWrite(LIFT_PWM, 0);
}

void driveLiftDown() {
  digitalWrite(LIFT_DIR, LIFT_DOWN_DIR_HIGH ? HIGH : LOW);
  analogWrite(LIFT_PWM, static_cast<uint8_t>((255UL * LIFT_PWM_PERCENT) / 100UL));
}

void driveLiftUp() {
  digitalWrite(LIFT_DIR, LIFT_DOWN_DIR_HIGH ? LOW : HIGH);
  analogWrite(LIFT_PWM, static_cast<uint8_t>((255UL * LIFT_PWM_PERCENT) / 100UL));
}

void enterLiftFault(bool keep_magnet_on) {
  stopLiftMotor();
  if (!keep_magnet_on) setMagnet(false);
  lift_cycle_state = LIFT_STATE_FAULT;
  lift_state_started_ms = millis();
  gripper_status = 'm';
  close_cycle_armed = false;
}

void startPickupCycle() {
  const bool top = topLimitActive();
  const bool bottom = bottomLimitActive();
  if (top && bottom) {
    // Conflicting switches: stop, but do not drop a possibly held mine.
    enterLiftFault(true);
    return;
  }

  proximity_raise_timer_started = false;
  proximity_raise_timer_started_ms = 0;
  lift_state_started_ms = millis();

  // If the lower switch is already pressed, do not push farther into the stop.
  // Enter the bottom wait state immediately and engage the magnet.
  if (bottom) {
    stopLiftMotor();
    setMagnet(true);
    lift_cycle_state = LIFT_STATE_BOTTOM_WAIT_PROX;
    gripper_status = 'c';
    return;
  }

  setMagnet(false);
  lift_cycle_state = LIFT_STATE_MOVING_DOWN;
  gripper_status = 'm';
  driveLiftDown();
}

void startRaiseToTop() {
  const bool top = topLimitActive();
  const bool bottom = bottomLimitActive();
  if (top && bottom) {
    enterLiftFault(false);
    return;
  }
  if (top) {
    stopLiftMotor();
    setMagnet(false);
    lift_cycle_state = LIFT_STATE_IDLE;
    gripper_status = 'o';
    close_cycle_armed = true;
    return;
  }
  lift_cycle_state = LIFT_STATE_MOVING_UP;
  lift_state_started_ms = millis();
  gripper_status = 'm';
  driveLiftUp();
}

void updateGripperMotor() {
  const bool top = topLimitActive();
  const bool bottom = bottomLimitActive();
  const uint32_t now = millis();

  // Both endpoint switches active at once is a wiring fault; stop immediately.
  // Keep the magnet state unchanged in a fault so a carried mine is not dropped.
  if (top && bottom && lift_cycle_state != LIFT_STATE_FAULT) {
    enterLiftFault(true);
    return;
  }

  switch (lift_cycle_state) {
    case LIFT_STATE_IDLE:
      stopLiftMotor();
      if (top) {
        gripper_status = 'o';
        setMagnet(false);  // At the upper endpoint, release the mine.
      } else if (bottom) {
        gripper_status = 'c';
        setMagnet(true);   // If already on the lower endpoint, keep magnet engaged.
      }
      break;

    case LIFT_STATE_MOVING_DOWN:
      // The lower switch blocks only downward motion. Once it is pressed,
      // stop immediately and enter the wait-for-proximity state.
      if (bottom) {
        stopLiftMotor();
        setMagnet(true);
        lift_cycle_state = LIFT_STATE_BOTTOM_WAIT_PROX;
        lift_state_started_ms = now;
        proximity_raise_timer_started = false;
        proximity_raise_timer_started_ms = 0;
        gripper_status = 'c';
      } else if (top && now - lift_state_started_ms >= LIMIT_DEPART_TIMEOUT_MS) {
        // Moving DOWN should release the top switch. If it does not, stop safely.
        enterLiftFault(false);
      } else if (now - lift_state_started_ms >= LIFT_DOWN_TIMEOUT_MS) {
        enterLiftFault(false);
      } else {
        driveLiftDown();
        gripper_status = 'm';
      }
      break;

    case LIFT_STATE_BOTTOM_WAIT_PROX:
      // Stay stopped at the bottom with the magnet ON until any proximity sensor
      // first detects an object. Then latch that event and wait a full 4 seconds.
      stopLiftMotor();
      setMagnet(true);
      gripper_status = 'c';

      if (!proximity_raise_timer_started && proximity_triggered) {
        proximity_raise_timer_started = true;
        proximity_raise_timer_started_ms = now;
      }

      if (proximity_raise_timer_started &&
          now - proximity_raise_timer_started_ms >= PROXIMITY_RAISE_DELAY_MS) {
        // Raise only after the 4-second delay from the first proximity detection.
        lift_cycle_state = LIFT_STATE_MOVING_UP;
        lift_state_started_ms = now;
        gripper_status = 'm';
        driveLiftUp();
      }
      break;

    case LIFT_STATE_MOVING_UP:
      // The upper switch blocks only upward motion. Reverse/down commands are
      // allowed to move away from the top; hitting the bottom is handled below.
      if (top) {
        stopLiftMotor();
        setMagnet(false);  // Release the mine only at the upper endpoint.
        lift_cycle_state = LIFT_STATE_IDLE;
        lift_state_started_ms = now;
        proximity_raise_timer_started = false;
        gripper_status = 'o';
      } else if (bottom && now - lift_state_started_ms >= LIMIT_DEPART_TIMEOUT_MS) {
        // Moving UP should release the bottom switch. Keep magnet ON if it fails.
        enterLiftFault(true);
      } else if (now - lift_state_started_ms >= LIFT_UP_TIMEOUT_MS) {
        enterLiftFault(true);
      } else {
        setMagnet(true);
        driveLiftUp();
        gripper_status = 'm';
      }
      break;

    case LIFT_STATE_FAULT:
      stopLiftMotor();
      // Fault remains latched until the cause is checked and Arduino is reset.
      break;
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
  if (wheel == 'r') assignWheelTarget(right_wheel, value);
  else if (wheel == 'l') assignWheelTarget(left_wheel, value);
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

  // ROS2 roverInterface sends gp<value> / gn<value> continuously.
  // Negative (CLOSE) starts: down -> bottom switch -> magnet ON -> wait for
  // proximity -> wait 4 s -> up -> top switch -> magnet OFF.
  // A positive (OPEN/up) command can override the automatic sequence at any
  // point, so the operator can move away from a pressed endpoint switch.
  if (token[0] == 'g' && (token[1] == 'p' || token[1] == 'n')) {
    float value = atof(token + 2);
    if (token[1] == 'n') value = -value;
    value = clampFloat(value, -MAX_GRIPPER_COMMAND, MAX_GRIPPER_COMMAND);

    if (lift_cycle_state == LIFT_STATE_FAULT) {
      return;
    }

    // An OPEN/up request overrides waiting-at-bottom or downward motion.
    // Do not restart the movement timer on every repeated ROS command.
    if (value > GRIPPER_COMMAND_EPSILON) {
      close_cycle_armed = true;
      if (lift_cycle_state != LIFT_STATE_MOVING_UP) {
        startRaiseToTop();
      }
      return;
    }

    // Ignore repeated CLOSE and neutral updates during an automatic sequence;
    // they must not cancel the proximity timer or restart the pickup cycle.
    if (lift_cycle_state != LIFT_STATE_IDLE) {
      return;
    }

    if (value < -GRIPPER_COMMAND_EPSILON) {
      if (close_cycle_armed) {
        close_cycle_armed = false;
        startPickupCycle();
      }
    } else {
      // Once idle, neutral rearms the next close press.
      close_cycle_armed = true;
    }
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
        setWheelTarget('r', 'p', v);
        setWheelTarget('l', 'p', v);
        last_command_ms = millis();
        break;
      case 's': case 'S':
        setWheelTarget('r', 'n', v);
        setWheelTarget('l', 'n', v);
        last_command_ms = millis();
        break;
      case 'a': case 'A':
        setWheelTarget('r', 'p', v);
        setWheelTarget('l', 'n', v);
        last_command_ms = millis();
        break;
      case 'd': case 'D':
        setWheelTarget('r', 'n', v);
        setWheelTarget('l', 'p', v);
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
  Serial.print('p');
  Serial.print(proximity_triggered ? '1' : '0');
  Serial.print(',');
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

  // Raw digital proximity states are diagnostic only; the interface consumes p0/p1.
  static uint8_t diagnostic_divider = 0;
  diagnostic_divider = (diagnostic_divider + 1) % 5;
  if (diagnostic_divider == 0) {
    for (uint8_t i = 0; i < PROXIMITY_COUNT; ++i) {
      Serial.print('q');
      Serial.print(i + 1);
      Serial.print(':');
      Serial.print(proximity_values[i]);
      Serial.print(',');
    }
  }
  Serial.println();
}

// Optional raw debug line (not used by roverInterface).
void sendRawSensors() {
  Serial.print("P:");
  for (uint8_t i = 0; i < PROXIMITY_COUNT; ++i) {
    Serial.print(proximity_values[i]);
    if (i + 1 < PROXIMITY_COUNT) Serial.print(',');
  }
  Serial.print(" M:");
  Serial.println(metal_detected ? 1 : 0);
}

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

  for (uint8_t i = 0; i < PROXIMITY_COUNT; ++i) pinMode(PROXIMITY_PINS[i], INPUT);
  pinMode(METAL_DETECTOR_PIN, INPUT);  // Matches the original FreeRTOS firmware.

  pinMode(SIREN_PIN, OUTPUT);
  digitalWrite(SIREN_PIN, SIREN_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(MAGNET_PIN, OUTPUT);
  setMagnet(false);  // Keep electromagnet off at startup.

  pinMode(LIFT_PWM, OUTPUT);
  pinMode(LIFT_DIR, OUTPUT);
  analogWrite(LIFT_PWM, 0);
  pinMode(LIMIT_SW_TOP, INPUT);     // Requires external pull-downs as in reference firmware.
  pinMode(LIMIT_SW_BOTTOM, INPUT);

  if (BUZZER_PIN >= 0) {
    pinMode(static_cast<uint8_t>(BUZZER_PIN), OUTPUT);
    digitalWrite(static_cast<uint8_t>(BUZZER_PIN), BUZZER_ACTIVE_HIGH ? LOW : HIGH);
  }

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
  metal_last_raw = (digitalRead(METAL_DETECTOR_PIN) == METAL_ACTIVE_STATE);
  metal_detected = metal_last_raw;
  metal_last_change_ms = millis();
  updateSensors();
  updateAlarmOutputs();
  if (topLimitActive() && !bottomLimitActive()) gripper_status = 'o';
  else if (bottomLimitActive() && !topLimitActive()) gripper_status = 'c';
  else gripper_status = 'm';

  // This one startup line is harmless: roverInterface ignores unknown tokens.
  Serial.println("ROVER_ARDUINO_READY");
}

void loop() {
  readSerial();
  updateCameraServo();
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
