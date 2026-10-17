/*
 * Differential drive (2 DC motors + lift) - FreeRTOS version
 * Board  : Arduino Mega 2560
 * Library: "FreeRTOS" by Phillip Stevens (feilipu), header <Arduino_FreeRTOS.h>
 *
 * =====================================================================
 *  AUTO MODE - SPEED CONTROL PARAMETERS (edit the AUTO_* constants below)
 * =====================================================================
 *  Signal chain of one wheel in auto mode:
 *
 *    ROS command [rad/s] --x COUNTS_PER_RAD--> target [counts/s]
 *    target - measured speed = error [counts/s]
 *    PID(error)                              = motor duty [% of full PWM]
 *    measured speed [counts/s] --/ COUNTS_PER_RAD--> status to ROS [rad/s]
 *
 *  ENC_COUNTS_PER_REV (PPR) = 2400 counts per WHEEL revolution (all 4 quadrature
 *  edges, gear ratio included).   COUNTS_PER_RAD = 2400 / 2*pi = 381.97 counts/s
 *  for every 1 rad/s.   Example: 10 rad/s = 3819.7 counts/s.
 *
 *  AUTO_KP  Proportional gain, unit: % duty per (count/s) of speed error.
 *           The immediate reaction to the error. Too low = sluggish and never
 *           reaches the target, too high = oscillation / buzzing.
 *           Example: 0.02 -> an error of 500 counts/s (1.3 rad/s) adds 10 % duty.
 *  AUTO_KI  Integral gain, unit: % duty per (count/s * s).
 *           Accumulates the remaining error and removes the steady-state offset.
 *           It is what actually holds the speed under load. Too high = overshoot.
 *           The integrator is frozen while the output is saturated (anti-windup).
 *  AUTO_KD  Derivative gain (on the measurement, not on the error).
 *           Damps fast changes. Keep 0 unless needed: the speed is derived from
 *           encoder counts, so the derivative amplifies quantisation noise.
 *
 *  AUTO_TOL_CPS  Error margin ("target reached" band), unit: counts/s (in rad/s:
 *           divide by 381.97). When |target - measured| <= AUTO_TOL_CPS the wheel
 *           counts as on target. If the target is 0 the output is forced to 0 and
 *           the integrator is cleared inside this band, so a stopped wheel does not
 *           twitch. Do not set it below the speed resolution of ONE encoder count
 *           per PID period (= 1 / period = 100 counts/s at 10 ms), unless the
 *           measurement filter below smooths it.
 *  AUTO_PWM_MIN_PCT  Minimum duty (%) that still moves the robot (motor dead
 *           band). Every non-zero PID output is lifted to at least this value.
 *  AUTO_PWM_MAX_PCT  Maximum duty (%) the PID may output (hard ceiling). While the
 *           gripper cycle runs the drive motors are additionally limited to
 *           CYCLE_MAX_PCT (50 %).
 *  VEL_FILTER_ALPHA  Smoothing of the measured speed (first order low pass,
 *           0 < alpha <= 1, 1 = no filter). Lower = smoother but slower.
 *
 *  These values are loaded every time auto mode is entered (AUTOM command or the
 *  first ROS frame). After entering, P / I / D / TOL / PWMMAX / PWMMIN typed in the
 *  Serial Monitor still override them for live tuning (until the next reset).
 *
 *  Tuning: start with AUTO_KI = 0, raise AUTO_KP until the wheel follows a
 *  constant command quickly without oscillating, then add AUTO_KI until the
 *  steady-state error vanishes. Use "TEL 50" and the T,... lines to look at it.
 * =====================================================================
 *
 * Serial Monitor: 115200, "Newline". Commands are case-insensitive.
 *
 *  DRIVE (open loop, they switch the PID loop OFF)
 *   W / S / A / D     forward / backward / turn left / turn right for 2 s
 *   SPEED XX          duty as % of max (0..100) used by W/A/S/D
 *   MOVE X FF         motor X (A or B) at 20% for FF seconds, e.g. "MOVE B 1.5"
 *   STOP              stop both drive motors, PID loop off
 *
 *  CLOSED LOOP (one PID per motor, run inside the Timer3 ISR)
 *   PID XX            PID period in ms (1..1000, default 10, see notes)
 *   P x / I x / D x   PID gains, e.g. "P 2.5"  "I 3.4"  "D 0.1"
 *   MODE POS|VEL      target = position (counts) or speed (counts/s)
 *   TGT A|B|ALL v     target of motor/encoder A (right) / B (left)
 *   LOOP ON|OFF       enable / disable the closed loop
 *   TOL v             "target reached" tolerance (counts or counts/s)
 *   PWMMAX v [A|B]    max duty % of a motor (both if A|B omitted)
 *   PWMMIN v [A|B]    min duty % that still moves the robot
 *   ZERO              zero both encoders, targets and integrators
 *   RESET             re-init encoder pins, zero encoders/targets/PID state,
 *                     restore default inversions (ENCA, ENCB, B inverted)
 *
 *  CONFIGURATION
 *   INV A|B|LIFT      invert the default direction of a motor (DIR pin only)
 *   INV ENCA|ENCB     invert the counting direction of encoder A / B
 *   PPR A|B           print encoder count (signed 32-bit, rewinds to 0 past the limit)
 *   ENCDBG            print raw encoder pin levels (D19 D18 D21 D20) and EIMSK
 *   STAT / HELP       print configuration / commands
 *   TEL ms            telemetry line "T,..." every ms (0 = off, min 30)
 *
 *  AUTOMATIC MODE  (protocol of the ROS 2 rover_interface.cpp hardware interface)
 *   START_IN_AUTO_MODE (constant at the top of the code): true = auto mode is already
 *                     active after power-up / reset, false = manual mode until AUTOM or
 *                     the first ROS frame.
 *   AUTOM XX          enter auto mode: PID sampling time in ms (default
 *                     AUTO_DEFAULT_MS). Velocity PID is enabled with the AUTO_*
 *                     parameters above, telemetry is switched off.
 *                     "AUTOM OFF" leaves auto mode.
 *                     Auto mode is ALSO entered by itself on the first valid frame
 *                     from ROS (the Arduino resets when ROS opens the port).
 *
 *   ROS -> Arduino (one line per ROS write(), comma separated, trailing comma):
 *        rp12.34,lp12.34,gp00.00,sp00.00,
 *        r / l : right / left wheel velocity command in rad/s ('p' = +, 'n' = -),
 *                converted to counts/s with COUNTS_PER_RAD (PPR = 2400)
 *        g     : gripper command (ros2_control velocity). A change from 0 to
 *                non-zero (rising edge) = GRIP request. (token may be missing)
 *        s     : camera servo direction -1 / 0 / +1. The servo angle (0..180,
 *                start 90) is integrated at SERVO_RATE_DPS deg/s while non-zero.
 *   If no frame arrives for FRAME_TIMEOUT_MS the wheel targets are set to 0.
 *
 *   Arduino -> ROS (one line, comma separated, trailing comma):
 *        rp12.34,ln05.00,p1,m0,gc,
 *        r / l : measured wheel velocity in rad/s ('p'/'n' sign, 2 decimals)
 *        p1|p0 : 1 if any of the 5 proximity sensors is HIGH, or had a rising edge
 *                since the last message
 *        m1|m0 : 1 only in the ONE message after a metal-detector rising edge
 *        go|gc|gm|gf : gripper open (lift up, metal released) / closed (lift down,
 *                idle) / moving (sanity check, lifting, lowering, recovery) / fault
 *   FRAME-SYNCHRONOUS REPLY: the ROS hardware interface reads exactly ONE status
 *   line per control cycle (read() then write()). To keep both sides in step,
 *   once ROS frames are arriving the Arduino sends exactly ONE status line per
 *   received frame (it carries the newest PID measurement). The line count can
 *   therefore never run ahead of what ROS reads, whatever the ROS update rate is,
 *   so no stale data piles up in the serial buffer. The PID itself keeps running at
 *   its own sampling time. Typed "AUTOM XX" without ROS frames still streams one
 *   status line per PID sample, for testing.
 *   Wheel units: ENC_COUNTS_PER_REV (below) converts counts/s <-> rad/s.
 *
 *  GRIPPER (state machine, same for GRIP and the "g" token of the ROS frame)
 *   The gripper is idle (state D) after power-up: NO sanity check at boot.
 *   The limit switches are powered from the SAME supply as the magnet, so the
 *   magnet is ON whenever a limit switch has to be read (sanity check, lifting,
 *   lowering).
 *
 *   GRIP (command or ROS "g" rising edge) in state D:
 *     - first GRIP after a reset (or every GRIP if SANITY_EVERY_GRIP): sanity check
 *         magnet ON -> wait SWITCH_POWER_MS -> lower switch must be HIGH (upper LOW)
 *         -> lift up until the upper switch is HIGH -> down until the lower switch
 *         is HIGH, 10 s timeout for each leg. Failure -> GRIP disabled until reset
 *         (magnet is switched OFF). Magnet stays ON during the whole check.
 *       Then the GRIP request is armed with a fresh 10 s window.
 *     - otherwise the request is armed directly.
 *
 *   Trigger edges: a metal-detector rising edge OR a rising edge of ANY of the
 *   5 proximity sensors (D23..D31) has the SAME role (in state D): magnet ON for
 *   10 s (restarted by every new edge), siren for 2 s, opens the 10 s "metal near"
 *   window needed by the grip cycle.
 *
 *   D (DOWN)       lower HIGH. Magnet OFF until a trigger edge: magnet ON for 10 s
 *                  (every new edge restarts the 10 s). Lift motor idle. If the magnet is
 *                  ON, NO limit switch is HIGH and the distance is >= 20 cm, the lift
 *                  is moved down until the lower switch is HIGH.
 *   A (MOVING_UP)  needs a GRIP request + trigger edge within the last 10 s
 *                  + ultrasonic distance < 20 cm for US_CONFIRM_N consecutive readings
 *                  (= metal is on the magnet). Lift goes
 *                  up (magnet ON). If the distance stays >= 20 cm for US_CONFIRM_N
 *                  consecutive readings (metal fell) or the leg times out, the lift
 *                  is lowered again (magnet ON) and the cycle returns to D with the
 *                  10 s magnet timeout restarted.
 *   U (UP)         upper HIGH, distance < 20 cm (metal still held): lift stops,
 *                  magnet OFF for 5 s (no limit switch is read in this phase). The
 *                  metal counts as dropped only after US_CONFIRM_N consecutive
 *                  readings >= 20 cm. The magnet then goes back ON and the upper
 *                  switch must read HIGH (after SWITCH_POWER_MS) before B starts;
 *                  otherwise the cycle recovers.
 *   B (MOVING_DOWN) magnet ON again (the lower switch needs power), lift down to the
 *                  lower switch while the distance is >= 20 cm. Then the cycle is
 *                  finished (magnet OFF, back to D).
 *   While a cycle runs: GRIP and MGNT are ignored and motors A/B are limited to 50 %.
 *
 *  I/O
 *   PROX              print the 5 proximity sensors (D23, D25, D27, D29, D31)
 *   MGNT              toggle the magnet output (D38), only in state D
 *   SIREN             toggle the siren output (D37)
 *   MD                metal detector on D33: METAL / NONE (level)
 *
 * Motors: A = right (encoder A), B = left (encoder B).
 * Defaults: motor B inverted, both encoders inverted.
 *
 * Data flow
 *   Timer3 ISR  -> PID math only (writes pid[m].out)
 *   signalTask  -> turns PID outputs into motor signals
 *   cmd / motion tasks -> also produce motor signals
 *   motorQ[ch]  -> one 1-slot "latest value wins" queue per channel (A, B, lift, servo)
 *   motorTask   -> only task that touches the PWM / DIR hardware (HARDWAREMX)
 *   driveMx     -> serialises everything that decides what the drive motors do
 *                  (commands, PID forwarding, motion timeout), so a stale
 *                  command can never overwrite a newer one (e.g. after STOP)
 */

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>

/* ------------------------- AUTO MODE TUNING (see header) ------------------------- */
/* >>> Start-up behaviour.
   true  : the Arduino boots straight into auto mode (velocity PID on, AUTO_* parameters loaded,
           wheel targets 0). It stays silent until the first ROS frame arrives, then answers
           every frame with one status line. Typed commands (HELP, STAT, AUTOM OFF ...) still work.
   false : the Arduino boots in manual mode (prints "Ready. Type HELP ...", PID loop off).
           Auto mode is then entered by the AUTOM command or by itself on the first valid ROS frame. */
constexpr bool     START_IN_AUTO_MODE = true;

/* >>> Encoder: counts per WHEEL revolution as counted by this firmware
   (x4 quadrature edges, gear ratio included). PPR = 2400. */
constexpr float    ENC_COUNTS_PER_REV = 2400.0f;
constexpr float    COUNTS_PER_RAD     = ENC_COUNTS_PER_REV / TWO_PI;   /* 381.97 counts/s per rad/s */

constexpr float    AUTO_KP          = 0.02f;   /* % duty per (count/s) of error                    */
constexpr float    AUTO_KI          = 0.10f;   /* % duty per (count/s * s)                         */
constexpr float    AUTO_KD          = 0.0f;    /* % duty per (count/s^2), 0 = off                  */
constexpr float    AUTO_TOL_CPS     = 50.0f;   /* error margin in counts/s (50 = 0.13 rad/s)       */
constexpr float    AUTO_PWM_MIN_PCT = 20.0f;   /* smallest non-zero duty that moves the robot      */
constexpr float    AUTO_PWM_MAX_PCT = 100.0f;  /* duty ceiling                                     */
constexpr float    VEL_FILTER_ALPHA = 0.5f;    /* measured-speed low pass, 1.0 = no filter         */
constexpr float    AUTO_MAX_TARGET_CPS = 100000.0f;  /* sanity clamp on a ROS command (counts/s)   */

constexpr uint16_t AUTO_DEFAULT_MS  = 10;      /* PID sampling time in auto mode                   */
constexpr uint32_t FRAME_TIMEOUT_MS = 500;     /* no ROS frame for this long -> wheel targets 0    */
constexpr float    SERVO_RATE_DPS   = 60.0f;   /* camera servo speed for s = +-1 (negative value flips direction) */

/* rad/s <-> counts/s (the ONLY place where the PPR conversion happens) */
static inline float radsToCps(float rads) { return rads * COUNTS_PER_RAD; }
static inline float cpsToRads(float cps)  { return cps / COUNTS_PER_RAD; }

/* ------------------------------ Pin map ------------------------------ */
constexpr uint8_t MOTOR_R_PWM = 44; /* PWM1, Timer 5 OC5C */
constexpr uint8_t MOTOR_R_DIR = 42;
constexpr uint8_t MOTOR_L_PWM = 46; /* PWM2, Timer 5 OC5A */
constexpr uint8_t MOTOR_L_DIR = 40;

constexpr uint8_t ENCODER_R_A = 19;
constexpr uint8_t ENCODER_R_B = 18;
constexpr uint8_t ENCODER_L_A = 21;
constexpr uint8_t ENCODER_L_B = 20;

/* Lift: D7 is Timer4 OC4B, hardware PWM */
constexpr uint8_t LIFT_PWM = 7;
constexpr uint8_t LIFT_DIR = 49;
constexpr uint8_t LIMIT_SW_TOP = 53;    /* PB2 / PCINT2 */
constexpr uint8_t LIMIT_SW_BOTTOM = 51; /* PB0 / PCINT0 */

/* Extra I/O */
constexpr uint8_t NUM_PROX = 5;
constexpr uint8_t PROX_PINS[NUM_PROX] = {23, 25, 27, 29, 31};
constexpr uint8_t PROX_ACTIVE_LEVEL = HIGH;   /* level that means "object detected" (rising edge = trigger) */
constexpr uint8_t MGNT_PIN  = 38;
constexpr uint8_t SIREN_PIN = 37;
constexpr uint8_t MD_PIN    = 33;   /* metal detector */
constexpr uint8_t MD_METAL_LEVEL = HIGH;   /* level that means "metal present" */
bool mgntState = false, sirenState = false;
constexpr uint8_t US_TRIG_PIN = 43;        /* ultrasonic trigger */
constexpr uint8_t US_ECHO_PIN = 45;        /* ultrasonic echo */
constexpr uint8_t SERVO_PIN   = 11;        /* OC1A (Timer1): not used by PID(T3), PWM(T4/T5), millis(T0) */

/* ------------------------------ Channels ------------------------------ */
constexpr uint8_t NUM_MOTORS = 2;   /* A (right), B (left); motor m uses encoder m / pid[m] */
constexpr uint8_t LIFT_IDX   = 2;
constexpr uint8_t SERVO_IDX  = 3;   /* servo: "pwm" field of MotorCmd = angle 0..180 */
constexpr uint8_t NUM_CH     = 4;   /* A, B + lift + servo */
constexpr uint8_t CH_PWM[NUM_CH] = {MOTOR_R_PWM, MOTOR_L_PWM, LIFT_PWM, SERVO_PIN};
constexpr uint8_t CH_DIR[NUM_CH] = {MOTOR_R_DIR, MOTOR_L_DIR, LIFT_DIR, 255};   /* servo: no DIR pin */

/* Servo: Timer1 fast PWM, 2 MHz tick, 20 ms period; 5 % = 1 ms (0 deg) .. 10 % = 2 ms (180 deg) */
constexpr uint16_t SERVO_TOP       = 39999;
constexpr uint16_t SERVO_MIN_TICKS = 2000;   /* 5 %  */
constexpr uint16_t SERVO_MAX_TICKS = 4000;   /* 10 % */

/* Defaults (also restored by RESET) */
constexpr bool DEF_MOTOR_INV[NUM_CH] = {false, false, true};   /* B inverted */
constexpr bool DEF_ENC_FLIP[2]       = {true, false};           /* both encoders inverted */

bool motorInv[NUM_CH] = {false, false, true};   /* DIR pin only */

constexpr uint16_t PWM_TOP_16 = 799;   /* 20 kHz, 800 steps */
constexpr uint8_t  MOVE_PCT   = 20;    /* duty used by "MOVE X FF" */

/* While the gripper cycle (or the sanity check) runs, A and B are limited to this duty */
constexpr uint8_t  CYCLE_MAX_PCT = 50;
volatile bool      cycleActive   = false;

/* ------------------------------ RTOS objects ------------------------------ */
struct MotorCmd { uint8_t pwm; uint8_t fwd; };   /* pwm 0..100 %, logical direction */
QueueHandle_t     motorQ[NUM_CH];                /* length 1, written with xQueueOverwrite */
SemaphoreHandle_t serialMx, driveMx;
TaskHandle_t hMotor, hSignal, hMotion, hCmd, hTel;
TaskHandle_t hUs, hGrip, hAuto;

/* printf-style, task-safe print. NOTE: AVR printf has no %f, use dtostrf. */
void sayf(const char *fmt_P, ...) {
  char buf[96];
  va_list ap;
  va_start(ap, fmt_P);
  vsnprintf_P(buf, sizeof buf, fmt_P, ap);
  va_end(ap);
  xSemaphoreTake(serialMx, portMAX_DELAY);
  Serial.println(buf);
  xSemaphoreGive(serialMx);
}
#define SAY(fmt, ...) sayf(PSTR(fmt), ##__VA_ARGS__)

inline void lockDrive()   { xSemaphoreTake(driveMx, portMAX_DELAY); }
inline void unlockDrive() { xSemaphoreGive(driveMx); }

/* ------------------------------ PWM hardware ------------------------------ */
void pwmInit() {
  for (uint8_t i = 0; i < NUM_CH; i++) {
    if (i == SERVO_IDX) continue;          /* servo has no DIR pin, set up below */
    pinMode(CH_PWM[i], OUTPUT);
    pinMode(CH_DIR[i], OUTPUT);
    digitalWrite(CH_DIR[i], LOW);
  }
  /* Timer5: OC5A (46) + OC5C (44), fast PWM mode 14, 20 kHz */
  TCCR5A = _BV(COM5A1) | _BV(COM5C1) | _BV(WGM51);
  TCCR5B = _BV(WGM53) | _BV(WGM52) | _BV(CS50);
  ICR5 = PWM_TOP_16; OCR5A = 0; OCR5C = 0;
  /* Timer4: OC4B (7) = lift, same 20 kHz */
  TCCR4A = _BV(COM4B1) | _BV(WGM41);
  TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);
  ICR4 = PWM_TOP_16; OCR4B = 0;
  /* Timer1: OC1A (11) = servo, fast PWM mode 14, prescaler 8, 50 Hz. OCR1A = 0 -> no pulse until the first servo command */
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
  ICR1 = SERVO_TOP; OCR1A = 0;
}

/*
 * HARDWAREMX - ONLY called by motorTask.
 *   PIN : the channel's PWM pin (44, 46 or 7 for the lift)
 *   PWM : duty in percent 0..100
 *   DIR : logical direction 1 = forward, 0 = backward.
 * The INV flag is applied here, at the very last step, so the PID and the
 * encoders only ever see the logical direction.
 * The 16-bit OCRn writes are done with interrupts off: the grip ISR also writes
 * OCR4B, and both writes share the timer's TEMP register (a torn write could
 * otherwise leave a wrong duty).
 */
void HARDWAREMX(int PIN, int PWM, int DIR) {
  int ch = -1;
  for (uint8_t i = 0; i < NUM_CH; i++) if (CH_PWM[i] == PIN) { ch = i; break; }
  if (ch < 0) return;
  if (ch == SERVO_IDX) {                   /* PWM = angle 0..180 deg -> 5..10 % duty */
    uint32_t a = (uint32_t)constrain(PWM, 0, 180);
    uint16_t ticks = SERVO_MIN_TICKS + (uint16_t)((a * (SERVO_MAX_TICKS - SERVO_MIN_TICKS) + 90UL) / 180UL);
    const uint8_t sr = SREG;
    cli();
    OCR1A = ticks;
    SREG = sr;
    return;
  }
  PWM = constrain(PWM, 0, 100);
  bool level = (DIR != 0) ^ motorInv[ch];
  digitalWrite(CH_DIR[ch], level ? HIGH : LOW);
  const uint16_t duty = (uint16_t)PWM * 8;
  const uint8_t sreg = SREG;
  cli();
  switch (PIN) {
    case MOTOR_R_PWM: OCR5C = duty; break;
    case MOTOR_L_PWM: OCR5A = duty; break;
    case LIFT_PWM:    OCR4B = duty; break;
  }
  SREG = sreg;
}

/* ------------------------- Motor signal queues ------------------------- */
/*
 * Any task calls this to give a signal to a motor. A motor signal is a state,
 * not an event, so the queue has one slot and the newest signal replaces any
 * unread one: it never blocks, never fails, and an old command can never be
 * applied after a newer one.
 * While the gripper cycle runs, the drive motors A and B are capped to
 * CYCLE_MAX_PCT here (last line of defence; the PID ISR also limits its output).
 */
bool motorSend(uint8_t ch, uint8_t pwm, bool fwd) {
  if (ch >= NUM_CH || hMotor == NULL) return false;
  if (ch < NUM_MOTORS && cycleActive && pwm > CYCLE_MAX_PCT) pwm = CYCLE_MAX_PCT;
  MotorCmd c = {pwm, (uint8_t)fwd};
  xQueueOverwrite(motorQ[ch], &c);
  xTaskNotifyGive(hMotor);
  return true;
}

void stopDrive() {
  for (uint8_t m = 0; m < NUM_MOTORS; m++) motorSend(m, 0, true);
}

void motorTask(void *) {
  MotorCmd c;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (uint8_t ch = 0; ch < NUM_CH; ch++)
      while (xQueueReceive(motorQ[ch], &c, 0) == pdTRUE)
        HARDWAREMX(CH_PWM[ch], c.pwm, c.fwd);
  }
}

/* ------------------------------ Encoders ------------------------------ */
/*
 * Pure interrupt driven, no task and no Arduino attachInterrupt() layer.
 * The four encoder pins are the Mega's external interrupts INT0..INT3, set to
 * "any edge", and each ISR reads the port once and updates its counter:
 *   ENC A (right): A = D19 / PD2 / INT2,   B = D18 / PD3 / INT3
 *   ENC B (left) : A = D21 / PD0 / INT0,   B = D20 / PD1 / INT1
 * Both channels of one encoder share a handler, so a 2-bit field of PIND is the
 * whole quadrature state (bit0 = A, bit1 = B).
 *
 * Counters are signed 32-bit. Instead of wrapping to the opposite sign, a count
 * that would pass +2147483647 or -2147483648 rewinds to 0.
 */
static_assert(ENCODER_R_A == 19 && ENCODER_R_B == 18 && ENCODER_L_A == 21 && ENCODER_L_B == 20,
              "Encoder ISRs are written for D19/D18 (INT2/INT3) and D21/D20 (INT0/INT1)");

constexpr uint8_t ENC_A[2] = {ENCODER_R_A, ENCODER_L_A};
constexpr uint8_t ENC_B[2] = {ENCODER_R_B, ENCODER_L_B};
constexpr uint8_t ENC_INT_MASK = 0x0F;       /* EIMSK / EIFR bits for INT0..INT3 */

volatile int32_t encCount[2];
volatile uint8_t encPrev[2];
volatile bool    encFlip[2] = {true, false};   /* inverted by default */

/* index = (previous state << 2) | new state, state = (B << 1) | A  ->  -1, 0 or +1 */
static const int8_t QEMR[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};

static inline __attribute__((always_inline)) void encStep(uint8_t i, uint8_t cur) {
  int8_t d = QEMR[(encPrev[i] << 2) | cur];
  encPrev[i] = cur;
  if (!d) return;                              /* glitch / no movement */
  if (encFlip[i]) d = -d;
  int32_t v = encCount[i];
  if (d > 0) v = (v == INT32_MAX) ? 0 : v + 1; /* rewind to 0 past the 32-bit limit */
  else       v = (v == INT32_MIN) ? 0 : v - 1;
  encCount[i] = v;
}

ISR(INT2_vect) { encStep(0, (PIND >> 2) & 3); }   /* ENC A, channel A (D19) */
ISR(INT3_vect) { encStep(0, (PIND >> 2) & 3); }   /* ENC A, channel B (D18) */
ISR(INT0_vect) { encStep(1, PIND & 3); }          /* ENC B, channel A (D21) */
ISR(INT1_vect) { encStep(1, PIND & 3); }          /* ENC B, channel B (D20) */

/* Safe to call again (RESET): pull-ups, any-edge mode, zero counts, enable INT0..3 */
void encInit() {
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
  }
  const uint8_t sreg = SREG;
  cli();
  EIMSK &= ~ENC_INT_MASK;                      /* off while configuring */
  EICRA  = 0x55;                               /* ISCn1:0 = 01 -> any change, INT0..INT3 */
  encCount[0] = 0;
  encCount[1] = 0;
  encPrev[0] = (PIND >> 2) & 3;
  encPrev[1] = PIND & 3;
  EIFR   = ENC_INT_MASK;                       /* drop stale flags */
  EIMSK |= ENC_INT_MASK;
  SREG = sreg;
}

/* ------------------------- PID (Timer3 ISR) ------------------------- */
enum { MODE_POS = 0, MODE_VEL = 1 };

struct PidCtl {
  float   target;    /* counts (POS) or counts/s (VEL) */
  float   meas;      /* counts (POS) or filtered counts/s (VEL) */
  float   vel;       /* low-pass filtered speed in counts/s, tracked in every mode */
  float   integ;
  float   prevMeas;
  float   out;       /* signed duty %, already capped */
  int32_t lastCount;
  bool    reached;
};
PidCtl pid[NUM_MOTORS];

float kp = 0.2f, ki = 0.01f, kd = 0.001f;
float tol = 100.0f;
float pwmMax[NUM_MOTORS] = {100.0f, 100.0f};
float pwmMin[NUM_MOTORS] = {20.0f, 20.0f};
volatile uint8_t ctrlMode    = MODE_POS;
volatile bool    loopEnabled = false;
volatile bool    pidFresh    = false;
volatile uint8_t pidSeq      = 0;       /* +1 every PID sample */
volatile bool    autoMode    = false;   /* AUTOM: status messages + ROS input frames */
volatile bool    rosSync     = false;   /* ROS frames are arriving: ONE status line per received frame */
volatile bool    statusReq   = false;   /* a ROS frame was received, its status reply is due */
uint16_t         pidPeriodMs = 0;

/* ROS frame watchdog (written by the command task, read by autoStatusTask) */
volatile uint32_t lastFrameMs = 0;
volatile bool     frameSeen   = false;

/* Prescaler 256 -> 62.5 kHz tick: OCR3A = ms * 62.5 - 1 */
void setPidPeriod(uint16_t ms) {
  ms = constrain(ms, 1, 1000);
  uint16_t top = (uint16_t)(((uint32_t)ms * 625UL) / 10UL) - 1;
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = _BV(WGM32) | _BV(CS32);
  OCR3A = top;
  TCNT3 = 0;
  TIMSK3 = _BV(OCIE3A);
  pidPeriodMs = ms;
  interrupts();
}

/*
 * PID maths only. Nothing here touches a motor: the result goes to pid[m].out
 * and signalTask forwards it to the motor queues.
 *  - measurement is always the LOGICAL encoder value (INV ENCx already applied)
 *  - in VEL mode the speed (counts per sample / dt) goes through a first order
 *    low pass (VEL_FILTER_ALPHA) because one count per sample is a coarse step
 *  - anti-windup: the integrator is frozen while the output is saturated and
 *    the error would push it further into saturation
 *  - target reached: integrator reset and output 0. In POS mode this happens at
 *    any target; in VEL mode only when the target is 0 (otherwise the integrator
 *    is what holds the speed).
 *  - output capped to [-lim, +lim] (lim = pwmMax, or CYCLE_MAX_PCT while the
 *    gripper cycle runs) and lifted to pwmMin when non-zero
 */
ISR(TIMER3_COMPA_vect) {
  const float dt = pidPeriodMs * 0.001f;
  const bool  en = loopEnabled;
  const uint8_t mode = ctrlMode;
  const bool  capped = cycleActive;
  pidSeq++;

  for (uint8_t s = 0; s < NUM_MOTORS; s++) {
    PidCtl &c = pid[s];
    int32_t cnt = encCount[s];
    int32_t dlt = (int32_t)((uint32_t)cnt - (uint32_t)c.lastCount);   /* modular, no UB */
    if (dlt > 0x40000000L || dlt < -0x40000000L) dlt = 0;               /* counter rewound to 0 */
    c.vel += VEL_FILTER_ALPHA * (((float)dlt / dt) - c.vel);
    float meas = (mode == MODE_POS) ? (float)cnt : c.vel;
    c.lastCount = cnt;
    c.meas = meas;

    if (!en) { c.integ = 0; c.prevMeas = meas; c.out = 0; c.reached = false; continue; }

    float err = c.target - meas;
    bool reached = fabsf(err) <= tol;
    c.reached = reached;
    if (reached && (mode == MODE_POS || c.target == 0.0f)) {
      c.integ = 0; c.out = 0; c.prevMeas = meas;
      continue;
    }

    float lim = pwmMax[s];
    if (capped && lim > (float)CYCLE_MAX_PCT) lim = (float)CYCLE_MAX_PCT;
    float dTerm = -kd * (meas - c.prevMeas) / dt;   /* derivative on measurement */
    c.prevMeas = meas;

    float iNew = c.integ + err * dt;
    float uTry = kp * err + ki * iNew + dTerm;
    if (!((uTry > lim && err > 0) || (uTry < -lim && err < 0))) c.integ = iNew;

    float u = kp * err + ki * c.integ + dTerm;
    if (u >  lim) u =  lim;
    if (u < -lim) u = -lim;
    float mn = pwmMin[s];
    if (u > 0 && u <  mn) u =  mn;
    if (u < 0 && u > -mn) u = -mn;
    c.out = u;
  }
  if (en) pidFresh = true;
}

void resetPidState() {
  for (uint8_t s = 0; s < NUM_MOTORS; s++) {
    pid[s].integ = 0; pid[s].out = 0; pid[s].reached = false;
    pid[s].prevMeas = pid[s].meas;
  }
}

/*
 * Turns PID outputs into motor signals (motor m <- pid[m]).
 * The outputs are read INSIDE driveMx and loopEnabled is checked there too, so
 * a STOP / LOOP OFF / drive command that ran first can never be followed by a
 * stale PID value.
 */
void signalTask(void *) {
  for (;;) {
    vTaskDelay(1);
    if (!pidFresh) continue;
    lockDrive();
    float out[NUM_MOTORS];
    noInterrupts();
    for (uint8_t m = 0; m < NUM_MOTORS; m++) out[m] = pid[m].out;
    pidFresh = false;
    interrupts();
    if (loopEnabled)
      for (uint8_t m = 0; m < NUM_MOTORS; m++)
        motorSend(m, (uint8_t)(fabsf(out[m]) + 0.5f), out[m] >= 0);
    unlockDrive();
  }
}

/* ------------------------- Timed open-loop motion ------------------------- */
/* motionActive / motionEndMs are only touched with driveMx held. */
int speedPct = 50;
bool     motionActive = false;
uint32_t motionEndMs  = 0;

/* Caller must hold driveMx */
void startMotion(uint32_t durationMs) {
  motionEndMs  = millis() + durationMs;
  motionActive = true;
  if (hMotion) xTaskNotifyGive(hMotion);
}

/* Open-loop differential drive. Caller must NOT hold driveMx. */
void moveDrive(bool leftFwd, bool rightFwd) {
  lockDrive();
  loopEnabled = false;
  motorSend(0, speedPct, rightFwd);   /* A = right */
  motorSend(1, speedPct, leftFwd);    /* B = left  */
  startMotion(2000);
  unlockDrive();
}

/* Stop the drive motors, PID off, no motion pending. Caller must NOT hold driveMx. */
void haltDrive() {
  lockDrive();
  loopEnabled  = false;
  motionActive = false;
  stopDrive();
  unlockDrive();
}

/*
 * Event driven: sleeps until a new motion is started (notification) or until
 * the deadline, whichever comes first. The deadline check and the stop happen
 * under driveMx, so a command that arrives at the same moment is never undone.
 */
void motionTask(void *) {
  for (;;) {
    TickType_t wait = portMAX_DELAY;
    lockDrive();
    if (motionActive) {
      int32_t remain = (int32_t)(motionEndMs - millis());
      if (remain <= 0) {
        motionActive = false;
        if (!loopEnabled) stopDrive();
      } else {
        if (remain > 500) remain = 500;         /* re-check at least every 0.5 s */
        wait = pdMS_TO_TICKS(remain);
        if (wait == 0) wait = 1;
      }
    }
    unlockDrive();
    ulTaskNotifyTake(pdTRUE, wait);
  }
}

/* ------------------------------ Ultrasonic ------------------------------ */
/*
 * Own task (priority 2). One reading every 60 ms; the echo wait is capped at
 * ~2.5 ms in total.
 * Each reading is limited to US_MAX_X10 (25.0 cm): a reading above 25 cm, or no
 * echo at all, is treated as noise / "far" and counts as exactly 25.0 cm.
 * The published value usAvgX10 is the average of the last US_AVG_N readings in
 * tenths of cm (valid once usCount == US_AVG_N). usSeq increments on every new
 * reading, so the gripper can wait for fresh readings after a state change.
 * usRawX10 is the latest single (capped) reading.
 */
constexpr uint8_t  US_AVG_N            = 5;
constexpr uint16_t US_MAX_X10          = 250;    /* 25.0 cm cap */
constexpr uint16_t US_FAR_X10          = 200;    /* 20.0 cm: distance < this = metal on the magnet, >= this = no metal */
constexpr uint8_t  US_CONFIRM_N        = 5;      /* consecutive fresh readings >= 20 cm needed before a drop / failure is declared (5 x 60 ms = 300 ms) */
constexpr uint16_t US_START_TIMEOUT_US = 1000;
constexpr uint16_t US_MAX_ECHO_US      = 3500;
constexpr uint16_t US_PERIOD_MS        = 60;
volatile uint16_t usAvgX10 = US_MAX_X10;
volatile uint16_t usRawX10 = US_MAX_X10;   /* latest single reading (used for the drop confirmation) */
volatile uint32_t usSeq    = 0;
volatile uint8_t  usCount  = 0;

/* Consistent snapshot. Returns true when the average of 5 readings is available. */
static bool usGet(uint16_t &avgX10, uint16_t &rawX10, uint32_t &seq) {
  noInterrupts();
  avgX10 = usAvgX10;
  rawX10 = usRawX10;
  seq    = usSeq;
  const bool ok = usCount >= US_AVG_N;
  interrupts();
  return ok;
}

void ultrasonicTask(void *) {
  uint16_t ring[US_AVG_N] = {0};
  uint8_t  idx = 0, cnt = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(US_PERIOD_MS));
    digitalWrite(US_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN, LOW);
    uint32_t t0 = micros();
    while (!digitalRead(US_ECHO_PIN)) if (micros() - t0 > US_START_TIMEOUT_US) break;
    uint16_t x10 = US_MAX_X10;
    if (digitalRead(US_ECHO_PIN)) {
      uint32_t t1 = micros();
      while (digitalRead(US_ECHO_PIN)) if (micros() - t1 > US_MAX_ECHO_US) break;
      uint32_t w = micros() - t1;
      if (w <= US_MAX_ECHO_US) {
        uint32_t v = (w * 10UL) / 58UL;
        x10 = (v > US_MAX_X10) ? US_MAX_X10 : (uint16_t)v;
      }
    }
    ring[idx] = x10;
    idx = (idx + 1) % US_AVG_N;
    if (cnt < US_AVG_N) cnt++;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < cnt; i++) sum += ring[i];
    const uint16_t avg = (uint16_t)(sum / cnt);
    noInterrupts();
    usAvgX10 = avg;
    usRawX10 = x10;
    usCount  = cnt;
    usSeq++;
    interrupts();
  }
}

/* ------------------------------ Gripper / lift ------------------------------ */
/*
 * ONE state machine (gripPoll, run by gripperTask) for BOTH the manual GRIP
 * command and the "g" token of the ROS frames. No blocking delays: every
 * wait is a millis() deadline, so PID, status messages and sensors keep running.
 *
 *  Power-up: state D (READY). No sanity check at boot.
 *
 *  D --GRIP--> [first GRIP after reset only] SANITY_PWR -> SANITY_UP -> SANITY_DOWN -> D (armed)
 *                                                          (any failure -> FAULT)
 *  D --(armed GRIP + trigger edge < 10 s + distance < 20 cm)--> A
 *  A --(upper limit, distance < 20 cm)--> U --(5 s, distance >= 20 cm confirmed)--> B --(lower limit)--> D
 *  A / U / B failure (metal fell, timeout, metal not dropped) -> recovery:
 *      magnet ON, lift lowered, back to D with the 10 s magnet timeout restarted
 *      (the status letter shows B while lowering).
 *  Lift timeouts during recovery or B, and any sanity failure -> FAULT (F).
 *
 *  MAGNET / LIMIT SWITCHES: the limit switches share their supply with the magnet,
 *  so the magnet is ON in every state in which a limit switch is read:
 *  SANITY_PWR/UP/DOWN, A, B. Only in U (lift stopped, nothing read) is it OFF.
 *
 *  TRIGGER EDGES: metal detector rising edge OR a rising edge of any of the
 *  5 proximity sensors - identical behaviour (see gripPoll).
 *
 * A pin-change ISR cuts the lift PWM the instant the limit switch in the
 * direction of travel goes HIGH; the state machine does the rest.
 * Both switches are on PORTB (D51 = PB2, D53 = PB0).
 */
constexpr uint8_t  LIFT_SPEED_PCT    = 50;
constexpr uint32_t LIFT_TIMEOUT_MS   = 10000;  /* every lift leg, incl. sanity check */
constexpr bool     LIFT_UP_FWD       = true;   /* logical lift direction that raises the gripper (INV LIFT flips the wiring) */
constexpr uint32_t SIREN_ON_MS       = 2000;
constexpr uint32_t MAG_TIMEOUT_MS    = 10000;  /* magnet ON time after a trigger rising edge in D */
constexpr uint32_t METAL_WINDOW_MS   = 10000;  /* a trigger edge counts as "metal near" this long */
constexpr uint32_t GRIP_REQ_MS       = 10000;  /* a GRIP request waits this long for its conditions */
constexpr uint32_t DROP_WAIT_MS      = 5000;   /* U: lift and magnet off, let the metal fall */
constexpr uint32_t DROP_CHECK_MS     = 10000;  /* extra time to see distance >= 20 cm before giving up */
constexpr uint32_t SWITCH_POWER_MS   = 200;    /* after the magnet is switched ON, wait before trusting the limit switches */
constexpr bool     SANITY_EVERY_GRIP = false;  /* false: sanity check only on the first GRIP after a reset */

enum : uint8_t { GS_SANITY_PWR = 0, GS_SANITY_UP, GS_SANITY_DOWN, GS_READY, GS_UP, GS_TOP, GS_DOWN, GS_FAULT };
volatile uint8_t gripState    = GS_READY;
volatile bool    gripCmdReq   = false;         /* GRIP command / ROS "g" edge, consumed by gripPoll in D */
volatile bool    gripStartReq = false;         /* armed request waiting for edge + distance < 20 cm */
uint32_t         gripReqUntilMs = 0;
bool             sanityDone   = false;

static uint8_t   gripMask, gripTopMask, gripBotMask;
volatile bool    gripActive   = false;         /* lift running, ISR armed */
volatile bool    gripGoingUp  = false;
volatile bool    gripDone     = false;         /* ISR: limit of the travel direction reached */
volatile uint8_t gripLastPins = 0;
uint32_t         gripStartMs  = 0;             /* start of the current lift leg */
uint32_t         gripPhaseMs  = 0;             /* start of U / SANITY_PWR */
bool             gripRecovery = false;         /* B is a recovery descent */
bool             gripPaused   = false;         /* B: lift paused because distance < 20 cm */

volatile bool    magOffPending = false;        /* D: magnet OFF due at magOffAtMs */
uint32_t         magOffAtMs    = 0;
uint32_t         magOnMs       = 0;            /* when the magnet was last switched ON (limit switches need SWITCH_POWER_MS after it) */
bool             gripTopPowered = false;       /* U: magnet is back ON after the drop wait, upper switch check pending */
uint32_t         noSwSinceMs   = 0;            /* D: since when no limit switch reads HIGH while the magnet is ON (0 = not the case) */
bool             sirenTimer    = false;
uint32_t         sirenOffAtMs  = 0;

/* trigger rising edges */
volatile bool    mdEdgeLatch   = false;        /* metal detector edge, shown once in the auto status message */
volatile bool    proxEdgeLatch = false;        /* proximity edge (any sensor), included in the next auto status message */
bool             metalSeen     = false;        /* edge seen in D, window runs from metalEdgeMs */
uint32_t         metalEdgeMs   = 0;

/* consecutive-reading counter: fresh readings in a row that are >= 20 cm (metal not on the magnet) */
uint8_t          usFarStreak = 0;
uint8_t          usNearStreak = 0;             /* fresh readings in a row that are < 20 cm (metal on the magnet) */
uint32_t         usStreakSeq = 0;              /* last reading already counted */

void setMagnet(bool on) { if (on && !mgntState) magOnMs = millis(); mgntState = on; digitalWrite(MGNT_PIN,  on ? HIGH : LOW); }
void setSiren(bool on)  { sirenState = on; digitalWrite(SIREN_PIN, on ? HIGH : LOW); }
void setCycleActive(bool on) { cycleActive = on; }

void gripInit() {
  pinMode(LIMIT_SW_TOP, INPUT);        /* add external pull-downs if they float */
  pinMode(LIMIT_SW_BOTTOM, INPUT);
  gripTopMask = digitalPinToBitMask(LIMIT_SW_TOP);
  gripBotMask = digitalPinToBitMask(LIMIT_SW_BOTTOM);
  gripMask = gripTopMask | gripBotMask;
  gripLastPins = PINB & gripMask;
  PCMSK0 |= gripMask;                  /* PORTB bit n = PCINT n */
  PCICR  |= _BV(PCIE0);
}

ISR(PCINT0_vect) {
  uint8_t now = PINB & gripMask;
  gripLastPins = now;
  if (!gripActive) return;
  if (now & (gripGoingUp ? gripTopMask : gripBotMask)) {
    OCR4B = 0;                                   /* immediate stop */
    gripActive = false;
    gripDone = true;
  }
}

static void gripMove(bool up) {
  noInterrupts();
  gripLastPins = PINB & gripMask;
  gripDone = false; gripGoingUp = up; gripActive = true;
  interrupts();
  gripStartMs = millis();
  motorSend(LIFT_IDX, LIFT_SPEED_PCT, up == LIFT_UP_FWD);
}

static void gripHalt() {
  gripActive = false;
  motorSend(LIFT_IDX, 0, true);
}

/* "G" field of the manual STAT print: D down, A moving up, U up, B moving down, F fault */
char gripStatusChar() {
  switch (gripState) {
    case GS_SANITY_UP:   return 'A';
    case GS_SANITY_DOWN: return 'B';
    case GS_UP:          return 'A';
    case GS_TOP:         return 'U';
    case GS_DOWN:        return 'B';
    case GS_FAULT:       return 'F';
    default:             return 'D';
  }
}

/* Gripper letter of the ROS status message: o = open, c = closed, m = moving, f = fault.
   Mapping: lift down / idle = closed (0 rad), lift up (metal released) = open, any travel = moving.
   Swap 'c' and 'o' below if your URDF convention is the opposite. */
char gripRosChar() {
  switch (gripState) {
    case GS_READY: return 'c';
    case GS_TOP:   return 'o';
    case GS_FAULT: return 'f';
    default:       return 'm';
  }
}

/* messages only in manual mode: the automatic status stream must stay clean */
#define GSAY(fmt, ...) do { if (!autoMode) sayf(PSTR(fmt), ##__VA_ARGS__); } while (0)

static bool metalWindow(uint32_t now) { return metalSeen && (now - metalEdgeMs) < METAL_WINDOW_MS; }

/* Lift fault: stop everything, GRIP stays disabled until the Arduino is reset.
   magnetOff: sanity failures switch the magnet OFF (it was only ON to power the switches);
   other faults leave the magnet as it is. */
static void gripFault(bool magnetOff = false) {
  gripHalt();
  setCycleActive(false);
  gripStartReq = false;
  gripCmdReq = false;
  gripPaused = false;
  if (magnetOff) { setMagnet(false); magOffPending = false; }
  gripState = GS_FAULT;
}

/* Back to D. keepMagnet = recovery: magnet stays ON and its 10 s timeout restarts. */
static void gripToReady(bool keepMagnet) {
  gripHalt();
  setCycleActive(false);
  gripRecovery = false;
  gripPaused = false;
  metalSeen = false;
  gripState = GS_READY;
  if (keepMagnet) {
    setMagnet(true);
    magOffAtMs = millis() + MAG_TIMEOUT_MS;
    magOffPending = true;
  } else {
    setMagnet(false);
    magOffPending = false;
  }
}

/* Failure after the cycle started: lower the lift with the magnet ON, then return to D */
static void gripRecover() {
  gripHalt();
  setMagnet(true);
  magOffPending = false;
  gripRecovery = true;
  gripPaused = false;
  gripState = GS_DOWN;
  gripMove(false);
}

/* Start of the sanity check: the magnet goes ON FIRST (it powers the limit switches) */
static void gripBeginSanity(uint32_t now) {
  setCycleActive(true);
  magOffPending = false;
  setMagnet(true);
  gripPhaseMs = now;
  gripState = GS_SANITY_PWR;
  GSAY("GRIP: sanity check, magnet ON, powering limit switches");
}

/* Called continuously from gripperTask */
void gripPoll() {
  const uint32_t now = millis();
  const bool top = digitalRead(LIMIT_SW_TOP) == HIGH;
  const bool bot = digitalRead(LIMIT_SW_BOTTOM) == HIGH;
  uint16_t avg = 0, raw = 0; uint32_t seq = 0;
  const bool usOk = usGet(avg, raw, seq);

  /* Trigger sources: metal detector RISING edge and RISING edge of any of the
     5 proximity sensors. Both have the same role. In D an edge turns the magnet
     ON for 10 s (a new edge restarts the time), sounds the siren and opens the
     10 s "metal near" window. */
  static int8_t  mdWas = -1;
  static uint8_t proxWas = 0;
  static bool    proxInit = false;
  const bool md = digitalRead(MD_PIN) == MD_METAL_LEVEL;
  if (mdWas < 0) mdWas = md;
  uint8_t pm = 0;
  for (uint8_t i = 0; i < NUM_PROX; i++)
    if (digitalRead(PROX_PINS[i]) == PROX_ACTIVE_LEVEL) pm |= _BV(i);
  if (!proxInit) { proxWas = pm; proxInit = true; }
  const bool mdRise = md && !mdWas;
  const bool pxRise = (pm & (uint8_t)~proxWas) != 0;
  mdWas = md;
  proxWas = pm;
  if (mdRise) mdEdgeLatch = true;                /* reported once in the auto status */
  if (pxRise) proxEdgeLatch = true;
  if ((mdRise || pxRise) && gripState == GS_READY) {
    metalSeen = true;
    metalEdgeMs = now;
    setMagnet(true);
    magOffAtMs = now + MAG_TIMEOUT_MS;
    magOffPending = true;
    setSiren(true);
    sirenOffAtMs = now + SIREN_ON_MS;
    sirenTimer = true;
  }
  if (sirenTimer && (int32_t)(now - sirenOffAtMs) >= 0) { setSiren(false); sirenTimer = false; }

  /* Count consecutive fresh readings >= 20 cm. A single near reading resets the count,
     so one noisy / missing echo can never declare a drop or a failure by itself. */
  if (usOk && seq != usStreakSeq) {
    usStreakSeq = seq;
    if (raw >= US_FAR_X10) { if (usFarStreak < 255) usFarStreak++; usNearStreak = 0; }
    else                   { if (usNearStreak < 255) usNearStreak++; usFarStreak = 0; }
  }
  const bool metalOnMagnet = usOk && avg < US_FAR_X10;                     /* < 20 cm (average of 5) */
  const bool metalNear     = usOk && usNearStreak >= US_CONFIRM_N;         /* < 20 cm, confirmed by several readings */
  const bool metalGone     = usOk && usFarStreak >= US_CONFIRM_N;          /* >= 20 cm, confirmed */

  /* requests are only valid in D */
  if (gripState != GS_READY) { gripStartReq = false; gripCmdReq = false; }

  switch (gripState) {
    case GS_SANITY_PWR:                          /* magnet is ON, wait for the switch supply, then check */
      if (now - gripPhaseMs < SWITCH_POWER_MS) break;
      if (!bot || top) {
        GSAY("GRIP: sanity FAILED (lower switch must be HIGH and upper LOW at start), GRIP disabled");
        gripFault(true);
      } else {
        gripState = GS_SANITY_UP;
        gripMove(true);
        GSAY("GRIP: sanity check, going up");
      }
      break;

    case GS_SANITY_UP:                           /* magnet stays ON */
      if (gripDone || top) {
        gripHalt();
        gripState = GS_SANITY_DOWN;
        gripMove(false);
        GSAY("GRIP: sanity check, going down");
      } else if (now - gripStartMs > LIFT_TIMEOUT_MS) {
        GSAY("GRIP: sanity FAILED (upper switch timeout), GRIP disabled");
        gripFault(true);
      }
      break;

    case GS_SANITY_DOWN:                         /* magnet stays ON */
      if (gripDone || bot) {
        gripToReady(false);                      /* magnet OFF, back to D */
        sanityDone = true;
        gripReqUntilMs = now + GRIP_REQ_MS;      /* now arm the GRIP request that triggered the check */
        gripStartReq = true;
        GSAY("GRIP: sanity OK, armed, waiting for trigger edge + metal (< 20 cm)");
      } else if (now - gripStartMs > LIFT_TIMEOUT_MS) {
        GSAY("GRIP: sanity FAILED (lower switch timeout), GRIP disabled");
        gripFault(true);
      }
      break;

    case GS_READY:                               /* D */
      if (magOffPending && (int32_t)(now - magOffAtMs) >= 0) {
        setMagnet(false);                        /* 10 s after the last trigger edge */
        magOffPending = false;
      }
      /* General rule: magnet ON (limit switches powered), NO limit switch HIGH and distance >= 20 cm
         -> the lift is somewhere in between: move down until the lower switch is HIGH. */
      if (mgntState && !top && !bot && (now - magOnMs) >= SWITCH_POWER_MS) {
        if (!noSwSinceMs) noSwSinceMs = now;
        if ((now - noSwSinceMs) >= SWITCH_POWER_MS && metalGone) {
          noSwSinceMs = 0;
          gripStartReq = false; gripCmdReq = false;
          setCycleActive(true);
          GSAY("GRIP: no limit switch HIGH, going down to the lower switch");
          gripRecover();                         /* magnet ON, lift down, back to D (10 s magnet timeout restarts) */
          break;
        }
      } else {
        noSwSinceMs = 0;
      }
      if (gripCmdReq) {
        gripCmdReq = false;
        if (SANITY_EVERY_GRIP || !sanityDone) {
          gripBeginSanity(now);                  /* sanity check first (magnet ON), request is armed afterwards */
          break;
        }
        gripReqUntilMs = now + GRIP_REQ_MS;
        gripStartReq = true;
      }
      if (gripStartReq) {
        if ((int32_t)(now - gripReqUntilMs) >= 0) {
          gripStartReq = false;
          GSAY("GRIP: request expired");
        } else if (metalNear && metalWindow(now)) {        /* distance < 20 cm (several readings) = metal on the magnet */
          gripStartReq = false;
          magOffPending = false;
          setMagnet(true);                       /* ON before the lift moves: powers the limit switches */
          setCycleActive(true);
          usFarStreak = 0; usNearStreak = 0; usStreakSeq = seq;   /* count only readings taken while lifting */
          gripState = GS_UP;
          gripMove(true);
          GSAY("GRIP: moving up");
        }
      }
      break;

    case GS_UP:                                  /* A, magnet ON */
      if (metalGone) {                           /* >= 20 cm for US_CONFIRM_N readings in a row */
        GSAY("GRIP: metal fell, recovering");
        gripRecover();
      } else if (gripDone || top) {              /* upper limit reached */
        gripHalt();
        if (metalOnMagnet && usFarStreak == 0) { /* < 20 cm and no pending far reading: metal is held */
          setMagnet(false);                      /* drop the metal (no switch is read in U) */
          gripPhaseMs = now;
          usFarStreak = 0; usNearStreak = 0; usStreakSeq = seq;   /* drop confirmation starts fresh */
          gripTopPowered = false;
          gripState = GS_TOP;
          GSAY("GRIP: top, releasing");
        } else if (now - gripStartMs > LIFT_TIMEOUT_MS) {
          GSAY("GRIP: metal not confirmed at top, recovering");
          gripRecover();
        }                                        /* else: readings not conclusive yet, wait */
      } else if (now - gripStartMs > LIFT_TIMEOUT_MS) {
        GSAY("GRIP: timeout going up, recovering");
        gripRecover();
      }
      break;

    case GS_TOP:                                 /* U: lift stopped, magnet OFF for the drop */
      if (!gripTopPowered) {
        if (now - gripPhaseMs >= DROP_WAIT_MS) {
          if (metalGone) {                       /* >= 20 cm for US_CONFIRM_N readings in a row: metal dropped */
            setMagnet(true);                     /* magnet back ON: powers the limit switches */
            gripTopPowered = true;
            GSAY("GRIP: metal dropped, magnet ON, checking upper switch");
          } else if (now - gripPhaseMs >= DROP_WAIT_MS + DROP_CHECK_MS) {
            GSAY("GRIP: metal not dropped, recovering");
            gripRecover();
          }
        }
      } else if (now - magOnMs >= SWITCH_POWER_MS) {
        if (top) {                               /* still on the upper switch: go down */
          gripState = GS_DOWN;
          gripRecovery = false;
          gripPaused = false;
          gripMove(false);
          GSAY("GRIP: lowering");
        } else {
          GSAY("GRIP: upper switch not HIGH after the drop, recovering");
          gripRecover();
        }
      }
      break;

    case GS_DOWN:                                /* B, magnet ON */
      if (gripDone || bot) {                     /* lower limit */
        if (gripRecovery) { gripToReady(true);  GSAY("GRIP: recovered, ready"); }
        else              { gripToReady(false); GSAY("GRIP: done, ready"); }
      } else if (now - gripStartMs > LIFT_TIMEOUT_MS) {
        GSAY("GRIP: timeout going down, FAULT, GRIP disabled");
        gripFault();
      } else if (!gripRecovery && usOk) {        /* move only while distance >= 20 cm */
        if (avg < US_FAR_X10 && !gripPaused) {
          gripHalt();
          gripPaused = true;
        } else if (avg >= US_FAR_X10 && gripPaused) {
          const uint32_t s = gripStartMs;        /* keep the leg timeout running */
          gripMove(false);
          gripStartMs = s;
          gripPaused = false;
        }
      }
      break;

    case GS_FAULT:
    default:
      break;
  }
}

void gripperTask(void *) {
  for (;;) {
    gripPoll();
    taskYIELD();
  }
}

/* ------------------------------ Telemetry ------------------------------ */
volatile uint16_t telPeriodMs = 0;

void telemetryTask(void *) {
  for (;;) {
    uint16_t p = telPeriodMs;
    if (!p) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    TickType_t t = pdMS_TO_TICKS(p);
    vTaskDelay(t ? t : 1);
    float tg[2], me[2], ou[2];
    noInterrupts();
    for (uint8_t s = 0; s < NUM_MOTORS; s++) { tg[s] = pid[s].target; me[s] = pid[s].meas; ou[s] = pid[s].out; }
    interrupts();
    SAY("T,%lu,%ld,%ld,%d,%ld,%ld,%d", (unsigned long)millis(),
        (long)tg[0], (long)me[0], (int)ou[0], (long)tg[1], (long)me[1], (int)ou[1]);
  }
}

/* ------------------------- Automatic status message ------------------------- */
/*
 * Format read by rover_interface.cpp:
 *    rp12.34,ln05.00,p1,m0,gc,
 *
 * When ROS frames are arriving (rosSync) ONE line is sent per received frame
 * (statusReq, set by handleRosFrame): the ROS read() takes one line per cycle,
 * so the two sides stay in lock-step and nothing queues up. Without ROS frames
 * (typed AUTOM) one line is sent per PID sample (pidSeq).
 * The ROS watchdog runs on every PID sample in both cases.
 */
/* counts/s -> sign letter + rad/s with 2 decimals (ip.fp) */
static void splitVel(float cps, char &sg, long &ip, long &fp) {
  long x = lroundf(fabsf(cpsToRads(cps)) * 100.0f);
  sg = (cps < 0.0f && x != 0) ? 'n' : 'p';
  ip = x / 100;
  fp = x % 100;
}

void autoStatusTask(void *) {
  uint8_t last = pidSeq;
  for (;;) {
    if (!autoMode) {
      mdEdgeLatch = false; proxEdgeLatch = false; statusReq = false;
      vTaskDelay(pdMS_TO_TICKS(50));
      last = pidSeq;
      continue;
    }
    const bool tick = (pidSeq != last);
    const bool due  = rosSync ? statusReq : tick;
    if (!tick && !due) { taskYIELD(); continue; }

    if (tick) {
      last = pidSeq;
      uint32_t lf; bool fs;
      noInterrupts(); lf = lastFrameMs; fs = frameSeen; interrupts();
      /* ROS watchdog: frames stopped -> stop the wheels */
      if (fs && (uint32_t)(millis() - lf) > FRAME_TIMEOUT_MS) {
        noInterrupts();
        for (uint8_t m = 0; m < NUM_MOTORS; m++) pid[m].target = 0;
        interrupts();
      }
    }
    if (!due) continue;

    statusReq = false;                               /* a frame arriving from now on asks for a new line */
    float vr, vl;
    bool mdEdge, pxEdge;
    noInterrupts();
    vr = pid[0].meas; vl = pid[1].meas;
    mdEdge = mdEdgeLatch;  mdEdgeLatch = false;      /* edge since the last message, then cleared */
    pxEdge = proxEdgeLatch; proxEdgeLatch = false;
    interrupts();

    bool pxLevel = pxEdge;
    for (uint8_t i = 0; i < NUM_PROX; i++)
      if (digitalRead(PROX_PINS[i]) == PROX_ACTIVE_LEVEL) pxLevel = true;

    char sr, sl; long ri, rf, li, lfr;
    splitVel(vr, sr, ri, rf);
    splitVel(vl, sl, li, lfr);
    SAY("r%c%ld.%02ld,l%c%ld.%02ld,p%c,m%c,g%c,",
        sr, ri, rf, sl, li, lfr,
        pxLevel ? '1' : '0', mdEdge ? '1' : '0', gripRosChar());
  }
}

/* ----------------------------- UART parser ----------------------------- */
constexpr uint8_t CMD_BUF_LEN = 64;

/* Strict number parsing: whole token must be a finite number ("abc" is an error, not 0) */
static bool parseF(const char *s, float &v) {
  if (!s || !*s) return false;
  char *e;
  double d = strtod(s, &e);
  if (e == s || *e || !isfinite(d)) return false;
  v = (float)d;
  return true;
}
static bool parseI(const char *s, long &v) {
  if (!s || !*s) return false;
  char *e;
  long l = strtol(s, &e, 10);
  if (e == s || *e) return false;
  v = l;
  return true;
}

int motorIdx(const char *s, bool allowLift = false) {
  if (!s) return -1;
  if (allowLift && !strcmp(s, "LIFT")) return LIFT_IDX;
  if (!s[0] || s[1]) return -1;
  char c = s[0];
  if (c >= 'A' && c < 'A' + NUM_MOTORS) return c - 'A';
  if (c >= '1' && c < '1' + NUM_MOTORS) return c - '1';
  return -1;
}

int encIdx(const char *s) {
  if (!s) return -1;
  if (!strncmp(s, "ENC", 3)) s += 3;
  if (!s[0] || s[1]) return -1;
  if (s[0] == 'A' || s[0] == '1') return 0;
  if (s[0] == 'B' || s[0] == '2') return 1;
  return -1;
}

void printInv() {
  SAY("INV: A=%d B=%d LIFT=%d ENCA=%d ENCB=%d",
      motorInv[0], motorInv[1], motorInv[LIFT_IDX], encFlip[0], encFlip[1]);
}

void printGains() {
  char a[14], b[14], c[14];
  dtostrf(kp, 1, 3, a); dtostrf(ki, 1, 3, b); dtostrf(kd, 1, 3, c);
  SAY("CFG: mode=%s loop=%d period=%u P=%s I=%s D=%s",
      ctrlMode == MODE_POS ? "POS" : "VEL", loopEnabled, pidPeriodMs, a, b, c);
}

void printLimits() {
  char t[14];
  dtostrf(tol, 1, 2, t);
  SAY("LIM: tol=%s max=%d,%d min=%d,%d", t,
      (int)pwmMax[0], (int)pwmMax[1], (int)pwmMin[0], (int)pwmMin[1]);
}

void printStat() {
  printInv(); printGains(); printLimits();
  SAY("RTOS stack free: cmd=%u tel=%u sig=%u mot=%u mtr=%u",
      (unsigned)uxTaskGetStackHighWaterMark(hCmd),
      (unsigned)uxTaskGetStackHighWaterMark(hTel),
      (unsigned)uxTaskGetStackHighWaterMark(hSignal),
      (unsigned)uxTaskGetStackHighWaterMark(hMotion),
      (unsigned)uxTaskGetStackHighWaterMark(hMotor));
  uint16_t ud, ur; uint32_t sq;
  usGet(ud, ur, sq);
  SAY("GRIP st=%c sanity=%d US=%u.%u cm stack grip=%u us=%u auto=%u",
      gripStatusChar(), sanityDone, (unsigned)(ud / 10), (unsigned)(ud % 10),
      (unsigned)uxTaskGetStackHighWaterMark(hGrip),
      (unsigned)uxTaskGetStackHighWaterMark(hUs),
      (unsigned)uxTaskGetStackHighWaterMark(hAuto));
}

void printHelp() {
  SAY("W/A/S/D | SPEED xx | MOVE x s | STOP | GRIP | PROX | MGNT | SIREN | MD");
  SAY("PID ms | P x | I x | D x | MODE POS/VEL | TGT A/B/ALL v");
  SAY("LOOP ON/OFF | TOL v | PWMMAX v [A/B] | PWMMIN v [A/B] | ZERO | RESET");
  SAY("INV A/B/LIFT/ENCA/ENCB | PPR A/B | ENCDBG | TEL ms | STAT");
  SAY("AUTOM xx|OFF  (xx = PID ms, default %u)", (unsigned)AUTO_DEFAULT_MS);
  SAY("ROS frame in : see header (r l g s tokens)");
  SAY("status out   : see header (r l p m g tokens)");
}

/*
 * RESET: stop everything, re-initialise the encoder pins, zero the encoders,
 * targets and PID state, and restore the default inversions.
 * PID gains, period, mode, TOL and PWMMAX/PWMMIN are left untouched.
 */
void resetAll() {
  haltDrive();

  /* restore default inversions */
  for (uint8_t i = 0; i < NUM_CH; i++) motorInv[i] = DEF_MOTOR_INV[i];
  noInterrupts();
  for (uint8_t s = 0; s < 2; s++) encFlip[s] = DEF_ENC_FLIP[s];
  interrupts();

  encInit();                               /* pull-ups, zero counts, re-arm INT0..INT3 */

  noInterrupts();
  for (uint8_t s = 0; s < NUM_MOTORS; s++) {
    pid[s].lastCount = 0; pid[s].target = 0; pid[s].meas = 0; pid[s].vel = 0;
  }
  resetPidState();
  interrupts();
}

/*
 * Enter auto mode: velocity PID, loaded with the AUTO_* parameters from the top of
 * this file (gains, error margin, min / max duty), telemetry off.
 * Used by AUTOM and by the first ROS frame.
 */
void startAuto(uint16_t ms, bool announce) {
  autoMode  = false;
  rosSync   = false;                       /* back to "one line per PID sample" until ROS frames arrive */
  statusReq = false;
  telPeriodMs = 0;                         /* telemetry lines would only pollute the ROS stream */
  haltDrive();
  setPidPeriod(ms);                        /* PID period = sampling time */
  noInterrupts();
  ctrlMode = MODE_VEL;                     /* targets are counts/s */
  kp  = AUTO_KP;  ki = AUTO_KI;  kd = AUTO_KD;
  tol = AUTO_TOL_CPS;
  for (uint8_t s = 0; s < NUM_MOTORS; s++) {
    pwmMax[s] = AUTO_PWM_MAX_PCT;
    pwmMin[s] = AUTO_PWM_MIN_PCT;
    pid[s].target = 0;
    pid[s].vel = 0;
    pid[s].meas = 0;
    pid[s].lastCount = encCount[s];
  }
  resetPidState();
  frameSeen = false;
  interrupts();
  lockDrive();
  stopDrive();
  noInterrupts(); resetPidState(); loopEnabled = true; interrupts();
  unlockDrive();
  if (announce) SAY("AUTOM = %u ms", (unsigned)ms);
  autoMode = true;
}

/* ---- ROS input frame: rp12.34,lp12.34,gp00.00,sp00.00, (tokens arrive upper-cased) ---- */
static bool rosTok(const char *t, char key, float &v) {
  if (t[0] != key) return false;
  if (t[1] != 'P' && t[1] != 'N') return false;
  float x;
  if (!parseF(t + 2, x)) return false;
  v = (t[1] == 'N') ? -x : x;
  return true;
}

void handleRosFrame(char *tok[], uint8_t nt) {
  static bool  gripPrev  = false;
  static bool  servoInit = false;
  static float angle     = 90.0f;

  float r = 0, l = 0, g = 0, s = 0, v;
  bool hr = false, hl = false, hg = false, hs = false;
  for (uint8_t i = 0; i < nt; i++) {
    const char *t = tok[i];
    if      (rosTok(t, 'R', v)) { r = v; hr = true; }
    else if (rosTok(t, 'L', v)) { l = v; hl = true; }
    else if (rosTok(t, 'G', v)) { g = v; hg = true; }
    else if (rosTok(t, 'S', v)) { s = v; hs = true; }
  }
  if (!hr || !hl) { SAY("ERR: bad frame (r and l tokens required)"); return; }

  if (!autoMode) startAuto(AUTO_DEFAULT_MS, false);   /* first ROS frame switches auto mode on */

  const uint32_t now = millis();
  uint32_t dtm = frameSeen ? (uint32_t)(now - lastFrameMs) : 0;
  if (dtm > 100) dtm = 100;

  /* wheels: rad/s -> counts/s with PPR = 2400 (A = right, B = left) */
  float t[2] = { radsToCps(r), radsToCps(l) };
  noInterrupts();
  for (uint8_t m = 0; m < NUM_MOTORS; m++) {
    t[m] = constrain(t[m], -AUTO_MAX_TARGET_CPS, AUTO_MAX_TARGET_CPS);
    if (t[m] == 0.0f || t[m] * pid[m].target < 0.0f) pid[m].integ = 0;
    pid[m].target = t[m];
  }
  lastFrameMs = now;
  frameSeen = true;
  interrupts();

  /* camera servo: direction -1/0/+1 integrated into an angle */
  if (!servoInit) {
    servoInit = true;
    motorSend(SERVO_IDX, (uint8_t)angle, true);       /* centre on the first frame */
  }
  if (hs && fabsf(s) > 0.05f && dtm) {
    s = constrain(s, -1.0f, 1.0f);
    angle += s * SERVO_RATE_DPS * (float)dtm * 0.001f;
    angle = constrain(angle, 0.0f, 180.0f);
    motorSend(SERVO_IDX, (uint8_t)(angle + 0.5f), true);
  }

  /* gripper: rising edge of a non-zero command = GRIP request */
  const bool gOn = hg && fabsf(g) > 0.001f;
  if (gOn && !gripPrev && gripState == GS_READY) gripCmdReq = true;
  gripPrev = gOn;

  /* ROS is talking: answer this frame with exactly one status line */
  rosSync   = true;
  statusReq = true;
}

void processLine(char *line) {
  for (char *p = line; *p; p++) *p = toupper((unsigned char)*p);
  char *tok[4] = {NULL, NULL, NULL, NULL};
  uint8_t nt = 0;
  for (char *p = strtok(line, " ,\t"); p && nt < 4; p = strtok(NULL, " ,\t")) tok[nt++] = p;
  if (!nt) return;
  const char *t0 = tok[0];
  float fv;
  long  lv;

  /* ---- single-letter drive commands ---- */
  if (nt == 1 && !strcmp(t0, "W")) { moveDrive(true, true);   SAY("Forward 2 s");  return; }
  if (nt == 1 && !strcmp(t0, "S")) { moveDrive(false, false); SAY("Backward 2 s"); return; }
  if (nt == 1 && !strcmp(t0, "A")) { moveDrive(false, true);  SAY("Turn left 2 s"); return; }
  if (nt == 1 && !strcmp(t0, "D")) { moveDrive(true, false);  SAY("Turn right 2 s"); return; }

  /* ---- ROS frame, e.g. rp12.34,lp12.34,gp00.00,sp00.00, ---- */
  if ((t0[0] == 'R' || t0[0] == 'L') && (t0[1] == 'P' || t0[1] == 'N') &&
      (isdigit((unsigned char)t0[2]) || t0[2] == '.')) {
    handleRosFrame(tok, nt);
    return;
  }

  /* ---- gains: "P 2.5", "I 3.4", "D 0.1" ---- */
  if (nt == 2 && (!strcmp(t0, "P") || !strcmp(t0, "I") || !strcmp(t0, "D"))) {
    if (!parseF(tok[1], fv) || fv < 0) { SAY("ERR: gain must be a number >= 0"); return; }
    noInterrupts();
    if (t0[0] == 'P') kp = fv; else if (t0[0] == 'I') ki = fv; else kd = fv;
    interrupts();
    printGains();
    return;
  }

  if (!strcmp(t0, "PID")) {
    if (!parseI(tok[1], lv) || lv < 1 || lv > 1000) { SAY("ERR: PID XX, XX = 1..1000 ms"); return; }
    setPidPeriod((uint16_t)lv);
    printGains();

  } else if (!strcmp(t0, "AUTOM")) {
    if (nt >= 2 && !strcmp(tok[1], "OFF")) {
      autoMode = false;
      rosSync = false;
      haltDrive();
      SAY("AUTOM = OFF");
      return;
    }
    lv = AUTO_DEFAULT_MS;                        /* default sampling time */
    if (nt >= 2 && (!parseI(tok[1], lv) || lv < 1 || lv > 1000)) { SAY("ERR: AUTOM XX|OFF, XX = 1..1000 ms"); return; }
    startAuto((uint16_t)lv, true);

  } else if (!strcmp(t0, "TGT")) {
    int e = (nt >= 3) ? (!strcmp(tok[1], "ALL") ? 2 : encIdx(tok[1])) : -1;
    if (e < 0 || !parseF(tok[2], fv)) { SAY("ERR: TGT A|B|ALL value"); return; }
    noInterrupts();
    for (uint8_t s = 0; s < NUM_MOTORS; s++)
      if (e == 2 || e == s) { pid[s].target = fv; pid[s].integ = 0; }
    interrupts();
    SAY("TGT %s = %ld", tok[1], (long)fv);

  } else if (!strcmp(t0, "LOOP")) {
    bool on = (nt >= 2) && !strcmp(tok[1], "ON");
    if (!on && !(nt >= 2 && !strcmp(tok[1], "OFF"))) { SAY("ERR: LOOP ON|OFF"); return; }
    if (on) {
      lockDrive();
      motionActive = false;
      stopDrive();                               /* drop any open-loop command still running */
      noInterrupts(); resetPidState(); loopEnabled = true; interrupts();
      unlockDrive();
    } else {
      haltDrive();
    }
    SAY("LOOP = %d", loopEnabled);

  } else if (!strcmp(t0, "MODE")) {
    if (nt < 2 || (strcmp(tok[1], "POS") && strcmp(tok[1], "VEL"))) { SAY("ERR: MODE POS|VEL"); return; }
    haltDrive();
    noInterrupts();
    ctrlMode = !strcmp(tok[1], "POS") ? MODE_POS : MODE_VEL;
    for (uint8_t s = 0; s < NUM_MOTORS; s++) { pid[s].target = 0; pid[s].lastCount = encCount[s]; }
    resetPidState();
    interrupts();
    printGains();

  } else if (!strcmp(t0, "TOL")) {
    if (!parseF(tok[1], fv) || fv < 0) { SAY("ERR: TOL v >= 0"); return; }
    noInterrupts(); tol = fv; interrupts();
    printLimits();

  } else if (!strcmp(t0, "PWMMAX") || !strcmp(t0, "PWMMIN")) {
    bool isMax = !strcmp(t0, "PWMMAX");
    int e = tok[2] ? encIdx(tok[2]) : 2;
    if (!parseF(tok[1], fv) || fv < 0 || fv > 100 || e < 0) { SAY("ERR: %s 0..100 [A|B]", t0); return; }
    for (uint8_t s = 0; s < NUM_MOTORS; s++) {
      if (e != 2 && e != s) continue;
      if (isMax && fv < pwmMin[s]) { SAY("ERR: max < min"); return; }
      if (!isMax && fv > pwmMax[s]) { SAY("ERR: min > max"); return; }
    }
    noInterrupts();
    for (uint8_t s = 0; s < NUM_MOTORS; s++)
      if (e == 2 || e == s) { if (isMax) pwmMax[s] = fv; else pwmMin[s] = fv; }
    interrupts();
    printLimits();

  } else if (!strcmp(t0, "ZERO")) {
    noInterrupts();
    for (uint8_t s = 0; s < NUM_MOTORS; s++) {
      encCount[s] = 0;
      pid[s].lastCount = 0; pid[s].target = 0; pid[s].meas = 0; pid[s].vel = 0;
    }
    resetPidState();
    interrupts();
    SAY("Encoders and targets zeroed");

  } else if (!strcmp(t0, "RESET")) {
    resetAll();
    SAY("RESET done");
    printInv();

  } else if (!strcmp(t0, "INV")) {
    if (nt < 2) { SAY("ERR: INV A|B|LIFT|ENCA|ENCB"); return; }
    if (!strncmp(tok[1], "ENC", 3)) {
      int e = encIdx(tok[1]);
      if (e < 0) { SAY("ERR: INV ENCA|ENCB"); return; }
      noInterrupts();
      encFlip[e] = !encFlip[e];
      encCount[e] = (encCount[e] == INT32_MIN) ? 0 : -encCount[e];   /* keep position consistent */
      pid[e].lastCount = encCount[e];
      pid[e].vel = 0;
      pid[e].integ = 0;
      interrupts();
      SAY("INV ENC%c = %d", 'A' + e, encFlip[e]);
    } else {
      int m = motorIdx(tok[1], true);
      if (m < 0) { SAY("ERR: INV A|B|LIFT"); return; }
      motorInv[m] = !motorInv[m];              /* DIR pin only, PID unaffected */
      if (m == LIFT_IDX) SAY("INV LIFT = %d", motorInv[m]);
      else SAY("INV %c = %d", 'A' + m, motorInv[m]);
    }

  } else if (!strcmp(t0, "PPR")) {
    int e = encIdx(tok[1]);
    if (e < 0) { SAY("ERR: PPR A|B"); return; }
    noInterrupts(); int32_t v = encCount[e]; interrupts();
    SAY("ENC%c = %ld", 'A' + e, (long)v);

  } else if (!strcmp(t0, "ENCDBG")) {
    /* raw pin levels + interrupt enable mask: turn a wheel by hand and repeat */
    uint8_t pd = PIND;
    SAY("ENCDBG: D19=%d D18=%d D21=%d D20=%d EIMSK=%02X",
        (pd >> 2) & 1, (pd >> 3) & 1, pd & 1, (pd >> 1) & 1, EIMSK);

  } else if (!strcmp(t0, "SPEED")) {
    if (!parseI(tok[1], lv) || lv < 0 || lv > 100) { SAY("ERR: SPEED XX, XX = 0..100"); return; }
    speedPct = (int)lv;
    SAY("Speed = %d %%", speedPct);

  } else if (!strcmp(t0, "MOVE") || (!strncmp(t0, "MOVE", 4) && t0[4] && !t0[5])) {
    /* "MOVE B 1.5" or "MOVEB 1.5" */
    const char *mt, *tt;
    if (t0[4]) { mt = t0 + 4; tt = tok[1]; } else { mt = tok[1]; tt = tok[2]; }
    int m = motorIdx(mt);
    if (m < 0 || !parseF(tt, fv) || fv <= 0.0f || fv > 30.0f) { SAY("ERR: MOVE X FF (X=A|B, FF=0..30 s)"); return; }
    lockDrive();
    loopEnabled = false;
    stopDrive();
    motorSend(m, MOVE_PCT, true);
    startMotion((uint32_t)(fv * 1000.0f));
    unlockDrive();
    SAY("Motor %c @%d%% for %d ds", 'A' + m, MOVE_PCT, (int)(fv * 10.0f));

  } else if (!strcmp(t0, "STOP")) {
    haltDrive();
    SAY("STOP");

  } else if (!strcmp(t0, "GRIP")) {
    if (gripState == GS_FAULT) {
      SAY("ERR: GRIP unavailable (lift fault), reset the Arduino");
    } else if (gripState != GS_READY) {
      SAY("GRIP ignored: cycle in progress");
    } else {
      gripCmdReq = true;                         /* handled by gripPoll: sanity check first if not done yet */
      if (SANITY_EVERY_GRIP || !sanityDone)
        SAY("GRIP: sanity check first (magnet ON), then armed");
      else
        SAY("GRIP: armed, waiting for trigger edge + metal (< 20 cm)");
    }

  } else if (!strcmp(t0, "PROX")) {
    SAY("PROX: P23=%d P25=%d P27=%d P29=%d P31=%d",
        digitalRead(PROX_PINS[0]), digitalRead(PROX_PINS[1]),
        digitalRead(PROX_PINS[2]), digitalRead(PROX_PINS[3]),
        digitalRead(PROX_PINS[4]));

  } else if (!strcmp(t0, "MGNT")) {
    if (gripState != GS_READY) { SAY("ERR: magnet owned by the gripper cycle"); return; }
    magOffPending = false;
    setMagnet(!mgntState);
    SAY("MGNT = %d", mgntState);

  } else if (!strcmp(t0, "SIREN")) {
    sirenState = !sirenState;
    digitalWrite(SIREN_PIN, sirenState ? HIGH : LOW);
    SAY("SIREN = %d", sirenState);

  } else if (!strcmp(t0, "MD")) {
    int lvl = digitalRead(MD_PIN);
    SAY("MD: %s (pin %d = %d)", lvl == MD_METAL_LEVEL ? "METAL" : "NONE", MD_PIN, lvl);

  } else if (!strcmp(t0, "TEL")) {
    lv = 0;
    if (tok[1] && !parseI(tok[1], lv)) { SAY("ERR: TEL ms (0 = off)"); return; }
    telPeriodMs = (lv <= 0) ? 0 : (uint16_t)constrain(lv, 30, 5000);
    SAY("TEL = %u ms", telPeriodMs);

  } else if (!strcmp(t0, "STAT")) {
    printStat();

  } else if (!strcmp(t0, "HELP")) {
    printHelp();

  } else {
    SAY("ERR: unknown command: %s", t0);
  }
}

/*
 * Receives and parses the serial lines.
 *  - The hardware RX buffer is only 64 bytes (~5.5 ms at 115200). The task does
 *    not sleep for a whole RTOS tick between reads any more (that was 15 ms with
 *    the library's default tick); it drains the buffer and yields, so other
 *    ready tasks still run but characters are picked up within microseconds.
 *  - A line longer than the buffer is discarded up to its newline (before, the
 *    tail of an over-long line was executed as a new command).
 */
void commandTask(void *) {
  static char buf[CMD_BUF_LEN];
  uint8_t len = 0;
  bool overflow = false;
  for (;;) {
    while (Serial.available() > 0) {
      int r = Serial.read();
      if (r < 0) break;
      char c = (char)r;
      if (c == '\n' || c == '\r') {
        if (overflow)  SAY("ERR: line too long (max %d)", CMD_BUF_LEN - 1);
        else if (len)  { buf[len] = 0; processLine(buf); }
        len = 0;
        overflow = false;
      } else if (overflow) {
        /* skip the rest of the line */
      } else if (len < CMD_BUF_LEN - 1) {
        buf[len++] = c;
      } else {
        overflow = true;
      }
    }
    taskYIELD();
  }
}

/* ------------------------------ Arduino ------------------------------- */
void setup() {
  Serial.begin(115200);
  for (uint8_t i = 0; i < NUM_PROX; i++) pinMode(PROX_PINS[i], INPUT);
  pinMode(MD_PIN, INPUT);
  pinMode(US_TRIG_PIN, OUTPUT); digitalWrite(US_TRIG_PIN, LOW);
  pinMode(US_ECHO_PIN, INPUT);
  pinMode(MGNT_PIN, OUTPUT);   digitalWrite(MGNT_PIN, LOW);
  pinMode(SIREN_PIN, OUTPUT);  digitalWrite(SIREN_PIN, LOW);
  pwmInit();
  encInit();
  gripInit();

  serialMx = xSemaphoreCreateMutex();
  driveMx  = xSemaphoreCreateMutex();
  bool ok = serialMx && driveMx;
  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    motorQ[ch] = xQueueCreate(1, sizeof(MotorCmd));
    ok &= motorQ[ch] != NULL;
  }
  if (!ok) { Serial.println(F("ERR: RTOS object creation failed - raise configTOTAL_HEAP_SIZE")); return; }

  ok &= xTaskCreate(motorTask,     "motor",  180, NULL, 3, &hMotor)  == pdPASS;
  ok &= xTaskCreate(signalTask,    "signal", 180, NULL, 2, &hSignal) == pdPASS;
  ok &= xTaskCreate(motionTask,    "motion", 160, NULL, 1, &hMotion) == pdPASS;
  ok &= xTaskCreate(commandTask,   "cmd",    600, NULL, 1, &hCmd)    == pdPASS;
  ok &= xTaskCreate(telemetryTask, "tel",    380, NULL, 1, &hTel)    == pdPASS;
  ok &= xTaskCreate(ultrasonicTask, "us",    180, NULL, 2, &hUs)     == pdPASS;
  ok &= xTaskCreate(gripperTask,   "grip",   340, NULL, 1, &hGrip)   == pdPASS;
  ok &= xTaskCreate(autoStatusTask, "auto",  320, NULL, 1, &hAuto)   == pdPASS;
  if (!ok) Serial.println(F("ERR: task creation failed - raise configTOTAL_HEAP_SIZE"));

  if (START_IN_AUTO_MODE) {
    startAuto(AUTO_DEFAULT_MS, false); /* velocity PID on, AUTO_* parameters loaded, no text on the serial line */
    rosSync = true;                    /* stay silent until the first ROS frame, then one status line per frame */
  } else {
    setPidPeriod(10);                  /* default PID sampling time: 10 ms (auto mode sets AUTO_DEFAULT_MS) */
    Serial.println(F("Ready. Type HELP for commands."));
  }
  /* The scheduler starts automatically when setup() returns.
     The gripper stays idle (state D) until the first GRIP command, which runs the sanity check. */
}

void loop() {
  /* empty: everything runs in tasks */
}
