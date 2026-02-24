/*
CaelumSufflamen.ino
==============================================================================
CAELUM SUFFLAMEN — DETERMINISTIC AVIONICS RUNTIME SKELETON
(SENSORS → PUBLISHED SNAPSHOTS → DERIVED STATE → SAFETY → POLICY → ACTUATION → TELEMETRY)
==============================================================================

0) EXECUTIVE SUMMARY (ONE PARAGRAPH)
------------------------------------------------------------------------------
Caelum Sufflamen (“sky brakes”) is a single-threaded, millis()-scheduled flight
software scaffold for a small rocket airbrake module. The program is built as a
deterministic pipeline: time-gated sensor epochs publish *validity-qualified,
time-stamped snapshots*; derived computations consume only those snapshots;
actuation is permitted only when an explicit multi-layer safety predicate holds;
telemetry exports a stable CSV schema that makes system health and timing
verifiable after the fact.

The design goal is not “maximum features,” but “maximum reviewability”:
bounded work per loop iteration, explicit data contracts, and fail-safe defaults.

------------------------------------------------------------------------------
1) WHAT PROBLEM THIS ARCHITECTURE SOLVES
------------------------------------------------------------------------------
Small flight systems often fail for non-obvious software reasons:
  • timing jitter from blocking calls and ad-hoc delays,
  • silent propagation of stale sensor reads,
  • actuator commands emitted from partially-initialized or invalid state,
  • telemetry that changes field order and becomes unverifiable.

This program attacks those failure modes structurally:

  (A) Determinism by construction:
      Every loop iteration performs a finite, bounded set of operations.
      “Not due” epochs return immediately.

  (B) Traceability by publication:
      Every subsystem publishes a snapshot containing:
        - numerical fields
        - valid flag (semantic correctness)
        - t_ms timestamp (updated on each attempt)
      Downstream logic consumes snapshots, never raw sensors.

  (C) Safety by gating:
      The actuator is forced idle unless a strict, explicit predicate is true.

  (D) Verifiability by telemetry:
      Stable CSV schema + ages + warning mask enable post-flight auditing.

------------------------------------------------------------------------------
2) CORE CONCEPT: PUBLISHED SNAPSHOTS + VALIDITY + AGE
------------------------------------------------------------------------------
The system is organized around “published snapshots” (single-writer structs).
A snapshot represents the best known state of a subsystem at a known time.

Snapshot fields:
  - valid : indicates whether published values are semantically usable
  - t_ms  : timestamp (millis()) associated with the most recent *attempt*
  - data  : floats/flags describing the subsystem

Why t_ms updates even on failure:
  It distinguishes “never attempted” from “attempted but failed,” and allows
  ages to reflect real timing even under intermittent faults.

Age operator (informal):
  Age(S, now) = now - S.t_ms if S.valid == true
                ⊥            if S.valid == false
Implementation uses a sentinel (0xFFFFFFFF) and prints age = -1 in telemetry.

------------------------------------------------------------------------------
3) SYSTEM PIPELINE (BOTTOM-UP DATAFLOW)
------------------------------------------------------------------------------
A. Sensor Epochs (hardware transactions)
  1) IMU (MPU6050) → ImuSample imu_s
     - publishes accel/gyro, roll/pitch (gravity-vector approximation),
       acceleration norm, and motion_bad plausibility flag.

  2) AUX accel (LIS3DH) → AuxAccelSample aux_s
     - publishes accel vector and motion_bad heuristic.

  3) Barometer (BMP5xx) → BaroSample baro_s
     - publishes pressure (hPa) and temperature (°C).

  4) Magnetometer (LIS2MDL) → MagSample mag_s
     - publishes raw/cal field vectors, norm, interference flag,
       and heading validity + heading angle when permitted.

B. Derived Epochs (math-only)
  5) Aux vertical linear acceleration → AuxVertLinAccel aux_vz_s
     - projects AUX accel into world +Z using IMU roll/pitch,
       subtracts gravity to yield a_lin_z (m/s²) for Kalman input.

  6) Estimator → FlightState fs_s
     - pressure → altitude (barometric approximation)
     - 2-state Kalman filter:
         state: altitude (m), vertical speed (m/s)
         input: vertical linear accel (optional)
         meas : baro altitude
     - publishes fused altitude and vertical speed.

C. Safety / Policy / Actuation
  7) Arming FSM (runtime gate)
     - DISARMED → SAFE → ARMED
     - requires physical switch AND software token.

  8) Airbrake policy (bounded logic)
     - computes command01 ∈ [0,1] plus valid flag
     - may refuse actuation under invalid/stale estimator.

  9) Actuator application (safety predicate)
     - commands servo only when permitted; otherwise forces idle.

D. Telemetry / Commands
 10) Telemetry emission at fixed cadence
     - stable CSV header + stable field order
     - includes ages and a warning bitmask.

 11) Non-blocking command interface
     - bounded line reader
     - commands mutate configuration, arming token, and policy enable state.

------------------------------------------------------------------------------
4) DETERMINISM CONTRACT (HARD REQUIREMENTS)
------------------------------------------------------------------------------
Definition (informal): behavior is deterministic when the same time history of
inputs leads to repeatable bounded execution with no hidden waits.

Contract enforcement mechanisms:
  (D1) Time-gated epochs:
      Each epoch begins with: if (now - last_ms < PERIOD) return false;
      Therefore each epoch performs work only when due.

  (D2) One hardware transaction per due epoch:
      Each sensor epoch attempts at most one I²C acquisition per invocation.

  (D3) Derived computations are bounded scalar math:
      No allocations, no unbounded loops, no retries, no waits.

  (D4) loop() contains no blocking calls:
      - command ingestion consumes only already-buffered bytes
      - no delay() inside loop()
      - no “wait until sensor ready” polling loops

Operator-only exception:
  Magnetometer calibration capture intentionally blocks and must be invoked only
  as a deliberate ground procedure (CAL_MAG), not during flight.

------------------------------------------------------------------------------
5) SAFETY MODEL: THE ACTUATOR IS SAFETY-CRITICAL
------------------------------------------------------------------------------
Airbrake motion is permitted only when *all* independent gates pass.

Layer 1 — Compile-time gate:
  ACTUATION_ENABLED == 0  ⇒ actuator output always forced idle.

Layer 2 — Runtime arming gate:
  arm_state == ARMED is required (physical switch + software token).

Layer 3 — Policy validity gate:
  compute_airbrake_policy() must explicitly return out.valid == true.

Safety predicate Π (informal):
  Π ≜ (ACTUATION_ENABLED != 0) ∧ (arm_state == ARMED) ∧ (pol.valid == true)

Safety invariant:
  If Π is false, actuator output is forced to idle.

This invariant is enforced redundantly:
  - inside AirbrakeActuator::setCommand01() (compile-time + runtime enable)
  - inside loop() (policy validity + arming gating before applying command)

Fail-safe default:
  Any invalid sensor, invalid estimator, stale estimate, disarmed state, or
  disabled policy forces idle actuation.

------------------------------------------------------------------------------
6) HEADING PIPELINE (VALIDITY IS EXPLICIT, NOT ASSUMED)
------------------------------------------------------------------------------
Heading is computed only when prerequisites are satisfied:

  (H1) Calibration exists (cfg.valid && cfg.mag_cal.valid)
  (H2) Attitude is plausible (imu_s.valid && !imu_s.motion_bad)
  (H3) Field magnitude is plausible (norm within [MIN, MAX])
       to suppress operation under interference.

If any gate fails:
  mag_s.valid = false and heading is exported as NaN.

------------------------------------------------------------------------------
7) CONFIGURATION PERSISTENCE (CRC-PROTECTED EEPROM)
------------------------------------------------------------------------------
Configuration is stored as a structured EEPROM blob:
  - magic constant (slot ownership)
  - version (schema compatibility)
  - Config payload (parameters)
  - CRC32 over preceding bytes (integrity)

Load semantics are conservative:
  wrong magic, wrong version, or CRC mismatch ⇒ reject blob.
In a safety-relevant system, “unknown” is treated as “invalid,” not “usable.”

------------------------------------------------------------------------------
8) TELEMETRY CONTRACT (STABLE SCHEMA = VERIFIABLE SYSTEM)
------------------------------------------------------------------------------
Telemetry is CSV with a fixed field order declared by print_header().
Field order is a parsing contract.

Validity conventions:
  - floating-point invalid → literal token "nan"
  - age invalid/unavailable → integer -1
This preserves parse alignment and avoids ambiguity with real zeros.

Warning mask:
  A compact bitfield summarizes key health and safety states so that logs can be
  scanned quickly and verified formally.

------------------------------------------------------------------------------
9) MAIN LOOP ORDERING (WHY THIS ORDER IS NON-NEGOTIABLE)
------------------------------------------------------------------------------
loop() executes in this order:

  1) Non-blocking command ingestion
  2) Sensor epochs (IMU, AUX, BARO, MAG)
  3) Derived epochs (aux vertical accel, estimator)
  4) Arming FSM update + buzzer chirps
  5) Policy evaluation
  6) Safety-gated actuation (command or idle)
  7) Telemetry emission

Rationale:
  - derived computations consume freshest snapshots
  - arming is evaluated before committing actuation
  - failure at any stage defaults to idle
  - telemetry reflects same-cycle policy/arming state

------------------------------------------------------------------------------
10) INTENDED USE AND LIMITATIONS
------------------------------------------------------------------------------
This firmware is an engineering scaffold intended for review, verification, and
incremental extension. It does not claim to be a complete flight controller.
Any airbrake policy must be validated by simulation, ground tests, and careful
review before enabling actuation.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <EEPROM.h>

// Sensor libraries (Adafruit unified sensor drivers)
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// C stdlib helpers used for parsing
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

// -----------------------------------------------------------------------------
// Servo backend selection
// -----------------------------------------------------------------------------
#if defined(ARDUINO_TEENSY41) || defined(TEENSYDUINO)
#include <PWMServo.h>
using ServoBackend = PWMServo;
#else
#include <Servo.h>
using ServoBackend = Servo;
#endif

// ============================================================================
// SECTION 1: Compile-time safety switches
// ============================================================================
#ifndef ACTUATION_ENABLED
#define ACTUATION_ENABLED 0
#endif

// Policy compile-time enable gate (default OFF).
#ifndef AIRBRAKE_POLICY_ENABLED
#define AIRBRAKE_POLICY_ENABLED 0
#endif

// ============================================================================
// SECTION 2: Hardware constants
// ============================================================================
static constexpr uint8_t PIN_ARM_SWITCH     = 7;
static constexpr uint8_t PIN_AIRBRAKE_SERVO = 9;
static constexpr uint8_t PIN_BUZZER         = 6;

// ============================================================================
// SECTION 2A: I²C wiring declarations (bus + pins + sensor addresses)
// ============================================================================
static TwoWire& I2C_BUS = Wire;

static constexpr uint8_t  PIN_I2C_SDA = 18;
static constexpr uint8_t  PIN_I2C_SCL = 19;
static constexpr uint32_t I2C_CLK_HZ  = 400000u;

static constexpr uint8_t BMP_I2C_ADDR_DEFAULT = 0x47;
static constexpr uint8_t BMP_I2C_ADDR_ALT     = 0x46;

static constexpr uint8_t LIS2MDL_I2C_ADDR = 0x1E;

static constexpr uint8_t LIS3DH_I2C_ADDR_SA0_0 = 0x18;
static constexpr uint8_t LIS3DH_I2C_ADDR_SA0_1 = 0x19;

// ============================================================================
// SECTION 3: Sensor objects
// ============================================================================
static Adafruit_MPU6050 mpu;
static Adafruit_BMP5xx  bmp;
static Adafruit_LIS2MDL mag;
static Adafruit_LIS3DH  lis3dh;



// ============================================================================
// SECTION 4: Type definitions  (DOCUMENTED DATA CONTRACTS)
// ============================================================================
//
// DESIGN RULE (PUBLISHED SNAPSHOTS)
// ---------------------------------------------------------------------------
// The structs below are “published snapshots”: single-writer, multi-reader
// records that capture *what is known* about a subsystem at a specific time.
//
// Common fields used across snapshots:
//   - valid : semantic usability of the numerical fields
//   - t_ms  : millis() timestamp associated with the most recent acquisition
//            attempt (updated on success and on failure)
//   - data  : numerical/flag payload (units and interpretation documented)
//
// WHY timestamp updates on failure:
//   It separates “never attempted” from “attempted but failed,” and enables
//   truthful age/staleness reporting even under intermittent faults.
//
// READER CONTRACT (APPLIES TO ALL SNAPSHOTS)
// ---------------------------------------------------------------------------
// Downstream logic must treat numerical fields as meaningful *only* when:
//   (a) valid == true
//   (b) all consumed floats are finite (isfinite() == true)
//   (c) age is within subsystem-specific freshness limits
//
// ============================================================================

/*
ImuSample
------------------------------------------------------------------------------
ROLE
  Published snapshot of the primary IMU (MPU6050) epoch.

CONTENTS
  - Proper acceleration (ax,ay,az) in m/s^2, in the sensor/body frame.
  - Angular rate (gx,gy,gz) in rad/s, in the sensor/body frame.
  - Roll and pitch (deg) computed from the accelerometer gravity vector
    approximation (quasi-static assumption).
  - Acceleration norm a_norm (m/s^2) used for plausibility gating.
  - motion_bad: heuristic flag indicating that roll/pitch (and any downstream
    projection using them) may be unreliable due to strong linear acceleration,
    vibration, or transient disturbances.

VALIDITY SEMANTICS
  valid == true  => all fields were updated coherently from a successful read.
  valid == false => epoch attempt failed; timestamp still updates; numeric
                    fields may be stale and must not be used.

TIMING SEMANTICS
  t_ms is set to the time of the most recent attempt (success or failure).

FIELD UNITS
  ax,ay,az      : m/s^2
  gx,gy,gz      : rad/s
  roll_deg      : deg
  pitch_deg     : deg
  a_norm        : m/s^2
*/
struct ImuSample {
  float ax = NAN, ay = NAN, az = NAN;   // Body-frame proper acceleration (m/s^2)
  float gx = NAN, gy = NAN, gz = NAN;   // Body-frame angular rate (rad/s)
  float roll_deg  = NAN;                // Roll estimate from accel (deg)
  float pitch_deg = NAN;                // Pitch estimate from accel (deg)
  float a_norm = NAN;                   // ||a|| (m/s^2), used for plausibility
  bool  valid = false;                  // Semantic usability gate
  bool  motion_bad = false;             // High-dynamics heuristic gate
  uint32_t t_ms = 0;                    // Timestamp of most recent attempt
};

/*
AuxAccelSample
------------------------------------------------------------------------------
ROLE
  Published snapshot of the auxiliary accelerometer (LIS3DH) epoch.

INTENDED USE
  Provides an independent acceleration measurement stream that can be projected
  into a world vertical axis and used as the Kalman input (u) after gravity
  removal. The auxiliary channel improves robustness if the primary IMU stream
  becomes unavailable or if cross-validation is desired.

VALIDITY + MOTION HEURISTIC
  valid indicates acquisition success.
  motion_bad is an analogous heuristic to ImuSample::motion_bad, based on accel
  magnitude plausibility; it is used to refuse use of the sample for derived
  vertical linear acceleration under high dynamics.

FIELD UNITS
  ax,ay,az      : m/s^2
  a_norm        : m/s^2
*/
struct AuxAccelSample {
  float ax = NAN, ay = NAN, az = NAN;   // Body-frame proper acceleration (m/s^2)
  float a_norm = NAN;                   // ||a|| (m/s^2)
  bool  valid = false;                  // Acquisition success gate
  bool  motion_bad = false;             // Plausibility gate
  uint32_t t_ms = 0;                    // Timestamp of most recent attempt
};

/*
AuxVertLinAccel
------------------------------------------------------------------------------
ROLE
  Published snapshot of derived vertical acceleration quantities computed from:
    - AuxAccelSample (body acceleration)
    - ImuSample roll/pitch (orientation estimate)

DERIVED QUANTITIES
  a_wz_mps2:
    Projection of the auxiliary accelerometer measurement onto the world +Z axis
    (up). This is a *world-frame* component but still includes gravity under the
    sign convention used by the projection.

  a_lin_z_mps2:
    Vertical *linear* acceleration in world coordinates (up), after gravity
    removal. This is the intended Kalman filter input u.

VALIDITY SEMANTICS
  valid == true only when:
    - aux_s is valid and not motion_bad, with finite ax/ay/az
    - imu_s is valid and not motion_bad, with finite roll/pitch
    - projection math yields finite values

FIELD UNITS
  a_wz_mps2      : m/s^2 (world +Z component of measured accel)
  a_lin_z_mps2   : m/s^2 (world +Z linear accel, gravity removed)
*/
struct AuxVertLinAccel {
  float a_wz_mps2    = NAN;             // World +Z component (m/s^2), includes gravity
  float a_lin_z_mps2 = NAN;             // Linear world +Z accel (m/s^2), gravity removed
  bool  valid = false;                  // Semantic usability gate for Kalman input
  uint32_t t_ms = 0;                    // Timestamp of most recent computation attempt
};

/*
BaroSample
------------------------------------------------------------------------------
ROLE
  Published snapshot of the barometer (BMP5xx) epoch.

INTENDED USE
  Supplies the measurement z for altitude estimation via a pressure→altitude
  model. Also supplies temperature for logging and potential compensation.

VALIDITY SEMANTICS
  valid == true  => pressure and temperature were updated from a successful read.
  valid == false => epoch attempt failed; timestamp still updates; numeric fields
                    may be stale and must not be used.

FIELD UNITS
  temp_c     : degrees Celsius (°C)
  press_hpa  : hectopascals (hPa)
*/
struct BaroSample {
  float temp_c = NAN;                   // Temperature (°C)
  float press_hpa = NAN;                // Pressure (hPa)
  bool  valid = false;                  // Acquisition success gate
  uint32_t t_ms = 0;                    // Timestamp of most recent attempt
};

/*
FlightState
------------------------------------------------------------------------------
ROLE
  Published fused vertical state estimate produced by the estimator epoch.

STATE VARIABLES
  altitude_m:
    Altitude in meters relative to the selected reference pressure model.
    If a pad baseline pressure is captured, altitude is approximately “meters
    above baseline.” Otherwise it is “meters above sea-level reference estimate.”

  vz_mps:
    Vertical speed in m/s, positive upward.

VALIDITY SEMANTICS
  valid == true only when:
    - a finite baro-derived altitude measurement exists for update/seed
    - Kalman state has been seeded and updated coherently for this epoch

TIMING
  t_ms marks the estimator publication time for freshness gating.
*/
struct FlightState {
  float altitude_m = NAN;               // Altitude (m)
  float vz_mps = NAN;                   // Vertical speed (m/s), +up
  bool  valid = false;                  // Semantic usability gate
  uint32_t t_ms = 0;                    // Timestamp of most recent estimator update
};

/*
MagCal
------------------------------------------------------------------------------
ROLE
  Magnetometer calibration parameters used to correct raw LIS2MDL samples before
  heading computation.

CALIBRATION MODEL (SIMPLIFIED)
  Hard-iron bias (offset):
    m' = m_raw - off

  Soft-iron scaling (axis-wise scale):
    m_cal = m' * scl

  This is an axis-wise approximation (no axis coupling). It substantially
  improves heading quality for many practical assemblies.

FIELDS
  off_* : hard-iron offsets (uT)
  scl_* : soft-iron scale factors (dimensionless)
  valid : true when calibration is believed usable
  r*    : diagnostic radii (half-range per axis) observed during capture

DIAGNOSTICS
  rx,ry,rz are stored to enable verification of sufficient excitation/coverage.
*/
struct MagCal {
  float off_x = 0.0f, off_y = 0.0f, off_z = 0.0f;  // Offsets (uT)
  float scl_x = 1.0f, scl_y = 1.0f, scl_z = 1.0f;  // Scale factors (unitless)
  bool  valid = false;                             // Calibration usability gate
  float rx = NAN, ry = NAN, rz = NAN;              // Diagnostic radii (uT)
};

/*
MagSample
------------------------------------------------------------------------------
ROLE
  Published snapshot of the magnetometer epoch plus derived heading results.

CONTENTS
  raw_*:
    Raw LIS2MDL field components (uT) in the sensor/body frame.

  cal_*:
    Calibrated components (uT) after MagCal correction (if available).

  norm_uT:
    Field magnitude (uT) computed from calibrated components (preferred) or
    raw components if no calibration. Used for interference plausibility gating.

  heading_deg:
    Tilt-compensated magnetic heading (deg), compensated using IMU roll/pitch
    and adjusted by declination. Only meaningful when valid==true.

  interference:
    True when norm_uT is outside plausible bounds; indicates likely magnetic
    disturbance (wiring currents, steel, electronics, etc.).

VALIDITY SEMANTICS
  valid == true only when:
    - calibration exists and is marked valid
    - IMU roll/pitch are available and not motion_bad
    - norm is plausible and finite
    - heading computation yields a finite result

TIMING
  t_ms corresponds to the magnetometer acquisition time (attempt time).
*/
struct MagSample {
  float raw_x = NAN, raw_y = NAN, raw_z = NAN;      // Raw field (uT)
  float cal_x = NAN, cal_y = NAN, cal_z = NAN;      // Calibrated field (uT)
  float norm_uT = NAN;                              // ||B|| (uT)
  float heading_deg = NAN;                          // Heading (deg), valid-gated
  bool  valid = false;                              // Heading usability gate
  bool  interference = false;                       // Plausibility/interference flag
  uint32_t t_ms = 0;                                // Timestamp of most recent attempt
};

/*
ArmingState
------------------------------------------------------------------------------
ROLE
  Enumerated arming finite-state machine state.

SAFETY INTENT
  This enum is used as the definitive runtime gate for actuation.
  Only ARMED permits the possibility of actuator motion (still subject to other
  safety gates, e.g., compile-time actuation enable and policy validity).

STATES
  DISARMED:
    Physical switch is off; software token cleared; actuation forced idle.

  SAFE:
    Physical switch is on but software token not present; actuation forced idle.

  ARMED:
    Physical switch on + token present; policy may produce valid outputs; actuator
    may move only if additional gates allow.
*/
enum class ArmingState : uint8_t {
  DISARMED = 0,
  SAFE     = 1,
  ARMED    = 2
};

/*
AirbrakePolicyOutput
------------------------------------------------------------------------------
ROLE
  Policy-layer output record consumed by the actuation layer.

FIELDS
  command01:
    Normalized command in [0,1], where mapping to servo microseconds is defined
    by the actuator abstraction.

  valid:
    Policy-level permission flag:
      valid == true  => policy authorizes actuation for this cycle
      valid == false => policy refuses actuation; actuator must be forced idle

SAFETY NOTE
  valid is necessary but not sufficient for motion: arming and compile-time
  actuation gates must also allow.
*/
struct AirbrakePolicyOutput {
  float command01 = 0.0f;               // Normalized command ∈ [0,1]
  bool  valid = false;                  // Policy permission flag
};

/*
Config
------------------------------------------------------------------------------
ROLE
  Runtime configuration bundle used across sensor conversion, heading, and
  actuator calibration.

PERSISTENCE
  This struct is stored in EEPROM inside EepromBlob with magic/version/CRC.

FIELDS
  sea_level_hpa:
    Reference pressure for sea-level-based altitude conversion (fallback mode).

  baro_baseline_hpa:
    Launch-site baseline pressure captured at pad. When finite, altitude is
    computed relative to this baseline, reducing weather bias.

  declination_deg:
    Magnetic declination (deg) added to magnetic heading to produce true heading
    estimate (within the limits of sensor quality).

  mag_cal:
    Magnetometer calibration parameters.

  servo_us_min/max/idle:
    Servo pulse widths (microseconds) defining the actuator mapping and the idle
    fail-safe position.

  valid:
    Indicates that configuration fields have been set/loaded and are permitted
    to be used as authoritative. Conservative logic may still gate on individual
    field finiteness (e.g., baseline presence).
*/
struct Config {
  float sea_level_hpa     = 1013.25f;   // Reference pressure (hPa)
  float baro_baseline_hpa = NAN;        // Pad baseline (hPa); finite => relative altitude mode

  float declination_deg = 0.0f;         // Magnetic declination (deg)

  MagCal mag_cal = MagCal();            // Magnetometer calibration payload

  int16_t servo_us_min  = 1000;         // Servo min pulse (us)
  int16_t servo_us_max  = 2000;         // Servo max pulse (us)
  int16_t servo_us_idle = 1000;         // Servo idle pulse (us), fail-safe

  bool valid = false;                   // Config usability gate
};

/*
EepromBlob
------------------------------------------------------------------------------
ROLE
  EEPROM persistence record for configuration with integrity and compatibility
  protection.

SCHEMA GUARDS
  magic:
    Identifies the EEPROM slot contents as belonging to this firmware family.

  version:
    Identifies the struct layout schema. Must be incremented whenever any field
    layout changes in Config or EepromBlob.

INTEGRITY
  crc:
    CRC-32 computed over all bytes of the blob except the crc field itself.
    Used to detect corruption, partial writes, and wear-induced faults.

reserved_pad:
  Padding/reserved field to reduce churn across minor schema changes and to keep
  alignment explicit. May be repurposed only with a version bump.
*/
struct EepromBlob {
  uint32_t magic;                       // Slot ownership / family identifier
  uint32_t version;                     // Layout schema version
  Config   cfg;                         // Persisted configuration payload
  uint32_t reserved_pad;                // Reserved/padding
  uint32_t crc;                         // CRC-32 over preceding bytes (excluding this field)
};

/*
KalmanAlt2
------------------------------------------------------------------------------
ROLE
  Internal 2-state Kalman filter record for vertical motion estimation.

STATE
  x0: altitude (m)
  x1: vertical speed (m/s)

COVARIANCE
  P is stored explicitly as four scalars:
    [P00 P01]
    [P10 P11]
  Symmetry is enforced by assignment (P10 = P01) after predict/update.

TUNING PARAMETERS
  q_acc:
    Continuous-time acceleration process noise intensity (units consistent with
    the discretized Q terms used in predict). Larger => trusts acceleration input
    less / allows faster dynamics.

  r_meas:
    Measurement noise variance for altitude measurement (m^2). Larger => trusts
    baro altitude less.

SEEDING
  seeded:
    False until the first valid altitude measurement is used to initialize x0.

TIMING
  t_ms:
    Timestamp of the most recent filter update (publication-aligned).
*/
struct KalmanAlt2 {
  float x0 = 0.0f;                      // Altitude (m)
  float x1 = 0.0f;                      // Vertical speed (m/s)

  float P00 = 1.0f, P01 = 0.0f;         // Covariance matrix elements
  float P10 = 0.0f, P11 = 1.0f;

  float q_acc  = 4.0f;                  // Process noise intensity (accel)
  float r_meas = 4.0f;                  // Measurement noise variance (altitude)

  bool seeded = false;                  // Seed gate
  uint32_t t_ms = 0;                    // Timestamp of last update
};



// ============================================================================
// SECTION 4A: Kalman filter primitives (scalar, deterministic)
// ============================================================================
//
// This section implements a *strictly bounded*, allocation-free, branch-light
// 2-state Linear Kalman Filter specialized for vertical motion:
//
//   State vector (implicit):
//     x = [ x0 ; x1 ] where
//       x0 = altitude (m)
//       x1 = vertical speed (m/s)
//
//   Control input:
//     u = vertical acceleration (m/s^2) along the global +Z (up) axis.
//
//   Measurement:
//     z = altitude measurement (m) from barometer (or equivalent).
//
// The implementation is deliberately *scalar* (hand-expanded 2×2 math):
//   - Avoids generic matrix libraries
//   - Minimizes runtime overhead
//   - Supports deterministic execution suitable for epoch scheduling
//
// Numerical conventions:
//   - All covariance terms are in standard KF units:
//       P00: var(alt)          [m^2]
//       P11: var(vz)           [(m/s)^2]
//       P01=P10: cov(alt, vz)  [m^2/s]
//   - Process noise is modeled as continuous-time white acceleration noise with
//     spectral density q_acc, discretized into Q(dt) for a constant-acceleration
//     state model.
//   - Measurement noise is modeled as altitude variance r_meas [m^2].
//
// Robustness conventions:
//   - Update step includes a guard against non-positive innovation variance S.
//   - Covariance update uses a Joseph-stabilized form, then enforces symmetry
//     (P01=P10) via averaging to mitigate floating-point asymmetry drift.
//
// Determinism contract:
//   - No loops
//   - No heap usage
//   - O(1) scalar arithmetic per call
//

//------------------------------------------------------------------------------
// kf_alt2_reset
//------------------------------------------------------------------------------
// PURPOSE
//   Initialize (or reinitialize) the 2-state altitude filter to a known state.
//
// CONTRACT
//   Preconditions:
//     - now_ms is a monotonic timestamp (typically millis()).
//     - alt0 is a finite altitude estimate in meters.
//   Postconditions:
//     - State is set to [alt0, 0].
//     - Covariance is set to a conservative diagonal prior.
//     - seeded flag is asserted.
//     - Internal timestamp is updated to now_ms.
//
// DESIGN NOTES
//   - Initial covariance values represent prior uncertainty. The chosen values
//     (25 m^2) correspond to a 1σ altitude uncertainty of 5 m, and a 1σ velocity
//     uncertainty of 5 m/s if interpreted similarly; these are placeholders and
//     should be tuned to match expected initialization quality.
//   - Off-diagonal covariances are set to 0, indicating no prior coupling.
//
// FAILURE MODES
//   - If alt0 is NaN/Inf, subsequent predict/update math can propagate NaNs.
//     Upstream validation is expected.
//
static inline void kf_alt2_reset(KalmanAlt2& kf, float alt0, uint32_t now_ms) {
  // State mean (posterior) after initialization.
  kf.x0 = alt0;   // altitude [m]
  kf.x1 = 0.0f;   // vertical speed [m/s] (unknown => neutral prior)

  // Covariance prior (posterior at initialization).
  // Conservative diagonal covariance: large enough to allow fast convergence,
  // small enough to avoid unstable gains in the first few updates.
  kf.P00 = 25.0f; kf.P01 = 0.0f;
  kf.P10 = 0.0f;  kf.P11 = 25.0f;

  // Administrative state.
  kf.seeded = true;
  kf.t_ms = now_ms;
}

//------------------------------------------------------------------------------
// kf_alt2_predict_u
//------------------------------------------------------------------------------
// PURPOSE
//   Perform the discrete-time *prediction* (time update) step using a control
//   input: vertical acceleration u_acc_mps2.
//
// MODEL
//   Continuous-time kinematics (constant acceleration over dt):
//     x0(t+dt) = x0(t) + dt*x1(t) + 0.5*dt^2*u
//     x1(t+dt) = x1(t) + dt*u
//
//   State transition matrix F and control matrix G (implicit):
//     F = [1  dt
//          0   1]
//
//     G = [0.5*dt^2
//           dt     ]
//
// COVARIANCE PROPAGATION
//   P <- F P F^T + Q
//
//   With continuous-time white acceleration noise (spectral density q_acc),
//   the standard discretization for this 2-state model yields:
//
//     Q00 = 0.25 * dt^4 * q
//     Q01 = 0.50 * dt^3 * q
//     Q11 =        dt^2 * q
//
//   where q = q_acc.
//
// CONTRACT
//   Preconditions:
//     - kf.seeded is true (filter has been initialized).
//     - dt is the elapsed time in seconds for the prediction interval.
//     - dt is finite and non-negative; typical operation requires dt > 0.
//     - u_acc_mps2 is finite.
//   Postconditions:
//     - (x0, x1) are advanced by dt using u_acc_mps2.
//     - P is advanced by dt with process noise q_acc.
//     - P symmetry is enforced by construction (P10 = P01).
//
// NUMERICAL STABILITY / DETERMINISM
//   - Computation is explicitly scalar and bounded.
//   - dt powers are computed once to reduce floating-point operations.
//   - The covariance propagation is expanded by hand for 2×2 to avoid matrix ops.
//
// FAILURE MODES
//   - If dt is extremely large, dt^4 may overflow float, producing Inf.
//   - If dt is negative, the state update becomes non-physical; upstream time
//     handling is expected to prevent this.
//   - If q_acc is negative, Q becomes negative definite and can destabilize P;
//     q_acc must represent a variance density and should be ≥ 0.
//
static inline void kf_alt2_predict_u(KalmanAlt2& kf, float dt, float u_acc_mps2) {
  // Precompute powers of dt used in kinematic and noise terms.
  const float dt2 = dt * dt;

  // -------------------------------
  // 1) Predict the state mean: x <- f(x, u)
  // -------------------------------
  // Altitude integrates velocity and acceleration.
  kf.x0 = kf.x0 + dt * kf.x1 + 0.5f * dt2 * u_acc_mps2;
  // Velocity integrates acceleration.
  kf.x1 = kf.x1 + dt * u_acc_mps2;

  // Snapshot old covariance terms (P at time k).
  const float P00 = kf.P00, P01 = kf.P01, P10 = kf.P10, P11 = kf.P11;

  // -------------------------------
  // 2) Propagate covariance by F P F^T (no process noise yet)
  // -------------------------------
  // For F = [[1, dt],[0,1]]:
  //   nP00 = P00 + dt*(P10+P01) + dt^2*P11
  //   nP01 = P01 + dt*P11
  //   nP10 = P10 + dt*P11
  //   nP11 = P11
  const float nP00_ap = P00 + dt * (P10 + P01) + dt2 * P11;
  const float nP01_ap = P01 + dt * P11;
  const float nP10_ap = P10 + dt * P11;
  const float nP11_ap = P11;

  // -------------------------------
  // 3) Add discretized acceleration process noise Q(dt)
  // -------------------------------
  const float dt3 = dt2 * dt;
  const float dt4 = dt2 * dt2;

  const float q   = kf.q_acc;           // acceleration noise spectral density
  const float Q00 = 0.25f * dt4 * q;
  const float Q01 = 0.50f * dt3 * q;
  const float Q11 = dt2 * q;

  // Final predicted covariance.
  // Symmetry is enforced explicitly: P10 == P01.
  kf.P00 = nP00_ap + Q00;
  kf.P01 = nP01_ap + Q01;
  kf.P10 = kf.P01;
  kf.P11 = nP11_ap + Q11;
}

//------------------------------------------------------------------------------
// kf_alt2_update_z
//------------------------------------------------------------------------------
// PURPOSE
//   Perform the discrete-time *measurement update* using an altitude measurement
//   z_alt (meters).
//
// MEASUREMENT MODEL
//   z = H x + v, where
//     H = [1  0]  (measures altitude directly)
//     v ~ N(0, R), with R = r_meas (altitude variance)
//
// INNOVATION FORM
//   y = z - H x_pred = z_alt - x0
//   S = H P_pred H^T + R = P00 + R
//
// KALMAN GAIN (2×1, scalar S)
//   K = P H^T S^-1
//     = [P00; P10] / S
//
// STATE UPDATE
//   x <- x + K y
//
// COVARIANCE UPDATE (Joseph form, numerically stable)
//   P <- (I - K H) P (I - K H)^T + K R K^T
//
//   This implementation expands Joseph form explicitly for 2×2 and then
//   enforces symmetry by averaging the off-diagonal terms.
//
// CONTRACT
//   Preconditions:
//     - kf.seeded is true.
//     - z_alt is finite.
//     - kf.r_meas is finite and ideally > 0.
//   Postconditions:
//     - State mean is corrected toward measurement by the Kalman gain.
//     - Covariance is reduced consistently with measurement information.
//     - If S is non-positive (degenerate), update is skipped and the filter
//       remains in predicted state.
//
// ROBUSTNESS / FAILURE MODES
//   - Guard: if S <= ~0, invS becomes invalid; update is rejected.
//   - If r_meas is set too small (overconfident sensor model), K can become
//     very large and inject measurement noise directly into the state.
//   - If r_meas is set too large (underconfident sensor model), updates become
//     weak and the filter behaves like a dead-reckoner.
//
static inline void kf_alt2_update_z(KalmanAlt2& kf, float z_alt) {
  // Innovation (measurement residual): new information not explained by prediction.
  const float y = z_alt - kf.x0;

  // Innovation covariance S = P00 + R for H = [1 0].
  const float S = kf.P00 + kf.r_meas;

  // Degeneracy guard: reject update if S is not strictly positive.
  // This prevents division by zero and negative variances due to numeric issues.
  if (!(S > 1e-9f)) return;

  const float invS = 1.0f / S;

  // Kalman gain components (K = [K0; K1]).
  const float K0 = kf.P00 * invS;  // altitude gain
  const float K1 = kf.P10 * invS;  // velocity gain via covariance coupling

  // -------------------------------
  // 1) State mean update: x <- x + K*y
  // -------------------------------
  kf.x0 = kf.x0 + K0 * y;
  kf.x1 = kf.x1 + K1 * y;

  // Snapshot prior predicted covariance (P^-).
  const float P00 = kf.P00, P01 = kf.P01, P10 = kf.P10, P11 = kf.P11;
  const float R   = kf.r_meas;

  // -------------------------------
  // 2) Joseph covariance update:
  //    P <- (I - K H) P (I - K H)^T + K R K^T
  //    For H=[1 0], (I-KH) becomes:
  //      A = [1-K0   0
  //           -K1    1]
  // -------------------------------
  const float a00 = 1.0f - K0;
  const float a10 = -K1;

  // Compute B = A * P.
  const float b00 = a00 * P00;
  const float b01 = a00 * P01;
  const float b10 = a10 * P00 + P10;
  const float b11 = a10 * P01 + P11;

  // Compute nP = B * A^T.
  float nP00 = b00 * a00;
  float nP01 = b00 * a10 + b01;
  float nP10 = b10 * a00;
  float nP11 = b10 * a10 + b11;

  // Add KRK^T term (measurement noise contribution in Joseph form).
  nP00 += K0 * R * K0;
  nP01 += K0 * R * K1;
  nP10 += K1 * R * K0;
  nP11 += K1 * R * K1;

  // Enforce symmetry to suppress floating-point asymmetry drift.
  const float sym01 = 0.5f * (nP01 + nP10);
  kf.P00 = nP00;
  kf.P01 = sym01;
  kf.P10 = sym01;
  kf.P11 = nP11;
}





// ============================================================================
// SECTION 5: Global state (single-writer discipline)
// ============================================================================
static ImuSample       imu_s;
static AuxAccelSample  aux_s;
static AuxVertLinAccel aux_vz_s;
static BaroSample      baro_s;
static FlightState     fs_s;
static MagSample       mag_s;

static KalmanAlt2 kf_alt;

static ArmingState arm_state = ArmingState::DISARMED;
static bool sw_arm_token = false;

static Config cfg;

// ============================================================================
// SECTION 6: Common utilities
// ============================================================================
static inline float rad2deg(float r) { return r * 57.29577951308232f; }
static inline float deg2rad(float d) { return d * 0.017453292519943295f; }

static inline float ema_update(float prev, float x, float alpha) {
  if (isnan(prev)) return x;
  return prev + alpha * (x - prev);
}

static inline uint32_t age_ms(uint32_t now_ms, uint32_t t_ms, bool valid) {
  if (!valid) return 0xFFFFFFFFu;
  return now_ms - t_ms;
}


static inline float wrap360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

// ============================================================================
// SECTION 7: EEPROM persistence (CRC-protected)
// ============================================================================
static constexpr uint32_t EEPROM_MAGIC   = 0x524B5443u; // 'RKTC'
static constexpr uint32_t EEPROM_VERSION = 1u;
static constexpr int      EEPROM_BASE    = 0;

/*
crc32_update(crc, data)
------------------------------------------------------------------------------
ROLE
  Update a running CRC-32 (IEEE 802.3 / Ethernet polynomial) with one byte.

MODEL
  - Reflected polynomial form: 0xEDB88320
  - Initialization and finalization are handled by crc32_bytes().

CONTRACT
  Preconditions:
    - crc is the current running CRC value.
    - data is the next byte.
  Postconditions:
    - returns the updated running CRC.

DETERMINISM / BOUNDS
  - Fixed 8-iteration loop per byte.
  - No allocations, no I/O.
*/
static uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (int i = 0; i < 8; i++) {
    const uint32_t mask = (uint32_t)-(int)(crc & 1u);
    crc = (crc >> 1) ^ (0xEDB88320u & mask);
  }
  return crc;
}

/*
crc32_bytes(p, n)
------------------------------------------------------------------------------
ROLE
  Compute CRC-32 over a contiguous byte buffer.

CRC SCOPE (IMPORTANT)
  This routine computes CRC over exactly n bytes starting at p. For EEPROM
  integrity, callers choose n such that it includes:
    - magic
    - version
    - config payload
    - padding/reserved fields (if any)
  but excludes the stored crc field itself.

CONTRACT
  Preconditions:
    - p points to readable memory of at least n bytes.
  Postconditions:
    - returns a CRC-32 suitable for detecting corruption.

DETERMINISM / BOUNDS
  - O(n) time with 8 bit-steps per byte.
*/
static uint32_t crc32_bytes(const uint8_t* p, size_t n) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) crc = crc32_update(crc, p[i]);
  return ~crc;
}
/*
eeprom_save_cfg()
------------------------------------------------------------------------------
ROLE
  Persist the current in-RAM configuration (cfg) to EEPROM as a CRC-protected
  EepromBlob record.

SCHEMA STABILITY (MAGIC + VERSION)
  The EEPROM record begins with:
    - magic   : identifies the slot as belonging to this firmware family
    - version : identifies the layout of the stored struct

  Together they prevent interpreting random EEPROM contents or incompatible
  older/newer layouts as valid configuration.

CRC SCOPE
  CRC is computed over the entire EepromBlob except the crc field itself.
  This detects:
    - bit flips
    - partial writes
    - unintended overwrites

CONTRACT
  Preconditions:
    - cfg contains desired configuration values to persist.
  Postconditions:
    - EEPROM contains a blob with correct magic/version and crc.
    - Serial prints "EEPROM,SAVE,OK".

DETERMINISM / BOUNDS
  - EEPROM.put() may be relatively slow but is bounded; called only on command.
  - No dynamic allocation.

FAILURE MODES
  - Power loss mid-write can corrupt the blob; load path detects via CRC.
  - EEPROM wear can corrupt; CRC detects but cannot correct.
*/
static void eeprom_save_cfg() {
  EepromBlob b;
  b.magic = EEPROM_MAGIC;
  b.version = EEPROM_VERSION;
  b.cfg = cfg;
  b.reserved_pad = 0u;
  b.crc = 0u;

  // CRC over all fields except the final crc field.
  b.crc = crc32_bytes((const uint8_t*)&b, sizeof(EepromBlob) - sizeof(uint32_t));

  EEPROM.put(EEPROM_BASE, b);
  Serial.println("EEPROM,SAVE,OK");
}

/*
eeprom_load_cfg()
------------------------------------------------------------------------------
ROLE
  Load a persisted configuration blob from EEPROM into cfg, but only if the blob
  passes:
    (1) magic check
    (2) version check
    (3) CRC integrity check

CONTRACT
  Postconditions (success):
    - cfg overwritten from EEPROM blob payload
    - cfg.valid set true
    - Serial prints "EEPROM,LOAD,OK"
  Postconditions (failure):
    - cfg is not overwritten (except cfg.valid may remain unchanged)
    - Serial prints a specific failure reason:
        - NO_MAGIC
        - BAD_VERSION
        - CRC_FAIL

WHY THIS DESIGN
  In flight-critical systems, incorrect configuration is worse than missing
  configuration. Therefore, the loader is conservative:
    - any doubt => refuse load, keep defaults or prior RAM cfg.

DETERMINISM / BOUNDS
  - EEPROM.get() bounded.
  - CRC computation bounded by blob size.
  - No dynamic allocation.

FAILURE MODES
  - If struct layout changes but version is not bumped, behavior is undefined.
    Versioning must be incremented when Config/EepromBlob layout changes.
*/
static void eeprom_load_cfg() {
  EepromBlob b;
  EEPROM.get(EEPROM_BASE, b);

  if (b.magic != EEPROM_MAGIC) {
    Serial.println("EEPROM,LOAD,NO_MAGIC");
    return;
  }
  if (b.version != EEPROM_VERSION) {
    Serial.println("EEPROM,LOAD,BAD_VERSION");
    return;
  }

  const uint32_t want = b.crc;

  // Compute CRC over the same bytes that were used when saving.
  b.crc = 0u;
  const uint32_t got =
      crc32_bytes((const uint8_t*)&b, sizeof(EepromBlob) - sizeof(uint32_t));

  if (got != want) {
    Serial.println("EEPROM,LOAD,CRC_FAIL");
    return;
  }

  cfg = b.cfg;
  cfg.valid = true;
  Serial.println("EEPROM,LOAD,OK");
}





// ============================================================================
// SECTION 8: Sensor init / scheduling
// ============================================================================
//
// This section defines:
//   (A) Epoch periods (milliseconds) for each subsystem.
//   (B) Per-epoch “last serviced” timestamps (single-writer by setup()/epoch fn).
//   (C) Sensor initialization routines that configure sensor operating modes.
//
// Design intent:
//   - Deterministic cadence control: each epoch routine uses (now - last_ms)
//     against its PERIOD_MS to decide whether work is due.
//   - Separation of concerns:
//       * init_*() performs one-time hardware bring-up + configuration.
//       * *_update() performs time-gated acquisition and snapshot publication.
//   - Fail-fast initialization: init_*() returns false if the sensor cannot be
//     discovered/started on the I²C bus.
//
// Determinism contract (scheduling side):
//   - The PERIOD constants define the *maximum* service rate. Actual service
//     rate may be lower if loop() stalls or if telemetry/serial dominates.
//   - Each epoch uses a non-blocking time gate:
//         if ((now - last_ms) < PERIOD_MS) return false;
//     Thus, “not due” implies constant-time return.
//
// Timestamp semantics (critical for age/valid logic):
//   - last_*_ms tracks scheduler cadence: “when that epoch last attempted work”.
//   - Snapshot structs (imu_s, baro_s, etc.) track “when data was last attempted”
//     via each snapshot’s own t_ms field.
//   - The separation allows reporting both:
//       * scheduling regularity (via last_*_ms behavior)
//       * data freshness/validity (via snapshot t_ms + valid flags)
//
// Practical period selection (why these numbers are reasonable defaults):
//   IMU_PERIOD_MS      = 10 ms  (~100 Hz): attitude/accel support for projection.
//   AUX_PERIOD_MS      = 10 ms  (~100 Hz): auxiliary accel aligned to LIS3DH rate.
//   AUX_VZ_PERIOD_MS   = 10 ms  (~100 Hz): derived vertical accel paired to aux.
//   BARO_PERIOD_MS     = 20 ms  (~50 Hz): barometer bandwidth, reduced bus load.
//   MAG_PERIOD_MS      = 20 ms  (~50 Hz): heading smoothing, reduced bus load.
//   EST_PERIOD_MS      = 20 ms  (~50 Hz): estimator matches baro cadence.
//
// Notes on I²C bus load:
//   - At 400 kHz I²C, these cadences are typically safe for small sensor sets,
//     but bus utilization depends on library transaction sizes and overhead.
//   - The architecture guarantees “one transaction per due epoch per loop”,
//     limiting burstiness.
//
// Failure philosophy:
//   - init_*() returning false is treated as “sensor unavailable at boot”.
//     Downstream epochs will publish invalid snapshots as appropriate.
//   - No retries or waits are added here; this maintains bounded boot behavior.

// -----------------------------------------------------------------------------
// Epoch periods (milliseconds)
// -----------------------------------------------------------------------------
//
// Each PERIOD defines the minimum elapsed time required before the associated
// update routine is allowed to perform a hardware transaction or derived update.
//
static constexpr uint32_t IMU_PERIOD_MS      = 10;
static constexpr uint32_t AUX_PERIOD_MS      = 10;
static constexpr uint32_t BARO_PERIOD_MS     = 20;
static constexpr uint32_t EST_PERIOD_MS      = 20;
static constexpr uint32_t MAG_PERIOD_MS      = 20;
static constexpr uint32_t AUX_VZ_PERIOD_MS   = 10;

// -----------------------------------------------------------------------------
// Scheduler timestamps (milliseconds)
// -----------------------------------------------------------------------------
//
// These are updated by the corresponding update routine when it is *due*.
// They are not the same as snapshot timestamps (imu_s.t_ms, etc.).
//
static uint32_t last_imu_ms   = 0;
static uint32_t last_aux_ms   = 0;
static uint32_t last_baro_ms  = 0;
static uint32_t last_est_ms   = 0;
static uint32_t last_mag_ms   = 0;
static uint32_t last_auxvz_ms = 0;

//------------------------------------------------------------------------------
// init_imu
//------------------------------------------------------------------------------
// PURPOSE
//   Initialize and configure the MPU6050 IMU.
//
// CONFIGURATION CHOICES (WHAT THEY MEAN)
//   - Accelerometer range: ±8 g
//       Provides headroom for high dynamic events (boost vibration / thrust).
//       Larger range reduces sensitivity (LSB per m/s^2) but avoids saturation.
//   - Gyro range: ±500 deg/s
//       Balances saturation headroom vs resolution for typical rocket motion.
//   - Digital low-pass filter bandwidth: 21 Hz
//       Reduces high-frequency noise at the cost of response latency.
//
// CONTRACT
//   Returns:
//     - true  if the device is detected and configured successfully.
//     - false if begin() fails (device missing, wiring fault, bus issue).
//
// DETERMINISM / BOUNDS
//   - One-time I²C device discovery/config.
//   - No loops and no retries.
//   - Intended to be called once from setup().
//
// FAILURE MODES
//   - Wrong I²C wiring/pins, missing pull-ups, incorrect power, bad address,
//     bus contention, or sensor absent -> begin() returns false.
//
static bool init_imu() {
  if (!mpu.begin()) return false;

  // Configure measurement ranges and internal filtering.
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  return true;
}

//------------------------------------------------------------------------------
// init_baro
//------------------------------------------------------------------------------
// PURPOSE
//   Initialize and configure the BMP5xx barometer (pressure + temperature).
//
// ADDRESS SELECTION STRATEGY
//   The code attempts multiple addresses to tolerate different breakout wiring:
//     - BMP5XX_DEFAULT_ADDRESS / BMP5XX_ALTERNATIVE_ADDRESS (library macros)
//     - Explicit 0x47 / 0x46 (project-defined constants)
//   The redundancy improves portability across board variants.
//
// CONFIGURATION CHOICES
//   - Temperature oversampling: 2×
//   - Pressure oversampling:    16×
//   - IIR filter coefficient:   3
//
//   These settings trade off:
//     * noise reduction and repeatability (higher oversampling, IIR filtering)
//     * sample latency and power/bus usage
//
// OPTIONAL SETTINGS (COMPILE-TIME DEPENDENT)
//   Some Adafruit library versions expose output-data-rate and power-mode APIs.
//   These are guarded by #if defined(...) so compilation succeeds across
//   library revisions.
//
// CONTRACT
//   Returns:
//     - true  if the sensor is found at any attempted address and configured.
//     - false if all begin() attempts fail.
//
// DETERMINISM / BOUNDS
//   - Finite sequence of begin() attempts (no retry loops).
//   - One-time configuration; intended for setup().
//
// FAILURE MODES
//   - Address mismatch, missing device, wiring/bus fault -> returns false.
//   - Library API differences are handled via conditional compilation; however,
//     if macros differ unexpectedly, some configuration calls may be omitted.
//
static bool init_baro() {
  // Attempt library-default addresses first.
  bool ok = bmp.begin(BMP5XX_DEFAULT_ADDRESS, &I2C_BUS);
  if (!ok) ok = bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &I2C_BUS);

  // Fall back to explicit project addresses (covers some board variants).
  if (!ok) ok = bmp.begin(BMP_I2C_ADDR_DEFAULT, &I2C_BUS);
  if (!ok) ok = bmp.begin(BMP_I2C_ADDR_ALT, &I2C_BUS);
  if (!ok) return false;

  // Configure oversampling and filtering for stable altitude estimation.
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);

  // Optional APIs depending on library version.
#if defined(BMP5XX_ODR_50_HZ)
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
#endif
#if defined(BMP5XX_POWERMODE_NORMAL)
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
#endif

  // Ensure pressure channel is enabled (temperature is typically implicit).
  bmp.enablePressure(true);

  return true;
}

//------------------------------------------------------------------------------
// init_mag
//------------------------------------------------------------------------------
// PURPOSE
//   Initialize the LIS2MDL magnetometer on the configured I²C bus/address.
//
// CONTRACT
//   Returns:
//     - true  if the sensor is discovered and initialized.
//     - false otherwise.
//
// DETERMINISM / BOUNDS
//   - Single begin() attempt (no retries).
//   - Intended for setup().
//
// FAILURE MODES
//   - Missing sensor, wiring/power issues, wrong address -> false.
//   - If begin() succeeds but environment has severe interference, heading
//     validity is still gated later by runtime heuristics.
//
static bool init_mag() {
  return mag.begin(LIS2MDL_I2C_ADDR, &I2C_BUS);
}

//------------------------------------------------------------------------------
// init_lis3dh
//------------------------------------------------------------------------------
// PURPOSE
//   Initialize and configure the LIS3DH auxiliary accelerometer.
//
// ADDRESS SELECTION STRATEGY
//   LIS3DH address depends on SA0 strap:
//     - SA0=0 -> 0x18
//     - SA0=1 -> 0x19
//   Both are attempted to tolerate different wiring.
//
// CONFIGURATION CHOICES
//   - Range:  ±4 g
//       Provides improved resolution over ±8 g if dynamics permit.
//       If saturation is observed in flight logs, consider ±8 g.
//   - Data rate: 100 Hz
//       Matches AUX_PERIOD_MS=10 ms scheduling target.
//
// CONTRACT
//   Returns:
//     - true  if device is found at either address and configured.
//     - false if both begin() attempts fail.
//
// DETERMINISM / BOUNDS
//   - Two fixed begin() attempts; no loops.
//   - One-time configuration; intended for setup().
//
// FAILURE MODES
//   - Address mismatch, absent device, wiring fault -> false.
//
static bool init_lis3dh() {
  bool ok = lis3dh.begin(LIS3DH_I2C_ADDR_SA0_0, &I2C_BUS);
  if (!ok) ok = lis3dh.begin(LIS3DH_I2C_ADDR_SA0_1, &I2C_BUS);
  if (!ok) return false;

  // Configure measurement range and output data rate.
  lis3dh.setRange(LIS3DH_RANGE_4_G);
  lis3dh.setDataRate(LIS3DH_DATARATE_100_HZ);

  return true;
}




// ============================================================================
// SECTION 9: IMU update (MPU6050 epoch)
// ============================================================================
/*
imu_update()
------------------------------------------------------------------------------
ROLE
  Sample the MPU6050 at a fixed cadence and publish an ImuSample containing:
    - body-frame acceleration (m/s^2)
    - body-frame angular rate (rad/s)
    - roll/pitch estimates (deg) derived from the gravity vector
    - acceleration norm and a "motion_bad" quality flag

  This routine is the sole writer of imu_s.

COORDINATE FRAME + SIGN CONVENTIONS (CRITICAL)
  The Adafruit sensor event conventions provide axes in the sensor's configured
  orientation. The *meaning* of ax/ay/az depends on physical mounting.

  Definitions used here:
    - "Body frame" (B): axes aligned with the IMU breakout board.
    - Acceleration (ax,ay,az): proper acceleration measured in B, in m/s^2.
      At rest, magnitude ~ g and direction opposite gravity (depending on axis).
    - Roll and pitch:
        roll  = atan2(ay, az)           (deg)
        pitch = atan2(-ax, sqrt(ay^2+az^2)) (deg)

  These formulas assume:
    - Small linear acceleration relative to gravity (quasi-static) so the
      accelerometer vector approximates gravity direction.
    - Standard aerospace interpretation where roll is rotation about +X_B and
      pitch about +Y_B, but the exact mapping depends on mounting.

  If mounting differs (axis swaps/sign flips), adjust:
    - the mapping from raw ax/ay/az to logical body axes
    - and/or the roll/pitch formulas accordingly.

CONTRACT
  Returns:
    - false when epoch not due (no work performed).
    - true when due (attempted); imu_s.valid indicates success/failure.

PUBLISH POLICY
  - On success:
      imu_s.valid = true
      imu_s.t_ms  = now
      all fields updated coherently
  - On failure:
      imu_s.valid = false
      imu_s.t_ms  = now
      numeric fields may remain stale; validity must be checked by readers

DETERMINISM / BOUNDS
  - At most one I²C read transaction when due.
  - No loops, no dynamic allocation.

MOTION QUALITY HEURISTIC
  - During high dynamics (thrust, vibration), accel magnitude can deviate far
    from 1g and roll/pitch derived from accel become unreliable.
  - motion_bad flags accel norm outside a broad window (6..20 m/s^2).

STEP-BY-STEP
  (1) Time-gate on IMU_PERIOD_MS.
  (2) Acquire accel+gyro events in one call.
  (3) If acquisition fails: publish invalid with fresh timestamp.
  (4) Compute acceleration norm.
  (5) Compute roll and pitch from accel vector.
  (6) Publish snapshot and motion_bad flag.
*/
static bool imu_update() {
  const uint32_t now = millis();
  if ((now - last_imu_ms) < IMU_PERIOD_MS) return false;
  last_imu_ms = now;

  sensors_event_t a, g, t;
  const bool ok = mpu.getEvent(&a, &g, &t);

  if (!ok) {
    imu_s.valid = false;
    imu_s.t_ms  = now;
    return true;
  }

  const float ax = a.acceleration.x;
  const float ay = a.acceleration.y;
  const float az = a.acceleration.z;

  const float anorm = sqrtf(ax*ax + ay*ay + az*az);

  const float roll_deg  = rad2deg(atan2f(ay, az));
  const float pitch_deg = rad2deg(atan2f(-ax, sqrtf(ay*ay + az*az)));

  imu_s.ax = ax; imu_s.ay = ay; imu_s.az = az;
  imu_s.gx = g.gyro.x; imu_s.gy = g.gyro.y; imu_s.gz = g.gyro.z;

  imu_s.a_norm    = anorm;
  imu_s.roll_deg  = roll_deg;
  imu_s.pitch_deg = pitch_deg;

  imu_s.motion_bad = (anorm < 6.0f) || (anorm > 20.0f);

  imu_s.valid = true;
  imu_s.t_ms  = now;
  return true;
}


// ============================================================================
// SECTION 9A: Auxiliary accelerometer epoch (LIS3DH @ 100 Hz)
// ============================================================================
/*
lis3dh_update()
------------------------------------------------------------------------------
ROLE
  Perform one time-gated acquisition from the LIS3DH auxiliary accelerometer
  and publish the result into the AuxAccelSample snapshot (aux_s).

  This routine is the *sole writer* of aux_s and therefore owns:
    - aux_s.ax, ay, az
    - aux_s.a_norm
    - aux_s.motion_bad
    - aux_s.valid
    - aux_s.t_ms

  It performs at most one I²C transaction when the epoch is due.

WHY AN AUXILIARY ACCELEROMETER EXISTS
  The LIS3DH provides an independent acceleration measurement channel used for:
    - Projection into the world +Z axis (aux_vertical_linaccel_update()).
    - Providing a vertical linear acceleration input to the Kalman filter.
    - Cross-validation against the primary IMU channel if needed.

SCHEDULING MODEL
  This function is time-gated using:
      (now - last_aux_ms) >= AUX_PERIOD_MS

  If not due:
      - returns false
      - performs no I²C access
      - leaves aux_s unchanged

  If due:
      - updates last_aux_ms
      - attempts exactly one sensor read
      - returns true (indicating an epoch attempt occurred)

UNITS AND FRAME
  ev.acceleration.x/y/z are provided by the Adafruit Unified Sensor API
  in units of m/s^2, expressed in the LIS3DH sensor (body) frame.

  The mapping from sensor axes to rocket body axes depends on physical mounting.
  This function does not remap axes; it publishes raw sensor-frame values.
  Any required remapping must be handled explicitly in derived logic.

VALIDITY SEMANTICS
  On successful acquisition (ok == true):
      aux_s.valid = true
      aux_s.t_ms  = now
      aux_s.ax/ay/az and a_norm updated coherently.

  On acquisition failure (ok == false):
      aux_s.valid = false
      aux_s.t_ms  = now
      Numeric fields may remain from previous cycle and must not be trusted
      unless valid == true.

MOTION PLAUSIBILITY HEURISTIC
  aux_s.motion_bad is set using a coarse acceleration magnitude window:

      motion_bad = (a_norm < 6.0f) || (a_norm > 20.0f)

  Rationale:
    - At rest in 1g, |a| ≈ 9.81 m/s^2.
    - Values far below or far above nominal gravity suggest:
         * saturation
         * violent vibration
         * free-fall-like transient
         * sensor fault
    - Derived computations (e.g., gravity-based tilt or projection) may become
      unreliable under such conditions.

  The window [6, 20] m/s^2 is intentionally broad and should be tuned based on:
      - expected boost acceleration profile
      - vibration environment
      - sensor range configuration

DETERMINISM AND BOUNDS
  - At most one I²C transaction per due epoch.
  - No loops.
  - No dynamic allocation.
  - O(1) scalar math.

CONTRACT
  Returns:
    false  -> epoch not due; no work performed.
    true   -> epoch due; acquisition attempted; aux_s.valid indicates success.

FAILURE MODES
  - I²C bus fault or sensor error: ok == false.
  - Saturation: acceleration components may clip at sensor range limits.
  - Mounting misalignment: axes not aligned with rocket body; requires
    explicit transformation elsewhere.

STEP-BY-STEP
  (1) Read current time (millis).
  (2) If epoch not due, return false immediately.
  (3) Update last_aux_ms to enforce period.
  (4) Attempt to read LIS3DH sample.
  (5) If read fails:
        - mark aux_s.valid = false
        - update timestamp
        - return true
  (6) Compute acceleration magnitude (Euclidean norm).
  (7) Publish ax/ay/az, a_norm, valid=true, timestamp.
  (8) Compute motion_bad heuristic.
*/
static bool lis3dh_update() {
  const uint32_t now = millis();
  if ((now - last_aux_ms) < AUX_PERIOD_MS) return false;
  last_aux_ms = now;

  sensors_event_t ev;
  const bool ok = lis3dh.getEvent(&ev);

  if (!ok) {
    aux_s.valid = false;
    aux_s.t_ms = now;
    return true;
  }

  const float ax = ev.acceleration.x;
  const float ay = ev.acceleration.y;
  const float az = ev.acceleration.z;

  const float anorm = sqrtf(ax*ax + ay*ay + az*az);

  aux_s.ax = ax;
  aux_s.ay = ay;
  aux_s.az = az;
  aux_s.a_norm = anorm;
  aux_s.valid = true;
  aux_s.t_ms = now;

  aux_s.motion_bad = (anorm < 6.0f) || (anorm > 20.0f);

  return true;
}


// ============================================================================
// SECTION 9B: Derived aux vertical linear acceleration epoch (NO sensor I/O)
// ============================================================================
static constexpr float G0_MPS2 = 9.80665f;
/*
aux_vertical_linaccel_update()
------------------------------------------------------------------------------
ROLE
  Compute vertical acceleration in a world-aligned frame and remove gravity to
  obtain vertical *linear* acceleration suitable as the Kalman input u.

  This routine is the sole writer of aux_vz_s.

COORDINATE FRAMES (EXPLICIT)
  - Body frame (B): aligned with the LIS3DH sensor axes (after mounting).
  - World frame (W): defined as:
        +Z_W is "up" (opposite gravity)
        X_W and Y_W span the horizontal plane

  Roll and pitch from imu_update() describe orientation of B relative to W
  (yaw is irrelevant for the vertical axis projection).

SIGN CONVENTION
  - a_wz_mps2 is the acceleration component along +Z_W.
  - Gravity in world coordinates is approximately:
        a_gravity_W = -g * Z_W
    meaning that if an accelerometer measured "true acceleration including
    gravity," stationary would read approximately -g in +Z_W.
  - However, accelerometers measure *proper acceleration*, which for many
    conventions reads +g upward when stationary. The chosen projection and
    subtraction assume the current implementation’s sign conventions.

  The implemented computation does:
      a_wz = projection_of_body_accel_onto_Zw
      a_lin_z = a_wz - g0
  This matches the earlier code’s assumption that a_wz includes +g at rest.

  If mounting or sensor sign conventions differ, adjust:
    - axis mapping of (ax,ay,az)
    - projection formula sign
    - whether to subtract +g0 or add g0

CONTRACT
  Returns:
    - false when epoch not due.
    - true when due (attempted); aux_vz_s.valid indicates success.

DEPENDENCY VALIDATION
  Requires:
    - aux_s.valid and not aux_s.motion_bad
    - imu_s.valid and not imu_s.motion_bad
    - finite roll/pitch and accel components

DETERMINISM / BOUNDS
  - Pure math, no sensor I/O.
  - Bounded runtime.

STEP-BY-STEP
  (1) Time-gate on AUX_VZ_PERIOD_MS.
  (2) Default aux_vz_s to invalid with fresh timestamp.
  (3) Validate dependency snapshots and finiteness.
  (4) Convert roll/pitch to radians; compute sin/cos once.
  (5) Project aux accel from body frame onto world vertical axis Z_W.
  (6) Subtract g0 to obtain linear vertical acceleration.
  (7) Publish aux_vz_s.valid=true.
*/
static bool aux_vertical_linaccel_update() {
  const uint32_t now = millis();
  if ((now - last_auxvz_ms) < AUX_VZ_PERIOD_MS) return false;
  last_auxvz_ms = now;

  aux_vz_s.valid = false;
  aux_vz_s.t_ms  = now;
  aux_vz_s.a_wz_mps2    = NAN;
  aux_vz_s.a_lin_z_mps2 = NAN;

  const bool aux_ok =
      aux_s.valid &&
      !aux_s.motion_bad &&
      isfinite(aux_s.ax) && isfinite(aux_s.ay) && isfinite(aux_s.az);

  const bool imu_ok =
      imu_s.valid &&
      !imu_s.motion_bad &&
      isfinite(imu_s.roll_deg) &&
      isfinite(imu_s.pitch_deg);

  if (!aux_ok || !imu_ok) return true;

  const float roll  = deg2rad(imu_s.roll_deg);
  const float pitch = deg2rad(imu_s.pitch_deg);

  const float sr = sinf(roll);
  const float cr = cosf(roll);
  const float sp = sinf(pitch);
  const float cp = cosf(pitch);

  const float ax = aux_s.ax;
  const float ay = aux_s.ay;
  const float az = aux_s.az;

  // Projection of body acceleration onto world +Z axis using roll/pitch.
  const float a_wz =
      (-sp) * ax
    + (sr * cp) * ay
    + (cr * cp) * az;

  // Remove gravity component to obtain linear acceleration.
  const float a_lin_z = a_wz - G0_MPS2;

  aux_vz_s.a_wz_mps2    = a_wz;
  aux_vz_s.a_lin_z_mps2 = a_lin_z;
  aux_vz_s.valid        = true;
  return true;
}

// ============================================================================
// SECTION 10: Barometer update (BMP epoch)
// ============================================================================
/*
baro_update()
------------------------------------------------------------------------------
ROLE
  Sample the BMP5xx barometer at BARO_PERIOD_MS cadence and publish a BaroSample
  snapshot (pressure in hPa, temperature in °C).

  This routine is the sole writer of baro_s.

UNITS AND CONVERSIONS (EXPLICIT)
  - Adafruit BMP5xx exposes:
      bmp.pressure    in Pascals (Pa)
      bmp.temperature in degrees Celsius (°C)
  - This module publishes:
      baro_s.press_hpa = bmp.pressure / 100.0
    since:
      1 hPa = 100 Pa

TRACEABILITY SEMANTICS (valid + timestamp)
  - baro_s.t_ms is updated on every due epoch, regardless of success.
  - baro_s.valid distinguishes “fresh usable data” from “attempt failed”.
  - This supports meaningful staleness/age evaluation downstream.

DETERMINISM / BOUNDS
  - Time-gated: does no work if epoch is not due.
  - When due: performs at most one sensor acquisition attempt:
      bmp.performReading()
  - No loops, no delays, no dynamic allocation.

CONTRACT
  Returns:
    - false if not due (no side effects).
    - true  if due (attempt performed; baro_s.valid indicates success).

PUBLISH POLICY
  On success:
    - baro_s.valid = true
    - baro_s.press_hpa, baro_s.temp_c updated
    - baro_s.t_ms = now
  On failure:
    - baro_s.valid = false
    - baro_s.t_ms = now
    - numeric fields may remain stale; readers must check valid flag.

FAILURE MODES
  - I²C bus errors, sensor internal errors, or wiring faults can cause
    performReading() to return false. The failure is contained and reported via
    baro_s.valid=false with a fresh timestamp.
*/
static bool baro_update() {
  const uint32_t now = millis();
  if ((now - last_baro_ms) < BARO_PERIOD_MS) return false;
  last_baro_ms = now;

  // Attempt exactly one acquisition when due.
  if (!bmp.performReading()) {
    baro_s.valid = false;
    baro_s.t_ms  = now;
    return true;
  }

  // Publish in module units (hPa, °C).
  baro_s.temp_c    = bmp.temperature;
  baro_s.press_hpa = bmp.pressure * 0.01f;  // Pa -> hPa
  baro_s.valid     = true;
  baro_s.t_ms      = now;
  return true;
}



// ============================================================================
// SECTION 11: Estimation update (Kalman altitude + vertical speed)
// ============================================================================
static float pressure_to_altitude_m(float press_hpa, float ref_hpa) {
  if (press_hpa <= 0.0f || ref_hpa <= 0.0f) return NAN;
  return 44330.0f * (1.0f - powf(press_hpa / ref_hpa, 0.190294957f));
}


/*
estimate_update()
------------------------------------------------------------------------------
ROLE
  Publish a fused vertical state snapshot:
    - altitude (m)
    - vertical speed (m/s)
  by combining:
    - barometric altitude measurement (pressure -> altitude model)
    - optional vertical linear acceleration input (aux_vz_s)

  This function is the sole writer of fs_s and the sole routine that advances
  the internal Kalman filter state kf_alt.

CONTRACT
  Preconditions:
    - baro_update() has executed at some cadence (BARO_PERIOD_MS).
    - cfg.sea_level_hpa is a reasonable reference pressure.
    - If cfg.baro_baseline_hpa is set, it represents the launch-site reference.
  Postconditions:
    - When epoch not due: returns false, performs no work.
    - When due:
        - fs_s.t_ms updated to now regardless of success/failure.
        - fs_s.valid true only when a finite altitude measurement exists and
          the filter has been properly updated/seeded.
        - On invalid baro or invalid altitude conversion: fs_s.valid=false.
        - kf_alt seeded on first valid measurement after reset.

DETERMINISM / BOUNDS
  - No blocking calls.
  - One update per due epoch.
  - Pure scalar arithmetic; no dynamic allocation.

BASELINE SELECTION (WHY TWO REFERENCE PRESSURES EXIST)
  Pressure->altitude conversion requires a reference pressure p_ref.
  Two operational modes:
    (A) Relative altitude mode (preferred for flight control):
        - Capture baseline pressure at pad using CAP_BASELINE.
        - Use cfg.baro_baseline_hpa as p_ref.
        - Altitude becomes "meters above baseline," reducing weather bias.
    (B) Sea-level reference mode (fallback):
        - Use cfg.sea_level_hpa as p_ref when baseline not captured.
        - Altitude becomes "meters above sea-level estimate," more error-prone
          due to local pressure/weather variation.

  The code selects:
    p_ref = cfg.baro_baseline_hpa if finite else cfg.sea_level_hpa.

DT GUARD (WHY dt IS CHECKED)
  dt is derived from millis() deltas and is used in the Kalman predict step.
  If dt is extremely small (0 or near 0):
    - dt², dt³, dt⁴ terms in Q become ~0, and numerical updates can become
      meaningless or produce divide-by-small downstream behavior.
  If dt is extremely large (scheduler stall or wrap mishandling):
    - covariance can inflate dramatically and destabilize the filter.

  Mitigation strategy here:
    - Only run predict when dt > 0.0001 s (0.1 ms).
    - The scheduler cadence ensures dt typically ~ 0.02 s.
    - On reference-pressure changes, kf_alt is unseeded (reset) to prevent
      inconsistent state when measurement model changes.

STEP-BY-STEP
  (1) Time-gate at EST_PERIOD_MS.
  (2) Require baro_s.valid; on failure publish fs_s.valid=false.
  (3) Select reference pressure p_ref:
        baseline if finite else sea-level setting.
  (4) Convert pressure -> altitude z_alt using ISA barometric formula.
  (5) If unseeded: seed filter with z_alt, publish altitude and vz=0.
  (6) Compute dt from scheduler delta in seconds.
  (7) Choose acceleration input u:
        - u = aux_vz_s.a_lin_z_mps2 if aux_vz_s valid and finite
        - else u = 0 (coast model)
  (8) Predict state/covariance using constant-acceleration kinematics.
  (9) Update filter with measurement z_alt.
 (10) Publish fs_s from kf_alt.

FAILURE MODES
  - baro invalid: fs_s.valid=false; timestamp updated.
  - p_ref invalid (<=0): altitude conversion returns NaN -> invalid publish.
  - aux vertical accel invalid: u forced to 0 (safe fallback).
*/
static bool estimate_update() {
  const uint32_t now = millis();
  const uint32_t dt_ms = now - last_est_ms;
  if (dt_ms < EST_PERIOD_MS) return false;
  last_est_ms = now;

  // --- (2) Measurement availability gate ------------------------------------
  if (!baro_s.valid) {
    fs_s.valid = false;
    fs_s.t_ms = now;
    return true;
  }

  // --- (3) Reference pressure selection -------------------------------------
  float ref_hpa = cfg.sea_level_hpa;
  if (!isnan(cfg.baro_baseline_hpa)) ref_hpa = cfg.baro_baseline_hpa;

  // --- (4) Convert pressure -> altitude -------------------------------------
  const float alt_raw = pressure_to_altitude_m(baro_s.press_hpa, ref_hpa);
  if (isnan(alt_raw) || !isfinite(alt_raw)) {
    fs_s.valid = false;
    fs_s.t_ms = now;
    return true;
  }

  // --- (5) Seed behavior -----------------------------------------------------
  if (!kf_alt.seeded) {
    kf_alt2_reset(kf_alt, alt_raw, now);
    fs_s.altitude_m = kf_alt.x0;
    fs_s.vz_mps     = kf_alt.x1;
    fs_s.valid      = true;
    fs_s.t_ms       = now;
    return true;
  }

  // --- (6) dt computation + guard -------------------------------------------
  const float dt = dt_ms * 0.001f;
  if (dt > 0.0001f && isfinite(dt)) {
    // --- (7) Input acceleration selection -----------------------------------
    const float u_acc =
        (aux_vz_s.valid && isfinite(aux_vz_s.a_lin_z_mps2))
          ? aux_vz_s.a_lin_z_mps2
          : 0.0f;

    // --- (8) Predict step ----------------------------------------------------
    kf_alt2_predict_u(kf_alt, dt, u_acc);
  }

  // --- (9) Measurement update ------------------------------------------------
  kf_alt2_update_z(kf_alt, alt_raw);

  // --- (10) Publish fused state snapshot ------------------------------------
  fs_s.altitude_m = kf_alt.x0;
  fs_s.vz_mps     = kf_alt.x1;
  fs_s.valid      = true;
  fs_s.t_ms       = now;

  kf_alt.t_ms = now;
  return true;

}



// ============================================================================
// SECTION 12: Magnetometer + heading (tilt-compensated)
// ============================================================================

/*
SECTION INTENT
------------------------------------------------------------------------------
This section provides the math primitives for producing a *tilt-compensated*
magnetic heading from a 3-axis magnetometer, plus a lightweight smoothing
mechanism (EMA) and a coarse interference gate based on field magnitude.

Design goals:
  - Explicit calibration application (hard-iron + soft-iron scale).
  - Deterministic, bounded math suitable for real-time loop integration.
  - Clear separation between:
      (A) calibration transform (mag_apply_cal)
      (B) heading math (heading_tilt_comp_deg)
      (C) higher-level validity gating (performed by mag_update elsewhere)

Key exported concepts:
  - ema_heading_deg: retained EMA-smoothed heading (degrees, [0,360))
  - EMA_ALPHA_HEADING: smoothing factor for heading updates
  - MAG_NORM_MIN_uT / MAG_NORM_MAX_uT: plausible magnetic-field magnitude window

NOTES ON UNITS AND FRAMES
------------------------------------------------------------------------------
- Magnetometer inputs are in microtesla (uT) per Adafruit unified sensor API.
- roll_deg and pitch_deg are in degrees and must be consistent with IMU
  conventions used elsewhere.
- Declination is in degrees and is *added* to convert from magnetic north to
  (approximate) true north.

The heading model assumes:
  - The magnetometer axes are fixed in the body frame (sensor frame).
  - roll/pitch describe body attitude relative to a world frame where +Z is up.
  - Yaw is the heading being estimated; yaw is not needed for the tilt
    compensation itself.

SENSOR ENVIRONMENT ASSUMPTION
------------------------------------------------------------------------------
The Earth’s magnetic field magnitude depends on location but is typically on
the order of ~25..65 uT. The chosen window [10, 100] uT is intentionally wide
to tolerate moderate disturbances while still flagging severe interference.
*/

// Retained exponential moving average (EMA) of heading in degrees.
// Updated only when heading is considered valid by higher-level gating.
static float ema_heading_deg = NAN;

// EMA smoothing factor.
// Interpretation: alpha=0.20 means ~20% new sample, ~80% previous EMA each update.
static constexpr float EMA_ALPHA_HEADING = 0.20f;

// Coarse plausibility bounds for |B| in microtesla (uT).
// Used to detect gross interference or sensor faults.
static constexpr float MAG_NORM_MIN_uT = 10.0f;
static constexpr float MAG_NORM_MAX_uT = 100.0f;

/*
mag_apply_cal(x, y, z, c)
------------------------------------------------------------------------------
ROLE
  Apply a simple magnetometer calibration model (hard-iron offsets + per-axis
  soft-iron scale) in-place to a 3D magnetic field vector.

CALIBRATION MODEL
  This routine applies:

      x_cal = (x_raw - off_x) * scl_x
      y_cal = (y_raw - off_y) * scl_y
      z_cal = (z_raw - off_z) * scl_z

  where:
    - off_* correct hard-iron bias (constant additive field from nearby magnets/
      ferromagnetic parts).
    - scl_* correct per-axis gain distortion (soft-iron scaling approximation).

  This is a *diagonal* correction model (no axis cross-coupling). It is not a
  full ellipsoid fit, but is deterministic and robust for embedded use.

CONTRACT
  Preconditions:
    - x, y, z are finite magnetometer samples (raw or intermediate).
    - c contains calibration parameters; c.valid is checked by the caller.
  Postconditions:
    - x, y, z replaced with calibrated values.

DETERMINISM / BOUNDS
  - O(1) scalar operations; no loops, no allocations.

FAILURE MODES
  - If calibration fields are NaN/inf, outputs become NaN/inf.
    Caller should gate by c.valid and finiteness checks if required.
*/
static inline void mag_apply_cal(float& x, float& y, float& z, const MagCal& c) {
  x = (x - c.off_x) * c.scl_x;
  y = (y - c.off_y) * c.scl_y;
  z = (z - c.off_z) * c.scl_z;
}

/*
heading_tilt_comp_deg(mx, my, mz, roll_deg, pitch_deg, declination_deg)
------------------------------------------------------------------------------
ROLE
  Compute a *tilt-compensated* compass heading (degrees, wrapped to [0,360))
  from:
    - calibrated magnetometer body-frame components (mx,my,mz) in uT
    - roll and pitch angles (deg) describing attitude
    - local magnetic declination (deg)

WHY TILT COMPENSATION EXISTS
  A naive compass heading uses only mx,my (assuming the sensor is level).
  When the airframe is pitched/rolled, the magnetometer’s horizontal components
  are contaminated by the vertical component, producing large yaw errors.

  Tilt compensation reprojects the measured magnetic field vector onto the
  horizontal plane using roll/pitch before computing atan2.

MATHEMATICAL MODEL (BOTTOM-UP)
  Let the measured (calibrated) magnetic field vector in body frame be:

      m_B = [mx, my, mz]^T

  With roll (φ) and pitch (θ), the horizontal components can be expressed as:

      Xh = mx * cosθ + mz * sinθ
      Yh = mx * sinφ * sinθ + my * cosφ - mz * sinφ * cosθ

  Then heading (magnetic) is:

      hdg_mag = atan2(Yh, Xh)  (radians)

  Convert to degrees, add declination, and wrap to [0,360).

ANGLE CONVENTIONS AND DEPENDENCIES
  - roll_deg and pitch_deg must match the conventions used to derive them.
  - The formulas above assume a conventional aerospace interpretation where:
      roll  is rotation about body X axis,
      pitch is rotation about body Y axis,
    and that the chosen sign conventions align with the trig expressions.

  If sensor axes or attitude conventions differ (mounting swaps/sign flips),
  the heading will be biased or mirrored; adjust axis mapping and/or formulas.

DECLINATION APPLICATION
  declination_deg is added:
      true_heading ≈ magnetic_heading + declination
  Positive declination means magnetic north is east of true north (common GIS
  convention). Ensure sign matches the chosen declination source.

CONTRACT
  Preconditions:
    - mx,my,mz are finite calibrated samples.
    - roll_deg, pitch_deg are finite.
  Postconditions:
    - returns heading in degrees in [0,360).
    - no global state is mutated.

DETERMINISM / BOUNDS
  - O(1) scalar trig operations; no loops, no allocation.

FAILURE MODES
  - If inputs are NaN/inf, output becomes NaN.
  - In extreme interference, Xh and Yh may be near zero; atan2 remains defined
    but heading becomes noise-dominated. Higher-level gating should suppress.

STEP-BY-STEP
  (1) Convert roll/pitch degrees to radians.
  (2) Compute tilt-compensated horizontal components (Xh, Yh).
  (3) Compute atan2(Yh, Xh) and convert to degrees.
  (4) Add declination and wrap to [0,360).
*/
static float heading_tilt_comp_deg(float mx, float my, float mz,
                                   float roll_deg, float pitch_deg,
                                   float declination_deg) {
  const float roll  = deg2rad(roll_deg);
  const float pitch = deg2rad(pitch_deg);

  const float Xh = mx * cosf(pitch) + mz * sinf(pitch);

  const float Yh =
      mx * sinf(roll) * sinf(pitch)
    + my * cosf(roll)
    - mz * sinf(roll) * cosf(pitch);

  float hdg = rad2deg(atan2f(Yh, Xh));
  hdg = wrap360(hdg + declination_deg);
  return hdg;
}

/*
mag_update()
------------------------------------------------------------------------------
ROLE
  Sample the LIS2MDL magnetometer at MAG_PERIOD_MS cadence and publish a
  magnetometer snapshot plus a tilt-compensated heading estimate.

  This routine is the sole writer of mag_s.
  The EMA-smoothed heading output is maintained in ema_heading_deg.

WHAT "VALID" MEANS HERE
  mag_s.valid indicates that a heading was computed and is considered trustworthy
  under the current gating rules. A valid heading requires:
    (V1) Magnetometer calibration available (cfg.valid && cfg.mag_cal.valid)
    (V2) Attitude available and not flagged as motion_bad (imu_s.valid, etc.)
    (V3) Magnetic field norm within plausible bounds (no interference)
  If any gate fails, mag_s.valid=false and heading_deg is set to NAN.

UNITS AND SENSOR SEMANTICS
  - Adafruit unified sensor API returns magnetic field in microtesla (uT).
  - norm_uT = sqrt(mx^2 + my^2 + mz^2) provides a coarse interference check.

INTERFERENCE HEURISTIC (WHY IT EXISTS)
  The heading computation assumes the measured field is dominated by Earth's
  field plus modest distortions. Nearby ferromagnetic materials, current loops,
  or electronics can distort the field magnitude dramatically.
  A broad “plausible norm” window is used:
    MAG_NORM_MIN_uT .. MAG_NORM_MAX_uT
  This is a heuristic and should be tuned per airframe environment.

TILT COMPENSATION DEPENDENCY
  Tilt-compensated heading requires roll/pitch. Those are derived from the IMU
  accelerometer gravity estimate and become unreliable under strong linear
  acceleration or vibration. Therefore the imu motion_bad gate is required.

TRACEABILITY SEMANTICS
  - mag_s.t_ms is updated on every due epoch regardless of validity.
  - raw/cal fields are always updated to aid debugging even when heading is
    declared invalid.

DETERMINISM / BOUNDS
  - Time-gated.
  - Exactly one magnetometer read per due call.
  - No loops, no delays, no dynamic allocation.

CONTRACT
  Returns:
    - false if not due (no work done).
    - true if due (attempt performed; heading validity encoded in mag_s.valid).

STEP-BY-STEP
  (1) Time-gate using last_mag_ms and MAG_PERIOD_MS.
  (2) Read magnetometer sample (raw).
  (3) Apply calibration if available.
  (4) Compute norm and interference flag.
  (5) Validate prerequisites: calibration, IMU attitude, no interference.
  (6) If valid, compute tilt-compensated heading and update EMA.
  (7) Publish mag_s snapshot.
*/
static bool mag_update() {
  const uint32_t now = millis();
  if ((now - last_mag_ms) < MAG_PERIOD_MS) return false;
  last_mag_ms = now;

  // --- (2) Acquire one sample ------------------------------------------------
  sensors_event_t ev;
  mag.getEvent(&ev);

  const float rawx = ev.magnetic.x;
  const float rawy = ev.magnetic.y;
  const float rawz = ev.magnetic.z;

  // --- (3) Apply calibration if available -----------------------------------
  float mx = rawx;
  float my = rawy;
  float mz = rawz;

  const bool cal_ok = cfg.valid && cfg.mag_cal.valid;
  if (cal_ok) mag_apply_cal(mx, my, mz, cfg.mag_cal);

  // --- (4) Norm + interference heuristic ------------------------------------
  const float norm_uT = sqrtf(mx*mx + my*my + mz*mz);
  const bool interf = (norm_uT < MAG_NORM_MIN_uT) || (norm_uT > MAG_NORM_MAX_uT);

  // --- Publish raw/cal regardless (observability) ----------------------------
  mag_s.raw_x = rawx; mag_s.raw_y = rawy; mag_s.raw_z = rawz;
  mag_s.cal_x = mx;   mag_s.cal_y = my;   mag_s.cal_z = mz;
  mag_s.norm_uT = norm_uT;
  mag_s.interference = interf;
  mag_s.t_ms = now;

  // --- (5) Prerequisite gates -----------------------------------------------
  const bool imu_ok =
      imu_s.valid &&
      !imu_s.motion_bad &&
      isfinite(imu_s.roll_deg) &&
      isfinite(imu_s.pitch_deg);

  if (!cal_ok || !imu_ok || interf || !isfinite(norm_uT)) {
    mag_s.valid = false;
    mag_s.heading_deg = NAN;
    return true;
  }

  // --- (6) Tilt-compensated heading -----------------------------------------
  const float hdg =
      heading_tilt_comp_deg(mx, my, mz,
                            imu_s.roll_deg, imu_s.pitch_deg,
                            cfg.declination_deg);

  mag_s.heading_deg = hdg;
  mag_s.valid = true;

  // EMA is only updated when heading is considered valid.
  ema_heading_deg = ema_update(ema_heading_deg, hdg, EMA_ALPHA_HEADING);
  return true;
}

/*
mag_calibrate_capture(duration_ms)
------------------------------------------------------------------------------
ROLE
  Perform a simple magnetometer calibration capture (hard-iron + soft-iron scale)
  by collecting axis-wise min/max values while the sensor is rotated through
  many orientations.

  This routine updates:
    - cfg.mag_cal (offsets + scale factors + diagnostic radii)
    - cfg.valid = true

INTENTIONALLY BLOCKING (IMPORTANT SAFETY NOTE)
  This function is NOT part of the deterministic flight loop.
  It is operator-invoked (via CAL_MAG command) and uses:
    - a while loop over time
    - delay(20)
  The blocking behavior is acceptable only in a controlled calibration context.

CALIBRATION MODEL (BOTTOM-UP)
  A raw magnetometer measurement can be modeled as:
      m_raw = A * m_true + b
  where:
      b  is hard-iron bias (constant offset)
      A  includes soft-iron scaling and axis coupling

  This routine implements a simplified approximation:
    - hard-iron bias b estimated as midpoint of min/max per axis:
        off = (max + min) / 2
    - soft-iron scaling estimated from per-axis “radius”:
        r = (max - min) / 2
      and scale factors that normalize radii:
        scl_axis = r_avg / r_axis
      where r_avg = mean(r_x, r_y, r_z)

  This does NOT estimate axis coupling (no full ellipsoid fit), but yields a
  robust improvement over uncalibrated heading for many practical setups.

CONTRACT
  Preconditions:
    - Magnetometer initialized and responsive.
    - Operator rotates sensor slowly through many orientations.
  Postconditions:
    - cfg.mag_cal.valid set true iff radii are positive and finite.
    - Always prints calibration results to Serial (diagnostics).
    - cfg.valid set true.

SAMPLE/PRINT POLICY
  - Samples are taken as fast as loop allows but throttled by delay(20).
  - Progress prints every ~250ms to show remaining time.

FAILURE MODES
  - If motion is insufficient (not enough orientation coverage), min/max may be
    too close => radii near zero => calibration invalid.
  - If sensor reports NaNs/inf (rare), results may be invalid; validity flag
    will remain false due to radius checks.

STEP-BY-STEP
  (1) Initialize min values to huge +, max to huge -.
  (2) Loop until duration_ms elapsed:
        - read magnetometer
        - update min/max per axis
        - periodically print remaining time
        - delay(20)
  (3) Compute offsets as midpoints.
  (4) Compute radii as half-ranges.
  (5) Compute average radius and per-axis scale.
  (6) Validate and store calibration; print OFF/SCL/RAD lines.
*/
static void mag_calibrate_capture(uint32_t duration_ms) {
  float minx = 1e9f, miny = 1e9f, minz = 1e9f;
  float maxx = -1e9f, maxy = -1e9f, maxz = -1e9f;

  const uint32_t t0 = millis();
  uint32_t last_print = 0;

  Serial.println("CAL,START");

  while ((millis() - t0) < duration_ms) {
    sensors_event_t ev;
    mag.getEvent(&ev);

    const float x = ev.magnetic.x;
    const float y = ev.magnetic.y;
    const float z = ev.magnetic.z;

    // Update bounds only for finite samples; ignore pathological values.
    if (isfinite(x)) { if (x < minx) minx = x; if (x > maxx) maxx = x; }
    if (isfinite(y)) { if (y < miny) miny = y; if (y > maxy) maxy = y; }
    if (isfinite(z)) { if (z < minz) minz = z; if (z > maxz) maxz = z; }

    const uint32_t now = millis();
    if ((now - last_print) >= 250) {
      last_print = now;
      const uint32_t elapsed = now - t0;
      const uint32_t rem = (elapsed < duration_ms) ? (duration_ms - elapsed) : 0u;
      Serial.print("CAL,PROG,");
      Serial.println(rem);
    }

    delay(20);
  }

  MagCal c;

  // Hard-iron offsets (midpoint of bounds)
  c.off_x = (maxx + minx) * 0.5f;
  c.off_y = (maxy + miny) * 0.5f;
  c.off_z = (maxz + minz) * 0.5f;

  // Radii (half-range)
  c.rx = (maxx - minx) * 0.5f;
  c.ry = (maxy - miny) * 0.5f;
  c.rz = (maxz - minz) * 0.5f;

  const bool radii_ok =
      isfinite(c.rx) && isfinite(c.ry) && isfinite(c.rz) &&
      (c.rx > 1e-6f) && (c.ry > 1e-6f) && (c.rz > 1e-6f);

  if (radii_ok) {
    const float ravg = (c.rx + c.ry + c.rz) / 3.0f;
    c.scl_x = ravg / c.rx;
    c.scl_y = ravg / c.ry;
    c.scl_z = ravg / c.rz;
    c.valid = isfinite(c.scl_x) && isfinite(c.scl_y) && isfinite(c.scl_z);
    Serial.println(c.valid ? "CAL,DONE,OK" : "CAL,DONE,FAIL");
  } else {
    c.scl_x = 1.0f; c.scl_y = 1.0f; c.scl_z = 1.0f;
    c.valid = false;
    Serial.println("CAL,DONE,FAIL");
  }

  cfg.mag_cal = c;
  cfg.valid = true;

  Serial.print("CAL,OFF,");
  Serial.print(c.off_x, 6); Serial.print(',');
  Serial.print(c.off_y, 6); Serial.print(',');
  Serial.println(c.off_z, 6);

  Serial.print("CAL,SCL,");
  Serial.print(c.scl_x, 6); Serial.print(',');
  Serial.print(c.scl_y, 6); Serial.print(',');
  Serial.println(c.scl_z, 6);

  Serial.print("CAL,RAD,");
  Serial.print(c.rx, 6); Serial.print(',');
  Serial.print(c.ry, 6); Serial.print(',');
  Serial.println(c.rz, 6);
}

// ============================================================================
// SECTION 13: Airbrake actuator abstraction
// ============================================================================

/*
CLASS INTENT: AirbrakeActuator
------------------------------------------------------------------------------
This class provides a minimal, deterministic abstraction layer between
flight-control logic and the physical servo hardware that actuates the
airbrake mechanism.

Design goals:
  - Encapsulate all servo pulse generation in one location.
  - Separate *command intent* (normalized [0,1]) from hardware units (µs).
  - Provide a runtime enable gate that forces safe idle on disable.
  - Maintain deterministic, bounded execution (no blocking, no allocation).

ARCHITECTURAL ROLE
  High-level control (policy) produces a normalized command u ∈ [0,1].
  AirbrakeActuator is responsible for:
    - mapping u to a pulse width,
    - enforcing idle behavior when disabled,
    - abstracting backend differences (Servo vs PWMServo).

SAFETY PHILOSOPHY
  - The actuator should default to a known idle position.
  - Disabling must *immediately* drive the servo to idle.
  - No hidden state should cause motion when disabled.write_us

STATE VARIABLES (PRIVATE)
  pin_        : GPIO pin used for servo output.
  us_min_     : pulse width (µs) corresponding to minimum deployment.
  us_max_     : pulse width (µs) corresponding to maximum deployment.
  us_idle_    : pulse width (µs) used when disabled or forced idle.
  enabled_    : runtime gate (true allows motion, false forces idle).
  servo_      : backend servo driver object.

BACKEND ABSTRACTION
  Depending on platform:
    - Teensy: PWMServo (write() expects microseconds directly).
    - Arduino Servo: writeMicroseconds() used explicitly.
  write_us_() encapsulates this distinction.
*/















































// ============================================================================
// SECTION 13: Airbrake actuator abstraction
// ============================================================================

/*
AirbrakeActuator
------------------------------------------------------------------------------
Minimal deterministic actuator wrapper.

Responsibilities:
  - Attach servo backend and store calibration limits.
  - Map normalized command [0,1] to microsecond pulse width.
  - Enforce runtime enable gate and compile-time actuation gate.
  - Provide an explicit idle command path.
*/
class AirbrakeActuator {
public:

/*
begin(pin, us_min, us_max, us_idle)
------------------------------------------------------------------------------
ROLE
  Initialize and attach the servo backend to the specified GPIO pin and
  configure pulse-width calibration limits.

CALIBRATION SEMANTICS
  us_min:
    Pulse width (µs) corresponding to u01 = 0.0
    (e.g., fully retracted or minimum brake extension).

  us_max:
    Pulse width (µs) corresponding to u01 = 1.0
    (e.g., maximum brake extension).

  us_idle:
    Pulse width (µs) commanded when actuator is disabled or forced idle.
    Often equal to us_min but may differ for mechanical preload reasons.

CONTRACT
  Preconditions:
    - pin is a valid PWM-capable pin for the target board.
    - us_min, us_max, us_idle are mechanically safe for the servo.
  Postconditions:
    - Servo is attached to pin.
    - Servo is immediately commanded to idle pulse.
    - enabled_ is set true (actuation allowed, subject to higher-level gates).
    - Returns true (current implementation does not fail attach).

DETERMINISM / BOUNDS
  - Single servo_.attach() call.
  - Single pulse write.
  - No loops, no dynamic allocation.

FAILURE MODES
  - If pin is invalid or hardware misconfigured, servo may not respond.
  - No internal validation of pulse ranges; incorrect calibration may cause
    mechanical overtravel. Such validation is a system integration task.
*/

  bool begin(uint8_t pin, int us_min, int us_max, int us_idle) {
    pin_ = pin;
    us_min_  = us_min;
    us_max_  = us_max;
    us_idle_ = us_idle;

    servo_.attach(pin_);
    write_us_(us_idle_);
    enabled_ = true;
    return true;
  }


/*
AirbrakeActuator::setCommand01(u01)
------------------------------------------------------------------------------
ROLE
  Convert a normalized command u01 ∈ [0,1] into a servo pulse width and apply it
  to the actuator output, subject to both runtime and compile-time safety gates.

SAFETY GATES (TWO LAYERS)
  Layer A (runtime):
    enabled_ must be true.
    - enabled_ is controlled by arming state logic.
    - disabling forces idle immediately.

  Layer B (compile-time):
    ACTUATION_ENABLED must be nonzero at compile time.
    - If compiled off, no runtime state can cause motion.

  Both layers must allow actuation for a non-idle command to be emitted.

CONTRACT
  Preconditions:
    - u01 may be outside [0,1]; function clamps it deterministically.
    - Servo must have been attached in begin().
  Postconditions:
    - If enabled_ == false => idle pulse is emitted.
    - Else if ACTUATION_ENABLED == 0 => idle pulse is emitted.
    - Else => a pulse in [us_min_, us_max_] is emitted based on u01.

MAPPING (LINEAR INTERPOLATION)
  The mapping used is:
      us = (1-u01)*us_min_ + u01*us_max_
  meaning:
      u01 = 0 -> us_min_
      u01 = 1 -> us_max_

  This is a reasonable baseline; mechanical linkage may require a nonlinear map
  in future revisions.

DETERMINISM / BOUNDS
  - Constant time, no loops.
  - No dynamic allocation.

FAILURE MODES
  - None internal; incorrect calibration or reversed servo direction manifests
    as mechanical behavior. Swap us_min/us_max or invert u01 if needed.
*/
void setCommand01(float u01) {
  // (1) Clamp normalized input.
  if (u01 < 0.0f) u01 = 0.0f;
  if (u01 > 1.0f) u01 = 1.0f;

  // (2) Runtime gate.
  if (!enabled_) {
    write_us_(us_idle_);
    return;
  }

  // (3) Compile-time gate.
#if ACTUATION_ENABLED
  // (4) Linear mapping to microseconds.
  const int us = (int)lroundf((1.0f - u01) * us_min_ + u01 * us_max_);
  write_us_(us);
#else
  write_us_(us_idle_);
#endif
}



/*
setEnabled(en)
------------------------------------------------------------------------------
ROLE
  Runtime enable/disable gate for actuator motion.

SAFETY BEHAVIOR
  - If en == false:
      * enabled_ set false
      * servo immediately commanded to idle pulse (us_idle_)
  - If en == true:
      * enabled_ set true
      * no immediate movement is commanded here
        (movement occurs only via setCommand01())

RATIONALE
  This method allows the arming state machine to:
    - disable actuation instantly on DISARM or SAFE,
    - guarantee that no residual command persists,
    - separate logical arming state from servo driver logic.

CONTRACT
  Preconditions:
    - begin() has been called.
  Postconditions:
    - enabled_ reflects en.
    - If disabling, servo output is forced to idle.

DETERMINISM / BOUNDS
  - Constant-time.
  - No loops, no allocation.

FAILURE MODES
  - None internally; hardware-level servo issues are external.
*/
void setEnabled(bool en) {
  enabled_ = en;

  // Immediate failsafe: drive to idle when disabling.
  if (!enabled_) {
    write_us_(us_idle_);
  }
}


  /*
  forceIdle()
  ----------------------------------------------------------------------------
  Force idle pulse regardless of enable state.
  */
  void forceIdle() { write_us_(us_idle_); }

private:
  /*
  write_us_(us)
  ----------------------------------------------------------------------------
  Backend-specific microsecond write.
  */
  void write_us_(int us) {
#if defined(ARDUINO_TEENSY41) || defined(TEENSYDUINO)
    servo_.write(us);               // PWMServo expects microseconds
#else
    servo_.writeMicroseconds(us);   // Servo expects microseconds explicitly
#endif
  }

  ServoBackend servo_;
  uint8_t pin_ = 255;

  int us_min_  = 1000;
  int us_max_  = 2000;
  int us_idle_ = 1000;

  bool enabled_ = false;
};

static AirbrakeActuator airbrake;












// ============================================================================
// SECTION 13A: Non-blocking buzzer chirps (optional observability)
// ============================================================================
struct BuzzerChirp {
  bool active = false;
  uint8_t remaining = 0;
  bool high = false;
  uint32_t t_next = 0;
  uint16_t on_ms = 40;
  uint16_t off_ms = 60;
};

static BuzzerChirp buzzer;

static void buzzer_start(uint8_t pulses, uint16_t on_ms, uint16_t off_ms) {
  buzzer.active = (pulses > 0);
  buzzer.remaining = pulses;
  buzzer.high = false;
  buzzer.t_next = millis();
  buzzer.on_ms = on_ms;
  buzzer.off_ms = off_ms;
  digitalWrite(PIN_BUZZER, LOW);
}
/*
buzzer_update()
------------------------------------------------------------------------------
ROLE
  Advance the non-blocking buzzer chirp state machine by at most one edge
  transition per call.

CONTRACT
  Preconditions:
    - buzzer_start() has configured the BuzzerChirp state when a chirp is desired.
    - PIN_BUZZER is configured as OUTPUT.
  Postconditions:
    - If buzzer.active == false: no GPIO changes occur.
    - If buzzer.active == true:
        - toggles output state when now >= buzzer.t_next
        - decrements buzzer.remaining after each completed low phase
        - deactivates and forces LOW when remaining reaches 0

DETERMINISM / BOUNDS
  - No blocking calls.
  - No loops.
  - Constant time.

SAFETY
  - On completion, output is forced LOW to prevent a stuck-on buzzer.
  - Time comparison uses signed delta for wrap-tolerant scheduling.

STEP-BY-STEP
  (1) Return immediately when inactive.
  (2) Check whether the scheduled transition time has been reached.
  (3) Toggle the output (high <-> low).
  (4) If transitioning to low:
        - decrement remaining pulse count
        - if remaining is now 0: stop and force LOW
        - else schedule next high transition after off_ms
  (5) If transitioning to high:
        - schedule next low transition after on_ms
*/
static void buzzer_update() {
  if (!buzzer.active) return;

  const uint32_t now = millis();

  // Wrap-tolerant "now >= t_next" check.
  if ((int32_t)(now - buzzer.t_next) < 0) return;

  // Toggle output level.
  buzzer.high = !buzzer.high;
  digitalWrite(PIN_BUZZER, buzzer.high ? HIGH : LOW);

  if (!buzzer.high) {
    // A full pulse is counted on the falling edge (transition to LOW).
    if (buzzer.remaining > 0u) buzzer.remaining--;

    if (buzzer.remaining == 0u) {
      buzzer.active = false;
      digitalWrite(PIN_BUZZER, LOW);
      return;
    }

    buzzer.t_next = now + buzzer.off_ms;
  } else {
    buzzer.t_next = now + buzzer.on_ms;
  }
}
// ============================================================================
// SECTION 14: Arming state machine
// ============================================================================

/*
SECTION INTENT
------------------------------------------------------------------------------
This section implements the *arming state machine* that governs whether the
airbrake actuator is permitted to move.

This layer is safety-critical.

It provides:
  - A physical switch input (PIN_ARM_SWITCH).
  - A software token requirement (sw_arm_token).
  - A three-state arming model:
        DISARMED  -> SAFE -> ARMED
  - A hard runtime gate into the AirbrakeActuator.

DESIGN PHILOSOPHY
------------------------------------------------------------------------------
Actuation must require *both*:

  (1) Physical intent (hardware arm switch HIGH)
  (2) Software authorization (correct ARM token command)

This prevents:
  - Accidental actuation due to switch bump alone.
  - Accidental actuation due to stray serial command alone.
  - Latent actuator enable after switch drop.

STATE DEFINITIONS
------------------------------------------------------------------------------
ArmingState::DISARMED
  - Physical switch LOW.
  - sw_arm_token cleared.
  - Actuator disabled and forced to idle.

ArmingState::SAFE
  - Physical switch HIGH.
  - No valid software token.
  - Actuator disabled and forced to idle.
  - System electrically “ready” but logically not armed.

ArmingState::ARMED
  - Physical switch HIGH.
  - sw_arm_token true (correct token provided).
  - Actuator runtime-enabled.
  - Policy layer may command motion (subject to its own gates).

SAFETY INVARIANT
------------------------------------------------------------------------------
Actuator motion is permitted only if:

    arm_state == ARMED

and higher layers (policy + compile-time gates) also allow motion.

This state machine is the *first runtime gate* in the actuation chain.
*/

/*
read_arm_switch()
------------------------------------------------------------------------------
ROLE
  Read the physical arming switch input and return its logical state.

HARDWARE ASSUMPTION
  PIN_ARM_SWITCH is configured as INPUT_PULLDOWN in setup().
  Therefore:
      HIGH  -> switch asserted
      LOW   -> switch not asserted

CONTRACT
  Returns:
    true  -> switch asserted
    false -> switch not asserted

DETERMINISM / BOUNDS
  - Single digitalRead().
  - Constant time.
*/
static bool read_arm_switch() {
  return digitalRead(PIN_ARM_SWITCH) == HIGH;
}

/*
set_arm_state(st)
------------------------------------------------------------------------------
ROLE
  Transition the global arming state and enforce the corresponding actuator
  enable/disable behavior.

WHY THIS FUNCTION EXISTS
  Centralizing state transitions ensures:
    - actuator enable logic is consistent,
    - forced idle is guaranteed on non-ARMED states,
    - no duplicated safety logic exists elsewhere.

CONTRACT
  Preconditions:
    - st is a valid ArmingState enum value.
  Postconditions:
    - arm_state updated.
    - AirbrakeActuator::setEnabled() called with:
          true  if st == ARMED
          false otherwise.
    - If not ARMED, actuator is explicitly forced to idle.

SAFETY GUARANTEE
  Any transition away from ARMED immediately forces:
      enabled_ = false
      servo pulse = us_idle_

  This ensures no stale non-zero command persists.

DETERMINISM / BOUNDS
  - O(1) logic.
  - No loops.
  - No dynamic allocation.
*/
static void set_arm_state(ArmingState st) {
  arm_state = st;

  const bool en = (arm_state == ArmingState::ARMED);

  // Runtime actuator gate.
  airbrake.setEnabled(en);

  // Explicit mechanical failsafe for non-ARMED states.
  if (!en) {
    airbrake.forceIdle();
  }
}

/*
update_arming()
------------------------------------------------------------------------------
ROLE
  Evaluate physical switch and software token to determine the current arming
  state. This function is called once per loop() iteration.

INPUTS
  - Physical switch (read_arm_switch()).
  - sw_arm_token (set only via valid ARM command).

STATE TRANSITION LOGIC
------------------------------------------------------------------------------
Let:
    sw = physical switch state
    tok = sw_arm_token

Transitions:

  Case 1: sw == false
      -> DISARMED
      -> sw_arm_token cleared (token invalidated)
      Rationale:
          Dropping the physical switch must immediately disarm and revoke token.

  Case 2: sw == true AND tok == false
      -> SAFE
      Rationale:
          Physical switch engaged but no valid software token.

  Case 3: sw == true AND tok == true
      -> ARMED
      Rationale:
          Both hardware and software conditions satisfied.

SECURITY / SAFETY PROPERTY
------------------------------------------------------------------------------
- The token is automatically cleared when switch goes LOW.
- Therefore, re-arming after a switch drop requires reissuing the ARM command.
- This prevents “remembered token” arming after power-cycle or switch bounce.

CONTRACT
  Preconditions:
    - sw_arm_token reflects last valid ARM command.
  Postconditions:
    - arm_state updated deterministically based on current inputs.
    - Actuator gating enforced via set_arm_state().

DETERMINISM / BOUNDS
  - No loops.
  - No blocking.
  - Purely combinational logic with constant time.

FAILURE MODES
  - Switch bounce may cause rapid transitions; higher-level debounce could be
    added if necessary.
  - If PIN_ARM_SWITCH wiring is incorrect, state machine may misinterpret state.
*/
static void update_arming() {
  const bool sw = read_arm_switch();

  // Case 1: physical switch LOW -> hard disarm and clear token.
  if (!sw) {
    sw_arm_token = false;
    set_arm_state(ArmingState::DISARMED);
    return;
  }

  // Case 2: switch HIGH but no valid token -> SAFE.
  if (sw && !sw_arm_token) {
    set_arm_state(ArmingState::SAFE);
    return;
  }

  // Case 3: switch HIGH and token valid -> ARMED.
  set_arm_state(ArmingState::ARMED);
}


// ============================================================================
// SECTION 15: Airbrake policy (deterministic, bounded)
// ============================================================================

/*
SECTION INTENT
------------------------------------------------------------------------------
This section defines the *airbrake control policy* — the logic layer that
translates estimated flight state (altitude + vertical speed) into a bounded,
rate-limited normalized airbrake command u ∈ [0,1].

This policy is:

  - Deterministic (no blocking, no dynamic allocation).
  - Fully bounded in execution time.
  - Explicitly gated by compile-time and runtime enables.
  - Strictly dependent on published estimator snapshots (fs_s).
  - Structured as a minimal finite state machine (FSM).

CRITICAL ARCHITECTURAL PRINCIPLE
------------------------------------------------------------------------------
The policy:
    DOES NOT directly command hardware.
It only returns:
    { command01, valid }

The caller (loop()) applies hardware actuation only if:

    arm_state == ARMED
    AND ACTUATION_ENABLED != 0
    AND pol.valid == true

This separation ensures reviewability and safety containment.

-------------------------------------------------------------------------------
POLICY OBJECTIVE (APOGEE SHAPING TEMPLATE)
-------------------------------------------------------------------------------
This implementation attempts a conservative form of "apogee shaping":

  - During coast phase (after motor burnout),
  - If vertical velocity (vz) is high,
  - Deploy airbrakes proportionally to reduce climb rate,
  - Disable braking once ascent velocity becomes non-positive.

This is NOT a full optimal control solution.
It is a bounded, reviewable heuristic template.

-------------------------------------------------------------------------------
STATE MACHINE OVERVIEW
-------------------------------------------------------------------------------
PolicyMode states:

  DISABLED
    - Initial/reset sentinel.
    - Transition immediately to ARMED_WAIT when active.

  ARMED_WAIT
    - Ignore near-pad altitude noise.
    - Wait until altitude >= POL_MIN_ALT_M.

  COAST
    - Monitor climb rate.
    - Wait until vz >= POL_START_BRAKE_VZ_MPS.

  BRAKE
    - Actively compute and output bounded brake command.
    - Remain until vz <= 0 (apogee approach).

  DONE
    - Disable further braking after ascent ends.

-------------------------------------------------------------------------------
RUNTIME ENABLE GATES
-------------------------------------------------------------------------------
Two enable layers exist:

1) Compile-time:
      AIRBRAKE_POLICY_ENABLED
   If zero, entire policy returns invalid output.

2) Runtime:
      policy_runtime_enable
   Controlled by serial command:
      POLICY 0|1

Both must allow execution for the policy to act.

-------------------------------------------------------------------------------
GLOBAL POLICY STATE
-------------------------------------------------------------------------------
This section owns the policy-global variables used by compute_airbrake_policy().

  policy_runtime_enable
    Runtime gate; defaults false.

  pol_mode
    Current FSM state.

  pol_cmd_prev
    Last commanded value (after slew limiting).
    Used to enforce rate limiting.

  pol_t_prev_ms
    Timestamp of last policy evaluation.
    Used to compute dt for slew limiting.

  pol_active_since_ms
    Timestamp when BRAKE mode entered.
    Used to suppress rapid transitions.

-------------------------------------------------------------------------------
PARAMETER RATIONALE
-------------------------------------------------------------------------------
All parameters are deliberately simple scalars; they are not persisted to EEPROM
in this revision.

  POL_MIN_ALT_M
    Ignore low-altitude noise and ground transients.

  POL_START_BRAKE_VZ_MPS
    Threshold to begin braking; avoids premature deployment.

  POL_TARGET_VZ_MPS
    Desired reduced climb rate.

  POL_FULL_BRAKE_VZ_MPS
    Above this, saturate to maximum brake command.

  POL_MAX_CMD01
    Hard cap on deployment (never exceed 80%).

  POL_SLEW_PER_SEC
    Max change in command per second.
    Protects mechanical linkage and avoids oscillation.

  POL_MAX_EST_AGE_MS
    Reject estimator data older than this.
    Prevents stale-state actuation.

  POL_MIN_ACTIVE_MS
    Prevent rapid toggling near transition boundaries.

-------------------------------------------------------------------------------
IMPORTANT COMPILATION / ORDERING CONSTRAINT
-------------------------------------------------------------------------------
The policy helper functions (policy_reset, compute_airbrake_policy) reference
pol_mode and other policy globals. Therefore, the declaration order is:

  (1) enum class PolicyMode
  (2) policy globals (policy_runtime_enable, pol_mode, ...)
  (3) constants (POL_...)
  (4) helpers (clamp01, rate_limit, policy_reset)
  (5) compute_airbrake_policy()

Violating this ordering yields compile errors like:
    'pol_mode' was not declared in this scope
*/

/* --------------------------------------------------------------------------
 * (1) Policy FSM state enumeration
 * -------------------------------------------------------------------------- */
enum class PolicyMode : uint8_t {
  DISABLED   = 0,
  ARMED_WAIT = 1,
  COAST      = 2,
  BRAKE      = 3,
  DONE       = 4
};

/* --------------------------------------------------------------------------
 * (2) Policy globals (owned by this section)
 * -------------------------------------------------------------------------- */

/*
policy_runtime_enable
------------------------------------------------------------------------------
ROLE
  Runtime gate for policy execution, controlled by serial commands.

SEMANTICS
  - false: policy computes no valid output (always returns valid=false).
  - true : policy may become active if other gates pass (compile-time + arming).

SAFETY
  Disabling policy at runtime should cause:
    - policy_reset() to clear internal mode/command history
    - output validity to drop to false
*/
static bool policy_runtime_enable = false;

/*
pol_mode
------------------------------------------------------------------------------
ROLE
  Holds the current state of the PolicyMode FSM.

SAFETY
  - Any unexpected value triggers a defensive reset.
  - Reset returns pol_mode to DISABLED (sentinel).
*/
static PolicyMode pol_mode = PolicyMode::DISABLED;

/*
pol_cmd_prev
------------------------------------------------------------------------------
ROLE
  Stores the last command produced *after slew limiting*.

WHY IT EXISTS
  Slew limiting requires the previous output, not the previous target.
  This ensures:
    |u(k) - u(k-1)| is bounded.
*/
static float pol_cmd_prev = 0.0f;

/*
pol_t_prev_ms
------------------------------------------------------------------------------
ROLE
  Stores the timestamp of the last policy evaluation.

WHY IT EXISTS
  - To compute dt for slew limiting.
  - dt_s = (now - pol_t_prev_ms) * 0.001

WRAPAROUND NOTE
  millis() wraps every ~49 days; subtraction on uint32_t is well-defined
  modulo 2^32 and is a standard embedded scheduling technique.
*/
static uint32_t pol_t_prev_ms = 0;

/*
pol_active_since_ms
------------------------------------------------------------------------------
ROLE
  Stores the timestamp when BRAKE mode began.

WHY IT EXISTS
  It enables a small "minimum active time" window (POL_MIN_ACTIVE_MS) that
  suppresses rapid toggling or chattering mode logic at the moment BRAKE starts.
*/
static uint32_t pol_active_since_ms = 0;

/* --------------------------------------------------------------------------
 * (3) Policy constants (non-persistent)
 * -------------------------------------------------------------------------- */

// ignore near-pad noise
static constexpr float POL_MIN_ALT_M            = 40.0f;

// brake if climbing fast
static constexpr float POL_START_BRAKE_VZ_MPS   = 30.0f;

// aim to reduce climb rate toward this
static constexpr float POL_TARGET_VZ_MPS        = 15.0f;

// saturate command above this
static constexpr float POL_FULL_BRAKE_VZ_MPS    = 45.0f;

// hard cap on deployment (never exceed 80%)
static constexpr float POL_MAX_CMD01            = 0.80f;

// command slew limit (per second)
static constexpr float POL_SLEW_PER_SEC         = 1.0f;

// estimator freshness gate
static constexpr uint32_t POL_MAX_EST_AGE_MS    = 100;

// avoid rapid toggling right after BRAKE begins
static constexpr uint32_t POL_MIN_ACTIVE_MS     = 300;

/* --------------------------------------------------------------------------
 * (4) Utility functions
 * -------------------------------------------------------------------------- */

/*
clamp01(x)
------------------------------------------------------------------------------
ROLE
  Clamp a floating-point value to the interval [0,1].

CONTRACT
  - Returns 0 if x < 0.
  - Returns 1 if x > 1.
  - Returns x otherwise.

DETERMINISM
  - O(1), branch-only.

NUMERICAL NOTE
  This clamp does not special-case NaN.
  If x is NaN:
    - both comparisons are false
    - function returns NaN
Call sites that require non-NaN must gate finiteness explicitly.
*/
static float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

/*
rate_limit(prev, target, dt_s, slew_per_s)
------------------------------------------------------------------------------
ROLE
  Apply a symmetric slew-rate limit to a command signal.

MATHEMATICAL MODEL
  Let:
      max_step = slew_per_s * dt_s

  Then:

      if target - prev >  max_step -> prev + max_step
      if target - prev < -max_step -> prev - max_step
      else                           target

This guarantees:

    |u(k) - u(k-1)| ≤ slew_per_s * dt_s

SAFETY PURPOSE
  - Prevents abrupt servo movement.
  - Reduces mechanical stress.
  - Improves control stability.

CONTRACT
  Preconditions:
    - dt_s ≥ 0  (negative dt should not occur with uint32_t millis math)
    - slew_per_s ≥ 0
  Postconditions:
    - Output differs from prev by at most max_step.

DETERMINISM
  - O(1), no loops.

NUMERICAL NOTES
  - If dt_s is extremely large (e.g., long stall), max_step becomes large and
    limiting becomes weak. This is acceptable because other gates exist:
      * estimator staleness gate
      * arming/policy runtime gates
*/
static float rate_limit(float prev, float target, float dt_s, float slew_per_s) {
  const float max_step = slew_per_s * dt_s;
  const float d = target - prev;

  if (d >  max_step) return prev + max_step;
  if (d < -max_step) return prev - max_step;
  return target;
}

/*
policy_reset()
------------------------------------------------------------------------------
ROLE
  Reset internal policy state to a safe baseline.

WHEN CALLED
  - When arming state drops below ARMED.
  - When estimator becomes invalid or stale.
  - When runtime policy is disabled.
  - When unexpected state is encountered.

POSTCONDITIONS
  - pol_mode = DISABLED
  - pol_cmd_prev = 0
  - pol_t_prev_ms set to current time (for future dt computation)
  - pol_active_since_ms cleared

SAFETY PROPERTY
  Ensures that no stale command or mode persists across:
    - disarm events
    - estimator faults
    - runtime disable toggles

DETERMINISM
  - O(1)
  - No blocking

NOTE ABOUT millis()
  policy_reset() samples millis() once; this bounds dt_s on the next call and
  avoids undefined dt behavior if pol_t_prev_ms were left at 0 indefinitely.
*/
static void policy_reset() {
  pol_mode = PolicyMode::DISABLED;
  pol_cmd_prev = 0.0f;
  pol_t_prev_ms = millis();
  pol_active_since_ms = 0u;
}

/* --------------------------------------------------------------------------
 * (5) Policy output computation
 * -------------------------------------------------------------------------- */

/*
compute_airbrake_policy()
------------------------------------------------------------------------------
ROLE
  Produce the normalized airbrake command u ∈ [0,1] along with a validity flag
  indicating whether actuation is permitted by the *policy layer*.

  This function owns the policy decision. It must not directly command hardware.
  The caller applies actuation only when:
    (a) runtime arming state == ARMED
    (b) compile-time ACTUATION_ENABLED != 0
    (c) returned out.valid == true

CONTRACT
  Preconditions:
    - All inputs are consumed from published snapshots (fs_s).
    - fs_s fields are interpreted only if fs_s.valid==true.
    - millis() is monotonic modulo wrap-around.
  Postconditions:
    - Returns out.valid == false when:
        - policy compiled out OR runtime disabled OR not armed
        - estimator invalid/stale
        - internal state requires waiting (ARMED_WAIT / COAST / DONE)
    - Returns out.valid == true only in BRAKE mode with a bounded u ∈ [0,1].
    - Internal policy state (pol_mode, pol_cmd_prev, pol_t_prev_ms) is updated
      deterministically and remains bounded.

DETERMINISM / BOUNDS
  - No blocking calls.
  - No sensor I/O.
  - No dynamic allocation.
  - Runtime is bounded scalar math with a constant number of branches.

SAFETY / FAILSAFE
  - Any invalid or stale estimator input forces:
      out.valid = false
      policy state reset (policy_reset)
  - Any non-ARMED state forces policy reset to avoid latent actuation on re-arm.
  - Slew limiting prevents large command steps that may excite mechanics.

ALGORITHM (high-level)
  The policy is a small finite state machine:
    DISABLED   : initialization / reset sentinel
    ARMED_WAIT : ignore pad-region noise; wait until altitude exceeds threshold
    COAST      : monitor climb rate; wait until vz exceeds start threshold
    BRAKE      : compute command to reduce vz toward a target, with slew limit
    DONE       : disable actuation after apogee approach (vz <= 0)

STEP-BY-STEP (detailed)
  (1) Initialize output as (command01=0, valid=false).
  (2) Compile-time gate:
        - if AIRBRAKE_POLICY_ENABLED == 0 -> return invalid output.
  (3) Runtime enable gate:
        - if policy_runtime_enable == false -> return invalid output.
  (4) Arming gate:
        - if arm_state != ARMED -> reset policy and return invalid output.
  (5) Estimator validity gate:
        - require fs_s.valid and finite altitude/vz.
        - reject stale estimator ages above POL_MAX_EST_AGE_MS.
        - on failure: reset policy and return invalid output.
  (6) Compute dt_s from now - pol_t_prev_ms for slew limiting.
  (7) State machine progression:
        - DISABLED -> ARMED_WAIT
        - ARMED_WAIT -> COAST once altitude >= POL_MIN_ALT_M
        - COAST -> BRAKE once vz >= POL_START_BRAKE_VZ_MPS
        - BRAKE -> DONE once vz <= 0
  (8) In BRAKE:
        (a) Map vz to an unslewed command u:
              vz <= target -> u = 0
              vz >= full    -> u = 1
              else          -> linear ramp between (target..full)
        (b) Apply max command clamp POL_MAX_CMD01.
        (c) Apply slew limiting based on dt_s and POL_SLEW_PER_SEC.
        (d) Publish out.command01 and out.valid=true.

FAILURE MODES
  - Time discontinuity (millis wrap): dt_s may become very large.
    Mitigation:
      * estimator staleness gate resets on stale data
      * slew limiting bounds *rate* for reasonable dt
  - Estimator glitch (NaNs): immediately resets and returns invalid.
  - Unexpected FSM state: defensive reset.

TRACEABILITY
  - Policy state is exported via telemetry (pol_mode).
  - Output validity is explicit in out.valid.
*/
static AirbrakePolicyOutput compute_airbrake_policy() {
  AirbrakePolicyOutput out;
  out.command01 = 0.0f;
  out.valid = false;

#if !AIRBRAKE_POLICY_ENABLED
  // Policy compiled out: never request actuation.
  return out;
#else
  // --------------------------------------------------------------------------
  // (1) Runtime enable gate
  // --------------------------------------------------------------------------
  if (!policy_runtime_enable) {
    // Explicitly refuse actuation; leave policy state unchanged.
    // (Optional behavior would be to reset here; current design keeps state so
    //  toggling runtime enable back on can resume from a known mode if desired.
    //  If "fresh start on enable" is preferred, call policy_reset() on disable.)
    return out;
  }

  const uint32_t now = millis();

  // --------------------------------------------------------------------------
  // (2) Arming gate: any non-ARMED state forces a reset
  // --------------------------------------------------------------------------
  if (arm_state != ArmingState::ARMED) {
    policy_reset();
    return out;
  }

  // --------------------------------------------------------------------------
  // (3) Estimator validity + finiteness gate
  // --------------------------------------------------------------------------
  if (!fs_s.valid || !isfinite(fs_s.altitude_m) || !isfinite(fs_s.vz_mps)) {
    policy_reset();
    return out;
  }

  // --------------------------------------------------------------------------
  // (4) Estimator staleness gate
  // --------------------------------------------------------------------------
  const uint32_t est_age = age_ms(now, fs_s.t_ms, fs_s.valid);
  if (est_age == 0xFFFFFFFFu || est_age > POL_MAX_EST_AGE_MS) {
    policy_reset();
    return out;
  }

  const float alt = fs_s.altitude_m;
  const float vz  = fs_s.vz_mps;

  // --------------------------------------------------------------------------
  // (5) Time base for slew limiting (dt_s)
  // --------------------------------------------------------------------------
  if (pol_t_prev_ms == 0u) pol_t_prev_ms = now;
  const uint32_t dt_ms = now - pol_t_prev_ms;
  pol_t_prev_ms = now;

  // Convert to seconds; dt_s is used only for rate limiting.
  const float dt_s = (dt_ms > 0u) ? (dt_ms * 0.001f) : 0.0f;

  // --------------------------------------------------------------------------
  // (6) State initialization
  // --------------------------------------------------------------------------
  if (pol_mode == PolicyMode::DISABLED) {
    pol_mode = PolicyMode::ARMED_WAIT;
    pol_cmd_prev = 0.0f;
    pol_active_since_ms = 0u;
  }

  // --------------------------------------------------------------------------
  // (7) ARMED_WAIT: ignore near-pad altitude noise
  // --------------------------------------------------------------------------
  if (pol_mode == PolicyMode::ARMED_WAIT) {
    if (alt >= POL_MIN_ALT_M) {
      pol_mode = PolicyMode::COAST;
    }
    pol_cmd_prev = 0.0f;
    return out; // valid=false
  }

  // --------------------------------------------------------------------------
  // (8) COAST: wait for climb-rate condition to begin braking
  // --------------------------------------------------------------------------
  if (pol_mode == PolicyMode::COAST) {
    if (vz >= POL_START_BRAKE_VZ_MPS) {
      pol_mode = PolicyMode::BRAKE;
      pol_active_since_ms = now;
    }
    pol_cmd_prev = 0.0f;
    return out; // valid=false
  }

  // --------------------------------------------------------------------------
  // (9) DONE: refuse further actuation until reset/disarm
  // --------------------------------------------------------------------------
  if (pol_mode == PolicyMode::DONE) {
    pol_cmd_prev = 0.0f;
    return out; // valid=false
  }

  // --------------------------------------------------------------------------
  // (10) BRAKE: compute bounded command, apply caps and slew limiting
  // --------------------------------------------------------------------------
  if (pol_mode == PolicyMode::BRAKE) {
    // Apogee approach: once ascent ends (vz <= 0), stop braking.
    if (vz <= 0.0f) {
      pol_mode = PolicyMode::DONE;
      pol_cmd_prev = 0.0f;
      return out; // valid=false
    }

    // Minimum active-time window: suppress certain future extensions that might
    // transition modes rapidly right after BRAKE begins. In this simple policy,
    // the window is primarily telemetry/traceability and a hook for future logic.
    if (pol_active_since_ms != 0u && (now - pol_active_since_ms) < POL_MIN_ACTIVE_MS) {
      // No mode transition is performed here; command computation continues.
    }

    // ------------------------------------------------------------------------
    // (10a) Map vz to an unslewed command u ∈ [0,1]
    // ------------------------------------------------------------------------
    float u = 0.0f;

    if (vz <= POL_TARGET_VZ_MPS) {
      // Already at or below target climb rate -> no braking requested.
      u = 0.0f;
    } else if (vz >= POL_FULL_BRAKE_VZ_MPS) {
      // Very high climb rate -> saturate request.
      u = 1.0f;
    } else {
      // Linear ramp between target and full-brake threshold.
      const float num = (vz - POL_TARGET_VZ_MPS);
      const float den = (POL_FULL_BRAKE_VZ_MPS - POL_TARGET_VZ_MPS);
      u = (den > 1e-6f) ? (num / den) : 0.0f;
    }

    // ------------------------------------------------------------------------
    // (10b) Clamp and apply policy maximum deployment
    // ------------------------------------------------------------------------
    u = clamp01(u);

    // Apply hard cap (e.g., 0.80 means never exceed 80% deployment).
    u *= POL_MAX_CMD01;

    // ------------------------------------------------------------------------
    // (10c) Slew limit relative to previous commanded output
    // ------------------------------------------------------------------------
    const float u_slewed =
        (dt_s > 0.0f) ? rate_limit(pol_cmd_prev, u, dt_s, POL_SLEW_PER_SEC) : u;

    pol_cmd_prev = clamp01(u_slewed);

    // ------------------------------------------------------------------------
    // (10d) Publish valid output
    // ------------------------------------------------------------------------
    out.command01 = pol_cmd_prev;
    out.valid = true;
    return out;
  }

  // --------------------------------------------------------------------------
  // (11) Defensive default: unknown state => reset and refuse actuation
  // --------------------------------------------------------------------------
  policy_reset();
  return out;
#endif
}


// ============================================================================
// SECTION 16: Telemetry + commands
// ============================================================================

/*
SECTION INTENT
------------------------------------------------------------------------------
This section defines the *external observability and control interface* of the
flight software via Serial:

  (A) Fixed-format telemetry stream (CSV rows).
  (B) Structured command interface (single-line ASCII commands).
  (C) Warning bitmask construction for compact fault visibility.

Design priorities:
  - Deterministic emission (fixed cadence, fixed field count).
  - Stable CSV schema for post-flight analysis tools.
  - No dynamic allocation.
  - Clear separation between data generation and presentation.
  - Explicit fault visibility through warn_mask.
  - Bounded parsing that never blocks loop().

-------------------------------------------------------------------------------
TELEMETRY TIMING MODEL
-------------------------------------------------------------------------------
TLM_PERIOD_MS
  Defines the fixed telemetry cadence (50 ms = 20 Hz).

last_tlm_ms
  Tracks last emission time using millis() delta scheduling.

This ensures:
  - At most one telemetry row per period.
  - No blocking delays.
  - Jitter bounded by loop execution time.

-------------------------------------------------------------------------------
HEADER CONTROL
-------------------------------------------------------------------------------
hdr_enable
  If true, print_header() is emitted on boot or on HDR 1 command.
  If false, header suppressed.

Purpose:
  - Allows log parsers to re-sync.
  - Avoids repeated header spam in long sessions.

-------------------------------------------------------------------------------
COMMAND PIPELINE OVERVIEW
-------------------------------------------------------------------------------
read_line(out, cap)
  - Non-blocking collection of bytes into newline-terminated command lines.

handle_command(line)
  - Tokenizes, validates arguments, and applies state changes.

The intended loop() pattern is:
  - read one line per iteration (or none)
  - execute at most one command per iteration
This caps per-iteration work and preserves determinism.
*/

// ---------------------------------------------------------------------------
// 16.0 Telemetry cadence state
// ---------------------------------------------------------------------------

static constexpr uint32_t TLM_PERIOD_MS = 50;
static uint32_t last_tlm_ms = 0;

static bool hdr_enable = true;

// ---------------------------------------------------------------------------
// 16.1 Printing helpers (schema-safe tokens)
// ---------------------------------------------------------------------------

/*
print_f_or_nan(x, decimals)
------------------------------------------------------------------------------
ROLE
  Print a float in fixed decimals if finite; otherwise print literal "nan".

WHY THIS EXISTS
  - Keeps CSV column alignment without inventing sentinel numeric values.
  - Offline parsers can treat "nan" as missing data explicitly.

CONTRACT
  - If isfinite(x) -> Serial.print(x, decimals)
  - Else           -> Serial.print("nan")

DETERMINISM
  - O(1), no allocation.
*/
static inline void print_f_or_nan(float x, int decimals) {
  if (isfinite(x)) Serial.print(x, decimals);
  else Serial.print("nan");
}

// ---------------------------------------------------------------------------
// 16.2 Header emission
// ---------------------------------------------------------------------------

/*
print_header()
------------------------------------------------------------------------------
ROLE
  Emit a single CSV header row describing the telemetry field order.

SCHEMA STABILITY REQUIREMENT
  The order and number of fields printed here must match exactly the order used
  in emit_tlm().

FIELD GROUPING (SEMANTIC LAYERS)
  1) Time + Arming
  2) Barometer
  3) Estimator
  4) IMU
  5) Auxiliary accel
  6) Derived vertical acceleration
  7) Magnetometer
  8) Configuration
  9) Warning mask
 10) Kalman internal parameters
 11) Policy

DETERMINISM
  - Single Serial.println() call.
  - Fixed string literal.
*/
static void print_header() {
  Serial.println(
    "HDR,"
    "ms,"
    "arm_state,act_en,"
    "press_hpa,temp_c,baro_age_ms,"
    "alt_m,vz_mps,est_age_ms,"
    "roll_deg,pitch_deg,a_norm,imu_age_ms,motion_bad,"
    "aux_ax,aux_ay,aux_az,aux_anorm,aux_age_ms,aux_motion_bad,"
    "aux_a_wz,aux_a_lin_z,aux_vz_age_ms,aux_vz_valid,"
    "hdg_deg,mag_valid,mag_interf,mag_age_ms,mag_norm_uT,decl_deg,mag_cal_valid,"
    "cfg_valid,baseline_hpa,"
    "warn_mask,"
    "kf_seeded,kf_P00,kf_P11,kf_r,kf_q_acc,"
    "pol_en,pol_mode,pol_cmd,pol_valid"
  );
}

// ---------------------------------------------------------------------------
// 16.3 Warning mask construction
// ---------------------------------------------------------------------------

/*
build_warn_mask(now_ms)
------------------------------------------------------------------------------
ROLE
  Construct a compact bitmask encoding current subsystem warning conditions.

BIT ASSIGNMENTS
  Bit 0  : Barometer invalid
  Bit 1  : IMU invalid
  Bit 2  : Estimator invalid
  Bit 3  : Baseline pressure not captured
  Bit 4  : Not ARMED
  Bit 5  : Magnetometer heading invalid
  Bit 6  : Magnetometer interference flagged
  Bit 7  : Magnetometer calibration invalid
  Bit 8  : Kalman filter not seeded
  Bit 9  : Auxiliary accel invalid
  Bit 10 : Derived vertical accel invalid

DESIGN PRINCIPLE
  - Each bit flags a condition that reduces trust in some telemetry values.
  - The mask is diagnostic only; gating is implemented elsewhere.

DETERMINISM
  - O(1), fixed number of bit tests.
*/
static uint16_t build_warn_mask(uint32_t /*now_ms*/) {
  uint16_t w = 0;

  if (!baro_s.valid) w |= (1u << 0);
  if (!imu_s.valid)  w |= (1u << 1);
  if (!fs_s.valid)   w |= (1u << 2);

  if (isnan(cfg.baro_baseline_hpa))     w |= (1u << 3);
  if (arm_state != ArmingState::ARMED)  w |= (1u << 4);

  if (!mag_s.valid)         w |= (1u << 5);
  if (mag_s.interference)   w |= (1u << 6);
  if (!(cfg.valid && cfg.mag_cal.valid)) w |= (1u << 7);

  if (!kf_alt.seeded) w |= (1u << 8);

  if (!aux_s.valid)    w |= (1u << 9);
  if (!aux_vz_s.valid) w |= (1u << 10);

  return w;
}

// ---------------------------------------------------------------------------
// 16.4 Telemetry row emission
// ---------------------------------------------------------------------------

/*
emit_tlm(pol)
------------------------------------------------------------------------------
ROLE
  Emit exactly one telemetry CSV row containing:
    - time
    - arming/actuation state
    - sensor snapshots + ages
    - derived quantities
    - warning mask
    - selected Kalman parameters
    - policy status and output

FIELD INVARIANTS (FOR LOG PARSERS)
  (I1) Field count is constant across all rows.
  (I2) Field order matches print_header().
  (I3) Invalid float values printed as literal token "nan".
  (I4) Invalid ages printed as integer -1.
  (I5) Validity flags printed for applicable subsystems.
  (I6) warn_mask always present.

AGE SEMANTICS
  age_ms(now, t_ms, valid) returns:
    - 0xFFFFFFFF sentinel when valid == false
    - otherwise now - t_ms (modulo wrap)

emit_tlm prints:
  -1  when sentinel is returned
  age when valid

DETERMINISM
  - Bounded Serial prints.
  - No loops other than print calls.
*/
static void emit_tlm(const AirbrakePolicyOutput& pol) {
  const uint32_t now = millis();

  // Ages derived from validity-qualified timestamps.
  const uint32_t baro_age_u    = age_ms(now, baro_s.t_ms,    baro_s.valid);
  const uint32_t imu_age_u     = age_ms(now, imu_s.t_ms,     imu_s.valid);
  const uint32_t aux_age_u     = age_ms(now, aux_s.t_ms,     aux_s.valid);
  const uint32_t est_age_u     = age_ms(now, fs_s.t_ms,      fs_s.valid);
  const uint32_t aux_vz_age_u  = age_ms(now, aux_vz_s.t_ms,  aux_vz_s.valid);

  // Magnetometer age uses its timestamp directly; usability is indicated by mag_s.valid.
  const uint32_t mag_age_u = now - mag_s.t_ms;

  // Convert invalid-age sentinel to -1 (parser invariant).
  const int baro_age   = (baro_age_u   == 0xFFFFFFFFu) ? -1 : (int)baro_age_u;
  const int imu_age    = (imu_age_u    == 0xFFFFFFFFu) ? -1 : (int)imu_age_u;
  const int aux_age    = (aux_age_u    == 0xFFFFFFFFu) ? -1 : (int)aux_age_u;
  const int est_age    = (est_age_u    == 0xFFFFFFFFu) ? -1 : (int)est_age_u;
  const int aux_vz_age = (aux_vz_age_u == 0xFFFFFFFFu) ? -1 : (int)aux_vz_age_u;
  const int mag_age    = (int)mag_age_u;

  const uint16_t warn = build_warn_mask(now);

  // Reflection of actuation enable at this instant (policy may still be invalid).
  const bool act_en = (arm_state == ArmingState::ARMED) && (ACTUATION_ENABLED != 0);

  Serial.print("TLM,");
  Serial.print(now);

  // arm_state,act_en
  Serial.print(',');
  Serial.print((int)arm_state);
  Serial.print(',');
  Serial.print(act_en ? 1 : 0);

  // press_hpa,temp_c,baro_age_ms
  Serial.print(',');
  if (baro_s.valid) Serial.print(baro_s.press_hpa, 2); else Serial.print("nan");
  Serial.print(',');
  if (baro_s.valid) Serial.print(baro_s.temp_c, 2);    else Serial.print("nan");
  Serial.print(',');
  Serial.print(baro_age);

  // alt_m,vz_mps,est_age_ms
  Serial.print(',');
  if (fs_s.valid) print_f_or_nan(fs_s.altitude_m, 2); else Serial.print("nan");
  Serial.print(',');
  if (fs_s.valid) print_f_or_nan(fs_s.vz_mps, 2);     else Serial.print("nan");
  Serial.print(',');
  Serial.print(est_age);

  // roll_deg,pitch_deg,a_norm,imu_age_ms,motion_bad
  Serial.print(',');
  if (imu_s.valid) Serial.print(imu_s.roll_deg, 2);  else Serial.print("nan");
  Serial.print(',');
  if (imu_s.valid) Serial.print(imu_s.pitch_deg, 2); else Serial.print("nan");
  Serial.print(',');
  if (imu_s.valid) Serial.print(imu_s.a_norm, 2);    else Serial.print("nan");
  Serial.print(',');
  Serial.print(imu_age);
  Serial.print(',');
  Serial.print((imu_s.valid && imu_s.motion_bad) ? 1 : 0);

  // aux_ax,aux_ay,aux_az,aux_anorm,aux_age_ms,aux_motion_bad
  Serial.print(',');
  if (aux_s.valid) Serial.print(aux_s.ax, 3); else Serial.print("nan");
  Serial.print(',');
  if (aux_s.valid) Serial.print(aux_s.ay, 3); else Serial.print("nan");
  Serial.print(',');
  if (aux_s.valid) Serial.print(aux_s.az, 3); else Serial.print("nan");
  Serial.print(',');
  if (aux_s.valid) Serial.print(aux_s.a_norm, 2); else Serial.print("nan");
  Serial.print(',');
  Serial.print(aux_age);
  Serial.print(',');
  Serial.print((aux_s.valid && aux_s.motion_bad) ? 1 : 0);

  // aux_a_wz,aux_a_lin_z,aux_vz_age_ms,aux_vz_valid
  Serial.print(',');
  if (aux_vz_s.valid) Serial.print(aux_vz_s.a_wz_mps2, 3);    else Serial.print("nan");
  Serial.print(',');
  if (aux_vz_s.valid) Serial.print(aux_vz_s.a_lin_z_mps2, 3); else Serial.print("nan");
  Serial.print(',');
  Serial.print(aux_vz_age);
  Serial.print(',');
  Serial.print(aux_vz_s.valid ? 1 : 0);

  // hdg_deg,mag_valid,mag_interf,mag_age_ms,mag_norm_uT,decl_deg,mag_cal_valid
  Serial.print(',');
  if (mag_s.valid) print_f_or_nan(ema_heading_deg, 2); else Serial.print("nan");
  Serial.print(',');
  Serial.print(mag_s.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(mag_s.interference ? 1 : 0);
  Serial.print(',');
  Serial.print(mag_age);
  Serial.print(',');
  print_f_or_nan(mag_s.norm_uT, 2);
  Serial.print(',');
  Serial.print(cfg.valid ? cfg.declination_deg : 0.0f, 3);
  Serial.print(',');
  Serial.print((cfg.valid && cfg.mag_cal.valid) ? 1 : 0);

  // cfg_valid,baseline_hpa
  Serial.print(',');
  Serial.print(cfg.valid ? 1 : 0);
  Serial.print(',');
  if (isnan(cfg.baro_baseline_hpa)) Serial.print("nan");
  else Serial.print(cfg.baro_baseline_hpa, 2);

  // warn_mask
  Serial.print(',');
  Serial.print(warn);

  // kf_seeded,kf_P00,kf_P11,kf_r,kf_q_acc
  Serial.print(',');
  Serial.print(kf_alt.seeded ? 1 : 0);
  Serial.print(',');
  Serial.print(kf_alt.P00, 4);
  Serial.print(',');
  Serial.print(kf_alt.P11, 4);
  Serial.print(',');
  Serial.print(kf_alt.r_meas, 4);
  Serial.print(',');
  Serial.print(kf_alt.q_acc, 4);

  // pol_en,pol_mode,pol_cmd,pol_valid
  Serial.print(',');
  Serial.print((AIRBRAKE_POLICY_ENABLED != 0 && policy_runtime_enable) ? 1 : 0);
  Serial.print(',');
  Serial.print((int)pol_mode);
  Serial.print(',');
  Serial.print(pol.command01, 3);
  Serial.print(',');
  Serial.print(pol.valid ? 1 : 0);

  Serial.println();
}

/*
telemetry_tick(pol)
------------------------------------------------------------------------------
ROLE
  Deterministic telemetry scheduler hook.

USAGE MODEL
  Called from loop() each iteration with the most recent policy output.

BEHAVIOR
  - Emits at most one row per TLM_PERIOD_MS.
  - Uses millis() delta scheduling.
  - Avoids blocking delays.

DETERMINISM
  - O(1).
*/
static inline void telemetry_tick(const AirbrakePolicyOutput& pol) {
  const uint32_t now = millis();
  if ((now - last_tlm_ms) >= TLM_PERIOD_MS) {
    last_tlm_ms = now;
    emit_tlm(pol);
  }
}

// ---------------------------------------------------------------------------
// 16.5 Help + status output
// ---------------------------------------------------------------------------

static void print_help() {
  Serial.println("HELP:");
  Serial.println(" HELP");
  Serial.println(" STATUS");
  Serial.println(" ARM <token>");
  Serial.println(" DISARM");
  Serial.println(" CAP_BASELINE");
  Serial.println(" SET_SLP <hPa>");
  Serial.println(" SET_DECL <deg>");
  Serial.println(" CAL_MAG <seconds> (5..60)");
  Serial.println(" CLEAR_MAG_CAL");
  Serial.println(" SAVE_CFG");
  Serial.println(" LOAD_CFG");
  Serial.println(" HDR 0|1");
  Serial.println(" SET_KF_R <m2>");
  Serial.println(" SET_KF_QACC <(m/s^2)^2>");
  Serial.println(" SET_KF_QVZ <(m/s^2)^2>  (alias)");
  Serial.println(" POLICY 0|1");
  Serial.println(" POL_RESET");
}

/*
print_status()
------------------------------------------------------------------------------
ROLE
  Emit a human-readable one-line status report for console debugging.

OUTPUT
  "STATUS,ms=...,arm_state=...,baro_age=..., ..."

NOTES
  - Unlike telemetry CSV, this line is not schema-stable.
  - Intended for interactive monitoring, not offline parsing.
*/
static void print_status() {
  const uint32_t now = millis();

  Serial.print("STATUS,ms=");
  Serial.print(now);

  Serial.print(",arm_state=");
  Serial.print((int)arm_state);

  Serial.print(",sw_arm_token=");
  Serial.print(sw_arm_token ? 1 : 0);

  Serial.print(",arm_switch=");
  Serial.print(read_arm_switch() ? 1 : 0);

  Serial.print(",baro_age=");
  Serial.print(age_ms(now, baro_s.t_ms, baro_s.valid));

  Serial.print(",imu_age=");
  Serial.print(age_ms(now, imu_s.t_ms, imu_s.valid));

  Serial.print(",aux_age=");
  Serial.print(age_ms(now, aux_s.t_ms, aux_s.valid));

  Serial.print(",aux_vz_age=");
  Serial.print(age_ms(now, aux_vz_s.t_ms, aux_vz_s.valid));

  Serial.print(",est_age=");
  Serial.print(age_ms(now, fs_s.t_ms, fs_s.valid));

  Serial.print(",mag_age=");
  Serial.print((unsigned long)(now - mag_s.t_ms));

  Serial.print(",baseline_hpa=");
  if (isnan(cfg.baro_baseline_hpa)) Serial.print("nan");
  else Serial.print(cfg.baro_baseline_hpa, 2);

  Serial.print(",decl_deg=");
  Serial.print(cfg.valid ? cfg.declination_deg : 0.0f, 3);

  Serial.print(",mag_cal_valid=");
  Serial.print((cfg.valid && cfg.mag_cal.valid) ? 1 : 0);

  Serial.print(",kf_seeded=");
  Serial.print(kf_alt.seeded ? 1 : 0);

  Serial.print(",kf_r=");
  Serial.print(kf_alt.r_meas, 6);

  Serial.print(",kf_q_acc=");
  Serial.print(kf_alt.q_acc, 6);

  Serial.print(",pol_compile=");
  Serial.print(AIRBRAKE_POLICY_ENABLED ? 1 : 0);

  Serial.print(",pol_runtime=");
  Serial.print(policy_runtime_enable ? 1 : 0);

  Serial.print(",pol_mode=");
  Serial.print((int)pol_mode);

  Serial.println();
}

// ---------------------------------------------------------------------------
// 16.6 Non-blocking command line capture
// ---------------------------------------------------------------------------

/*
read_line(out, out_cap)
------------------------------------------------------------------------------
ROLE
  Non-blocking Serial line reader that assembles bytes into newline-terminated
  command strings.

LINE TERMINATION RULES
  - '\r' is ignored (supports CRLF and LF-only senders).
  - '\n' terminates a line.
  - On '\n':
      - internal buffer is null-terminated
      - content is copied to 'out' (bounded by out_cap)
      - 'out' is always null-terminated when out_cap > 0
      - internal index resets for next line
      - returns true

TRUNCATION SEMANTICS
  - Internal buffer has fixed capacity.
  - When incoming line exceeds capacity:
      - extra characters are dropped
      - function still waits for '\n'
      - truncated line is returned on '\n'

CONTRACT
  - Returns true only when a full line has been received.
  - Returns false otherwise.
*/
static bool read_line(char* out, size_t out_cap) {
  static char buf[256];
  static size_t n = 0;

  if (out_cap == 0) return false;

  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      buf[n] = 0;

      strncpy(out, buf, out_cap);
      out[out_cap - 1] = 0;

      n = 0;
      return true;
    }

    if (n < (sizeof(buf) - 1)) {
      buf[n++] = c;
    }
  }

  return false;
}

// ---------------------------------------------------------------------------
// 16.7 Command parsing helpers
// ---------------------------------------------------------------------------

/*
trim_spaces(s)
------------------------------------------------------------------------------
ROLE
  Remove leading and trailing ASCII spaces/tabs from a mutable C string.

DETERMINISM
  - O(L) where L is string length (bounded by read_line()).
*/
static void trim_spaces(char* s) {
  size_t len = strlen(s);
  while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t')) {
    s[len - 1] = 0;
    len--;
  }
  size_t i = 0;
  while (s[i] == ' ' || s[i] == '\t') i++;
  if (i > 0) memmove(s, s + i, strlen(s + i) + 1);
}

/*
upper_inplace(s)
------------------------------------------------------------------------------
ROLE
  Uppercase ASCII letters in-place for case-insensitive command matching.
*/
static void upper_inplace(char* s) {
  for (; *s; s++) *s = (char)toupper((unsigned char)*s);
}

/*
parse_*_after_space(...)
------------------------------------------------------------------------------
ROLE
  Parse a single argument appearing after the first space in the command line.

NOTES
  - These are intentionally permissive: atof/atoi accept leading whitespace.
  - For higher assurance, argument validation is applied in handle_command().
*/
static bool parse_float_after_space(const char* s, float& v_out) {
  const char* p = strchr(s, ' ');
  if (!p) return false;
  v_out = (float)atof(p + 1);
  return true;
}

static bool parse_int_after_space(const char* s, int& v_out) {
  const char* p = strchr(s, ' ');
  if (!p) return false;
  v_out = atoi(p + 1);
  return true;
}

static bool parse_token_after_space(const char* s, char* out, size_t cap) {
  const char* p = strchr(s, ' ');
  if (!p) return false;
  while (*p == ' ') p++;
  if (*p == 0) return false;
  strncpy(out, p, cap);
  out[cap - 1] = 0;
  return true;
}

// ---------------------------------------------------------------------------
// 16.8 Command dispatcher
// ---------------------------------------------------------------------------

/*
handle_command(line_in)
------------------------------------------------------------------------------
ROLE
  Parse and execute exactly one command line using bounded fixed buffers.

STATE IMPACT SCOPE
  Global state changes are restricted to:
    cfg + cfg.valid + cfg.mag_cal + cfg.baro_baseline_hpa
    sw_arm_token
    kf_alt tuning fields
    policy_runtime_enable and policy_reset()
    hdr_enable

DETERMINISM
  - No dynamic allocation.
  - Parsing uses fixed-size buffers.
  - Dispatch is a bounded strcmp chain.

SAFETY NOTES
  - Arming token required; disarm clears token.
  - Baseline capture clears Kalman seed to avoid model inconsistency.
  - Policy runtime enabling refused when compiled out.
*/
static void handle_command(const char* line_in) {
  char s[256];
  strncpy(s, line_in, sizeof(s));
  s[sizeof(s) - 1] = 0;

  trim_spaces(s);
  if (s[0] == 0) return;

  // Extract first token.
  char cmd[64];
  strncpy(cmd, s, sizeof(cmd));
  cmd[sizeof(cmd) - 1] = 0;
  char* sp = strchr(cmd, ' ');
  if (sp) *sp = 0;
  upper_inplace(cmd);

  if (strcmp(cmd, "HELP") == 0)   { print_help();   return; }
  if (strcmp(cmd, "STATUS") == 0) { print_status(); return; }

  if (strcmp(cmd, "HDR") == 0) {
    int v = 0;
    if (!parse_int_after_space(s, v)) { Serial.println("ERR,HDR,ARG"); return; }
    hdr_enable = (v != 0);
    Serial.print("OK,HDR,"); Serial.println(hdr_enable ? 1 : 0);
    if (hdr_enable) print_header();
    return;
  }

  if (strcmp(cmd, "LOAD_CFG") == 0) { eeprom_load_cfg(); return; }
  if (strcmp(cmd, "SAVE_CFG") == 0) { eeprom_save_cfg(); return; }

  if (strcmp(cmd, "SET_SLP") == 0) {
    float v = NAN;
    if (!parse_float_after_space(s, v)) { Serial.println("ERR,SET_SLP,ARG"); return; }
    if (!(v > 800.0f && v < 1100.0f))   { Serial.println("ERR,SET_SLP,RANGE"); return; }
    cfg.sea_level_hpa = v;
    cfg.valid = true;
    Serial.print("OK,SLP,"); Serial.println(cfg.sea_level_hpa, 2);
    return;
  }

  if (strcmp(cmd, "SET_DECL") == 0) {
    float v = NAN;
    if (!parse_float_after_space(s, v)) { Serial.println("ERR,SET_DECL,ARG"); return; }
    if (!(v > -180.0f && v < 180.0f))   { Serial.println("ERR,SET_DECL,RANGE"); return; }
    cfg.declination_deg = v;
    cfg.valid = true;
    Serial.print("OK,DECL,"); Serial.println(cfg.declination_deg, 3);
    return;
  }

  if (strcmp(cmd, "CAP_BASELINE") == 0) {
    if (!baro_s.valid) { Serial.println("ERR,BASELINE,BARO_INVALID"); return; }

    cfg.baro_baseline_hpa = baro_s.press_hpa;
    cfg.valid = true;

    Serial.print("OK,BASELINE,");
    Serial.println(cfg.baro_baseline_hpa, 2);

    // Reference pressure changed -> measurement model changed -> seed invalid.
    kf_alt.seeded = false;
    return;
  }

  if (strcmp(cmd, "CLEAR_MAG_CAL") == 0) {
    cfg.mag_cal = MagCal();
    cfg.valid = true;
    Serial.println("OK,MAG_CAL,CLEARED");
    return;
  }

  if (strcmp(cmd, "CAL_MAG") == 0) {
    int sec = 0;
    if (!parse_int_after_space(s, sec)) { Serial.println("ERR,CAL_MAG,ARG"); return; }
    if (sec < 5)  sec = 5;
    if (sec > 60) sec = 60;
    mag_calibrate_capture((uint32_t)sec * 1000u);
    return;
  }

  if (strcmp(cmd, "DISARM") == 0) {
    sw_arm_token = false;
    Serial.println("OK,DISARM");
    return;
  }

  if (strcmp(cmd, "ARM") == 0) {
    char tok[64];
    if (!parse_token_after_space(s, tok, sizeof(tok))) { Serial.println("ERR,ARM,ARG"); return; }
    if (strcmp(tok, "MASA") == 0) {
      sw_arm_token = true;
      Serial.println("OK,ARM,TOKEN_SET");
    } else {
      Serial.println("ERR,ARM,BAD_TOKEN");
    }
    return;
  }

  if (strcmp(cmd, "SET_KF_R") == 0) {
    float v = NAN;
    if (!parse_float_after_space(s, v)) { Serial.println("ERR,SET_KF_R,ARG"); return; }
    if (!(v > 1e-6f && v < 1e6f))       { Serial.println("ERR,SET_KF_R,RANGE"); return; }
    kf_alt.r_meas = v;
    Serial.print("OK,KF_R,"); Serial.println(kf_alt.r_meas, 6);
    return;
  }

  if (strcmp(cmd, "SET_KF_QACC") == 0 || strcmp(cmd, "SET_KF_QVZ") == 0) {
    float v = NAN;
    if (!parse_float_after_space(s, v)) { Serial.println("ERR,SET_KF_QACC,ARG"); return; }
    if (!(v > 0.0f && v < 1e6f))        { Serial.println("ERR,SET_KF_QACC,RANGE"); return; }
    kf_alt.q_acc = v;
    Serial.print("OK,KF_QACC,"); Serial.println(kf_alt.q_acc, 6);
    return;
  }

  if (strcmp(cmd, "POLICY") == 0) {
    int v = 0;
    if (!parse_int_after_space(s, v)) { Serial.println("ERR,POLICY,ARG"); return; }

#if AIRBRAKE_POLICY_ENABLED
    policy_runtime_enable = (v != 0);
    if (!policy_runtime_enable) policy_reset();
    Serial.print("OK,POLICY,"); Serial.println(policy_runtime_enable ? 1 : 0);
#else
    (void)v;
    Serial.println("ERR,POLICY,COMPILED_OFF");
#endif
    return;
  }

  if (strcmp(cmd, "POL_RESET") == 0) {
#if AIRBRAKE_POLICY_ENABLED
    policy_reset();
    Serial.println("OK,POL_RESET");
#else
    Serial.println("ERR,POL_RESET,COMPILED_OFF");
#endif
    return;
  }

  Serial.println("ERR,UNKNOWN");
}

/*
commands_tick()
------------------------------------------------------------------------------
ROLE
  Deterministic command processing hook.

BEHAVIOR
  - Reads at most one complete line per call.
  - Executes at most one command per call.
  - Never blocks loop().

USAGE
  Call from loop() each iteration.
*/
static inline void commands_tick() {
  char line[256];
  if (read_line(line, sizeof(line))) {
    handle_command(line);
  }
}




// ============================================================================
// SECTION 17: Setup / Loop
// ============================================================================

/*
setup()
------------------------------------------------------------------------------
ROLE
  Perform all one-time system initialization required before the deterministic
  runtime loop begins.

  setup() establishes:
    - Serial telemetry/command channel
    - I²C bus configuration (pins + clock)
    - GPIO modes (arming switch, buzzer)
    - Sensor initialization (IMU, baro, mag, aux accel)
    - EEPROM configuration load (CRC-protected)
    - Actuator backend attachment + fail-safe idle output
    - Estimator parameter initialization (Kalman tuning + seed state)
    - Policy state initialization
    - Boot-time diagnostics emission (human + machine readable)
    - Scheduler epoch baselines (last_* timestamps)
    - Snapshot timestamps initialized to "now" for consistent age semantics

DETERMINISM AND SAFETY NOTES
------------------------------------------------------------------------------
1) setup() is allowed to block briefly:
     - Serial.begin() and delay(200) provide host time to attach.
     - Sensor begin() calls may perform I²C transactions.

   This is acceptable because setup() runs exactly once before flight logic.

2) Fail-safe actuation:
     - Servo is attached and immediately commanded to idle.
     - Arming state is forced DISARMED.
     - Actuator enable is tied to arming state (redundant safety).

3) Conservative configuration:
     - eeprom_load_cfg() only adopts stored config if it passes magic/version/CRC.
     - Otherwise cfg remains default-initialized (safe baseline).

4) Scheduling baseline:
     - last_* timestamps are initialized to current millis() so that initial
       epoch functions do not “burst” multiple sensor reads on first loop
       iteration. This enforces controlled startup behavior.

CONTRACT
------------------------------------------------------------------------------
Preconditions:
  - Global objects for sensors (mpu, bmp, mag, lis3dh) exist.
  - Hardware wiring for I²C and GPIO is correct.
Postconditions:
  - Serial prints BOOT diagnostics and HELP text.
  - Sensors are initialized (or failures recorded in ok_* flags and printed).
  - cfg loaded (if valid) and cfg.valid reflects load acceptance.
  - airbrake actuator attached and forced to idle.
  - arm_state is DISARMED and actuator is disabled.
  - Kalman filter is unseeded (seeded=false) awaiting first valid measurement.
  - Policy state reset to DISABLED.
  - All scheduler timestamps initialized.

FAILURE MODES (NON-FATAL BY DESIGN)
------------------------------------------------------------------------------
  - If a sensor fails to initialize, the system continues running:
      - ok_* printed as FAIL
      - corresponding update epochs will publish valid=false snapshots
  This supports ground debug without reflashing and preserves deterministic loop.

STEP-BY-STEP (TRACEABLE)
------------------------------------------------------------------------------
  (1) Start Serial, allow host attach time.
  (2) Configure I²C pins for boards that support remapping (Teensy).
  (3) Begin I²C and set bus clock.
  (4) Configure GPIO for arm switch and buzzer; force buzzer low.
  (5) Initialize each sensor; capture success flags.
  (6) Load EEPROM configuration (may set cfg.valid=true on success).
  (7) Initialize actuator with servo bounds from cfg; force idle.
  (8) Initialize Kalman tuning parameters and clear seeded state.
  (9) Reset policy FSM.
 (10) Emit BOOT diagnostics summary.
 (11) Print HELP and optional telemetry header.
 (12) Force DISARMED arming state (also disables actuator).
 (13) Initialize scheduler timestamps and snapshot timestamps to now.
*/
void setup() {
  // --------------------------------------------------------------------------
  // (1) Serial bring-up (telemetry + commands)
  // --------------------------------------------------------------------------
  Serial.begin(115200);

  // Give the host a brief window to attach a serial monitor.
  // Acceptable here because setup() is one-time initialization, not flight loop.
  delay(200);

  // --------------------------------------------------------------------------
  // (2) I²C pin mapping (board-dependent)
  // --------------------------------------------------------------------------
#if defined(ARDUINO_TEENSY41) || defined(TEENSYDUINO)
  // Teensy supports re-mapping SDA/SCL pins via setSDA/setSCL.
  I2C_BUS.setSDA(PIN_I2C_SDA);
  I2C_BUS.setSCL(PIN_I2C_SCL);
#endif

  // --------------------------------------------------------------------------
  // (3) I²C bus initialization
  // --------------------------------------------------------------------------
  I2C_BUS.begin();
  I2C_BUS.setClock(I2C_CLK_HZ);

  // --------------------------------------------------------------------------
  // (4) GPIO configuration
  // --------------------------------------------------------------------------
  pinMode(PIN_ARM_SWITCH, INPUT_PULLDOWN);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // --------------------------------------------------------------------------
  // (5) Sensor initialization (captures success for diagnostic printing)
  // --------------------------------------------------------------------------
  const bool ok_imu  = init_imu();
  const bool ok_baro = init_baro();
  const bool ok_mag  = init_mag();
  const bool ok_lis3 = init_lis3dh();

  // --------------------------------------------------------------------------
  // (6) Load persisted configuration (CRC-protected)
  // --------------------------------------------------------------------------
  eeprom_load_cfg();

  // --------------------------------------------------------------------------
  // (7) Actuator initialization (fail-safe idle enforced)
  // --------------------------------------------------------------------------
  airbrake.begin(PIN_AIRBRAKE_SERVO, cfg.servo_us_min, cfg.servo_us_max, cfg.servo_us_idle);
  airbrake.forceIdle();

  // --------------------------------------------------------------------------
  // (8) Estimator initialization (tuning + seeded state)
  // --------------------------------------------------------------------------
  kf_alt.q_acc  = 4.0f;
  kf_alt.r_meas = 4.0f;
  kf_alt.seeded = false;

  // --------------------------------------------------------------------------
  // (9) Policy initialization
  // --------------------------------------------------------------------------
  policy_reset();

  // --------------------------------------------------------------------------
  // (10) Boot diagnostics (machine-readable CSV-like lines)
  // --------------------------------------------------------------------------
  Serial.println("BOOT,START");
  Serial.print("BOOT,I2C_SDA,"); Serial.println((int)PIN_I2C_SDA);
  Serial.print("BOOT,I2C_SCL,"); Serial.println((int)PIN_I2C_SCL);
  Serial.print("BOOT,I2C_HZ,");  Serial.println((unsigned long)I2C_CLK_HZ);

  Serial.print("BOOT,IMU,");    Serial.println(ok_imu  ? "OK" : "FAIL");
  Serial.print("BOOT,BARO,");   Serial.println(ok_baro ? "OK" : "FAIL");
  Serial.print("BOOT,MAG,");    Serial.println(ok_mag  ? "OK" : "FAIL");
  Serial.print("BOOT,LIS3DH,"); Serial.println(ok_lis3 ? "OK" : "FAIL");

  Serial.print("BOOT,ACT_EN,");     Serial.println(ACTUATION_ENABLED ? "1" : "0");
  Serial.print("BOOT,POLICY_EN,");  Serial.println(AIRBRAKE_POLICY_ENABLED ? "1" : "0");
  Serial.print("BOOT,EEP_VER,");    Serial.println((unsigned long)EEPROM_VERSION);

  Serial.print("BOOT,KF_R,");     Serial.println(kf_alt.r_meas, 6);
  Serial.print("BOOT,KF_QACC,");  Serial.println(kf_alt.q_acc, 6);

  Serial.println("BOOT,DONE");

  // --------------------------------------------------------------------------
  // (11) Operator help + optional telemetry header
  // --------------------------------------------------------------------------
  print_help();
  if (hdr_enable) print_header();

  // --------------------------------------------------------------------------
  // (12) Force safe arming baseline (disarmed -> actuator disabled + idle)
  // --------------------------------------------------------------------------
  set_arm_state(ArmingState::DISARMED);

  // --------------------------------------------------------------------------
  // (13) Scheduler baseline initialization (prevents startup burst behavior)
  // --------------------------------------------------------------------------
  const uint32_t now = millis();
  last_imu_ms   = now;
  last_aux_ms   = now;
  last_auxvz_ms = now;
  last_baro_ms  = now;
  last_est_ms   = now;
  last_mag_ms   = now;
  last_tlm_ms   = now;

  // Initialize snapshot timestamps for consistent age semantics.
  imu_s.t_ms      = now;
  aux_s.t_ms      = now;
  aux_vz_s.t_ms   = now;
  baro_s.t_ms     = now;
  fs_s.t_ms       = now;
  mag_s.t_ms      = now;
  kf_alt.t_ms     = now;
}


/*
loop()
------------------------------------------------------------------------------
ROLE
  Execute the single-threaded deterministic runtime scheduler.

  This function is the top-level orchestrator that:
    - ingests commands without blocking,
    - services time-gated sensor epochs (one transaction max per due epoch),
    - computes derived states strictly from published snapshots,
    - updates arming state (which hard-gates actuation),
    - evaluates airbrake policy (math-only, bounded),
    - applies actuation only under strict safety conditions,
    - emits fixed-cadence telemetry for post-flight analysis.

SYSTEM INVARIANTS ENFORCED HERE
  (I1) No blocking in loop():
       - no delay()
       - no waiting for Serial
       - no polling loops for sensors

  (I2) Sensor determinism:
       Each epoch routine is time-gated and attempts at most one acquisition
       per invocation.

  (I3) Single-writer discipline:
       Each global snapshot struct is written by exactly one routine.
       loop() never partially writes any snapshot.

  (I4) Actuation safety:
       Motion may occur iff:
         ACTUATION_ENABLED != 0
         AND arm_state == ARMED
         AND policy output pol.valid == true

CONTRACT
  Preconditions:
    - setup() has configured Serial, I²C, GPIO, and sensors.
    - All time bases use millis() (monotonic modulo wrap).
  Postconditions (per call):
    - At most one command line is handled (bounded work).
    - Each sensor update may run at most once per invocation, and only if due.
    - Derived and policy computations are bounded scalar arithmetic.
    - Actuator output is either:
        (a) commanded by policy (ARMED + pol.valid), or
        (b) forced to idle (all other cases).
    - Telemetry is emitted at most once per TLM_PERIOD_MS.

DETERMINISM / REAL-TIME BOUNDS
  - Total work per loop iteration is bounded:
      * command ingestion consumes only currently available bytes
      * each epoch function does O(1) work and at most one I²C transaction
      * derived and policy steps are O(1) math
      * telemetry printing is bounded by fixed field count
  - No dynamic allocation.

SAFETY ORDERING RATIONALE (why this sequence)
  The ordering is chosen to ensure:
    (S1) Derived states use freshest available sensor snapshots.
    (S2) Arming state is updated before any actuation decision is committed.
    (S3) Any failure in sensors/estimation/policy defaults to idle actuation.
    (S4) Telemetry reflects the *same-cycle* policy output applied (or withheld).

STEP-BY-STEP
  (1) Non-blocking command ingestion:
        - reads a complete line if already buffered; never waits
        - updates config/arming/policy enables deterministically
  (2) Sensor epochs (time-gated, 0 or 1 transaction each):
        - IMU (MPU6050)
        - AUX accel (LIS3DH)
        - BARO (BMP5xx)
        - MAG (LIS2MDL)
  (3) Derived epochs (no sensor I/O):
        - auxiliary vertical linear acceleration
        - altitude/vz Kalman estimation
  (4) Arming FSM update:
        - forces actuator disable when not ARMED
        - emits chirp state transitions (non-blocking)
  (5) Policy evaluation (bounded math only):
        - returns (command01, valid)
  (6) Apply actuation gate:
        - if ARMED and valid => apply command
        - else force idle
  (7) Telemetry:
        - emit one row at fixed cadence including policy fields

FAILURE MODES
  - Any sensor failure results in invalid snapshots with updated timestamps.
  - Any stale/invalid estimator causes policy reset and pol.valid=false.
  - Any non-ARMED state forces idle regardless of policy.
*/
void loop() {
  // --------------------------------------------------------------------------
  // (1) Non-blocking command ingestion
  // --------------------------------------------------------------------------
  char line[256];
  if (read_line(line, sizeof(line))) handle_command(line);

  // --------------------------------------------------------------------------
  // (2) Sensor epochs: time-gated; each does at most one I²C read when due
  // --------------------------------------------------------------------------
  (void)imu_update();
  (void)lis3dh_update();
  (void)baro_update();
  (void)mag_update();

  // --------------------------------------------------------------------------
  // (3) Derived epochs: math-only; no hardware transactions
  // --------------------------------------------------------------------------
  (void)aux_vertical_linaccel_update();
  (void)estimate_update();

  // --------------------------------------------------------------------------
  // (4) Arming FSM update + chirps (chirps must not block)
  // --------------------------------------------------------------------------
  const ArmingState prev_arm = arm_state;
  update_arming();

  if (arm_state != prev_arm) {
    if (arm_state == ArmingState::DISARMED) buzzer_start(1, 30, 60);
    if (arm_state == ArmingState::SAFE)     buzzer_start(2, 30, 60);
    if (arm_state == ArmingState::ARMED)    buzzer_start(3, 25, 55);
  }
  buzzer_update();

  // --------------------------------------------------------------------------
  // (5) Policy evaluation (bounded math; no I/O)
  // --------------------------------------------------------------------------
  const AirbrakePolicyOutput pol = compute_airbrake_policy();

  // --------------------------------------------------------------------------
  // (6) Safety-gated actuation
  // --------------------------------------------------------------------------
  if (arm_state == ArmingState::ARMED && pol.valid) {
    airbrake.setCommand01(pol.command01);
  } else {
    airbrake.forceIdle();
  }

  // --------------------------------------------------------------------------
  // (7) Fixed-rate telemetry emission (bounded printing)
  // --------------------------------------------------------------------------
  const uint32_t now = millis();
  if ((now - last_tlm_ms) >= TLM_PERIOD_MS) {
    last_tlm_ms = now;
    emit_tlm(pol);
  }
}