
## Caelum Sufflamen: 
Deterministic Instrumentation, Derived State Estimation with Formally Analyzed Kalman Filtering, Tilt-Compensated Heading, Safety-Gated Actuation, Telemetry, and CRC-Protected Configuration


  This README presents *Caelum Sufflamen* (Latin: “sky brakes”), a deterministic embedded flight-software architecture that enforces bounded execution, traceable publication of sensor and derived states, and safety-gated actuation. The implementation includes time-gated sensor epochs, derived-state estimation including a two-state Kalman filter for altitude and vertical velocity with formal observability, Riccati convergence, Lyapunov stability, and an equivalent LMI characterization of the steady-state solution. Tilt-compensated heading with calibration and interference gating, CRC-protected configuration persistence, and a stable telemetry/command interface support post-flight analysis and verification. Monte Carlo studies provide an illustrative evaluation of estimator accuracy and statistical consistency.


# Preface

## Document purpose

This manuscript is a unified explanation of `RocketAirbrakeModule` and the engineering constraints that shaped its structure. Emphasis is placed on:

- **Determinism:** bounded work per `loop()` iteration and per epoch.
- **Traceability:** validity flags and timestamps for data age.
- **Safety by construction:** compile-time and runtime actuation gates.
- **Review readiness:** policy logic intentionally inert until formally reviewed.

## Table of contents

- [Background and system context](#background-and-system-context)
  - [Motivation](#motivation)
  - [Core design principles](#core-design-principles)
  - [Module scope](#module-scope)
- [Hardware architecture](#hardware-architecture)
  - [Processing platform](#processing-platform)
  - [I²C bus strategy](#i²c-bus-strategy)
  - [Sensors](#sensors)
  - [Actuator and safety wiring](#actuator-and-safety-wiring)
- [Software architecture and determinism contract](#software-architecture-and-determinism-contract)
  - [Top-level structure](#top-level-structure)
  - [Cooperative scheduler](#cooperative-scheduler)
  - [Nominal epoch rates](#nominal-epoch-rates)
  - [Formal determinism invariants](#formal-determinism-invariants)
- [State model and data contracts](#state-model-and-data-contracts)
  - [Single-writer published state structs](#single-writer-published-state-structs)
  - [Validity and timestamp semantics](#validity-and-timestamp-semantics)
  - [Formal dataflow and contracts diagram](#formal-dataflow-and-contracts-diagram)
- [EEPROM persistence and CRC integrity](#eeprom-persistence-and-crc-integrity)
- [Sensor epochs](#sensor-epochs)
  - [IMU epoch (`imu_update`)](#imu-epoch-imu_update)
  - [Barometer epoch (`baro_update`)](#barometer-epoch-baro_update)
  - [Magnetometer epoch (`mag_update`)](#magnetometer-epoch-mag_update)
- [Derived state estimation](#derived-state-estimation)
- [Heading pipeline and calibration](#heading-pipeline-and-calibration)
- [Actuation and arming](#actuation-and-arming)
- [Telemetry and command interface](#telemetry-and-command-interface)
- [System requirements and traceability](#system-requirements-and-traceability)
- [Timing budget and worst-case execution time](#timing-budget-and-worst-case-execution-time)
- [Kalman-based altitude and vertical velocity estimation](#kalman-based-altitude-and-vertical-velocity-estimation)
- [Deterministic execution and safety guarantees](#deterministic-execution-and-safety-guarantees)
- [Operational procedures and test plan](#operational-procedures-and-test-plan)
- [Extension roadmap](#extension-roadmap)
- [Conclusion](#conclusion)
- [Appendix A: Telemetry field reference](#appendix-a-telemetry-field-reference)
- [Appendix B: Warning mask bit assignments](#appendix-b-warning-mask-bit-assignments)
- [Appendix C: Engineering invariants](#appendix-c-engineering-invariants)

# Background and system context

## Motivation

Small-scale flight systems occupy a challenging regime:

- **Fast dynamics:** sensor and estimator updates must be periodic and bounded.
- **Resource constraints:** CPU time, bus bandwidth, and memory are finite.
- **Safety criticality:** actuator outputs must be tightly controlled.
- **Accountability:** stable telemetry schemas and validity metadata are required for post-flight analysis.

A common failure mode in prototype flight code is architectural drift: ad-hoc control logic becomes entangled with sensor reads, printing, and timing delays. This increases the probability of timing jitter, hidden blocking calls, and unsafe actuator states.

## Core design principles

### Determinism as an explicit contract

Determinism is treated as a contract enforced by structure:

1. `loop()` does not block (no delays, no waits).
2. Each epoch function performs bounded work and returns immediately if not scheduled.
3. No internal retry loops occur inside epoch functions.
4. No dynamic allocation occurs in telemetry and command paths.

### Traceability via validity and age

Each state struct carries:

- `valid`: whether the most recent update succeeded and is semantically meaningful.
- `t_ms`: timestamp updated on every attempt (success or failure).

This yields:

1. telemetry ages remain meaningful through intermittent sensor faults,
2. downstream logic can detect staleness without ambiguous interpretation.

### Safety by construction

Actuation is prohibited unless two independent gates pass:

- **Compile-time gate:** `ACTUATION_ENABLED` (default `0`)
- **Runtime gate:** arming requires both a physical switch and a software token

Additionally, policy output must be explicitly marked valid before actuator application.

## Module scope

### Provided by the module

- deterministic sensor acquisition epochs
- derived-state estimation and plausibility gating
- tilt-compensated heading pipeline with calibration support
- CRC-protected EEPROM configuration container
- telemetry schema and command interface
- safety-gated actuator abstraction

### Deliberately excluded by default

- closed-loop airbrake policy logic (placeholder returns invalid output)
- pyro control and ignition sequencing
- full navigation and higher-order state estimation beyond basic derived state

# Hardware architecture

## Processing platform

The intended target is a Teensy 4.1 class microcontroller running an Arduino-compatible environment. The architecture assumes:

- a stable millisecond time base (`millis()`)
- I²C bus operation at 400 kHz
- serial port availability for telemetry and commands
- a PWM-capable pin for servo output

## I²C bus strategy

All sensors attach to a single selected bus:

- single-bus topology reduces concurrency ambiguity
- fixed probe order makes bring-up behavior repeatable
- fixed bus clock rate supports deterministic transaction time estimates

## Sensors

### MPU6050 IMU

- Provides accelerometer and gyroscope.
- Roll and pitch are derived from a gravity-vector approximation when dynamics are low.
- A plausibility gate (`motion_bad`) is computed from acceleration norm.

### BMP58x barometer (e.g., BMP585)

- Provides pressure and temperature.
- Pressure is used to derive altitude relative to a configurable reference.
- A captured baseline pressure supports pad-relative altitude.

### LIS2MDL magnetometer

- Provides magnetic field vector in microtesla.
- Heading computation is tilt-compensated using IMU roll/pitch.
- Calibration supports hard-iron offsets and diagonal scaling.
- Field magnitude plausibility gating suppresses heading under interference.

### LIS3DH auxiliary accelerometer

- Initialized to reserve presence and compatibility.
- Optional epoch for redundancy and vibration monitoring; may be disabled to preserve timing margin.

## Actuator and safety wiring

The actuator is a servo controlling an airbrake mechanism.

- PWM output is mediated through `AirbrakeActuator`.
- A physical arming switch is mapped to a GPIO input with pull-down.

The safety posture is strict: the physical switch alone is insufficient; the software token and compile-time gate must also pass.

# Software architecture and determinism contract

## Top-level structure

The codebase is organized by engineering concerns:

1. compile-time safety switches and constants
2. sensor objects and hardware bindings
3. state types (structs/enums) and single-writer discipline
4. EEPROM persistence and CRC integrity
5. epoch functions and scheduler contract
6. derived state and gating logic
7. actuation abstraction and arming FSM
8. telemetry schema, warning mask, and command parser

## Cooperative scheduler

A cooperative scheduler is implemented inside `loop()`:

- Each epoch checks `now - last_ms >= period`; if not due it returns immediately.
- Each scheduled epoch performs bounded work and returns.
- No `delay()` calls appear in the scheduler loop.

## Nominal epoch rates

- IMU: 100 Hz
- Barometer: 50 Hz
- Magnetometer: 50 Hz
- Estimator: 50 Hz
- Telemetry: 20 Hz

## Formal determinism invariants

1. No blocking in `loop()`.
2. Bounded epochs: each epoch performs at most one sensor transaction per scheduled invocation.
3. Epoch idempotence: epochs may be called every loop iteration.
4. No dynamic allocation on hot paths.

# State model and data contracts

## Single-writer published state structs

Each subsystem publishes into a single-writer state struct:

- `ImuSample imu_s`
- `BaroSample baro_s`
- `MagSample mag_s`
- `FlightState fs_s`

## Validity and timestamp semantics

Each state struct carries:

- `t_ms`: updated on every attempt (success or failure)
- `valid`: semantic correctness of published fields

Define an age operator for a state $S$ at time $t$:

$$
\operatorname{Age}(S,t)=
\begin{cases}
t - S.t\_ms, & S.valid = \text{true} \\
\bot, & S.valid = \text{false}
\end{cases}
$$ {#eq:age-operator}

## Formal dataflow and contracts diagram

![Figure: System dataflow with single-writer publication, validity/age propagation, and actuation gating.](figs/system-dataflow.png){#fig:system-dataflow width=90%}

**Contract C-1 (single-writer):** Each state struct has exactly one writer.  
**Contract C-2 (validity semantics):** `t_ms` updates on every attempt. `valid` reflects semantic correctness.  
**Contract C-3 (safety gating):** Actuator forced idle unless `ACTUATION_ENABLED ∧ ARMED ∧ pol.valid`.

# EEPROM persistence and CRC integrity

## Motivation

Configuration storage is safety-relevant: corrupted parameters can compromise derived state and actuation behavior. The design stores a structured blob with explicit integrity checks.

## Blob structure and CRC32 method

A typical EEPROM blob includes:

- a magic constant
- schema version
- configuration struct
- reserved padding
- CRC32 over preceding fields

During CRC computation, the stored CRC field is treated as zero to avoid self-inclusion.

## Operational behavior

- `eeprom_load_cfg()` rejects blobs with wrong magic, wrong version, or CRC mismatch.
- `eeprom_save_cfg()` writes a newly computed CRC.

# Sensor epochs

## IMU epoch (`imu_update`)

### Functional role

`imu_update()` performs bounded acquisition and publishes:

- accelerometer $(a_x,a_y,a_z)$
- gyroscope $(\omega_x,\omega_y,\omega_z)$
- roll/pitch derived from the gravity-vector approximation
- acceleration norm $\|a\|$ and a plausibility flag

### Roll and pitch from gravity vector

Under low translational acceleration, the accelerometer measures gravity in sensor coordinates. Let $(a_x,a_y,a_z)$ be the measured acceleration. Then:

$$
\phi = \tan^{-1}\!\left(\frac{a_y}{a_z}\right), \qquad
\theta = \tan^{-1}\!\left(\frac{-a_x}{\sqrt{a_y^2+a_z^2}}\right)
$$ {#eq:imu-roll-pitch}

where $\phi$ is roll and $\theta$ is pitch.

### Motion plausibility gate

Compute:

$$
\|a\| = \sqrt{a_x^2 + a_y^2 + a_z^2}
$$ {#eq:accel-norm}

A conservative range check is used to detect regimes where the gravity-vector approximation is unreliable (free-fall, severe vibration, or large accelerations). When out-of-range, `motion_bad` is asserted and tilt compensation is suppressed.

## Barometer epoch (`baro_update`)

`baro_update()` publishes pressure (hPa) and temperature (°C) with validity and timestamps.

## Magnetometer epoch (`mag_update`)

`mag_update()` publishes:

- magnetic vector $(m_x,m_y,m_z)$
- magnetic norm $\|m\|$
- interference flag when $\|m\|$ is implausible
- heading when gated valid
- calibration-valid flag

# Derived state estimation

## Pressure-to-altitude approximation

Altitude is approximated from pressure as:

$$
h \approx 44330 \left(1 - \left(\frac{P}{P_0}\right)^{0.190294957}\right)
$$ {#eq:pressure-altitude}

where $P$ is measured pressure and $P_0$ is a reference pressure (baseline if captured, else sea-level reference).

## EMA filtering (optional baseline)

Altitude EMA update:

$$
\hat{h}_k = \hat{h}_{k-1} + \alpha (h_k - \hat{h}_{k-1})
$$ {#eq:ema-altitude}

Vertical speed approximation:

$$
v_z \approx \frac{\hat{h}_k - \hat{h}_{k-1}}{\Delta t}
$$ {#eq:ema-vz}

A similar EMA may be applied to velocity. In practice, the EMA and Kalman paths can coexist, with selection via a configuration switch.

## Baseline vs. sea-level reference

- If a baseline is captured (`baro_baseline_hpa` finite), altitude becomes pad-relative.
- Otherwise, `sea_level_hpa` is used for approximate MSL altitude.

# Heading pipeline and calibration

## Tilt compensation motivation

A magnetometer measures the 3D field vector. Without tilt compensation, computed yaw/heading is corrupted by vertical components when the sensor frame is rolled/pitched. Tilt compensation rotates the measured field into an estimated horizontal plane using roll/pitch.

## Tilt-compensated heading

Let $(m_x,m_y,m_z)$ be the calibrated magnetic field, roll $\phi$, pitch $\theta$. A standard tilt compensation approximation is:

$$
X_h = m_x \cos\theta + m_z \sin\theta
$$ {#eq:tilt-xh}

$$
Y_h = m_x \sin\phi \sin\theta + m_y \cos\phi - m_z \sin\phi \cos\theta
$$ {#eq:tilt-yh}

Heading:

$$
\psi = \tan^{-1}\!\left(\frac{Y_h}{X_h}\right)
$$ {#eq:heading-psi}

Then apply declination correction and wrap to $[0,360)$.

## Calibration model

Hard-iron offsets $(o_x,o_y,o_z)$ and diagonal scales $(s_x,s_y,s_z)$:

$$
m'_x = (m_x - o_x)s_x,\quad
m'_y = (m_y - o_y)s_y,\quad
m'_z = (m_z - o_z)s_z
$$ {#eq:mag-calibration}

## Interference gate

Earth-field magnitude is typically tens of microtesla. A conservative range gate (example):

- `MAG_NORM_MIN_uT = 10`
- `MAG_NORM_MAX_uT = 100`

Outside this range, heading is invalidated and `mag_interference` is asserted.

# Actuation and arming

## Actuator abstraction

The actuator is wrapped by `AirbrakeActuator`:

- command input `command01 ∈ [0,1]`
- controlled enable/disable behavior
- safe idle enforcement

## Safety gates

Motion is permitted only if:

1. `ACTUATION_ENABLED == 1` (compile-time)
2. arming state is `ARMED` (runtime)
3. policy output is valid (`pol.valid == true`)

If any condition fails, actuator output is forced to idle.

## Formal arming state diagram

![Figure: Arming finite-state machine with explicit guards, actions, and safety invariant.](figs/arming-fsm.png){#fig:arming-fsm width=85%}

## Policy placeholder

The policy function is intentionally inert: it returns an invalid output unless replaced by reviewed logic. This ensures that even an armed system remains actuator-idle until a formal policy implementation asserts `pol.valid = true`.

# Telemetry and command interface

## Telemetry schema stability

Telemetry uses a CSV line format. A header line (`HDR,...`) defines field order. Field order is treated as a parsing contract; extensions should append fields rather than reorder.

## Age computation

Ages are computed from `t_ms` fields and current time. Invalid states return a sentinel (conceptually $\bot$). This enables downstream analysis to distinguish:

- invalid-now
- valid-but-stale
- valid-and-fresh

## Warning mask

A compact `warn_mask` bitfield summarizes safety and health states for fast parsing and alerting.

## Command set (conceptual)

The command interface supports:

- status/inspection (`STATUS`)
- header enable/disable (`HDR 0|1`)
- configuration edits (`SET_SLP`, `SET_DECL`, `CAP_BASELINE`)
- magnetometer calibration (`CAL_MAG`, `CLEAR_MAG_CAL`)
- EEPROM persistence (`SAVE_CFG`, `LOAD_CFG`)
- safety/arming (`ARM <token>`, `DISARM`)

# System requirements and traceability

## Requirements philosophy

Requirements are presented as verifiable statements mapped to code sections/functions and telemetry observables proving satisfaction.

## Requirements traceability table

Table: Requirements traceability mapping to code and telemetry observables. {#tbl:req-trace}

| Req ID | Requirement description | Code mapping | Telemetry observables |
|:--|:--|:--|:--|
| R-1 | Acquire IMU data at fixed 100 Hz with bounded execution and publish roll/pitch and plausibility metadata. | `imu_update()`, `ImuSample` | `roll_deg`, `pitch_deg`, `a_norm`, `imu_age_ms`, `motion_bad` |
| R-2 | Acquire barometer pressure/temperature at fixed 50 Hz and publish validity and age. | `baro_update()`, `BaroSample` | `press_hpa`, `temp_c`, `baro_age_ms` |
| R-3 | Compute altitude from pressure relative to selectable reference: baseline if captured, else sea-level. | `pressure_to_altitude_m()`, `estimate_update()` | `alt_m`, `cfg_valid`, `baseline_hpa` |
| R-4 | Compute vertical speed as filtered derivative or fused Kalman state at 50 Hz. | `estimate_update()` | `vz_mps`, `est_age_ms` |
| R-5 | Compute tilt-compensated heading only when calibration is valid, IMU tilt is plausible, and magnetic norm is plausible. | `mag_update()`, `heading_tilt_comp_deg()` | `hdg_deg`, `mag_valid`, `mag_interf`, `mag_norm_uT`, `mag_cal_valid` |
| R-6 | Guarantee actuator idle unless compile-time gate, runtime arming, and policy validity all pass. | `ACTUATION_ENABLED`, `update_arming()`, `compute_airbrake_policy()` | `arm_state`, `act_en`, `warn_mask` bit 4 |
| R-7 | Provide stable schema telemetry at fixed 20 Hz including health summary mask. | `print_header()`, `emit_tlm()`, `build_warn_mask()` | `HDR`, `TLM`, `warn_mask` |
| R-8 | Persist configuration with CRC integrity and reject corrupted or incompatible EEPROM blobs. | `eeprom_save_cfg()`, `eeprom_load_cfg()`, CRC32 | `EEPROM,LOAD,OK/CRC_FAIL/BAD_VERSION/NO_MAGIC` |

## Telemetry-based verification notes

- R-1–R-5 can be verified by age fields and plausibility gates.
- R-6 is verified by invariants: actuator remains idle while `pol.valid = false` and/or not armed.
- R-8 is verified by status lines across power cycles and deliberate corruption tests.

# Timing budget and worst-case execution time

## Purpose

A timing budget provides a schedulability argument and a regression baseline.

## Assumptions

- I²C at 400 kHz
- Teensy-class CPU with compute margin
- Each library call executes a bounded number of transfers (no blocking waits)
- Telemetry output at 115200 baud; formatting dominates CPU time, transmission is buffered

## WCET table

Table: Conservative WCET estimates for scheduled epochs and worst-case alignment. {#tbl:wcet}

| Function / epoch | Rate | WCET (µs) | Dominant costs |
|:--|:--:|--:|:--|
| `imu_update()` | 100 Hz | ≤ 300 | I²C burst read + trig/sqrt |
| `baro_update()` | 50 Hz | ≤ 450 | Pressure + temperature read and conversion |
| `mag_update()` | 50 Hz | ≤ 260 | Mag read + cal + norm + heading trig |
| `estimate_update()` | 50 Hz | ≤ 120 | Scalar compute + KF update |
| `update_arming()` | cont. | ≤ 20 | GPIO + state logic |
| `compute_airbrake_policy()` | cont. | ≤ 10 | Placeholder |
| `emit_tlm()` | 20 Hz | ≤ 900 | Formatting + multiple `Serial.print` |
| **Worst-case aligned loop** | — | **≈ 2060** | All epochs coincide with telemetry |

## Schedulability argument

Worst-case aligned loop work $\approx 2.06$ ms leaves margin within the fastest period (10 ms IMU cadence). This supports additional bounded logic without violating the determinism contract.

## Recommended timing regression instrumentation

An optional diagnostic mechanism is recommended:

- timestamp at epoch entry/exit
- track maximum observed duration per epoch
- emit a 1 Hz diagnostic line with observed maxima

# Kalman-based altitude and vertical velocity estimation

## Model and filter

The estimator is a deterministic two-state discrete-time Kalman filter with state

$$
x_k =
\begin{bmatrix}
z_k\\
v_k
\end{bmatrix},\quad
A =
\begin{bmatrix}
1 & \Delta t\\
0 & 1
\end{bmatrix},\quad
B =
\begin{bmatrix}
\tfrac{1}{2}\Delta t^2\\
\Delta t
\end{bmatrix},\quad
H =
\begin{bmatrix}
1 & 0
\end{bmatrix}
$$ {#eq:kf-model}

Process and measurement equations:

$$
x_{k+1} = A x_k + B u_k + w_k,\qquad
y_k = H x_k + v_k
$$ {#eq:kf-process-measurement}

with $u_k$ the measured vertical linear acceleration input and $(w_k, v_k)$ noise.

**Assumption 1.** $w_k$ and $v_k$ are zero-mean independent sequences with covariances $Q \succ 0$ and $R > 0$.

A white-acceleration process noise model yields

$$
Q = q_{\text{acc}}
\begin{bmatrix}
\frac{\Delta t^4}{4} & \frac{\Delta t^3}{2}\\
\frac{\Delta t^3}{2} & \Delta t^2
\end{bmatrix},
\qquad q_{\text{acc}} > 0
$$ {#eq:kf-Q}

## Predict–update equations

Prediction:

$$
\hat{x}_k^- = A\hat{x}_{k-1} + B u_k,\qquad
P_k^- = A P_{k-1} A^T + Q
$$ {#eq:kf-predict}

Update:

$$
K_k = P_k^- H^T (H P_k^- H^T + R)^{-1}
$$ {#eq:kf-gain}

$$
\hat{x}_k = \hat{x}_k^- + K_k \left(y_k - H\hat{x}_k^- \right)
$$ {#eq:kf-state-update}

Joseph covariance update:

$$
P_k = (I-K_kH)P_k^-(I-K_kH)^T + K_k R K_k^T
$$ {#eq:kf-joseph}

## Observability and detectability

**Theorem 1 (Observability).** For $\Delta t \neq 0$, the pair $(A,H)$ is observable.

**Proof.** The observability matrix is

$$
\mathcal{O} =
\begin{bmatrix}
H\\
HA
\end{bmatrix}
=
\begin{bmatrix}
1 & 0\\
1 & \Delta t
\end{bmatrix}
$$ {#eq:observability-matrix}

with $\det(\mathcal{O}) = \Delta t \neq 0$, hence full rank. $\square$

**Corollary 1.** $(A,H)$ is detectable.

## Riccati convergence

The Riccati recursion induced by the Kalman filter is

$$
P_{k+1}
= A P_k A^T + Q
- A P_k H^T (H P_k H^T + R)^{-1} H P_k A^T
$$ {#eq:riccati-recursion}

**Theorem 2 (Existence and uniqueness of stabilizing solution).** Under Assumption 1 and Theorem 1, the discrete-time algebraic Riccati equation admits a unique stabilizing solution $P_\infty \succ 0$ and $P_k \to P_\infty$.

**Proof.** Observability implies detectability; with $Q \succ 0$ and $R > 0$, standard discrete-time Kalman filtering conditions hold, yielding a unique stabilizing solution and convergence of $P_k$. $\square$

## Lyapunov stability of estimation error

Define estimation error $e_k = x_k - \hat{x}_k$. In steady state, $K_k \to K_\infty$, giving LTI error dynamics

$$
e_{k+1} = F e_k + \eta_k,\qquad
F = A - K_\infty H,\qquad
\eta_k = w_k - K_\infty v_k
$$ {#eq:error-dynamics}

**Theorem 3 (Mean-square exponential stability).** Under Assumption 1 and Theorem 1, the steady-state estimation error dynamics are mean-square exponentially stable; equivalently, $\rho(F) < 1$.

**Proof.** At steady state, $P_\infty$ satisfies the algebraic Riccati equation which implies

$$
P_\infty - F P_\infty F^T = Q + K_\infty R K_\infty^T \triangleq W \succ 0
$$ {#eq:lyap-identity}

Pre- and post-multiplying by $P_\infty^{-1}$ yields a discrete Lyapunov inequality, implying $\rho(F)<1$ and exponential stability. Mean-square boundedness follows by standard linear stochastic system arguments with bounded driving noise covariance. $\square$

## LMI characterization of the steady-state solution

**Proposition 1 (Riccati inequality as an LMI).** For $P \succ 0$, the Riccati inequality

$$
P \succ APA^T + Q - APH^T (HPH^T + R)^{-1} HPA^T
$$ {#eq:riccati-ineq}

is equivalent (via Schur complement) to an LMI form suitable for convex optimization.

A common convex surrogate is:

$$
\min_{P \succ 0} \operatorname{trace}(P)
\quad \text{s.t. Riccati-LMI constraints}
$$ {#eq:lmi-surrogate}

Once $P$ is obtained, the corresponding steady-state gain is

$$
K = A P H^T (H P H^T + R)^{-1}
$$ {#eq:ss-gain}

## Monte Carlo performance evaluation (illustrative)

Illustrative study setup:

- 1000 independent trials
- $q_{\text{acc}} = 4.0$, $R = 4.0$
- $\Delta t = 0.02$ s
- ballistic ascent + coast + descent profile with Gaussian noise injections

Per-run metrics:

$$
\mathrm{RMSE}_z = \sqrt{\frac{1}{N}\sum_{k=1}^N (z_k-\hat{z}_k)^2},\qquad
\mathrm{RMSE}_v = \sqrt{\frac{1}{N}\sum_{k=1}^N (v_k-\hat{v}_k)^2}
$$ {#eq:rmse-metrics}

Consistency via NEES:

$$
\epsilon_k = e_k^T P_k^{-1} e_k,\qquad \mathbb{E}[\epsilon_k] \approx n = 2
$$ {#eq:nees}

Table: Example Monte Carlo performance over 1000 trials. {#tbl:monte-carlo}

| Metric | Mean | Std dev |
|:--|--:|--:|
| Altitude RMSE (m) | 0.84 | 0.12 |
| Velocity RMSE (m/s) | 0.41 | 0.09 |

## Kalman filter block diagram

![Figure: Discrete-time Kalman filter structure with coupled state and covariance propagation.](figs/kalman-block-diagram.png){#fig:kf-block width=85%}

## Covariance evolution example

![Figure: Illustrative covariance convergence under constant noise assumptions.](figs/covariance-convergence.png){#fig:covariance-conv width=70%}

## Comparison: EMA vs. Kalman estimator

Table: Comparison of exponential moving average and Kalman-based estimator. {#tbl:ema-vs-kf}

| Feature | EMA | Kalman |
|:--|:--:|:--:|
| Dynamic model | No | Yes |
| Uncertainty propagation | No | Yes |
| State coupling | No | Yes |
| Acceleration fusion | No | Yes |
| Optimal (Gaussian noise) | No | Yes |
| Handles dropouts | Limited | Robust |
| Velocity noise amplification | High | Low |
| Computational complexity | $O(1)$ | $O(1)$ |
| WCET deterministic | Yes | Yes |

# Deterministic execution and safety guarantees

## Formal statements

**Definition 1 (Deterministic epoch).** An epoch function is deterministic if it:

1. performs a bounded number of floating-point operations,
2. performs at most one sensor transaction per scheduled invocation, and
3. contains no blocking calls.

**Theorem 4 (Bounded execution time).** If all epochs satisfy Definition 1 and the scheduler invokes a finite set of epochs per loop iteration, then worst-case loop execution time is bounded.

**Proof.** Each epoch executes $O(1)$ work; the loop executes a finite sum of bounded costs, hence bounded. $\square$

**Definition 2 (Actuation permission predicate).**

$$
\Pi \triangleq
\texttt{ACTUATION\_ENABLED}
\wedge (\texttt{arm\_state} = \texttt{ARMED})
\wedge \texttt{pol.valid}
$$ {#eq:actuation-predicate}

**Theorem 5 (Actuator safety guarantee).** If $\Pi = 0$ then the actuator output is forced to the idle command.

**Proof.** The control logic implements

$$
u =
\begin{cases}
u_{\text{policy}}, & \Pi = 1\\
u_{\text{idle}}, & \Pi = 0
\end{cases}
$$ {#eq:actuator-safety}

thus the actuator cannot move unless $\Pi$ holds. $\square$

# Operational procedures and test plan

## Bench bring-up checklist

1. Power system with sensors on the selected I²C bus.
2. Confirm init status lines (e.g., `BOOT,IMU,OK`).
3. Confirm telemetry header (`HDR,...`) and telemetry lines (`TLM,...`).
4. Confirm warning mask reflects expected initial conditions (baseline missing until captured).

## Configuration and calibration

### Baseline capture

1. Place system at reference condition (pad).
2. Execute `CAP_BASELINE`.
3. Confirm `baseline_hpa` becomes finite and altitude becomes pad-relative.

### Magnetometer calibration

1. Execute `CAL_MAG <seconds>` (e.g., 20–40).
2. Rotate through broad orientations.
3. Confirm completion line and `mag_cal_valid = 1`.

### EEPROM persistence

1. Execute `SAVE_CFG`.
2. Power cycle.
3. Execute `LOAD_CFG`.
4. Confirm `EEPROM,LOAD,OK`.

## Arming safety test

1. Switch off: `arm_state` remains `DISARMED`.
2. Switch on without token: `SAFE`.
3. After `ARM <token>`: `ARMED`.
4. Confirm actuator remains idle (policy invalid by default).

# Extension roadmap

## Policy integration requirements

A reviewed policy should satisfy:

- bounded compute time
- explicit validity semantics
- explicit saturation behavior for `command01`
- safe fallbacks under missing or stale sensor data

## Telemetry evolution

CSV is human-friendly but bandwidth-expensive. A binary packet format with CRC may be added:

- fixed-size framing
- explicit schema versioning
- faster parsing and lower overhead

## Fault classification and redundancy

Potential extensions:

- barometer dropout detection vs. implausible pressure jumps
- magnetometer interference pattern classification
- LIS3DH vibration/shock classification for gating tilt compensation

# Conclusion

This paper consolidated a deterministic embedded flight-software structure built around explicit epoch scheduling, single-writer state publication, validity/age semantics, and strict actuation gating. A two-state Kalman estimator for altitude and vertical velocity was presented with formal observability, Riccati convergence, Lyapunov stability arguments, and an equivalent LMI characterization. The resulting architecture supports reviewable safety invariants and telemetry-based verification suitable for iterative flight-system development.

# Appendix A: Telemetry field reference

## Conceptual grouping

- **Time and safety:** `ms`, `arm_state`, `act_en`
- **Barometer:** `press_hpa`, `temp_c`, `baro_age_ms`
- **Derived state:** `alt_m`, `vz_mps`, `est_age_ms`
- **IMU:** `roll_deg`, `pitch_deg`, `a_norm`, `imu_age_ms`, `motion_bad`
- **Heading:** `hdg_deg`, `mag_valid`, `mag_interf`, `mag_age_ms`, `mag_norm_uT`, `decl_deg`, `mag_cal_valid`
- **Configuration and warnings:** `cfg_valid`, `baseline_hpa`, `warn_mask`

# Appendix B: Warning mask bit assignments

Table: Warning mask bit assignments. {#tbl:warn-mask}

| Bit | Meaning |
|:--:|:--|
| 0 | Barometer invalid |
| 1 | IMU invalid |
| 2 | Derived estimate invalid |
| 3 | Baseline missing |
| 4 | Not armed |
| 5 | Magnetometer heading invalid |
| 6 | Magnetometer interference detected |
| 7 | Magnetometer calibration missing |

# Appendix C: Engineering invariants

## Determinism invariants

1. `loop()` contains no `delay()` calls.
2. Each epoch performs at most one sensor transaction per scheduled invocation.
3. Epochs are safe to call every iteration (internal period gating).

## Safety invariants

1. If `arm_state != ARMED` then actuator output is forced idle.
2. If `ACTUATION_ENABLED == 0` then actuator output is forced idle regardless of runtime arming.
3. If `pol.valid == false` then actuator output is forced idle.
