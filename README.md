Please draft a cleaned, properly formatted, GitHub-ready Markdown language exclusively version: 


# Caelum Sufflamen  
### Deterministic Sensor Fusion & Airbrake Control Architecture for High-Power Rockets

> *Caelum Sufflamen* (Latin: “Braking the Sky”) is a rigorously engineered embedded system for real-time rocket state estimation and active airbrake control using deterministic scheduling and Kalman-based sensor fusion.

---

## 🚀 Project Overview

**Caelum Sufflamen** is a flight-computer architecture designed to:

- Estimate rocket altitude and vertical velocity in real time  
- Fuse barometric and inertial measurements using a Kalman Filter  
- Enforce deterministic execution timing  
- Provide a formally documented arming and actuation FSM  
- Maintain telemetry transparency for post-flight analysis  
- Enable safe, bounded, real-time control of deployable airbrakes  

The system prioritizes:

- **Determinism**
- **Mathematical rigor**
- **Engineering traceability**
- **Formal documentation**
- **Bounded real-time behavior**

---

## 🧠 Core Engineering Philosophy

This project is built around five foundational principles:

### 1. Deterministic Execution
- No unbounded loops  
- No dynamic allocation  
- Fixed-rate sensor epochs  
- Single hardware transaction per epoch  

### 2. State-Centric Design
- All control decisions derived from published state snapshots  
- No hidden global mutations  
- Explicit validity flags  

### 3. Explicit Uncertainty Modeling
- Measurement noise (R)  
- Process noise (Q)  
- Covariance propagation  
- Innovation monitoring  

### 4. Safety by Construction
- Compile-time arming gates  
- Runtime FSM gating  
- Safe actuator abstraction  
- Fail-safe behavior on invalid state  

### 5. Telemetry as Observability
- State publication  
- Innovation tracking  
- Covariance insight  
- Debug fields for post-flight validation  

---

## 📐 System Architecture

### High-Level Data FlowSensors → Deterministic Scheduler → State Estimator → FSM → Actuator
│ │ │ │ │
└──────────────┴──────── Telemetry ─────┴─────────────────┴─────────┘
### Functional Modules

| Module | Purpose |
|--------|----------|
| Sensor Epoch Scheduler | Time-bounded polling using `millis()` |
| Barometric Interface | Altitude measurement |
| IMU Interface | Acceleration measurement |
| Vertical Acceleration Estimator | Gravity-compensated acceleration |
| Kalman Filter | Altitude & velocity estimation |
| Arming FSM | Safe actuation control |
| Actuator Abstraction | Servo / airbrake interface |
| Telemetry Layer | Debug and flight analysis output |
| EEPROM Config | Persistent tuning parameters |

---

## 📊 Estimation Model

### State Vector

\[
x =
\begin{bmatrix}
h \\
v
\end{bmatrix}
\]

Where:
- \( h \): altitude (m)  
- \( v \): vertical velocity (m/s)  

---

### Discrete-Time Dynamics (Constant Acceleration Model)

\[
x_{k+1} =
\begin{bmatrix}
1 & \Delta t \\
0 & 1
\end{bmatrix}
x_k +
\begin{bmatrix}
\frac{1}{2} \Delta t^2 \\
\Delta t
\end{bmatrix}
a_k
\]

- Input \( a_k \) derived from IMU  
- Measurement \( z_k \) from barometer  

---

### Kalman Filter Equations

**Predict**

\[
\hat{x}_{k|k-1} = F\hat{x}_{k-1|k-1} + Bu_k
\]

\[
P_{k|k-1} = FP_{k-1|k-1}F^T + Q
\]

**Update**

\[
K_k = P H^T (H P H^T + R)^{-1}
\]

\[
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H\hat{x}_{k|k-1})
\]

\[
P_{k|k} = (I - K_k H)P_{k|k-1}
\]

---

## ⏱ Determinism Contract

The `loop()` function obeys strict rules:

- No blocking delays  
- No dynamic memory allocation  
- Bounded scalar arithmetic only  
- Maximum one hardware transaction per sensor per epoch  
- Fixed execution time per iteration  

This ensures predictable CPU load and prevents flight-time instability.

---

## 🔐 Arming Finite State Machine (FSM)

### States

- `DISARMED`
- `ARMED`
- `ACTIVE_CONTROL`
- `SAFE_LOCK`

### Transitions

| Condition | Transition |
|-----------|------------|
| Compile-time enable + runtime flag | DISARMED → ARMED |
| Launch detect | ARMED → ACTIVE_CONTROL |
| Fault / invalid state | → SAFE_LOCK |

All actuation passes through FSM gating.

---

## 📡 Telemetry Design

Telemetry includes:

- Raw barometric altitude  
- IMU acceleration  
- Gravity-compensated acceleration  
- Fused altitude  
- Fused velocity  
- Covariance terms  
- Innovation residual  
- Arming state  
- Actuator command  

This enables:

- Innovation whiteness checks  
- Covariance tuning  
- Q/R parameter validation  
- Post-flight replay analysis  

---

## 🧪 Validation Strategy

1. Static bench tests  
2. Drop tests (gravity-only model validation)  
3. Logged dataset replay  
4. Innovation monitoring  
5. Covariance growth observation  
6. Process noise tuning sweeps  

---

## 📂 Repository StructureCaelum-Sufflamen/
│
├── RocketAirbrakeModule.ino
├── src/
│ ├── estimator/
│ ├── sensors/
│ ├── control/
│ ├── telemetry/
│ └── fsm/
│
├── docs/
│ ├── IEEE_Report.pdf
│ ├── architecture_diagrams/
│ └── validation_notes/
│
├── data/
│ └── flight_logs/
│
└── README.md
---

## 📘 Theoretical Foundations

This project draws from:

- Linear state-space modeling  
- Discrete-time system propagation  
- Stochastic estimation  
- Covariance algebra  
- Gaussian uncertainty modeling  
- Control gating via finite-state machines  

For a pedagogically structured reference on Kalman filtering from first principles, see:

**Kalman Filter from the Ground Up (Third Edition, 2025)** :contentReference[oaicite:0]{index=0}

---

## 🛠 Hardware Platform

- Microcontroller: Arduino-class MCU  
- Barometric sensor (BMP/BME series)  
- MPU6050 or similar IMU  
- Servo-based airbrake mechanism  
- EEPROM for persistent configuration  

---

## 🧩 Configuration Parameters

| Parameter | Meaning |
|-----------|----------|
| Q | Process noise covariance |
| R | Measurement noise covariance |
| dt | Estimation time step |
| Launch threshold | Detect liftoff |
| Brake profile | Deployment mapping |

---

## ⚠ Safety Disclaimer

This system is experimental and intended for educational and research purposes.  
Flight use requires independent validation, redundancy, and regulatory compliance.

---

## 📈 Future Enhancements

- Extended Kalman Filter (EKF)  
- Unscented Kalman Filter (UKF)  
- Apogee prediction control law  
- Multi-rate fusion  
- Outlier rejection (Mahalanobis gating)  
- SD card high-rate logging  
- MATLAB/Python analysis toolkit  
- Hardware-in-the-loop simulation  

---

## 🧑‍💻 Author

Computer Engineering undergraduate specializing in:

- High-performance C++  
- Physics-based modeling  
- Sensor fusion  
- Deterministic embedded systems  
- State estimation and control  

---

## 📜 License

Specify license here (MIT, GPL, etc.).

---

## ⭐ Contributing

Contributions are welcome in the form of:

- Algorithm improvements  
- Simulation modules  
- Hardware abstraction layers  
- Documentation enhancements  
- Validation datasets  
