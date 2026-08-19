# CanSat Autonomous Glider Flight Software (Team 1094)

![C++](https://img.shields.io/badge/Language-C%2B%2B-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Teensy%20%7C%20Arduino-orange.svg)
![Build](https://img.shields.io/badge/Status-Flight%20Ready-brightgreen.svg)

Embedded flight software for an autonomous aerospace glider developed for the International CanSat Competition. The system handles multi-sensor fusion over dual I2C buses, real-time telemetry downlink, ground station command handling, state restoration via non-volatile memory (EEPROM/SD), dual camera triggering, and actuation for flight surface control and payload separation.

---

## Technical Overview

The core architecture executes an asynchronous, event-driven flight loop managing sensor updates, telemetry logging, state transitions, and high-frequency guidance adjustments.

```text
                  ┌───────────────────────────────────────────┐
                  │          Dual I2C / SPI Sensors           │
                  │ (BMP581, BNO085, SAM-M8Q GNSS, INA260)    │
                  └─────────────────────┬─────────────────────┘
                                        │
                                        ▼
┌───────────────────┐        ┌─────────────────────┐        ┌───────────────────┐
│ Ground Control    │◄──────►│ Flight Loop & State │───────►│ Storage & State   │
│ Station (115200)  │  UART  │ Engine (10 Hz / 1s) │        │ (EEPROM & SD Card)│
└───────────────────┘        └──────────┬──────────┘        └───────────────────┘
                                        │
                                        ▼
                  ┌───────────────────────────────────────────┐
                  │       Actuation & Guidance Systems        │
                  │   (Bezier Path Steering & Release Servos) │
                  └───────────────────────────────────────────┘
```

---

## Hardware & Peripheral Integration

* **Sensors & Busses:**
  * **BMP581 Precision Barometer (I2C Bus 0):** Altitude & atmospheric pressure tracking with exponential moving average filtering.
  * **BNO085 9-DOF IMU/AHRS (I2C Bus 1):** 100 Hz fused rotation vectors (heading calculation with magnetic declination correction) and 3-axis calibrated angular velocity.
  * **SparkFun u-blox SAM-M8Q GNSS (I2C Bus 1):** 5 Hz position fix (latitude, longitude, altitude) and GPS time synchronization.
  * **Adafruit INA260 (I2C Bus 1):** Real-time bus voltage and current draw tracking.
* **Actuation & Servos:**
  * **Release System:** Servo-actuated multi-stage separation (Probe separation, Nose cone deployment, Egg payload release).
  * **Steering Mechanisms:** Differential port and starboard servos for paraglider/flight surface control.
* **Storage & Recovery:**
  * **SD Card (SPI):** Real-time flight telemetry logging (`flight.csv`) and persistent flight state logging (`state.txt`).
  * **EEPROM:** Mid-Mission Flag (MMF) storage to retain critical flight state across un-commanded power resets.
* **Payload Operations:**
  * Ground-facing and release-view onboard action camera control via GPIO pulse triggering.

---

## Flight State Machine Architecture

The flight system operates as a deterministic finite-state machine (`sw_state`) governing autonomous operations from pad initialization to landing:

1. **`LAUNCH_PAD`:** Base calibration, sensor sanity checks, altitude zeroing, and launch detection (Alt ≥ 20m, Vel ≥ 10m/s).
2. **`ASCENT`:** Apogee monitoring via consecutive negative vertical rate filtering.
3. **`APOGEE`:** Automated onboard camera activation and descent sequence transition.
4. **`DESCENT`:** Descent rate tracking and altitude-triggered deployment.
5. **`PROBE_RELEASE`:** Mechanical probe deployment; active guidance, dynamic path generation, and steering logic execution.
6. **`PAYLOAD_RELEASE`:** Low-altitude payload/egg separation (≤ 4m).
7. **`LANDED`:** Disarm system, close storage files, enable visual beaconing, and cut camera recordings.

---

## Guidance, Navigation, & Control (GNC)

The flight system incorporates a dynamic path-following algorithm designed to steer the glider toward target drop zone coordinates:

* **Quadratic Bézier Path Planning:** Generates flight paths (P0 → P1 → P2) connecting deployment points (P0) through intermediate gate points (P1) to the target coordinate (P2).
* **Dynamic Path Redrawing:** Monitors cross-track error using dynamic distance checks (`check_and_redraw_path`); automatically recalculates path targets if wind drift exceeds 20 meters.
* **Vector Product Steering Error:** Computes direction errors via cross product calculations between current heading vectors and active Bézier targets, adjusting differential servo positions for left/right correction.
* **Braking Alignment Check:** Evaluates target-heading dot products to verify alignment prior to initiating descent flare and braking maneuvers.

---

## Telemetry & Command System

### Downlink Telemetry Format
Downlinked via serial telemetry (`Serial1`, 115200 baud) at 1 Hz in CSV format:

```csv
<TEAM_ID>,<MISSION_TIME>,<PACKET_COUNT>,<MODE>,<STATE>,<ALTITUDE>,<TEMP>,<PRESSURE>,<VOLTAGE>,<CURRENT>,<GYRO_R>,<GYRO_P>,<GYRO_Y>,<ACCEL_R>,<ACCEL_P>,<ACCEL_Y>,<GPS_TIME>,<GPS_ALT>,<GPS_LAT>,<GPS_LON>,<GPS_SATS>,<ECHO>
```

### Telecommand Set
Supported Ground Control Station (GCS) commands parsed via incoming serial frames:

| Command Structure | Description |
| :--- | :--- |
| `CMD,1094,CX,ON/OFF` | Enable / Disable telemetry transmission stream |
| `CMD,1094,ST,GPS` or `HH:MM:SS` | Synchronize internal mission clock |
| `CMD,1094,SIM,ENABLE/ACTIVATE/DISABLE` | Toggle simulation pressure override mode |
| `CMD,1094,SIMP,<PRESSURE>` | Inject pressure values during simulation testing |
| `CMD,1094,CAL` | Re-calibrate ground pressure and zero altitude |
| `CMD,1094,MEC,REL/ENG` | Probe release mechanism manually trigger |
| `CMD,1094,NOSE,REL/ENG` | Nose release mechanism manually trigger |
| `CMD,1094,EGG,REL/ENG` | Payload release mechanism manually trigger |
| `CMD,1094,MMF,TRUE/FALSE` | Toggle EEPROM Mid-Mission state retention flag |
| `CMD,1094,GCAM,ON/OFF` | Manual override ground camera trigger |
| `CMD,1094,DCAM,ON/OFF` | Manual override descent camera trigger |

---

## Repository Structure

```text
.
├── Metal_Hawk2a.ino     # Main system logic, sensor initialization, state machine, telecommands
├── Navigation.h         # Guidance, coordinate transforms, Bézier generation, & PID algorithms
└── README.md            # Software architecture documentation
```

---

## Build & Dependencies

Built in Arduino IDE / PlatformIO targeting Teensy or RP2040/RP2030 series processors.

### Required Libraries
* `Wire.h` & `SPI.h` (Standard Hardware Busses)
* `EEPROM.h` & `SD.h` (Non-Volatile Storage)
* `Adafruit_BMP5xx.h` (BMP581 Barometer)
* `Adafruit_BNO08x.h` (BNO085 AHRS/IMU)
* `Adafruit_INA260.h` (INA260 Power Monitor)
* `SparkFun_u-blox_GNSS_v3.h` (SAM-M8Q GPS Module)
* `Servo.h` (PWM Hardware Control)
