# Telemetry Data Dictionary

File: `data/ultrasonic_tracking_telemetry.csv`  
Sample Rate: $20\,\text{Hz}$ ($50\,\text{ms}$ loop cycle)  
Hardware: 2× ESP32 microcontrollers, 4× RCWL-1601 ultrasonic transducers

---

## Column Descriptions

| Column Header | Data Type | Units | Description |
|---|---|---|---|
| `timestamp_ms` | Integer | $\text{ms}$ | Microcontroller system uptime clock. Use to compute elapsed time: $t = (t_i - t_0) / 1000$. |
| `Profile` | Integer | Enum | Dynamic tracking profile mode (0 = Fast, 1 = Balanced, 2 = Precise). |
| `LO_cm` | Float | $\text{cm}$ | Left Outer transducer raw measured distance. Mounted at $(10, 0)$ angled at $45^\circ$. |
| `LI_cm` | Float | $\text{cm}$ | Left Inner transducer raw measured distance. Mounted at $(20, 0)$ angled at $65^\circ$. |
| `RI_cm` | Float | $\text{cm}$ | Right Inner transducer raw measured distance. Mounted at $(130, 0)$ angled at $115^\circ$. |
| `RO_cm` | Float | $\text{cm}$ | Right Outer transducer raw measured distance. Mounted at $(140, 0)$ angled at $135^\circ$. |
| `JitterLO_cm` | Float | $\text{cm}$ | Running standard deviation / jitter magnitude on Left Outer channel. Threshold is $15\,\text{cm}$. |
| `JitterLI_cm` | Float | $\text{cm}$ | Running jitter magnitude on Left Inner channel. |
| `JitterRI_cm` | Float | $\text{cm}$ | Running jitter magnitude on Right Inner channel. |
| `JitterRO_cm` | Float | $\text{cm}$ | Running jitter magnitude on Right Outer channel. |
| `RawX_cm` | Float | $\text{cm}$ | Unfiltered lateral position computed directly by the multilateration solver before Kalman filtering. |
| `RawY_cm` | Float | $\text{cm}$ | Unfiltered depth position computed directly by the multilateration solver before Kalman filtering. |
| `X_cm` | Float | $\text{cm}$ | Filtered lateral position output by the Kalman filter (sent to game over ESP-NOW). |
| `Y_cm` | Float | $\text{cm}$ | Filtered depth position output by the Kalman filter (sent to game over ESP-NOW). |
| `Vx_cm_s` | Float | $\text{cm/s}$ | Filtered lateral velocity estimate. |
| `Vy_cm_s` | Float | $\text{cm/s}$ | Filtered depth velocity estimate. |
| `Speed_cm_s` | Float | $\text{cm/s}$ | Instantaneous speed magnitude: $\sqrt{V_x^2 + V_y^2}$. |
| `NormX` | Float | $[0.0, 1.0]$ | Normalized playing area X coordinate mapped to screen resolution. |
| `NormY` | Float | $[0.0, 1.0]$ | Normalized playing area Y coordinate mapped to screen resolution. |
| `Confidence` | Float | $[0.0, 1.0]$ | Tracking confidence score based on sensor consensus. |
| `Detected` | Boolean | $\{0, 1\}$ | Flag indicating whether an active player target was detected within the valid cone. |
| `InDeadZone` | Boolean | $\{0, 1\}$ | Flag indicating whether the player penetrated the $60\,\text{cm}$ safety dead zone in front of the screen. |

---

## Key Data Insights for Teammate Reports
1. **Raw vs Filtered Plotting:** Plot `RawX_cm` vs `X_cm` over `timestamp_ms` to demonstrate how the V2 filter cleans up severe multipath spikes without adding noticeable phase lag.
2. **Channel Jitter Plotting:** Plot `JitterLO_cm`, `JitterLI_cm`, `JitterRI_cm`, and `JitterRO_cm` in separate subplots against the $15\,\text{cm}$ threshold to show sensor stability and anomaly detection.
3. **Dead Zone Compliance:** Point out the `InDeadZone` flag triggering when $Y < 60\,\text{cm}$, proving compliance with safety constraints.
