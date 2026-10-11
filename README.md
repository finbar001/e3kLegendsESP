# Ultrasonic Spatial Tracking Subassembly Kit

Standalone engineering package containing core firmware, empirical telemetry logs, mathematical models, and benchmark verification tools for the ultrasonic spatial tracking subassembly.

---

## Repository Structure

```
team_handover_pack/
├── README.md                           <- Technical overview and usage
├── firmware_core/                      <- Active operational firmware
│   ├── engg3000esp.ino                 <- Main Arduino entry point and 20 Hz loop
│   ├── config.h                        <- Field geometry, pinouts, and physical constants
│   └── comms.h / comms.cpp             <- ESP-NOW wireless telemetry
├── v1_baseline/                        <- Sprint 1 MVP baseline code (Commit 6492807)
│   ├── tracking.h / tracking.cpp       <- V1 unweighted solver and adaptive R implementation
│   └── sensors.h / sensors.cpp         <- V1 raw timing routines
├── v2_optimized/                       <- Sprint 2 Optimized prototype (Commit 2e81e80)
│   ├── tracking.h / tracking.cpp       <- V2 Levenberg-Marquardt solver, Huber R, dynamic damping
│   └── sensors.h / sensors.cpp         <- V2 median filtering and spike gating
├── data/                               <- Empirical testing data and metrics
│   ├── benchmark_metrics.csv           <- Quantitative comparison table with formulas
│   ├── ultrasonic_tracking_telemetry.csv <- Raw 20 Hz hardware log (479 frames)
│   └── telemetry_data_dictionary.md    <- Column definitions and units
├── docs/                               <- Technical documentation and formulations
│   ├── V1_vs_V2_TECHNICAL_COMPARISON.md<- Side-by-side architectural changes
│   └── FORMULAS_AND_MATHEMATICAL_MODELS.md <- Mathematical models and FLOP derivations
├── figures/                            <- Publication plots (300 DPI)
│   ├── fig1_simulation_trajectory.png  <- 2D trajectory and error profiles
│   └── fig2_empirical_telemetry.png    <- Raw vs filtered position and channel jitter
└── tools/
    └── run_benchmarks_and_plots.py     <- Python benchmark and plot generation script
```

---

## Verification & Figure Generation

Execute the benchmark runner to reproduce the publication figures and print verified performance metrics:

```bash
python3 tools/run_benchmarks_and_plots.py
```

Outputs:
- `figures/fig1_simulation_trajectory.png`: 2D field trajectory and dynamic error profile comparing Ground Truth, V1, and V2.
- `figures/fig2_empirical_telemetry.png`: Empirical ESP32 telemetry comparing raw multilateration spikes against filtered output, alongside 4 independent channel jitter lanes (`LO`, `LI`, `RI`, `RO`).

---

## Performance Benchmark Metrics

Empirical and simulated benchmarks comparing Prototype V1 (Commit `6492807`) against Prototype V2 (Commit `2e81e80`):

| Metric | V1 Baseline | V2 Subsystem | Quantified Improvement | Engineering Impact |
|---|---|---|---|---|
| Tracking RMSE | $8.29\,\text{cm}$ | $3.19\,\text{cm}$ | $61.5\%$ Error Reduction | High-accuracy spatial cursor positioning |
| Peak Error Shock | $63.89\,\text{cm}$ | $7.38\,\text{cm}$ | $88.4\%$ Spike Suppression | Suppresses multipath acoustic wall bounce spikes |
| Stationary Aiming Jitter ($\sigma$) | $5.40\,\text{cm}$ | $0.80\,\text{cm}$ | $85.2\%$ Jitter Reduction | Rock-solid cursor hold over active target holes |
| Multipath Spike Rejection | $0\%$ | $100\%$ | $100\%$ Outlier Rejection | Zero cursor teleportation during tracking |
| Firmware Execution Latency | $45.8\,\mu\text{s}$ | $21.4\,\mu\text{s}$ | $53.3\%$ Faster Execution | Reduced compute footprint on ESP32 |
| Frame CPU Budget ($50\,\text{ms}$ loop) | $0.09\%$ | $0.04\%$ | $55.6\%$ Lower CPU Load | $< 0.05\%$ duty cycle leaves bandwidth for ESP-NOW radio |
| User Setup Overhead | $15.0\,\text{min}$ | $0.0\,\text{min}$ | $100\%$ Eliminated | Fixed geometric origin allows plug-and-play setup |

Detailed mathematical derivations and complexity breakdowns are available in `docs/FORMULAS_AND_MATHEMATICAL_MODELS.md`.  
Full architectural and code comparisons are documented in `docs/V1_vs_V2_TECHNICAL_COMPARISON.md`.
