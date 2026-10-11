# Subsystem Technical Comparison: V1 Baseline vs V2 Optimized

This document details the exact architectural, mathematical, and algorithmic differences between **Prototype V1 (Sprint 1 MVP)** and **Prototype V2 (Sprint 2 Subsystem Optimization)** for the Ultrasonic Spatial Tracking Subassembly.

---

## High-Level Summary of Subsystem Upgrades

| Subsystem Component | V1 Baseline (Sprint 1 MVP) | V2 Optimized (Sprint 2 Prototype) | Engineering Impact |
|---|---|---|---|
| **Outlier Consensus Pruning** | None (All 4 sensor readings used indiscriminately) | 2-pass consensus check; discards worst sensor if residual $> 18\,\text{cm}$ | Rejects multipath bounce echoes off side walls |
| **Transducer Beam Weighting** | Flat/coarse angle weights; sensors outside beam cone given non-zero weight ($w = 0.15$) | Strict $\pm 35^\circ$ beam cone cutoff ($w = 0$ outside); quadratic cosine taper within beam | Eliminates corrupted peripheral readings dragging multilateration |
| **Multilateration Solver** | Unbounded Levenberg-Marquardt with cold initialization; damping $\lambda = 0.10$ | Warm-started from Kalman prediction; damping $\lambda = 0.15$; step clamped to $\pm 20\,\text{cm}$ | Guarantees numerical convergence in $\le 4$ iterations without matrix singularity |
| **Kalman Filter Covariance** | Inverted adaptive measurement noise ($R$ dropped on spikes, locking filter onto errors) | Robust Huber-adaptive covariance ($R$ scales up quadratically on spikes); $20\,\text{cm}$ innovation clamp | Prevents cursor teleportation when an acoustic echo glitches |
| **Velocity Damping ($\zeta$)** | Fixed damping ($\zeta = 0.92$) applied uniformly | Speed-scheduled damping ($\zeta = 0.84$ at rest, $\zeta = 0.97$ while moving) | Eliminates stationary jitter while maintaining fluid player momentum |
| **Aim Assist / Target Guidance** | Unconditional quadratic pull towards active mole holes | Directional release factor ($\vec{v} \cdot \vec{r}_{\text{mole}} < 0$) + displacement clamped to $\le 1.2\,\text{cm/frame}$ | Provides gentle capture assistance without resisting player lunges |
| **System Calibration Overhead** | Complex manual calibration routines required after physical movement | Zero manual calibration; pre-configured fixed geometric origin | True plug-and-play field deployment |

---

## Detailed Component Breakdowns

### 1. Transducer Beam Geometry & Outlier Consensus
- **V1 Problem (`v1_baseline/tracking.cpp`):**
  When a player lunged towards the side boundaries, ultrasonic waves bounced off nearby side walls and enclosure edges, causing phantom long-distance readings. V1 still assigned a $15\%$ weight to sensors facing away from the player, causing the multilateration solver to pull the player's position toward the wall.
- **V2 Solution (`v2_optimized/tracking.cpp`):**
  1. *Hard Beam Cutoff:* If deviation angle $|\Delta \theta_i| > 35^\circ$, weight is set strictly to $w_i = 0$.
  2. *Cosine Taper:* Inside the $35^\circ$ cone, weight follows $w_i = \cos^2\left(\frac{\Delta \theta_i}{35^\circ} \cdot \frac{\pi}{2}\right)$.
  3. *Consensus Pruning:* Before solving, each sensor's predicted distance from the previous Kalman state is compared to the measured distance. If the worst sensor has an error $> 18\,\text{cm}$, it is marked invalid and dropped from that frame's calculation.

---

### 2. Multilateration Solver Optimization
- **V1 Problem (`v1_baseline/tracking.cpp`):**
  The solver used a standard non-linear least squares approach with a fixed cold-start guess. Under noisy conditions, the Jacobian matrix $J^T J$ became ill-conditioned, producing step sizes $> 100\,\text{cm}$ that sent the tracker out of bounds.
- **V2 Solution (`v2_optimized/tracking.cpp`):**
  - **Warm Initialization:** The solver initializes its estimate $(\hat{x}_0, \hat{y}_0)$ directly from the Kalman filter's predicted position.
  - **Regularization (Levenberg-Marquardt):** A damping factor of $\lambda = 0.15$ is added along the diagonal: $(J^T W J + 0.15 I)$.
  - **Step Clamping:** Each iterative update step is clamped to $\pm 20\,\text{cm}$, preventing divergence even if multiple sensors experience transient dropouts.

---

### 3. Kalman Filter & Huber-Adaptive Measurement Covariance
- **V1 Problem (`v1_baseline/tracking.cpp`):**
  The V1 adaptive filter had an inverse noise equation:
  ```cpp
  // V1 FLAW: As error increased, R decreased!
  _adaptiveR = baseR / (1.0f + gain * innovation);
  ```
  When a large multipath spike occurred, $R$ dropped close to zero, causing the Kalman filter to treat the corrupted measurement as ground truth and jump violently across the screen.
- **V2 Solution (`v2_optimized/tracking.cpp`):**
  - Corrected to a **Huber-adaptive covariance model**:
    ```cpp
    // V2 FIX: Large innovations scale R UP, telling the filter to ignore the spike
    float normInv = min(3.0f, innovationMag / 25.0f);
    _adaptiveR = min(8.0f, baseR * (1.0f + 2.5f * normInv * normInv));
    ```
  - Clamped maximum innovation update to $20\,\text{cm}$ per frame. On an echo spike, the filter holds the previous trajectory and ignores the anomaly.

---

### 4. Dynamic Velocity Damping
- **V1 Problem:**
  Using a fixed damping ratio ($\zeta = 0.92$) forced an engineering compromise: high damping created severe cursor lag during fast body movements, while low damping produced visible cursor jitter when the player was stationary.
- **V2 Solution:**
  Implemented **speed-scheduled damping**:
  - At rest ($\text{speed} < 10\,\text{cm/s}$): $\zeta = 0.84$ (heavy damping suppresses hand/body jitter).
  - During rapid lunges ($\text{speed} \ge 60\,\text{cm/s}$): $\zeta = 0.97$ (low damping preserves momentum and gives instant responsiveness).

---

### 5. Target Guidance (Aim Assist) with Directional Release
- **V1 Problem:**
  Aim assist applied an unconditional gravitational pull toward active mole targets. When a player tried to lunge away to hit a different mole, the aim assist created a "sticky" drag force that resisted their movement.
- **V2 Solution:**
  Added a directional dot-product check:
  $$\text{alignment} = \frac{\vec{v} \cdot \vec{r}_{\text{mole}}}{\|\vec{v}\| \|\vec{r}_{\text{mole}}\|}$$
  If the player's velocity is moving away from the mole ($\text{alignment} < 0$), the assistance force is dynamically cut to zero ($\text{rel} \rightarrow 0$), giving full control back to the player.
