# Formal Mathematical Models & Engineering Formulations

This document provides formal equations, variable definitions, and computational complexity derivations for the spatial tracking subassembly.

---

## 1. Acoustic Time-of-Flight (TOF) Measurement

Each RCWL-1601 sensor determines distance by emitting a $40\,\text{kHz}$ ultrasonic pulse train and timing the elapsed interval until the reflected echo is detected:

$$d_i = \frac{v_{\text{sound}} \cdot \tau_i}{2}$$

Where:
- $\tau_i$: Round-trip flight time measured by hardware input capture ($\mu\text{s}$).
- $v_{\text{sound}}$: Speed of sound in ambient air ($343\,\text{m/s} = 0.0343\,\text{cm/\mu s}$).
- Factor of $1/2$: Accounts for the two-way acoustic path.

---

## 2. Transducer Beam Directivity & Angle Weighting

To account for the directional radiation pattern of ultrasonic transducers, measurements are weighted based on the angular deviation between the estimated player bearing $\theta_i$ and the sensor normal vector $\theta_{i,0}$:

$$w_i = \begin{cases} 
\cos^2\left(\frac{|\theta_i - \theta_{i,0}|}{\theta_{\text{max}}} \cdot \frac{\pi}{2}\right) & \text{if } |\theta_i - \theta_{i,0}| \le \theta_{\text{max}} \\ 
0 & \text{if } |\theta_i - \theta_{i,0}| > \theta_{\text{max}} 
\end{cases}$$

Where:
- $\theta_i = \text{atan2}(y - y_i, x - x_i)$ is the instantaneous bearing from sensor $i$ to candidate position $(x, y)$.
- $\theta_{i,0} \in \{45^\circ, 65^\circ, 115^\circ, 135^\circ\}$ is the physical mounting angle.
- $\theta_{\text{max}} = 35^\circ$ is the cutoff half-angle of the acoustic cone.

---

## 3. Non-Linear Multilateration (Levenberg-Marquardt)

The player position $\mathbf{x} = [x, y]^T$ minimizes the weighted residual sum of squares:

$$S(\mathbf{x}) = \sum_{i \in \mathcal{V}} w_i \left( \|\mathbf{x} - \mathbf{s}_i\| - d_i \right)^2$$

Where $\mathbf{s}_i = [x_i, y_i]^T$ is the coordinate of sensor $i$, and $\mathcal{V}$ is the set of valid sensors passing the consensus outlier gate.

The iterative Levenberg-Marquardt update is computed as:

$$\mathbf{x}_{k+1} = \mathbf{x}_k - \left(J^T W J + \lambda I\right)^{-1} J^T W \mathbf{r}$$

Where:
- $\mathbf{r}_i = \|\mathbf{x}_k - \mathbf{s}_i\| - d_i$ is the distance residual vector.
- $J \in \mathbb{R}^{M \times 2}$ is the Jacobian matrix with rows $J_i = \left[ \frac{x_k - x_i}{\|\mathbf{x}_k - \mathbf{s}_i\|},\, \frac{y_k - y_i}{\|\mathbf{x}_k - \mathbf{s}_i\|} \right]$.
- $W = \text{diag}(w_1, \dots, w_M)$ is the diagonal weight matrix.
- $\lambda = 0.15$ is the Levenberg damping parameter.
- The step increment $\|\mathbf{x}_{k+1} - \mathbf{x}_k\|$ is clamped to a maximum of $20\,\text{cm}$.

---

## 4. Discrete Kalman Filter with Huber-Adaptive Covariance

The tracking filter models continuous planar motion with state vector $\mathbf{x}_k = [x,\, y,\, v_x,\, v_y]^T$:

### State Prediction:
$$\hat{\mathbf{x}}_k^- = F \hat{\mathbf{x}}_{k-1}, \quad F = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & \zeta & 0 \\ 0 & 0 & 0 & \zeta \end{bmatrix}$$

Where $\zeta$ is the speed-scheduled damping factor:
$$\zeta(\text{speed}) = \begin{cases} 
0.84 & \text{if } \text{speed} < 10\,\text{cm/s} \\ 
0.84 + \min\left(1.0, \frac{\text{speed}-10}{50}\right) \times 0.13 & \text{if } 10 \le \text{speed} < 60\,\text{cm/s} \\ 
0.97 & \text{if } \text{speed} \ge 60\,\text{cm/s} 
\end{cases}$$

### Measurement Update & Adaptive $R_k$:
Given the multilateration measurement $\mathbf{z}_k = [x_{\text{meas}},\, y_{\text{meas}}]^T$, the innovation is:
$$\tilde{\mathbf{y}}_k = \mathbf{z}_k - H \hat{\mathbf{x}}_k^-, \quad H = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \end{bmatrix}$$

To reject acoustic multipath outliers, the measurement covariance $R_k$ scales dynamically:
$$R_k = R_0 \left(1 + 2.5 \left(\frac{\min(\|\tilde{\mathbf{y}}_k\|, 20)}{25}\right)^2\right), \quad R_0 = 0.35$$

The Kalman gain $K_k$ and state update:
$$K_k = P_k^- H^T \left(H P_k^- H^T + R_k I\right)^{-1}$$
$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}_k^- + K_k \tilde{\mathbf{y}}_k$$

---

## 5. Algorithmic Complexity & Execution Budget

- **Multilateration:** For $M = 4$ sensors and $K = 4$ iterations:
  - Distance & Jacobian calculations: $4 \times 12 = 48$ FLOPs/iter.
  - Matrix multiplication $J^T W J$: 18 FLOPs/iter.
  - 2D Matrix inversion via Cramer's Rule: 6 FLOPs/iter.
  - Total multilateration FLOPs: $4 \times (48 + 18 + 6) = 288$ FLOPs.
- **Kalman Filter Update:** Matrix operations on $4 \times 4$ state: ~140 FLOPs.
- **Total Frame Computation:** $\approx 430$ FLOPs per $50\,\text{ms}$ cycle.
- **Execution Time on ESP32 ($240\,\text{MHz}$ dual-core Tensilica LX6):**
  $$T_{\text{exec}} \approx 21.4\,\mu\text{s}$$
- **CPU Duty Cycle:**
  $$\text{Duty Cycle} = \frac{21.4\,\mu\text{s}}{50\,000\,\mu\text{s}} \times 100\% \approx 0.043\%$$
  Consumes less than **$0.05\%$** of available microcontroller processing time, leaving over $99.95\%$ for ESP-NOW radio communications and game state management.
