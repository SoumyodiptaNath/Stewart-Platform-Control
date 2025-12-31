# 🎾 Ball-Plate-SMC-LQR
**Robust Non-linear Stabilization & Trajectory Tracking**

[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A comparative study and implementation of **Sliding Mode Control (SMC)** and **Linear Quadratic Regulator (LQR)** for a 2-DOF Ball and Plate system. This repo features a full physics simulation, controller synthesis, and real-time Matplotlib animations.

### 🧠 Core Methodology
* **System Dynamics:** Modeled using state-space representations for an underactuated ball-on-plate setup with additive disturbances.
* **LQR Controller:** Optimal state-feedback control using $Q$ and $R$ matrix tuning for linearized stability.
* **SMC Controller:** Robust non-linear control utilizing a sliding surface to ensure performance despite model uncertainties and external perturbations.
* **Trajectory Tracking:** Dynamic reference tracking for circular paths using lambda-based state functions.

### ✨ Features
* ✅ **Physics Engine:** Euler-integration based custom simulator.
* ✅ **Comparison Mode:** Switch between LQR and SMC to observe robustness trade-offs.
* ✅ **Visualization:** 3-pane animated dashboard (XY Trajectory, Control Angles, Position Error).

### 🚀 Quick Start
```bash
git clone [https://github.com/SoumyodiptaNath/Ball-Plate-SMC-LQR.git](https://github.com/SoumyodiptaNath/Ball-Plate-SMC-LQR.git)
cd Ball-Plate-SMC-LQR
python main.py
