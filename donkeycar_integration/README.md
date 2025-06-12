# README: TUM Race Trajectory Optimizer Integration for DonkeyCar

## Overview
This document outlines how to integrate the TUMFTM-based Race Trajectory Optimizer into the DonkeyCar autonomous driving stack. The optimizer computes optimal racing lines based on centerline input and can be run either standalone or as a part of DonkeyCar's path-following pipeline using a virtual environment.

## Key Features
- Supports multiple optimization methods: `mincurv`, `geometric`, `shortest_path`, `mintime`
- Replaces the default centerline path with a smoothed, optimized trajectory
- Works directly with DonkeyCar paths when integrated via a part

---

## Folder Structure
```
parts/
└── tumftm/
    ├── run_optimizer.sh               # Wrapper (can be modified for venv use)
    ├── src/
    │   ├── startup_tum_optimizer.py   # Main entrypoint
    │   ├── quadprog_wrapper.py        # Replaces quadprog with cvxopt
    │   └── race_trajectory_optimizer.py
    ├── config/full_config.yaml        # Tuning and track details
    ├── inputs/                        # Tracks and dynamic info
    ├── outputs/                       # Optimized CSV and PNG
    └── global_racetrajectory_optimization/  # Cloned from TUM's GitHub
```

---

## 🛠 Setup Instructions

### Requirements
- Python 3.8+
- Virtualenv or conda-based environment
- Packages from `requirements.txt`

### Installing
```bash
# Inside your virtualenv
pip install -r requirements.txt
```

### 🔺 Note
> **Old Jetsons like Nano or TX2 may not support the full optimizer pipeline** due to version incompatibilities in packages like `cvxopt`, `casadi`, or `matplotlib`. We recommend running the optimizer offline (e.g., on your PC) and transferring the optimized CSV to the Jetson for path following or using a newer jetson. 

> 📝 The modified `manage.py` is provided as part of this integration for reference.

---

## 🚗 DonkeyCar Integration

### Inside `manage.py`
```python
if os.path.exists(cfg.PATH_FILENAME):
    if optimized:
        print("[manage.py] Running TUM optimizer on saved path...")
        optimizer = TUMPathOptimizer(
            input_path=cfg.PATH_FILENAME,
            output_path=cfg.OPTIMIZED_PATH_FILENAME
        )
        optimizer.run()
        load_path(cfg.OPTIMIZED_PATH_FILENAME)
    else:
        load_path(cfg.PATH_FILENAME)
```

### Create the Part `TUMPathOptimizer`
```python
# parts/TUM_optimizer_part.py
class TUMPathOptimizer:
    def __init__(self, input_path, output_path):
        self.input_path = input_path
        self.output_path = output_path

    def run(self):
        import subprocess
        cmd = [
            "python3", "parts/tumftm/src/startup_tum_optimizer.py",
            "--track", self.input_path,
            "--config", "parts/tumftm/config/full_config.yaml"
        ]
        print("[TUMPathOptimizer] Running:", " ".join(cmd))
        subprocess.run(cmd, check=True)
```

---

## 🔄 Block Diagram of Data Flow
```
         +------------------+
         |   Record Path    |
         | (CsvThrottlePath)|
         +--------+---------+
                  |
                  v
         +--------+---------+
         | Save Path as CSV |
         +--------+---------+
                  |
        [--optimized flag set?]
             /         \
           Yes         No
          /             \
 +----------------+   +----------------+
 |  TUM Optimizer |   | Load Saved CSV |
 | (TUMPathOptimizer) |                |
 +--------+-------+   +----------------+
          |
          v
 +--------+----------+
 | PathPlot + PIDCtl |
 +-------------------+
```

---

## ✅ Result
- Optimized path used during path-following mode
- Trajectory and visual plots saved in `outputs/`
- Can switch between optimized and manual centerline by toggling the `--optimized` flag

---

## 🚀 Velocity Profile and Throttle Control
Once integrated, the velocity profile from the optimized trajectory can also be used to directly control the throttle of the car. This can result in significantly faster lap times and smoother driving.

However, if throttle becomes variable:
- The PID controller used for steering or speed might behave inconsistently without tuning.

### Proposed Solutions:
1. **Throttle Feedforward**:
   - Use the velocity from the optimizer as a feedforward term in your throttle controller.

2. **Adaptive PID Gains**:
   - Tune PID gains as a function of velocity to maintain stability across throttle changes.

3. **Lookup Tables**:
   - Predefine throttle values for each segment of the trajectory, synced with the optimizer's velocity profile.

4. **Velocity Tracking Controller**:
   - Implement a closed-loop controller that tries to match actual velocity to optimized target velocity (e.g., simple P-controller).

---

## 📎 References
- [TUM Global Trajectory Optimizer](https://github.com/TUMFTM/global_racetrajectory_optimization)
- [DonkeyCar Docs](https://docs.donkeycar.com)
