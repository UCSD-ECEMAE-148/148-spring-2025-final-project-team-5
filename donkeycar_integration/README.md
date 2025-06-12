Here’s your **final `README.md`** for the `optimizer_tumftm` part, fully documenting:

* 📌 Purpose of the optimizer
* ⚙️ Directory structure
* 🧩 How it integrates with DonkeyCar
* 🛠️ Setup instructions (with/without Docker)
* 🚗 How to run optimized path following
* 💡 Customization notes

---

```markdown
# 🏎️ TUM Race Trajectory Optimizer for DonkeyCar Integration

This module integrates the [TUMFTM](https://github.com/TUMFTM/global_racetrajectory_optimization) race trajectory optimizer into a DonkeyCar project. It allows you to record a path using GPS, run an optimization pass on that path, and drive the optimal racing line using the DonkeyCar autopilot.

---

## 📁 Project Structure (Simplified)

```

mycar/
├── manage.py                      # Updated DonkeyCar script with --optimized
├── myconfig.py
└── parts/
└── optimizer\_tumftm/
├── TUMPathOptimizer.py    # DonkeyCar part that wraps optimizer
├── config/
│   └── full\_config.yaml   # Main optimizer config
├── inputs/                # Tracks and metadata (e.g., donkey\_path.csv)
├── outputs/               # Optimized trajectories & plots
├── global\_racetrajectory\_optimization/
├── opt\_mintime\_traj/
├── scripts/
├── src/
└── requirements.txt       # Required packages (used with venv)

````

---

## ⚙️ What This Does

### 1. **Record Path**
Use DonkeyCar in manual mode to record a GPS path:
```bash
python manage.py drive --js
````

### 2. **Optimize Path**

Run:

```bash
python manage.py drive --optimized
```

This:

* Reads `cfg.PATH_FILENAME` (e.g., `donkey_path.csv`)
* Converts and runs the TUM optimizer
* Saves the result to `cfg.OPTIMIZED_PATH_FILENAME`
* Loads the optimized path into memory for autopilot use

---

## 🧩 DonkeyCar Integration: How It Works

Your `manage.py` uses this logic:

```python
from parts.optimizer_tumftm.TUMPathOptimizer import TUMPathOptimizer

if optimized:
    optimizer = TUMPathOptimizer(
        input_path=cfg.PATH_FILENAME,
        output_path=cfg.OPTIMIZED_PATH_FILENAME,
        config_path="parts/optimizer_tumftm/config/full_config.yaml"
    )
    optimizer.run()
    load_path(cfg.OPTIMIZED_PATH_FILENAME)
else:
    load_path(cfg.PATH_FILENAME)
```

---

## 🛠️ Setup Instructions

### ✅ Virtualenv (Recommended)

1. Activate your DonkeyCar environment:

```bash
source ~/envs/donkeycar_env/bin/activate
```

2. Install optimizer dependencies:

```bash
cd parts/optimizer_tumftm
pip install -r requirements.txt
```

3. (Optional) Test basic setup:

```bash
python src/test_quadprog_wrapper.py
```

---

## 🔄 Conversion from `run_optimizer.sh`

We avoid Docker by calling the optimizer’s Python entry point directly:

### `TUMPathOptimizer.py`

```python
import subprocess, os

class TUMPathOptimizer:
    def __init__(self, input_path, output_path, config_path=None, method="mincurv"):
        self.input_path = input_path
        self.output_path = output_path
        self.config_path = config_path or "config/full_config.yaml"
        self.method = method

    def run(self):
        track_name = os.path.splitext(os.path.basename(self.input_path))[0]
        cmd = [
            "python", "src/startup_tum_optimizer.py",
            "--track", track_name,
            "--method", self.method,
            "--config", self.config_path
        ]
        print("[TUM Optimizer] Running:", " ".join(cmd))
        subprocess.run(cmd, check=True)
```

---

## 🧠 Notes & Customization

* You can change the optimization method (`mincurv`, `geometric`, `mintime`, etc.) in `TUMPathOptimizer` or your config.
* Optimized paths will be saved to `outputs/`, such as:

  * `outputs/donkey_converted_track_trajectory_mincurv.csv`
  * `outputs/donkey_converted_track_analysis_mincurv.png`
* The optimizer uses your recorded path as input and creates a smooth, racing-line output optimized for your vehicle dynamics.
* You may tweak `config/full_config.yaml` to set bounds, velocity, or other path constraints.

---

## 📞 Credits

* Optimizer from [TUMFTM](https://github.com/TUMFTM/global_racetrajectory_optimization)
* Integration developed by Aryaman Jadhav as part of autonomous racing research

---

## ✅ To-Do

* [ ] Add full CI test to verify integration with dummy path
* [ ] Improve vehicle-specific dynamic config for Jetson cars
* [ ] Support throttle-aware PID tuning based on optimized curvature

---
