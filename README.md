# Race Trajectory Optimizer

A Python-based trajectory optimizer for full-size race tracks and 1/18 RC platforms (e.g. DonkeyCar, Traxxas Ford Fiesta ST).
Supports multiple optimization methods, CVXOPT-backed `quadprog`, and seamless DonkeyCar path conversion.

---

## 🚀 Features

* **Multiple methods**:

  * `geometric`
  * `mincurv`
  * `mincurv_iqp`
  * `mintime`
  * `shortest_path`

* **Quadprog wrapper**: Drop-in replacement of MATLAB’s `quadprog` using CVXOPT.

* **Config-driven**: Fully separate vehicle, track, optimization, and visualization parameters in YAML.

* **CLI interface**: Flexible flags for track, method, config file, output directory, and debug.

* **DonkeyCar compatibility**:

  * Converts raw DonkeyCar `x,y,throttle` logs to centerline+width CSV
  * Repeats last throttle value if input ends early
  * Exports optimized trajectory back into DonkeyCar pipeline

---

## 📦 Installation

### 1. Clone the repository

```bash
git clone https://github.com/UCSD-ECEMAE-148/148-spring-2025-final-project-team-5.git
cd 148-spring-2025-final-project-team-5
```

### 2. Install Python dependencies

```bash
pip install -r requirements.txt
```

### 3. (Recommended) Docker

```bash
docker build -f Dockerfile.tumftm -t race-trajectory-optimizer .
docker run --rm -it -v "$PWD":/app race-trajectory-optimizer bash
```

---

## 🛠️ Configuration

Edit `config/full_config.yaml` to define your scenario:

```yaml
vehicle:
  width: 0.285
  mass: 1.5
  length: 0.4
  max_velocity: 9.0
  min_velocity: 2.0
  max_lat_acc: 7.0
  max_long_acc: 5.0
  max_long_dec: -5.0

optimization:
  method: geometric            # geometric|mincurv|mincurv_iqp|mintime|shortest_path
  velocity_profile:
    method: curvature_based
    safety_factor: 0.9
    smoothing: 10

track:
  name: donkey_converted_track
  import_options:
    flip_imp_track: false
    min_track_width: 1.5
    num_laps: 1

visualization:
  save_png: true
  show_plots: false
```

---

## 📂 Directory Structure

```
race_trajectory_optimizer/
├── config/
│   ├── full_config.yaml
│   └── optimizer_config.yaml
├── Dockerfile
├── Dockerfile.tumftm
├── requirements.txt
├── inputs/
│   └── tracks/
│       ├── berlin_2018.csv
│       └── donkey_converted_track.csv
├── outputs/
├── scripts/
│   └── convert_donkey_path.py
├── src/
│   ├── race_trajectory_optimizer.py   # CLI entrypoint
│   └── opt_mintime_traj/              # TUMFTM port
├── optimize_track.sh
└── README.md
```

---

## 📥 Input Formats

### 1. Full-size track CSV

```csv
# x_m,y_m,w_tr_right_m,w_tr_left_m
0.0,0.0,3.5,3.5
1.0,0.5,3.5,3.5
...
```

### 2. DonkeyCar path CSV

```csv
# x_m,y_m,w_tr_right_m,w_tr_left_m,throttle
0.0,0.0,0.75,0.75,0.50
1.0,0.5,0.75,0.75,0.50
...
```

*Throttles are repeated if the raw DonkeyCar log ends early.*

Convert raw DonkeyCar logs:

```bash
python3 scripts/convert_donkey_path.py \
  --input inputs/tracks/raw_donkey.csv \
  --output inputs/tracks/donkey_converted_track.csv
```

---

## 📤 Output Formats

### 1. Trajectory CSV

```
outputs/<track_name>_trajectory_<method>.csv
```

Columns:

```
x_m, y_m, psi_rad, vx_mps, ax_mps2, ay_mps2, kappa_radpm
```

### 2. Analysis PNG

```
outputs/<track_name>_analysis_<method>.png
```

Contains:

* Track boundaries & optimized racing line
* Velocity profile heatmap
* Curvature & centrifugal accel plots

### 3. DonkeyCar-compatible Path CSV

```
outputs/donkey_optimized_path_<w_tr_right>_<w_tr_left>.csv
```

Columns:

```
x_m, y_m, throttle
```

> The last throttle value is repeated if the DonkeyCar log ends early, and `<w_tr_right>`/`<w_tr_left>` correspond to the half-widths set in your config’s `import_options`.

---

## ⚙️ Usage

### A. Shell script (default config)

```bash
./optimize_track.sh
```

### B. CLI interface

```bash
python3 src/race_trajectory_optimizer.py \
  --track berlin_2018 \
  --method mintime \
  --config config/full_config.yaml \
  --output-dir outputs/ \
  --debug
```

**Flags**

* `--track`       : Track identifier (filename in `inputs/tracks/`)
* `--method`      : `geometric` | `mincurv` | `mincurv_iqp` | `mintime` | `shortest_path`
* `--config`      : Path to YAML config
* `--output-dir`  : Directory for CSV/PNGs
* `--debug`       : Enable verbose logging

---

## 🤝 DonkeyCar Integration

1. **Convert** raw DonkeyCar logs
2. **Optimize** trajectory as above
3. **Export** the generated
   `donkey_optimized_path_<w_tr_right>_<w_tr_left>.csv`
4. **Feed** into DonkeyCar controller for path-following

---

## 🔮 Next Steps

* Integration into DonkeyCar (`donkeycar_integration/` directory)
* Real-time throttle scheduler using optimized velocity profiles

---

## 📄 License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.
