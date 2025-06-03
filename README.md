# Race Trajectory Optimizer: Project Documentation

## Overview
This document outlines the configuration, setup, usage, and output structure for the Race Trajectory Optimizer, adapted for both large-scale tracks and miniature RC cars (like the Traxxas Ford Fiesta ST 1/18 platform).

---

## 1. Setup Summary
### 🔧 Issues & Fixes
- **Initial Issue**: `quadprog` was unavailable in the original TUMFTM dependency chain.
- **Fix**: Created a `quadprog_wrapper.py` that internally uses `cvxopt` to emulate the same optimization functionality. This wrapper is loaded dynamically via:
  ```python
  import sys
  sys.path.insert(0, '/app/src')
  import quadprog_wrapper
  sys.modules['quadprog'] = quadprog_wrapper
  ```

### 🛠 Starting from Scratch
1. **Build the Docker Image**
   ```bash
   docker build -f Dockerfile.tumftm -t race-trajectory-optimizer-tumftm .
   ```

2. **Run the Docker Container**
   ```bash
   docker run --rm -it -v "$PWD":/app race-trajectory-optimizer-tumftm bash
   ```

3. **Convert Donkey Path (if needed)**
   ```bash
   python3 scripts/convert_donkey_path.py
   ```
   This generates `donkey_converted_track.csv` in the `inputs/tracks/` directory.

4. **Update Configuration File**
   Ensure `full_config.yaml` specifies:
   ```yaml
   track:
     name: donkey_converted_track
   ```

5. **Run the Optimizer**
   ```bash
   ./optimize_track.sh
   ```

---

## 2. Directory Structure
```
race_trajectory_optimizer/
├── Dockerfile / Dockerfile.tumftm
├── config/
│   ├── full_config.yaml
│   └── optimizer_config.yaml
├── inputs/
│   └── tracks/
│       ├── berlin_2018.csv
│       └── donkey_converted_track.csv
├── outputs/
├── src/
├── scripts/
├── run_final_optimizer.sh
├── optimize_track.sh
└── README.md
```

---

## 3. Configuration Files
### `full_config.yaml` (Primary Used)
Defines vehicle, track, optimization, and visualization parameters.

#### 🔹 Vehicle Parameters
```yaml
vehicle:
  width: 0.285
  max_velocity: 9.0
  min_velocity: 2.0
  max_lat_acc: 7.0
  max_long_acc: 5.0
  max_long_dec: -5.0
  mass: 1.5
  length: 0.4
```

#### 🔹 Optimization Method
```yaml
optimization:
  method: geometric
  velocity_profile:
    method: curvature_based
    safety_factor: 0.9
    smoothing: 10
```

#### 🔹 Track Settings
```yaml
track:
  name: donkey_converted_track
  import_options:
    flip_imp_track: false
    min_track_width: 1.5
    num_laps: 1
```

---

## 4. Input Format
### 📥 Converted Donkey Path Format (Centerline with Uniform Widths)
```csv
# x_m,y_m,w_tr_right_m,w_tr_left_m
0.0, 0.0, 0.75, 0.75
1.0, 0.5, 0.75, 0.75
...
```
This input is generated from the original DonkeyCar `x,y,throttle` CSV using `scripts/convert_donkey_path.py`.

---

## 5. Output Format
### 📤 Trajectory CSV Output
Each trajectory is saved under:
```
outputs/<track_name>_trajectory.csv
```
With the following format:
```csv
x_m,y_m,psi_rad,vx_mps,ax_mps2,ay_mps2,kappa_radpm
```
Where:
- `x_m, y_m`: Global coordinates
- `psi_rad`: Heading angle
- `vx_mps`: Velocity
- `ax_mps2`: Longitudinal acceleration
- `ay_mps2`: Lateral acceleration
- `kappa_radpm`: Curvature

---

## 6. Visualizations
Generated PNGs include:
- Track boundaries
- Velocity-colored racing line
- Velocity profile over distance

Saved to:
```
outputs/<track_name>_analysis.png
```

---

## 7. Running the Optimizer
### ✅ Inside Docker:
```bash
./optimize_track.sh
```

### ✅ Manual CLI:
```bash
python3 src/race_trajectory_optimizer.py --track donkey_converted_track
```

---

## 8. DonkeyCar Compatibility
The optimizer supports converting DonkeyCar paths into optimizable formats. Once optimized, the output can be exported back for vehicle following.

---

## 9. Conclusion
The Race Trajectory Optimizer is now capable of:
- Running geometric-based optimization
- Generating curvature-based velocity profiles
- Exporting fully formatted trajectories and visualizations
- Supporting miniature RC cars and standard tracks

**Next steps**: Integrate the generated optimized trajectories into DonkeyCar's path-following logic.
