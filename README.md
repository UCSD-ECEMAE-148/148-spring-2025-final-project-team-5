# Race Trajectory Optimizer

This repository contains a custom setup of the TUMFTM Race Trajectory Optimizer, tailored for use with small-scale autonomous vehicles like the Traxxas Ford Fiesta ST RC car. The optimizer computes an optimal racing line and velocity profile from a track centerline.

---

## 🚧 Setup Summary

### ✅ Dockerized Build

* Built with a `Dockerfile` that replicates exact dependency versions for reproducibility.
* Mounted volumes for persistent `inputs/` and `outputs/`.

### ❌ Original Issue

* **`quadprog` dependency failed** due to compilation errors on newer Linux environments.

### ✅ Fix

* Replaced `quadprog` with a **custom wrapper using `cvxopt`**, maintaining compatibility with original interfaces.
* Patched all downstream modules to call the wrapper seamlessly.

---

## ⚙️ Configuration Files

### `config/full_config.yaml`

Comprehensive config covering:

* **Track**: width, new start, flipping, etc.
* **Vehicle**: mass, dimensions, acceleration limits, max velocity.
* **Optimization**: method (`geometric`), velocity profile (`curvature_based`), smoothing params.
* **Output**: enables trajectory + statistics saving, visualization PNGs.
* **Visualization**: plot styling and annotations (e.g., arrows, braking zones).

### `config/optimizer_config.yaml`

Used by `main_globaltraj.py` or other modules:

* GGV diagram config
* Discretization points
* Cost weights (time, smoothness, tire usage)
* IPOPT/SQP solver parameters

---

## 🚗 Vehicle Model - Traxxas Ford Fiesta ST

Based on:

* [Official Traxxas specs](https://traxxas.com/74154-4-ford-fiesta-st-bl-2s?t=support)
* \[MPC student paper]\(attached PDF)

Key parameters:

```
mass: ~1.4 kg
width: ~0.3 m
length: ~0.5 m
max_velocity: 7–10 m/s (realistically tested)
friction_coeff: 1.2 (grippy surface)
```

These were converted and approximated in `full_config.yaml`.

---

## 🏁 Inputs

* `inputs/tracks/*.csv` → centerline or boundary-based tracks
* `inputs/veh_dyn_info/*.csv` → GGV diagrams or acceleration limits
* `config/*.yaml` → all config variants

---

## 📤 Outputs

* CSV trajectory with x/y/speed
* PNG plots (`racing_line_analysis.png`)
* JSON stats: lap time, max speed, etc.
* Optional `donkey_path.csv` for DonkeyCar integration

---

## 🚀 Running the Optimizer

```bash
./optimize_track.sh  # runs the geometric optimizer with full_config.yaml
```

This loads the input track, computes the optimized trajectory, saves output visuals and data.

---

## 🔐 GitHub Deployment

### Issue: `git push` failed due to removed password support

✅ Fixed by setting up SSH authentication:

```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
# Add ~/.ssh/id_ed25519.pub to GitHub SSH keys
```

Then update origin remote:

```bash
git remote set-url origin git@github.com:Aryamanj26/race-trajectory-optimizer.git
git push -u origin main
```

---

## 🧩 Future Extensions

* Integrate real-time controller node
* Enable DonkeyCar GPS/path integration
* Support nonlinear tire models
* Add friction maps for varying surfaces

---

For any issues, please open a GitHub Issue or contact Aryaman Jadhav.
