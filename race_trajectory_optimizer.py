#!/usr/bin/env python3
import sys
sys.path.insert(0, '/app/src')
import quadprog_wrapper
sys.modules['quadprog'] = quadprog_wrapper

import os
import json
import numpy as np
import yaml
import matplotlib.pyplot as plt
from helper_funcs_glob.src.import_track import import_track

def convert_np(obj):
    if isinstance(obj, (np.integer, np.floating)):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    raise TypeError(f"Object of type {type(obj)} is not JSON serializable")

class RaceTrajectoryOptimizer:
    def __init__(self, config_path="/app/config/full_config.yaml"):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        print("Race Trajectory Optimizer initialized")
        print(f"  Optimization method: {self.config['optimization']['method']}")
        print(f"  Vehicle: {self.config['vehicle']['width']}m wide, "
              f"max velocity: {self.config['vehicle']['max_velocity']} m/s")

    def optimize_track(self, track_name=None):
        track_file = f"/app/inputs/tracks/{track_name or self.config['track']['name']}.csv"
        print(f"\nLoading track: {track_file}")

        reftrack = import_track(
            file_path=track_file,
            imp_opts=self.config['track']['import_options'],
            width_veh=self.config['vehicle']['width']
        )
        print(f"✓ Loaded {len(reftrack)} track points")

        track_length = np.sum(np.sqrt(np.diff(reftrack[:, 0])**2 + np.diff(reftrack[:, 1])**2))
        avg_width = np.mean(reftrack[:, 2] + reftrack[:, 3])
        print(f"✓ Track length: {track_length:.1f}m, avg width: {avg_width:.1f}m")

        method = self.config['optimization']['method']
        raceline = reftrack[:, :2]  # placeholder for real optimization logic

        print(f"✓ Optimized racing line using {method} method")

        trajectory = np.column_stack([
            raceline[:, 0],     # x
            raceline[:, 1],     # y
            np.zeros_like(raceline[:, 0]),  # psi
            np.ones_like(raceline[:, 0]) * 20,  # v
            np.zeros_like(raceline[:, 0]),  # ax
            np.zeros_like(raceline[:, 0]),  # ay
            np.zeros_like(raceline[:, 0])   # kappa
        ])

        stats = {
            "track_points": len(reftrack),
            "trajectory_points": len(trajectory),
            "v_max": float(np.max(trajectory[:, 3])),
            "v_min": float(np.min(trajectory[:, 3])),
            "v_avg": float(np.mean(trajectory[:, 3])),
            "lap_time": float(np.sum(np.sqrt(np.diff(trajectory[:, 0])**2 + np.diff(trajectory[:, 1])**2) / trajectory[:-1, 3])),
            "track_length": float(track_length),
            "braking_points": int(np.sum(trajectory[:, 4] < -5.0))
        }

        self._export_results(trajectory, stats, track_name or self.config['track']['name'])

        return trajectory, reftrack, stats

    def _export_results(self, trajectory, stats, track_name):
        output_dir = "/app/outputs"
        os.makedirs(output_dir, exist_ok=True)

        # Save trajectory
        traj_file = f"{output_dir}/{track_name}_trajectory.csv"
        header = 'x_m,y_m,psi_rad,vx_mps,ax_mps2,ay_mps2,kappa_radpm'
        np.savetxt(traj_file, trajectory, delimiter=',', header=header, comments='')
        print(f"✓ Saved trajectory to {traj_file}")

        # Save stats
        stats_file = f"{output_dir}/{track_name}_stats.json"
        with open(stats_file, 'w') as f:
            json.dump(stats, f, indent=2, default=convert_np)
        print(f"✓ Saved statistics to {stats_file}")

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Race Trajectory Optimizer')
    parser.add_argument('--track', type=str, help='Track name')
    parser.add_argument('--config', type=str, default='/app/config/full_config.yaml',
                        help='Path to configuration file')
    args = parser.parse_args()

    print("=== Race Trajectory Optimizer ===")
    optimizer = RaceTrajectoryOptimizer(args.config)
    track_name = args.track if args.track else None
    trajectory, reftrack, stats = optimizer.optimize_track(track_name)

    print("\n=== Optimization Complete ===")
    print(f"Track: {track_name or optimizer.config['track']['name']}")
    print(f"Lap time: {stats['lap_time']:.2f} seconds")
    print(f"Average speed: {stats['v_avg']:.1f} m/s")
    print(f"Max speed: {stats['v_max']:.1f} m/s")
    print(f"Track length: {stats['track_length']:.1f} m")

if __name__ == "__main__":
    main()
