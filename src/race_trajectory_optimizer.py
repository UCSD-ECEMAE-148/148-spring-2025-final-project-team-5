#!/usr/bin/env python3
"""
Geometric Optimizer adapted to use full_config.yaml and support donkey tracks
"""
import sys
sys.path.insert(0, './src')
import quadprog_wrapper
sys.modules['quadprog'] = quadprog_wrapper

import os
import numpy as np
import matplotlib.pyplot as plt
import yaml
from matplotlib.collections import LineCollection

# TUMFTM functions
sys.path.insert(0, './global_racetrajectory_optimization')
from helper_funcs_glob.src.import_track import import_track
from helper_funcs_glob.src.prep_track import prep_track

class RaceTrajectoryOptimizer:
    def __init__(self, config_path="./config/full_config.yaml"):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        print("Race Trajectory Optimizer initialized")
        print(f"  Optimization method: {self.config['optimization']['method']}")
        print(f"  Vehicle: {self.config['vehicle']['width']}m wide, max velocity: {self.config['vehicle']['max_velocity']} m/s")

    def optimize_track(self, track_name=None):
        track_name = track_name or self.config['track']['name']
        track_file = f"./inputs/tracks/{track_name}.csv"
        print(f"\nLoading track: {track_file}")

        reftrack = import_track(
            file_path=track_file,
            imp_opts=self.config['track']['import_options'],
            width_veh=self.config['vehicle']['width']
        )
        print(f"✓ Loaded {len(reftrack)} track points")

        # Geometric optimization
        raceline = self._optimize_raceline(reftrack)

        # Create trajectory
        trajectory = self._create_trajectory(raceline, reftrack)

        # Export
        self._export_results(trajectory, track_name)
        self._visualize_results(trajectory, reftrack, track_name)
        return trajectory, reftrack

    def _optimize_raceline(self, reftrack):
        x, y = reftrack[:, 0], reftrack[:, 1]
        w_right, w_left = reftrack[:, 2], reftrack[:, 3]

        dx, dy = np.gradient(x), np.gradient(y)
        ds = np.sqrt(dx**2 + dy**2)
        ds[ds < 1e-6] = 1e-6
        dxn, dyn = dx / ds, dy / ds
        nx, ny = -dyn, dxn

        ddx, ddy = np.gradient(dxn), np.gradient(dyn)
        curvature = ddx * ny - ddy * nx

        from scipy.ndimage import gaussian_filter1d
        curvature_smooth = gaussian_filter1d(curvature, sigma=5, mode='wrap')

        max_offset = self.config['optimization']['geometric']['max_offset_fraction']
        lateral_offset = np.zeros_like(curvature)
        for i in range(len(curvature_smooth)):
            if curvature_smooth[i] > 0.01:
                lateral_offset[i] = -min(max_offset * w_right[i], w_right[i] - 0.5)
            elif curvature_smooth[i] < -0.01:
                lateral_offset[i] = min(max_offset * w_left[i], w_left[i] - 0.5)

        lateral_offset = gaussian_filter1d(lateral_offset, sigma=10, mode='wrap')
        raceline_x = x + nx * lateral_offset
        raceline_y = y + ny * lateral_offset
        raceline_x = gaussian_filter1d(raceline_x, sigma=3, mode='wrap')
        raceline_y = gaussian_filter1d(raceline_y, sigma=3, mode='wrap')
        return np.column_stack([raceline_x, raceline_y])

    def _create_trajectory(self, raceline, reftrack):
        dx, dy = np.gradient(raceline[:, 0]), np.gradient(raceline[:, 1])
        ddx, ddy = np.gradient(dx), np.gradient(dy)
        denom = (dx**2 + dy**2)**1.5
        denom[denom < 1e-6] = 1e-6
        curvature = np.abs(ddx * dy - dx * ddy) / denom

        from scipy.ndimage import gaussian_filter1d
        curvature = gaussian_filter1d(curvature, sigma=3, mode='wrap')

        a_lat_max = self.config['vehicle']['max_lat_acc']
        v_max = self.config['vehicle']['max_velocity']
        v_min = self.config['vehicle']['min_velocity']

        velocity = np.where(curvature > 0.001, np.sqrt(a_lat_max / curvature), v_max)
        velocity = gaussian_filter1d(velocity, sigma=10, mode='wrap')
        velocity = np.clip(velocity, v_min, v_max)

        heading = np.arctan2(dy, dx)
        ax = np.gradient(velocity) * 2
        ay = velocity**2 * curvature

        return np.column_stack([raceline[:, 0], raceline[:, 1], heading, velocity, ax, ay])

    def _export_results(self, traj, name):
        out = "./outputs"
        os.makedirs(out, exist_ok=True)
        np.savetxt(f"{out}/{name}_trajectory.csv", traj, delimiter=',',
                   header='x_m,y_m,psi_rad,vx_mps,ax_mps2,ay_mps2', comments='')
        print(f"✓ Saved trajectory to {out}/{name}_trajectory.csv")

    def _visualize_results(self, traj, reftrack, name):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 14))
        ax1.set_aspect('equal')
        x, y = reftrack[:, 0], reftrack[:, 1]
        dx, dy = np.gradient(x), np.gradient(y)
        ds = np.sqrt(dx**2 + dy**2)
        ds[ds < 1e-6] = 1e-6
        nx, ny = -dy / ds, dx / ds
        xr = x + nx * reftrack[:, 2]
        yr = y + ny * reftrack[:, 2]
        xl = x - nx * reftrack[:, 3]
        yl = y - ny * reftrack[:, 3]

        ax1.fill(np.concatenate([xr, xl[::-1]]), np.concatenate([yr, yl[::-1]]),
                 color='lightgray', alpha=0.5, label='Track surface')
        ax1.plot(xr, yr, 'k-', linewidth=2)
        ax1.plot(xl, yl, 'k-', linewidth=2)
        ax1.plot(x, y, 'b--', alpha=0.5, linewidth=1)

        points = np.array([traj[:, 0], traj[:, 1]]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap='RdYlGn')
        lc.set_array(traj[:, 3])
        lc.set_linewidth(3)
        ax1.add_collection(lc)
        plt.colorbar(lc, ax=ax1).set_label('Velocity (m/s)', rotation=270, labelpad=20)
        ax1.set_title(f'Optimized Racing Line - {name}')
        ax1.grid(True)

        dists = np.sqrt(np.diff(traj[:, 0])**2 + np.diff(traj[:, 1])**2)
        distance = np.concatenate([[0], np.cumsum(dists)])
        ax2.plot(distance, traj[:, 3], 'b-', linewidth=2)
        ax2.fill_between(distance, 0, traj[:, 3], alpha=0.3)
        ax2.set_title('Velocity Profile')
        ax2.set_xlabel('Distance (m)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.grid(True)

        plt.tight_layout()
        plt.savefig(f"./outputs/{name}_analysis.png", dpi=150, bbox_inches='tight')
        print(f"✓ Saved visualization to ./outputs/{name}_analysis.png")

if __name__ == "__main__":
    print("=== Race Trajectory Optimizer ===")
    optimizer = RaceTrajectoryOptimizer()
    trajectory, reftrack = optimizer.optimize_track()
    print("\n✓ Done!")
