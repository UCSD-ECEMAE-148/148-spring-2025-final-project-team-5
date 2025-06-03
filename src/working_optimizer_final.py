#!/usr/bin/env python3
"""
Working optimizer using TUMFTM functions with correct signatures
"""

# Install quadprog wrapper first
import sys
sys.path.insert(0, '/app/src')
import quadprog_wrapper
sys.modules['quadprog'] = quadprog_wrapper

import os
import numpy as np
import matplotlib.pyplot as plt
import yaml

# Add TUMFTM path
sys.path.insert(0, '/app/global_racetrajectory_optimization')

# Import working modules
from helper_funcs_glob.src.import_track import import_track
from helper_funcs_glob.src.prep_track import prep_track
from helper_funcs_glob.src.calc_min_bound_dists import calc_min_bound_dists

class WorkingOptimizer:
    def __init__(self, config_path="/app/config/optimizer_config.yaml"):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        print("Working optimizer initialized")
        
        # Setup import options for TUMFTM
        self.imp_opts = {
            "flip_imp_track": False,
            "set_new_start": False,
            "new_start": np.array([0.0, -47.0]),
            "min_track_width": 3.0,
            "num_laps": 1
        }
        
        # Vehicle width
        self.width_veh = 2.0  # meters
        
    def optimize_track(self, track_name="berlin_2018"):
        """Optimize a track using available functions"""
        
        # Load track
        track_file = f"/app/inputs/tracks/{track_name}.csv"
        print(f"Loading track: {track_file}")
        
        # Import track with correct parameter name
        reftrack_imp = import_track(
            file_path=track_file,  # Changed from import_path
            imp_opts=self.imp_opts,
            width_veh=self.width_veh
        )
        print(f"✓ Loaded {len(reftrack_imp)} track points")
        
        # Prepare track using TUMFTM function
        try:
            reftrack_prep = prep_track(reftrack_imp, self.imp_opts, self.width_veh)
            print("✓ Track prepared with TUMFTM prep_track")
            reftrack = reftrack_prep
        except Exception as e:
            print(f"⚠ Could not use prep_track: {e}")
            reftrack = reftrack_imp
        
        # Calculate track boundaries
        track_widths = reftrack[:, 2] + reftrack[:, 3]  # right + left widths
        avg_width = np.mean(track_widths)
        print(f"✓ Average track width: {avg_width:.2f}m")
        
        # Create an optimized racing line
        raceline = self._optimize_raceline(reftrack)
        
        # Calculate distances to boundaries
        try:
            dists = calc_min_bound_dists(reftrack, raceline)
            print(f"✓ Min distance to boundaries: {np.min(dists):.2f}m")
        except Exception as e:
            print(f"⚠ Could not calculate boundary distances: {e}")
        
        # Create trajectory with velocity profile
        trajectory = self._create_trajectory(raceline, reftrack)
        
        # Export
        self._export_results(trajectory, track_name)
        
        return trajectory, reftrack
        
    def _optimize_raceline(self, reftrack):
        """Create an optimized racing line using geometric approach"""
        x = reftrack[:, 0]
        y = reftrack[:, 1]
        w_right = reftrack[:, 2]
        w_left = reftrack[:, 3]
        
        # Calculate track direction
        dx = np.gradient(x)
        dy = np.gradient(y)
        
        # Normalize direction vectors
        ds = np.sqrt(dx**2 + dy**2)
        ds[ds < 1e-6] = 1e-6
        dx_norm = dx / ds
        dy_norm = dy / ds
        
        # Calculate normals (perpendicular to track direction)
        nx = -dy_norm
        ny = dx_norm
        
        # Calculate curvature
        ddx = np.gradient(dx_norm)
        ddy = np.gradient(dy_norm)
        curvature = ddx * ny - ddy * nx
        
        # Smooth the curvature
        from scipy.ndimage import gaussian_filter1d
        curvature_smooth = gaussian_filter1d(curvature, sigma=5, mode='wrap')
        
        # Lateral offset based on curvature
        max_offset = 0.7  # Maximum offset as fraction of track width
        lateral_offset = np.zeros_like(curvature)
        
        for i in range(len(curvature_smooth)):
            if curvature_smooth[i] > 0.01:  # Left turn
                # Move to the right (negative offset)
                lateral_offset[i] = -min(max_offset * w_right[i], w_right[i] - 1.0)
            elif curvature_smooth[i] < -0.01:  # Right turn
                # Move to the left (positive offset)
                lateral_offset[i] = min(max_offset * w_left[i], w_left[i] - 1.0)
        
        # Smooth the lateral offset
        lateral_offset = gaussian_filter1d(lateral_offset, sigma=10, mode='wrap')
        
        # Apply offset to create racing line
        raceline_x = x + nx * lateral_offset
        raceline_y = y + ny * lateral_offset
        
        # Additional smoothing of the racing line
        raceline_x = gaussian_filter1d(raceline_x, sigma=3, mode='wrap')
        raceline_y = gaussian_filter1d(raceline_y, sigma=3, mode='wrap')
        
        return np.column_stack([raceline_x, raceline_y])
        
    def _create_trajectory(self, raceline, reftrack):
        """Create trajectory with velocity profile"""
        n_points = len(raceline)
        
        # Calculate path curvature
        dx = np.gradient(raceline[:, 0])
        dy = np.gradient(raceline[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        # Calculate curvature
        denominator = (dx**2 + dy**2)**1.5
        denominator[denominator < 1e-6] = 1e-6
        curvature = np.abs(ddx * dy - dx * ddy) / denominator
        
        # Smooth curvature
        from scipy.ndimage import gaussian_filter1d
        curvature = gaussian_filter1d(curvature, sigma=3, mode='wrap')
        
        # Calculate velocity profile based on curvature
        a_lat_max = 20.0  # m/s^2 - maximum lateral acceleration
        v_max_global = self.config['optimization']['constraints']['max_velocity']
        v_min = 15.0  # minimum velocity
        
        velocity = np.zeros(n_points)
        for i in range(n_points):
            if curvature[i] > 0.001:
                v_curve = np.sqrt(a_lat_max / curvature[i])
                velocity[i] = min(v_curve, v_max_global)
            else:
                velocity[i] = v_max_global
        
        # Smooth velocity profile
        velocity = gaussian_filter1d(velocity, sigma=10, mode='wrap')
        velocity = np.clip(velocity, v_min, v_max_global)
        
        # Calculate heading
        heading = np.arctan2(dy, dx)
        
        # Calculate accelerations
        ax = np.gradient(velocity) * 2.0  # Simplified longitudinal acceleration
        ay = velocity**2 * curvature  # Lateral acceleration
        
        # Combine into trajectory
        trajectory = np.column_stack([
            raceline[:, 0],  # x
            raceline[:, 1],  # y
            heading,         # psi (heading)
            velocity,        # v (velocity)
            ax,              # ax (acceleration x)
            ay               # ay (acceleration y)
        ])
        
        return trajectory
        
    def _export_results(self, trajectory, track_name):
        """Export results"""
        output_dir = "/app/outputs"
        os.makedirs(output_dir, exist_ok=True)
        
        # Save trajectory
        traj_file = f"{output_dir}/{track_name}_trajectory.csv"
        np.savetxt(traj_file, trajectory, delimiter=',',
                   header='x_m,y_m,psi_rad,vx_mps,ax_mps2,ay_mps2', comments='')
        print(f"✓ Saved trajectory to: {traj_file}")
        
    def visualize(self, trajectory, reftrack):
        """Visualize results"""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 14))
        
        # Plot 1: Track and racing line
        ax1.set_aspect('equal')
        
        # Calculate track boundaries
        x_center = reftrack[:, 0]
        y_center = reftrack[:, 1]
        
        dx = np.gradient(x_center)
        dy = np.gradient(y_center)
        ds = np.sqrt(dx**2 + dy**2)
        ds[ds < 1e-6] = 1e-6
        
        # Normal vectors
        nx = -dy / ds
        ny = dx / ds
        
        # Boundaries
        x_right = x_center + nx * reftrack[:, 2]
        y_right = y_center + ny * reftrack[:, 2]
        x_left = x_center - nx * reftrack[:, 3]
        y_left = y_center - ny * reftrack[:, 3]
        
        # Plot track
        ax1.fill(np.concatenate([x_right, x_left[::-1]]),
                 np.concatenate([y_right, y_left[::-1]]),
                 color='lightgray', alpha=0.5, label='Track surface')
        ax1.plot(x_right, y_right, 'k-', linewidth=2, label='Track boundaries')
        ax1.plot(x_left, y_left, 'k-', linewidth=2)
        ax1.plot(x_center, y_center, 'b--', alpha=0.5, linewidth=1, label='Center line')
        
        # Plot racing line colored by velocity
        points = np.array([trajectory[:, 0], trajectory[:, 1]]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        from matplotlib.collections import LineCollection
        lc = LineCollection(segments, cmap='RdYlGn')
        lc.set_array(trajectory[:, 3])
        lc.set_linewidth(3)
        line = ax1.add_collection(lc)
        
        cbar = plt.colorbar(line, ax=ax1)
        cbar.set_label('Velocity (m/s)', rotation=270, labelpad=20)
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title(f'Optimized Racing Line - Berlin 2018')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='best')
        
        # Plot 2: Velocity profile
        distance = np.cumsum(np.sqrt(np.diff(trajectory[:, 0])**2 + 
                                    np.diff(trajectory[:, 1])**2))
        distance = np.concatenate([[0], distance])
        
        ax2.plot(distance, trajectory[:, 3], 'b-', linewidth=2)
        ax2.fill_between(distance, 0, trajectory[:, 3], alpha=0.3)
        ax2.set_xlabel('Distance (m)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity Profile')
        ax2.grid(True, alpha=0.3)
        ax2.set_xlim(0, distance[-1])
        ax2.set_ylim(0, max(trajectory[:, 3]) * 1.1)
        
        plt.tight_layout()
        plt.savefig('/app/outputs/racing_line_analysis.png', dpi=150, bbox_inches='tight')
        print("✓ Saved visualization")

if __name__ == "__main__":
    print("=== Working Race Trajectory Optimizer ===\n")
    
    optimizer = WorkingOptimizer()
    
    try:
        # Run optimization
        trajectory, reftrack = optimizer.optimize_track("berlin_2018")
        
        # Visualize
        optimizer.visualize(trajectory, reftrack)
        
        # Print statistics
        print("\n✓ Optimization complete!")
        print(f"  Track points: {len(reftrack)}")
        print(f"  Trajectory points: {len(trajectory)}")
        print(f"  Avg velocity: {np.mean(trajectory[:, 3]):.1f} m/s")
        print(f"  Max velocity: {np.max(trajectory[:, 3]):.1f} m/s")
        print(f"  Min velocity: {np.min(trajectory[:, 3]):.1f} m/s")
        
        # Calculate lap time
        distances = np.sqrt(np.diff(trajectory[:, 0])**2 + np.diff(trajectory[:, 1])**2)
        times = distances / trajectory[:-1, 3]
        lap_time = np.sum(times)
        print(f"  Estimated lap time: {lap_time:.2f} seconds ({lap_time/60:.2f} minutes)")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
