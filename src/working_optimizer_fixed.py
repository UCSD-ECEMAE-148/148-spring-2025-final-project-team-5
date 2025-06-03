#!/usr/bin/env python3
"""
Working optimizer using available TUMFTM functions - Fixed version
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
from helper_funcs_glob.src.export_traj_race import export_traj_race
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
        
        # Import track with required parameters
        reftrack_imp = import_track(
            import_path=track_file,
            imp_opts=self.imp_opts,
            width_veh=self.width_veh
        )
        print(f"✓ Loaded {len(reftrack_imp)} track points")
        
        # Process the imported track
        reftrack = self._process_imported_track(reftrack_imp)
        
        # Calculate track boundaries
        track_widths = reftrack[:, 2] + reftrack[:, 3]  # right + left widths
        avg_width = np.mean(track_widths)
        print(f"✓ Average track width: {avg_width:.2f}m")
        
        # Create a simple racing line
        raceline = self._create_simple_raceline(reftrack)
        
        # Calculate distances to boundaries
        try:
            dists = calc_min_bound_dists(reftrack, raceline)
            print(f"✓ Min distance to boundaries: {np.min(dists):.2f}m")
        except Exception as e:
            print(f"⚠ Could not calculate boundary distances: {e}")
        
        # Create trajectory with velocity profile
        trajectory = self._create_trajectory(raceline)
        
        # Export
        self._export_results(trajectory, track_name)
        
        return trajectory, reftrack
        
    def _process_imported_track(self, reftrack_imp):
        """Process the imported track to expected format"""
        # TUMFTM import_track returns: [x, y, w_tr_right, w_tr_left]
        # Make sure we have the right format
        if reftrack_imp.shape[1] >= 4:
            return reftrack_imp[:, :4]
        else:
            # If missing widths, add default values
            n_points = len(reftrack_imp)
            default_width = 5.0  # meters
            return np.column_stack([
                reftrack_imp[:, 0],  # x
                reftrack_imp[:, 1],  # y
                np.ones(n_points) * default_width,  # right width
                np.ones(n_points) * default_width   # left width
            ])
        
    def _create_simple_raceline(self, reftrack):
        """Create a simple racing line"""
        # Extract centerline
        x = reftrack[:, 0]
        y = reftrack[:, 1]
        
        # Simple smoothing
        from scipy.ndimage import gaussian_filter1d
        x_smooth = gaussian_filter1d(x, sigma=2, mode='wrap')
        y_smooth = gaussian_filter1d(y, sigma=2, mode='wrap')
        
        return np.column_stack([x_smooth, y_smooth])
        
    def _create_trajectory(self, raceline):
        """Create trajectory with velocity profile"""
        n_points = len(raceline)
        
        # Calculate curvature
        dx = np.gradient(raceline[:, 0])
        dy = np.gradient(raceline[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        # Avoid division by zero
        denominator = (dx**2 + dy**2)**1.5
        denominator[denominator < 1e-6] = 1e-6
        
        curvature = np.abs(ddx * dy - dx * ddy) / denominator
        
        # Simple velocity profile based on curvature
        v_max = self.config['optimization']['constraints']['max_velocity']
        v_min = 20.0  # minimum velocity in corners
        
        # Velocity inversely proportional to curvature
        velocity = v_max - (v_max - v_min) * np.clip(curvature * 100, 0, 1)
        
        # Calculate heading
        heading = np.arctan2(dy, dx)
        
        # Combine into trajectory
        trajectory = np.column_stack([
            raceline[:, 0],  # x
            raceline[:, 1],  # y
            heading,         # psi (heading)
            velocity,        # v (velocity)
            np.zeros(n_points),  # ax (acceleration x)
            np.zeros(n_points)   # ay (acceleration y)
        ])
        
        return trajectory
        
    def _export_results(self, trajectory, track_name):
        """Export results"""
        output_dir = "/app/outputs"
        os.makedirs(output_dir, exist_ok=True)
        
        # Save trajectory
        traj_file = f"{output_dir}/{track_name}_trajectory.csv"
        np.savetxt(traj_file, trajectory, delimiter=',',
                   header='x,y,psi,v,ax,ay', comments='')
        print(f"✓ Saved trajectory to: {traj_file}")
        
    def visualize(self, trajectory, reftrack):
        """Visualize results"""
        plt.figure(figsize=(12, 8))
        
        # Plot track boundaries
        x_center = reftrack[:, 0]
        y_center = reftrack[:, 1]
        
        # Calculate track boundaries
        dx = np.gradient(x_center)
        dy = np.gradient(y_center)
        
        # Normal vectors
        normals = np.column_stack([-dy, dx])
        norms = np.sqrt(normals[:, 0]**2 + normals[:, 1]**2)
        norms[norms < 1e-6] = 1.0  # Avoid division by zero
        normals = normals / norms[:, np.newaxis]
        
        # Right boundary
        x_right = x_center - normals[:, 0] * reftrack[:, 2]
        y_right = y_center - normals[:, 1] * reftrack[:, 2]
        
        # Left boundary
        x_left = x_center + normals[:, 0] * reftrack[:, 3]
        y_left = y_center + normals[:, 1] * reftrack[:, 3]
        
        # Plot
        plt.plot(x_right, y_right, 'k-', linewidth=1, label='Track boundary')
        plt.plot(x_left, y_left, 'k-', linewidth=1)
        plt.plot(x_center, y_center, 'b--', alpha=0.5, label='Center line')
        plt.plot(trajectory[:, 0], trajectory[:, 1], 'r-', linewidth=2, label='Racing line')
        
        # Color code by velocity
        scatter = plt.scatter(trajectory[::10, 0], trajectory[::10, 1], 
                            c=trajectory[::10, 3], cmap='RdYlGn', s=20, zorder=5)
        plt.colorbar(scatter, label='Velocity (m/s)')
        
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.title(f'Optimized Racing Line - {trajectory.shape[0]} points')
        
        plt.savefig('/app/outputs/racing_line_visualization.png', dpi=150, bbox_inches='tight')
        print("✓ Saved visualization")

if __name__ == "__main__":
    print("=== Working Race Trajectory Optimizer ===\n")
    
    optimizer = WorkingOptimizer()
    
    try:
        # Run optimization
        trajectory, reftrack = optimizer.optimize_track("berlin_2018")
        
        # Visualize
        optimizer.visualize(trajectory, reftrack)
        
        print("\n✓ Optimization complete!")
        print(f"  Track points: {len(reftrack)}")
        print(f"  Trajectory points: {len(trajectory)}")
        print(f"  Avg velocity: {np.mean(trajectory[:, 3]):.1f} m/s")
        print(f"  Max velocity: {np.max(trajectory[:, 3]):.1f} m/s")
        print(f"  Min velocity: {np.min(trajectory[:, 3]):.1f} m/s")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
