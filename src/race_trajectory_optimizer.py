#!/usr/bin/env python3
"""
Race Trajectory Optimizer using TUM optimizer
Supports multiple optimization methods: geometric, mincurv, mincurv_iqp, mintime, shortest_path
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import yaml
import time
import json
from matplotlib.collections import LineCollection

# Add TUM optimizer to path
sys.path.insert(0, './global_racetrajectory_optimization')
sys.path.insert(0, './opt_mintime_traj')

# Import TUM modules
import trajectory_planning_helpers as tph
import helper_funcs_glob

class RaceTrajectoryOptimizer:
    def __init__(self, config_path="./config/full_config.yaml"):
        """Initialize optimizer with configuration"""
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
            
        # Set up paths
        self.module_path = os.path.dirname(os.path.abspath(__file__))
        
        # Extract optimization method
        self.opt_type = self.config['optimization']['method']
        
        # Map our config names to TUM optimizer names
        self.method_mapping = {
            'geometric': 'mincurv',  # Use mincurv as fallback for geometric
            'mincurv': 'mincurv',
            'min_curv': 'mincurv',
            'mincurv_iqp': 'mincurv_iqp',
            'mintime': 'mintime',
            'min_time': 'mintime',
            'shortest_path': 'shortest_path'
        }
        
        # Get TUM method name
        self.tum_opt_type = self.method_mapping.get(self.opt_type, 'mincurv')
        
        print("Race Trajectory Optimizer initialized")
        print(f"  Optimization method: {self.opt_type} (TUM: {self.tum_opt_type})")
        print(f"  Vehicle: {self.config['vehicle']['width']}m wide, max velocity: {self.config['vehicle']['max_velocity']} m/s")
        
        # Load vehicle parameters for TUM optimizer
        self._setup_vehicle_params()

    def _setup_vehicle_params(self):
        """Set up vehicle parameters in TUM format from config"""
        # Main vehicle parameters
        self.pars = {
            "veh_params": {
                "width": self.config['vehicle']['width'],
                "length": self.config['vehicle']['length'],
                "curvlim": self.config['vehicle'].get('curvlim', 1000.0),
                "v_max": self.config['vehicle']['max_velocity'],
                "mass": self.config['vehicle']['mass'],
                "dragcoeff": self.config['vehicle'].get('dragcoeff', 0.75)
            },
            "optim_opts": {
                "width_opt": self.config['optimization'].get('mincurv', {}).get('width_opt', 
                                                                                 self.config['vehicle']['width'] + 0.2),
            },
            "stepsize_opts": {
                "stepsize_prep": self.config.get('track_preparation', {}).get('stepsize_prep', 1.0),
                "stepsize_reg": self.config.get('track_preparation', {}).get('stepsize_reg', 2.0),
                "stepsize_interp_after_opt": self.config.get('track_preparation', {}).get('stepsize_interp_after_opt', 0.2)
            },
            "reg_smooth_opts": {
                "k_reg": self.config.get('track_preparation', {}).get('spline_degree', 3),
                "s_reg": self.config.get('track_preparation', {}).get('smoothing_factor', 10)
            },
            "vel_calc_opts": {
                "vel_profile_conv_filt_window": self.config['optimization'].get('velocity_profile', {}).get('conv_filt_window', 7),
                "dyn_model_exp": 1.0
            }
        }
        
        # Add method-specific parameters
        if self.tum_opt_type in ['mincurv', 'mincurv_iqp']:
            mincurv_config = self.config['optimization'].get('mincurv', {})
            self.pars["optim_opts"]["iqp_iters_min"] = mincurv_config.get('iqp_iters_min', 3)
            self.pars["optim_opts"]["iqp_curverror_allowed"] = mincurv_config.get('iqp_curverror_allowed', 0.01)
            self.pars["veh_params"]["curvlim"] = mincurv_config.get('curvlim', 1000.0)
            
        elif self.tum_opt_type == 'mintime':
            mintime_config = self.config['optimization'].get('mintime', {})
            self.pars["optim_opts"]["var_friction"] = mintime_config.get('var_friction', None)
            self.pars["optim_opts"]["warm_start"] = mintime_config.get('warm_start', False)
            self.pars["optim_opts"]["safe_traj"] = mintime_config.get('safe_traj', False)
            self.pars["optim_opts"]["ax_pos_safe"] = mintime_config.get('ax_pos_safe', None)
            self.pars["optim_opts"]["ax_neg_safe"] = mintime_config.get('ax_neg_safe', None)
            self.pars["optim_opts"]["ay_safe"] = mintime_config.get('ay_safe', None)

    def optimize_track(self, track_name=None):
        """Main optimization function"""
        track_name = track_name or self.config['track']['name']
        track_file = f"./inputs/tracks/{track_name}.csv"
        print(f"\nLoading track: {track_file}")
        
        # Import track using TUM function
        reftrack_imp = helper_funcs_glob.src.import_track.import_track(
            imp_opts=self.config['track']['import_options'],
            file_path=track_file,
            width_veh=self.config['vehicle']['width']
        )
        print(f"✓ Loaded {len(reftrack_imp)} track points")
        
        # Prepare track (spline fitting)
        reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp = \
            helper_funcs_glob.src.prep_track.prep_track(
                reftrack_imp=reftrack_imp,
                reg_smooth_opts=self.pars["reg_smooth_opts"],
                stepsize_opts=self.pars["stepsize_opts"],
                debug=True,
                min_width=self.config['track']['import_options'].get('min_track_width', None)
            )
        
        # Run optimization based on method
        alpha_opt = self._run_optimization(
            reftrack_interp, 
            normvec_normalized_interp, 
            a_interp,
            coeffs_x_interp,
            coeffs_y_interp
        )
        
        # Create raceline
        raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, \
        t_vals_opt_interp, s_points_opt_interp, spline_lengths_opt, el_lengths_opt_interp = \
            tph.create_raceline.create_raceline(
                refline=reftrack_interp[:, :2],
                normvectors=normvec_normalized_interp,
                alpha=alpha_opt,
                stepsize_interp=self.pars["stepsize_opts"]["stepsize_interp_after_opt"]
            )
        
        # Calculate heading and curvature
        psi_vel_opt, kappa_opt = tph.calc_head_curv_an.calc_head_curv_an(
            coeffs_x=coeffs_x_opt,
            coeffs_y=coeffs_y_opt,
            ind_spls=spline_inds_opt_interp,
            t_spls=t_vals_opt_interp
        )
        
        # Calculate velocity profile
        if self.tum_opt_type == 'mintime':
            # Use mintime velocities if available
            vx_profile_opt = self.v_opt if hasattr(self, 'v_opt') else self._calc_velocity_profile(kappa_opt, el_lengths_opt_interp)
        else:
            vx_profile_opt = self._calc_velocity_profile(kappa_opt, el_lengths_opt_interp)
        
        # Calculate acceleration
        vx_profile_opt_cl = np.append(vx_profile_opt, vx_profile_opt[0])
        ax_profile_opt = tph.calc_ax_profile.calc_ax_profile(
            vx_profile=vx_profile_opt_cl,
            el_lengths=el_lengths_opt_interp,
            eq_length_output=False
        )
        
        # Create trajectory
        trajectory = np.column_stack([
            raceline_interp[:, 0],  # x
            raceline_interp[:, 1],  # y
            psi_vel_opt,            # heading
            vx_profile_opt,         # velocity
            ax_profile_opt,         # longitudinal acceleration
            vx_profile_opt**2 * kappa_opt  # lateral acceleration
        ])
        
        # Export and visualize
        if self.config['output']['save_trajectory']:
            self._export_results(trajectory, track_name)
            
        if self.config['output']['save_visualization']:
            self._visualize_results(trajectory, reftrack_interp, normvec_normalized_interp, track_name)
        
        return trajectory, reftrack_interp

    def _run_optimization(self, reftrack_interp, normvec_normalized_interp, a_interp, coeffs_x_interp, coeffs_y_interp):
        """Run the selected optimization method"""
        print(f"\nRunning {self.tum_opt_type} optimization...")
        
        if self.tum_opt_type == 'mincurv':
            alpha_opt = tph.opt_min_curv.opt_min_curv(
                reftrack=reftrack_interp,
                normvectors=normvec_normalized_interp,
                A=a_interp,
                kappa_bound=self.pars["veh_params"]["curvlim"],
                w_veh=self.pars["optim_opts"]["width_opt"],
                print_debug=True,
                plot_debug=False
            )[0]
            
        elif self.tum_opt_type == 'mincurv_iqp':
            alpha_opt, _, _ = tph.iqp_handler.iqp_handler(
                reftrack=reftrack_interp,
                normvectors=normvec_normalized_interp,
                A=a_interp,
                kappa_bound=self.pars["veh_params"]["curvlim"],
                w_veh=self.pars["optim_opts"]["width_opt"],
                print_debug=True,
                plot_debug=False,
                stepsize_interp=self.pars["stepsize_opts"]["stepsize_reg"],
                iters_min=self.pars["optim_opts"]["iqp_iters_min"],
                curv_error_allowed=self.pars["optim_opts"]["iqp_curverror_allowed"]
            )
            
        elif self.tum_opt_type == 'shortest_path':
            alpha_opt = tph.opt_shortest_path.opt_shortest_path(
                reftrack=reftrack_interp,
                normvectors=normvec_normalized_interp,
                w_veh=self.pars["optim_opts"]["width_opt"],
                print_debug=True
            )
            
        elif self.tum_opt_type == 'mintime':
            # For mintime, we need additional setup
            import opt_mintime_traj
            
            # Create dummy file paths
            file_paths = {
                "module": self.module_path,
                "tpamap": "dummy",
                "tpadata": "dummy",
                "mintime_export": "./outputs/mintime"
            }
            
            # Extend parameters for mintime
            self.pars["curv_calc_opts"] = {"iqp_iters_min": 3}
            self.pars["vehicle_params_mintime"] = {
                "wheelbase_front": self.config['vehicle']['wheelbase'] / 2,
                "wheelbase_rear": self.config['vehicle']['wheelbase'] / 2,
                "wheelbase": self.config['vehicle']['wheelbase']
            }
            self.pars["tire_params_mintime"] = {}
            self.pars["pwr_params_mintime"] = {}
            
            alpha_opt, self.v_opt, _, _, _ = opt_mintime_traj.src.opt_mintime.opt_mintime(
                reftrack=reftrack_interp,
                coeffs_x=coeffs_x_interp,
                coeffs_y=coeffs_y_interp,
                normvectors=normvec_normalized_interp,
                pars=self.pars,
                tpamap_path=file_paths["tpamap"],
                tpadata_path=file_paths["tpadata"],
                export_path=file_paths["mintime_export"],
                print_debug=True,
                plot_debug=False
            )
        else:
            # Fallback to simple geometric (if method='geometric')
            print("Using geometric optimization")
            alpha_opt = self._geometric_optimization(reftrack_interp, normvec_normalized_interp)
            
        return alpha_opt

    def _geometric_optimization(self, reftrack, normvec):
        """Simple geometric optimization based on config settings"""
        # Calculate curvature
        x, y = reftrack[:, 0], reftrack[:, 1]
        dx, dy = np.gradient(x), np.gradient(y)
        ddx, ddy = np.gradient(dx), np.gradient(dy)
        
        ds = np.sqrt(dx**2 + dy**2)
        ds[ds < 1e-6] = 1e-6
        dxn, dyn = dx / ds, dy / ds
        nx, ny = -dyn, dxn
        
        curvature = ddx * ny - ddy * nx
        
        # Smooth curvature
        from scipy.ndimage import gaussian_filter1d
        geo_config = self.config['optimization'].get('geometric', {})
        curvature_smooth = gaussian_filter1d(curvature, 
                                           sigma=geo_config.get('curvature_smoothing', 5), 
                                           mode='wrap')
        
        # Calculate lateral offset
        max_offset = geo_config.get('max_offset_fraction', 0.7)
        lateral_offset = np.zeros_like(curvature)
        
        for i in range(len(curvature_smooth)):
            if curvature_smooth[i] > 0.01:
                lateral_offset[i] = -min(max_offset * reftrack[i, 2], reftrack[i, 2] - 0.1)
            elif curvature_smooth[i] < -0.01:
                lateral_offset[i] = min(max_offset * reftrack[i, 3], reftrack[i, 3] - 0.1)
                
        # Smooth offset
        lateral_offset = gaussian_filter1d(lateral_offset, 
                                         sigma=geo_config.get('offset_smoothing', 10), 
                                         mode='wrap')
        
        # Convert to alpha (normalized offset)
        alpha = lateral_offset
        
        return alpha

    def _calc_velocity_profile(self, kappa, el_lengths):
        """Calculate velocity profile based on curvature"""
        # Get velocity profile config
        vel_config = self.config['optimization'].get('velocity_profile', {})
        
        # Vehicle limits
        v_max = self.config['vehicle']['max_velocity']
        v_min = self.config['vehicle']['min_velocity']
        a_lat_max = self.config['vehicle']['max_lat_acc']
        
        # Calculate velocity from curvature
        velocity = np.zeros_like(kappa)
        for i in range(len(kappa)):
            if abs(kappa[i]) > 0.001:
                v_curve = np.sqrt(a_lat_max / abs(kappa[i]))
                velocity[i] = min(v_curve, v_max)
            else:
                velocity[i] = v_max
                
        # Apply safety factor
        safety_factor = vel_config.get('safety_factor', 0.9)
        velocity = velocity * safety_factor
        
        # Smooth velocity
        from scipy.ndimage import gaussian_filter1d
        smoothing = vel_config.get('smoothing', 10)
        velocity = gaussian_filter1d(velocity, sigma=smoothing, mode='wrap')
        velocity = np.clip(velocity, v_min, v_max)
        
        return velocity

    def _export_results(self, traj, name):
        """Export trajectory to CSV"""
        output_config = self.config['output']
        out_dir = output_config.get('export_path', './outputs')
        os.makedirs(out_dir, exist_ok=True)
        
        filename = f"{out_dir}/{name}_trajectory_{self.opt_type}.csv"
        np.savetxt(filename, traj, delimiter=',',
                   header='x_m,y_m,psi_rad,vx_mps,ax_mps2,ay_mps2', 
                   comments='')
        print(f"✓ Saved trajectory to {filename}")

    def _visualize_results(self, traj, reftrack, normvec, name):
        """Visualize the optimized trajectory matching the original style"""
        viz_config = self.config['visualization']
        
        # Create figure with subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=viz_config['figure_size'])
        
        # Plot 1: Track and racing line
        ax1.set_aspect('equal')
        
        # Track preparation
        x, y = reftrack[:, 0], reftrack[:, 1]
        
        # Calculate track boundaries
        xr = x + normvec[:, 0] * reftrack[:, 2]
        yr = y + normvec[:, 1] * reftrack[:, 2]
        xl = x - normvec[:, 0] * reftrack[:, 3]
        yl = y - normvec[:, 1] * reftrack[:, 3]
        
        # Plot track surface
        if viz_config['track']['show_surface']:
            ax1.fill(np.concatenate([xr, xl[::-1]]), 
                     np.concatenate([yr, yl[::-1]]),
                     color=viz_config['track']['surface_color'], 
                     alpha=viz_config['track']['surface_alpha'], 
                     label='Track surface')
        
        # Plot boundaries
        if viz_config['track']['show_boundaries']:
            ax1.plot(xr, yr, 
                     color=viz_config['track']['boundary_color'], 
                     linewidth=viz_config['track']['boundary_width'])
            ax1.plot(xl, yl, 
                     color=viz_config['track']['boundary_color'], 
                     linewidth=viz_config['track']['boundary_width'])
        
        # Plot centerline
        if viz_config['track']['show_centerline']:
            ax1.plot(x, y, 
                     color=viz_config['track']['centerline_color'],
                     linestyle=viz_config['track']['centerline_style'],
                     alpha=0.5, 
                     linewidth=1, 
                     label='Centerline')
        
        # Plot racing line with velocity colors
        if viz_config['racing_line']['show_velocity_colors']:
            points = np.array([traj[:, 0], traj[:, 1]]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            lc = LineCollection(segments, cmap=viz_config['racing_line']['colormap'])
            lc.set_array(traj[:, 3])
            lc.set_linewidth(viz_config['racing_line']['line_width'])
            ax1.add_collection(lc)
            cbar = plt.colorbar(lc, ax=ax1)
            cbar.set_label('Velocity (m/s)', rotation=270, labelpad=20)
        else:
            ax1.plot(traj[:, 0], traj[:, 1], 'r-', 
                     linewidth=viz_config['racing_line']['line_width'],
                     label='Racing line')
        
        # Add title and labels
        ax1.set_title(f'Optimized Racing Line - {name} ({self.opt_type})')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Velocity profile
        if viz_config['velocity_profile']['show']:
            # Calculate distance along track
            dists = np.sqrt(np.diff(traj[:, 0])**2 + np.diff(traj[:, 1])**2)
            distance = np.concatenate([[0], np.cumsum(dists)])
            
            ax2.plot(distance, traj[:, 3], 
                     color=viz_config['velocity_profile']['color'], 
                     linewidth=2)
            ax2.fill_between(distance, 0, traj[:, 3], 
                             alpha=viz_config['velocity_profile']['fill_alpha'])
            ax2.set_title('Velocity Profile')
            ax2.set_xlabel('Distance (m)')
            ax2.set_ylabel('Velocity (m/s)')
            ax2.grid(True, alpha=0.3)
            
            # Add optimization info
            avg_velocity = np.mean(traj[:, 3])
            lap_time = np.sum(dists / traj[:-1, 3])
            
            info_text = f'Method: {self.opt_type}\n'
            info_text += f'Avg velocity: {avg_velocity:.2f} m/s\n'
            info_text += f'Est. lap time: {lap_time:.2f} s'
            
            ax2.text(0.02, 0.98, info_text, 
                    transform=ax2.transAxes, 
                    va='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
        
        # Save plot
        output_path = self.config['output'].get('export_path', './outputs')
        filename = f"{output_path}/{name}_analysis_{self.opt_type}.png"
        plt.savefig(filename, dpi=viz_config['dpi'], bbox_inches='tight')
        print(f"✓ Saved visualization to {filename}")
        
        plt.show()

if __name__ == "__main__":
    print("=== Race Trajectory Optimizer (TUM) ===")
    optimizer = RaceTrajectoryOptimizer()
    trajectory, reftrack = optimizer.optimize_track()
    print("\n✓ Done!")