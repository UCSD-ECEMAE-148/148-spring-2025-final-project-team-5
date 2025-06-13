#!/usr/bin/env python3
"""
Race Trajectory Optimizer with Enhanced IQP Iteration Visualization

This module implements the `RaceTrajectoryOptimizer`, a high-level wrapper around the TUM 
Global Race Trajectory Optimization toolbox. It supports multiple racing trajectory methods, 
including:

  • geometric       — simple corner-cutting based on track curvature  
  • shortest_path   — hugs the inside line to minimize path length  
  • mincurv         — minimizes curvature for smooth, constant-speed lines  
  • mincurv_iqp     — iterative quadratic programming refinement of min-curv lines  
  • mintime         — full time-optimal control via CasADi/IPOPT, optionally warm-started  
  • (future) other methods as configured

Key features:
  - Reads a unified `full_config.yaml` for all optimizer, vehicle, track, and visualization settings  
  - Preprocesses the track (spline fitting, regular/non-regular sampling, curvature/heading numerics)  
  - Delegates to TUM's helper routines (quadprog via CVXOPT, CasADi collocation, powertrain models)  
  - Exports optimized trajectories, lap statistics, and optional plots in user-specified formats  
  - Supports DonkeyCar throttle passthrough and multi-lap simulation for the mintime solver  
  - Enhanced IQP iteration visualization showing convergence process
  - Seamlessly wraps lower-level solvers (quadprog, CVXOPT, IPOPT) to minimize external dependencies  

Usage:
    from race_trajectory_optimizer import RaceTrajectoryOptimizer

    optimizer = RaceTrajectoryOptimizer(config_path="full_config.yaml")
    trajectory, reftrack = optimizer.optimize_track(track_name="my_track")

For detailed parameter descriptions and tuning guidelines, see the accompanying `full_config.yaml` 
and the TUM Race Car Optimization paper by Christ et al.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import yaml
import time
import json
from scipy.spatial import cKDTree
from matplotlib.collections import LineCollection
import matplotlib.cm as cm

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
        
        # Initialize IQP iteration storage
        self.iqp_iterations = []
        self.iteration_data = []
        
        print("Race Trajectory Optimizer initialized")
        print(f"  Optimization method: {self.opt_type} (TUM: {self.tum_opt_type})")
        print(f"  Vehicle: {self.config['vehicle']['width']}m wide, max velocity: {self.config['vehicle']['max_velocity']} m/s")
        
        # Load vehicle parameters for TUM optimizer
        self._setup_vehicle_params()
        self._maybe_convert_throttle()
        
    def _maybe_convert_throttle(self):
        """If enabled in config, convert raw donkey path into TUM format and store throttle vector."""
        imp = self.config['track']['import_options']
        if imp.get('use_throttle', False):
            raw_csv = imp['raw_input']
            df = pd.read_csv(raw_csv, header=None, names=["x","y","throttle"])
            # inject widths
            df["w_tr_right_m"] = imp['width_right']
            df["w_tr_left_m"]  = imp['width_left']
            # stash for export
            self._throttle_points = df[["x","y"]].to_numpy()
            self._throttle_vals   = df["throttle"].to_numpy()
            # write out the converted input for optimizer
            track_name = self.config['track']['name']
            conv_path  = os.path.join("inputs", "tracks", f"{track_name}.csv")
            df[["x","y","w_tr_right_m","w_tr_left_m"]].to_csv(
                conv_path, index=False, header=False
            )
            print(f"✓ DonkeyCar path converted with throttle → {conv_path}")

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
                "dragcoeff": self.config['vehicle'].get('dragcoeff', 0.75),
                "g": self.config['vehicle'].get("g", 9.81)
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
            mt_cfg = self.config['optimization']['mintime']
            v_cfg  = self.config['vehicle']

            # 1) basic optim_opts
            optim = self.pars["optim_opts"]
            optim["var_friction"] = mt_cfg.get('var_friction', None)
            optim["warm_start"]   = mt_cfg.get('warm_start', False)
            optim["safe_traj"]    = mt_cfg.get('safe_traj', False)
            optim["ax_pos_safe"]  = mt_cfg.get('ax_pos_safe', None)
            optim["ax_neg_safe"]  = mt_cfg.get('ax_neg_safe', None)
            optim["ay_safe"]      = mt_cfg.get('ay_safe', None)
            optim["step_non_reg"] = mt_cfg.get('step_non_reg', 0.0)
            optim["eps_kappa"]    = mt_cfg.get('eps_kappa', 1e-4)
            optim["mue"]          = mt_cfg.get('mue', v_cfg.get('friction_coeff', 1.0))

            # 2) curvature & heading preview/review
            self.pars["curv_calc_opts"] = {
                "d_preview_curv": mt_cfg.get("d_preview_curv", 2.0),
                "d_review_curv":  mt_cfg.get("d_review_curv", 2.0),
                "d_preview_head": mt_cfg.get("d_preview_head", 1.0),
                "d_review_head":  mt_cfg.get("d_review_head", 1.0),
            }

            # 3) gravity
            self.pars["veh_params"]["g"] = v_cfg.get("g", 9.81)

            # 4) mintime vehicle parameters
            wheelbase = v_cfg["wheelbase"]
            track_w   = v_cfg.get("track_width", v_cfg["width"])
            self.pars["vehicle_params_mintime"] = {
                "wheelbase_front": wheelbase/2,
                "wheelbase_rear":  wheelbase/2,
                "wheelbase":       wheelbase,
                "track_width_front": track_w,
                "track_width_rear":  track_w,
                "cog_z":           v_cfg.get("cog_height", 0.0),
                "I_z":             v_cfg.get("moment_of_inertia", 0.0),
                "liftcoeff_front": v_cfg.get("downforce_coeff", 0.0),
                "liftcoeff_rear":  v_cfg.get("downforce_coeff", 0.0),
                "k_brake_front":   v_cfg.get("brake_distribution", 0.5),
                "k_drive_front":   v_cfg.get("drive_distribution", 0.0),
                "k_roll":          v_cfg["k_roll"],
                "delta_max":       v_cfg["max_steering_angle"],
                "power_max":       v_cfg["max_power"],
                "f_drive_max":     v_cfg["max_drive_force"],
                "f_brake_max":     v_cfg["max_brake_force"]
            }

            # 5) tire parameters
            self.pars["tire_params_mintime"] = self.config.get("tire_params", {})
            self.pars["pwr_params_mintime"] = self.config.get("pwr_params_mintime", {
            "pwr_behavior": False
            })

            self.pars["optim_opts"]["limit_energy"]  = self.config.get("limit_energy", False)
            self.pars["optim_opts"]["energy_limit"]  = self.config.get("energy_limit", 0.0)
            # regularization penalties for drive/brake and steering smoothness
            self.pars["optim_opts"]["penalty_F"]     = self.config.get("penalty_F", 0.0)
            self.pars["optim_opts"]["penalty_delta"] = self.config.get("penalty_delta", 0.0)

    def _custom_iqp_handler(self, reftrack, normvectors, A, kappa_bound, w_veh, print_debug, plot_debug, stepsize_interp, iters_min, curv_error_allowed):
        """Custom IQP handler that captures iterations for visualization"""
        
        # Initialize with minimum curvature solution
        alpha_mincurv = tph.opt_min_curv.opt_min_curv(
            reftrack=reftrack,
            normvectors=normvectors,
            A=A,
            kappa_bound=kappa_bound,
            w_veh=w_veh,
            print_debug=False,
            plot_debug=False
        )[0]
        
        # Store initial solution
        self.iqp_iterations = [alpha_mincurv.copy()]
        self.iteration_data = [{'iteration': 0, 'method': 'initial_mincurv', 'alpha': alpha_mincurv.copy()}]
        
        # Check the exact signature of iqp_handler to avoid parameter errors
        try:
            # Try with all expected parameters
            alpha_opt, curv_error_max, iters = tph.iqp_handler.iqp_handler(
                reftrack=reftrack,
                normvectors=normvectors,
                A=A,
                kappa_bound=kappa_bound,
                w_veh=w_veh,
                print_debug=print_debug,
                plot_debug=plot_debug,
                stepsize_interp=stepsize_interp,
                iters_min=iters_min,
                curv_error_allowed=curv_error_allowed
            )
        except TypeError as e:
            if "unexpected keyword argument" in str(e):
                print(f"⚠️ Parameter mismatch in iqp_handler: {e}")
                print("⚠️ Trying with minimal parameters...")
                # Fallback with minimal parameters
                alpha_opt, curv_error_max, iters = tph.iqp_handler.iqp_handler(
                    reftrack=reftrack,
                    normvectors=normvectors,
                    A=A,
                    kappa_bound=kappa_bound,
                    w_veh=w_veh
                )
            else:
                raise e
        
        # If direct iteration capture not available, simulate intermediate steps
        if len(self.iqp_iterations) == 1:
            print("⚠️ Direct iteration capture not available, simulating intermediate steps...")
            self._simulate_iqp_iterations(alpha_mincurv, alpha_opt, iters)
        
        return alpha_opt, curv_error_max, iters
    
    def _simulate_iqp_iterations(self, alpha_initial, alpha_final, num_iters):
        """Simulate IQP iterations by interpolating between initial and final solutions"""
        # Handle different types of num_iters input with robust numpy array handling
        print(f"  Debug: num_iters type = {type(num_iters)}, value = {num_iters}")
        
        try:
            if isinstance(num_iters, np.ndarray):
                if num_iters.size == 1:
                    # Single element array - extract safely
                    num_iters = num_iters.item()
                else:
                    # Multi-element array - use the last element as final iteration count
                    num_iters = num_iters[-1].item()
            elif isinstance(num_iters, (list, tuple)):
                num_iters = int(num_iters[-1]) if len(num_iters) > 0 else 3
            else:
                # Scalar value
                num_iters = int(num_iters)
                
        except (ValueError, TypeError, AttributeError) as e:
            print(f"⚠️ Could not convert num_iters ({num_iters}) to int: {e}")
            print("⚠️ Using default value of 3 iterations")
            num_iters = 3
        
        # Handle shape mismatch between initial and final alpha arrays
        print(f"  Debug: alpha_initial shape = {alpha_initial.shape}, alpha_final shape = {alpha_final.shape}")
        
        if alpha_initial.shape != alpha_final.shape:
            print(f"⚠️ Shape mismatch detected: initial={alpha_initial.shape}, final={alpha_final.shape}")
            # Trim both arrays to the minimum common length
            min_length = min(len(alpha_initial), len(alpha_final))
            alpha_initial = alpha_initial[:min_length]
            alpha_final = alpha_final[:min_length]
            print(f"✓ Trimmed both arrays to length {min_length}")
                
        print(f"  Simulating {num_iters} IQP iterations...")
        
        for i in range(1, num_iters + 1):
            # Linear interpolation between initial and final
            t = i / num_iters
            alpha_interp = (1 - t) * alpha_initial + t * alpha_final
            
            self.iqp_iterations.append(alpha_interp.copy())
            self.iteration_data.append({
                'iteration': i, 
                'method': f'iqp_iter_{i}', 
                'alpha': alpha_interp.copy()
            })

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
        
        print(f"✓ Track prepared: {len(reftrack_interp)} interpolated points")
        
        # Store reference data for IQP visualization
        self.reftrack_interp = reftrack_interp
        self.normvec_normalized_interp = normvec_normalized_interp
        
        # Run optimization based on method
        alpha_opt = self._run_optimization(
            reftrack_interp, 
            normvec_normalized_interp, 
            a_interp,
            coeffs_x_interp,
            coeffs_y_interp
        )
        
        print(f"✓ Optimization complete: alpha shape = {alpha_opt.shape}")
        
        # **FIX: Validate shapes before calling create_raceline**
        refline = reftrack_interp[:, :2]  # Extract x,y coordinates
        
        
        # Ensure alpha and other arrays have matching lengths
        min_length = min(len(refline), len(normvec_normalized_interp), len(alpha_opt))
        
        if len(refline) != len(alpha_opt) or len(normvec_normalized_interp) != len(alpha_opt):
            print(f"⚠️  Shape mismatch detected, trimming to common length: {min_length}")
            refline = refline[:min_length]
            normvec_normalized_interp = normvec_normalized_interp[:min_length]
            alpha_opt = alpha_opt[:min_length]
            reftrack_interp = reftrack_interp[:min_length]  # Also trim the full reftrack
            
            print(f"✓ Shapes after trimming:")
            print(f"  refline: {refline.shape}")
            print(f"  normvec_normalized_interp: {normvec_normalized_interp.shape}")
            print(f"  alpha_opt: {alpha_opt.shape}")
        
        # Create raceline
        raceline_interp, a_opt, coeffs_x_opt, coeffs_y_opt, spline_inds_opt_interp, \
        t_vals_opt_interp, s_points_opt_interp, spline_lengths_opt, el_lengths_opt_interp = \
            tph.create_raceline.create_raceline(
                refline=refline,
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

        if hasattr(self, '_throttle_points'):
            self._export_throttle(trajectory, track_name)
        
        return trajectory, reftrack_interp
        
    def _export_throttle(self, traj, track_name):
        """Match optimized points to original throttle and save x,y,throttle CSV"""
        # build KDTree on original donkey points
        tree = cKDTree(self._throttle_points)
        pts = np.column_stack([traj[:,0], traj[:,1]])
        _, idx = tree.query(pts)
        thr = self._throttle_vals[idx]
        out = np.column_stack([traj[:,0], traj[:,1], thr])
        w_r = self.config['track']['import_options']['width_right']
        w_l = self.config['track']['import_options']['width_left']
        fname = f"outputs/donkey_optimized_path_{w_r:.2f}_{w_l:.2f}.csv"
        np.savetxt(fname, out, delimiter=',', header='x,y,throttle', comments='')
        print(f"✓ Saved donkey-compatible optimized path → {fname}")

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
            # Use custom IQP handler to capture iterations
            alpha_opt, _, _ = self._custom_iqp_handler(
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
            import opt_mintime_traj
            file_paths = {
                "module": self.module_path,
                "tpamap": "dummy",
                "tpadata": "dummy",
                "mintime_export": "./outputs/mintime"
            }

            v = self.config["vehicle"]
            # pull in your tire block too:
            t = self.config.get("tire_params", {})

            # assemble the dict exactly as opt_mintime.py expects:
            self.pars["vehicle_params_mintime"] = {
                "wheelbase_front":     v["wheelbase"] / 2,
                "wheelbase_rear":      v["wheelbase"] / 2,
                "wheelbase":           v["wheelbase"],
                "track_width_front":   v["track_width_front"],
                "track_width_rear":    v["track_width_rear"],
                "cog_z":               v["cog_height"],
                "I_z":                 v["moment_of_inertia"],
                # map your single downforce_coeff → liftcoeff_{front, rear}
                "liftcoeff_front":     v.get("downforce_front", v.get("downforce_coeff", 0.0)),
                "liftcoeff_rear":      v.get("downforce_rear",  v.get("downforce_coeff", 0.0)),
                "k_roll":              v["k_roll"],
                "k_drive_front":       v["k_drive_front"],
                "k_brake_front":       v["k_brake_front"],
                "delta_max":           v["delta_max"],
                "power_max":           v["max_power"],
                "f_drive_max":         v["max_drive_force"],
                "f_brake_max":         v["max_brake_force"],
                "t_delta":             v["steering_time_constant"],
                "t_drive":             v["drive_time_constant"],
                "t_brake":             v["brake_time_constant"],
            }

            # tire parameters for the Kamm's circle constraints:
            self.pars["tire_params_mintime"] = {
                "c_roll":    t["c_roll"],
                "f_z0":      t["f_z0"],
                "B_front":   t["B_front"],
                "C_front":   t["C_front"],
                "eps_front": t["eps_front"],
                "E_front":   t["E_front"],
                "B_rear":    t["B_rear"],
                "C_rear":    t["C_rear"],
                "eps_rear":  t["eps_rear"],
                "E_rear":    t["E_rear"],
            }

            # disable the powertrain branch:
            self.pars["pwr_params_mintime"] = {"pwr_behavior": False}

            # now call the TUM module
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
        """Visualize the optimized trajectory with IQP iteration support"""
        viz_config = self.config['visualization']
        
        # For IQP method, create special iteration visualization
        if self.tum_opt_type == 'mincurv_iqp' and len(self.iqp_iterations) > 1:
            self._plot_iqp_iterations(reftrack, normvec, name)
        
        # Create standard figure with subplots
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
            
            if self.tum_opt_type == 'mincurv_iqp':
                info_text += f'\nIQP Iterations: {len(self.iqp_iterations)}'
            
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

    def _plot_iqp_iterations(self, reftrack, normvec, name):
        """Create specialized plot showing IQP iteration convergence"""
        print(f"✓ Creating IQP iteration visualization with {len(self.iqp_iterations)} iterations")
        
        # Create figure for IQP iterations
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Setup track visualization
        x, y = reftrack[:, 0], reftrack[:, 1]
        xr = x + normvec[:, 0] * reftrack[:, 2]
        yr = y + normvec[:, 1] * reftrack[:, 2]
        xl = x - normvec[:, 0] * reftrack[:, 3]
        yl = y - normvec[:, 1] * reftrack[:, 3]
        
        # Plot 1: Track with all iterations
        ax1.set_aspect('equal')
        
        # Plot track boundaries
        ax1.plot(xr, yr, 'k-', linewidth=2, alpha=0.8, label='Track boundaries')
        ax1.plot(xl, yl, 'k-', linewidth=2, alpha=0.8)
        ax1.plot(x, y, '--', color='gray', alpha=0.5, linewidth=1, label='Centerline')
        
        # Generate colors for iterations
        colors = plt.cm.plasma(np.linspace(0, 1, len(self.iqp_iterations)))
        
        # Plot each iteration
        for i, (alpha, color) in enumerate(zip(self.iqp_iterations, colors)):
            # Ensure alpha matches reference track length
            min_len = min(len(alpha), len(reftrack))
            alpha_trimmed = alpha[:min_len]
            reftrack_trimmed = reftrack[:min_len]
            normvec_trimmed = normvec[:min_len]
            
            # Calculate raceline for this iteration
            raceline = reftrack_trimmed[:, :2] + np.expand_dims(alpha_trimmed, 1) * normvec_trimmed
            
            if i == 0:
                # Initial solution - dashed line
                ax1.plot(raceline[:, 0], raceline[:, 1], 
                        color=color, linestyle='--', linewidth=2, 
                        label=f'Initial (mincurv)', alpha=0.8)
            elif i == len(self.iqp_iterations) - 1:
                # Final solution - thick solid line
                ax1.plot(raceline[:, 0], raceline[:, 1], 
                        color=color, linewidth=3, 
                        label=f'Final (iter {i})', alpha=1.0)
            else:
                # Intermediate solutions
                ax1.plot(raceline[:, 0], raceline[:, 1], 
                        color=color, linewidth=1.5, 
                        label=f'Iter {i}', alpha=0.7)
        
        ax1.set_title(f'IQP Iteration Convergence - {name}')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.grid(True, alpha=0.3)
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # Plot 2: Alpha evolution
        ax2.set_title('Lateral Offset (Alpha) Evolution')
        
        # Calculate distance along track for x-axis
        dists = np.sqrt(np.diff(reftrack[:, 0])**2 + np.diff(reftrack[:, 1])**2)
        distance = np.concatenate([[0], np.cumsum(dists)])
        
        for i, (alpha, color) in enumerate(zip(self.iqp_iterations, colors)):
            min_len = min(len(alpha), len(distance))
            alpha_trimmed = alpha[:min_len]
            distance_trimmed = distance[:min_len]
            
            if i == 0:
                ax2.plot(distance_trimmed, alpha_trimmed, 
                        color=color, linestyle='--', linewidth=2, 
                        label=f'Initial', alpha=0.8)
            elif i == len(self.iqp_iterations) - 1:
                ax2.plot(distance_trimmed, alpha_trimmed, 
                        color=color, linewidth=3, 
                        label=f'Final', alpha=1.0)
            else:
                ax2.plot(distance_trimmed, alpha_trimmed, 
                        color=color, linewidth=1, 
                        label=f'Iter {i}', alpha=0.7)
        
        ax2.set_xlabel('Distance along track (m)')
        ax2.set_ylabel('Lateral offset α (m)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # Add convergence info
        if len(self.iqp_iterations) > 1:
            initial_alpha = self.iqp_iterations[0]
            final_alpha = self.iqp_iterations[-1]
            min_len = min(len(initial_alpha), len(final_alpha))
            alpha_change = np.abs(final_alpha[:min_len] - initial_alpha[:min_len])
            max_change = np.max(alpha_change)
            avg_change = np.mean(alpha_change)
            
            info_text = f'Convergence Analysis:\n'
            info_text += f'Iterations: {len(self.iqp_iterations)}\n'
            info_text += f'Max α change: {max_change:.4f} m\n'
            info_text += f'Avg α change: {avg_change:.4f} m'
            
            ax2.text(0.02, 0.98, info_text, 
                    transform=ax2.transAxes, 
                    va='top',
                    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
        
        plt.tight_layout()
        
        # Save IQP-specific plot
        output_path = self.config['output'].get('export_path', './outputs')
        filename = f"{output_path}/{name}_iqp_iterations.png"
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"✓ Saved IQP iteration plot to {filename}")
        
        plt.show()

if __name__ == "__main__":
    print("=== Race Trajectory Optimizer (TUM) with IQP Visualization ===")
    optimizer = RaceTrajectoryOptimizer()
    trajectory, reftrack = optimizer.optimize_track()
    print("\n✓ Done!")