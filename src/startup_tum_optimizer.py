#!/usr/bin/env python3
"""
Startup script for TUM Racing Line Optimizer with quadprog wrapper
"""

import sys
import os

# Step 1: Clean any existing quadprog from sys.modules
for key in list(sys.modules.keys()):
    if 'quadprog' in key:
        del sys.modules[key]

# Step 2: Install our wrapper BEFORE any other imports
sys.path.insert(0, '/app/src')
import quadprog_wrapper

# Install it as quadprog
sys.modules['quadprog'] = quadprog_wrapper

print("✓ Quadprog wrapper installed")

# Step 3: Now we can add TUM paths
sys.path.insert(0, '/app/global_racetrajectory_optimization')
sys.path.insert(0, '/app/opt_mintime_traj')

# Step 4: Import and run the main optimizer
try:
    # Import TUM modules to verify they work
    import trajectory_planning_helpers as tph
    print("✓ trajectory_planning_helpers imported")
    
    import helper_funcs_glob
    print("✓ helper_funcs_glob imported")
    
    # Import and run our optimizer
    from race_trajectory_optimizer import RaceTrajectoryOptimizer
    
    print("\n=== Starting Race Trajectory Optimizer ===")
    
    # Check for command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='TUM Racing Line Optimizer')
    parser.add_argument('--track', type=str, help='Track name (e.g., berlin_2018)')
    parser.add_argument('--method', type=str, help='Optimization method')
    parser.add_argument('--config', type=str, default='./config/full_config.yaml', help='Config file path')
    
    args = parser.parse_args()
    
    # Create optimizer
    optimizer = RaceTrajectoryOptimizer(config_path=args.config)
    
    # Override track/method if provided
    if args.method:
        optimizer.opt_type = args.method
        optimizer.tum_opt_type = optimizer.method_mapping.get(args.method, 'mincurv')
        print(f"  Using method: {args.method}")
    
    # Run optimization
    trajectory, reftrack = optimizer.optimize_track(track_name=args.track)
    
    print("\n✓ Optimization completed successfully!")
    
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)