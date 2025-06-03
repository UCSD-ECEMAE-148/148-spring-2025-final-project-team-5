#!/usr/bin/env python3
"""
Main TUMFTM runner with proper configuration
"""

# Install quadprog wrapper
import sys
sys.path.insert(0, '/app/src')
import quadprog_wrapper
sys.modules['quadprog'] = quadprog_wrapper

# Bypass version checks
import pkg_resources
original_require = pkg_resources.require
def patched_require(requirements):
    try:
        return original_require(requirements)
    except pkg_resources.VersionConflict:
        pass
pkg_resources.require = patched_require

import os
import shutil
import configparser

# Change to TUMFTM directory
os.chdir('/app/global_racetrajectory_optimization')
sys.path.insert(0, os.getcwd())

def setup_tumftm():
    """Setup TUMFTM with proper configuration"""
    
    # Create params.ini
    config = configparser.ConfigParser()
    
    config['GENERAL_OPTIONS'] = {
        'export_traj_race': 'True',
        'export_traj_ltpl': 'False', 
        'print_debug': 'True'
    }
    
    config['OPTIMIZATION_OPTIONS'] = {
        'optim_method': 'mincurv',  # or 'shortest_path'
        'optim_type': 'mintime',
        'mintime_opts_path': './params/mintime_opts.ini',
        'vehicle_params_path': './params/vehicle_params.ini', 
        'vel_calc_opts_path': './params/vel_calc_opts.ini'
    }
    
    config['TRACK_OPTIONS'] = {
        'track_name': 'berlin',
        'track_file_path': '/app/inputs/tracks/berlin_2018.csv',
        'trackbound_file_path': '',
        'use_circuit': 'True'
    }
    
    with open('params.ini', 'w') as f:
        config.write(f)
    
    print("Created params.ini")
    
    # Create output directories
    os.makedirs('outputs', exist_ok=True)
    os.makedirs('logs', exist_ok=True)
    
def run_optimization():
    """Run TUMFTM optimization"""
    try:
        # Import and run
        import main_globaltraj
        main_globaltraj.main()
        
        # Copy results to our output directory
        if os.path.exists('outputs/traj_race_cl.csv'):
            shutil.copy('outputs/traj_race_cl.csv', '/app/outputs/tumftm_result.csv')
            print("✓ Copied results to /app/outputs/tumftm_result.csv")
            
    except Exception as e:
        print(f"Error in optimization: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    print("=== TUMFTM Global Race Trajectory Optimization ===\n")
    
    # Setup
    setup_tumftm()
    
    # Run
    print("\nStarting optimization...")
    run_optimization()
    
    print("\nDone!")
