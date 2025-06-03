#!/usr/bin/env python3
"""
Integration module for TUMFTM without quadprog dependency
"""

import sys
import os

# Add TUMFTM to path
sys.path.insert(0, '/app/global_racetrajectory_optimization')

# Try importing TUMFTM modules
try:
    from helper_funcs_glob.src import import_track
    from helper_funcs_glob.src import prep_track
    print("✓ Successfully imported TUMFTM helper functions")
except ImportError as e:
    print(f"✗ Failed to import TUMFTM modules: {e}")
    print("Note: Some TUMFTM functionality may require quadprog")

# Import what we can without quadprog
available_modules = []

modules_to_try = [
    ('helper_funcs_glob.src.import_track', 'import_track'),
    ('helper_funcs_glob.src.export_traj_race', 'export_traj_race'),
    ('helper_funcs_glob.src.result_plots', 'result_plots'),
    ('helper_funcs_glob.src.calc_min_bound_dists', 'calc_min_bound_dists'),
]

for module_path, module_name in modules_to_try:
    try:
        exec(f"from {module_path} import *")
        available_modules.append(module_name)
        print(f"✓ Loaded {module_name}")
    except Exception as e:
        print(f"✗ Could not load {module_name}: {e}")

print(f"\nAvailable modules: {available_modules}")
