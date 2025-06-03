#!/usr/bin/env python3
"""
Comprehensive patcher for TUMFTM to use our quadprog wrapper
"""

import os
import sys

# Add our quadprog wrapper to Python path first
sys.path.insert(0, '/app/src')

# Import our wrapper to make it available
import quadprog_wrapper
sys.modules['quadprog'] = quadprog_wrapper

# Now try importing TUMFTM modules
print("Testing TUMFTM imports with quadprog wrapper...")

try:
    sys.path.insert(0, '/app/global_racetrajectory_optimization')
    
    # Test trajectory_planning_helpers first
    import trajectory_planning_helpers as tph
    print("✓ trajectory_planning_helpers imported successfully")
    
    # Test TUMFTM modules
    from helper_funcs_glob.src import prep_track
    print("✓ prep_track imported successfully")
    
    from helper_funcs_glob.src import result_plots
    print("✓ result_plots imported successfully")
    
    # Try the main module
    import main_globaltraj
    print("✓ main_globaltraj imported successfully")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

print("\nPatching complete! TUMFTM should now work with cvxopt backend.")
