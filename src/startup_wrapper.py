#!/usr/bin/env python3
"""
Startup wrapper that ensures quadprog is patched before any imports
"""

import sys
import os

# CRITICAL: Add our src to path and import wrapper FIRST
sys.path.insert(0, '/app/src')
import quadprog_wrapper
sys.modules['quadprog'] = quadprog_wrapper

print("✓ Quadprog wrapper installed")

# Now we can safely import everything else
sys.path.insert(0, '/app/global_racetrajectory_optimization')

def test_imports():
    """Test that all imports work"""
    try:
        import trajectory_planning_helpers
        print("✓ trajectory_planning_helpers OK")
        
        from helper_funcs_glob.src import prep_track
        print("✓ prep_track OK")
        
        from helper_funcs_glob.src import result_plots  
        print("✓ result_plots OK")
        
        from helper_funcs_glob.src import import_track
        print("✓ import_track OK")
        
        return True
    except Exception as e:
        print(f"✗ Import failed: {e}")
        return False

if __name__ == "__main__":
    if test_imports():
        print("\nAll imports successful! You can now run TUMFTM optimization.")
        
        # Try running a simple track import
        try:
            from helper_funcs_glob.src.import_track import import_track
            
            # Test with Berlin track
            track_path = "/app/inputs/tracks/berlin_2018.csv"
            if os.path.exists(track_path):
                track = import_track(track_path)
                print(f"\n✓ Successfully loaded track with {len(track)} points")
            else:
                print(f"\n✗ Track file not found: {track_path}")
                
        except Exception as e:
            print(f"\n✗ Error loading track: {e}")
