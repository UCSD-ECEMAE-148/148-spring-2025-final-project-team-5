#!/usr/bin/env python3
"""Test the setup with cvxopt"""

import sys
print("Python version:", sys.version)

# Test our quadprog wrapper
sys.path.insert(0, '/app/src')

try:
    import numpy as np
    print(f"✓ NumPy {np.__version__}")
except ImportError as e:
    print(f"✗ NumPy: {e}")

try:
    import cvxopt
    print(f"✓ cvxopt {cvxopt.__version__}")
except ImportError as e:
    print(f"✗ cvxopt: {e}")

try:
    import quadprog_wrapper
    print("✓ quadprog_wrapper (cvxopt-based)")
    
    # Test the wrapper
    G = np.eye(2)
    a = np.array([0., 0.])
    C = np.array([[1., 0.], [0., 1.], [-1., 0.], [0., -1.]])
    b = np.array([2., 2., 2., 2.])
    result = quadprog_wrapper.solve_qp(G, a, C.T, b)
    print("  quadprog_wrapper test: PASSED")
    print(f"  Solution: {result[0]}")
except Exception as e:
    print(f"✗ quadprog_wrapper: {e}")

try:
    # Don't import trajectory_planning_helpers yet as it needs patching
    print("✓ Ready for trajectory_planning_helpers (after patching)")
except Exception as e:
    print(f"Note: {e}")

print("\nSetup test complete!")
