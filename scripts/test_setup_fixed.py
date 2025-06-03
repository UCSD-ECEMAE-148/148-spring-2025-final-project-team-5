#!/usr/bin/env python3
"""Test the setup with fixed quadprog wrapper"""

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
    
    # Test 1: Simple unconstrained problem
    G = np.eye(2)
    a = np.array([3., 4.])
    result = quadprog_wrapper.solve_qp(G, a)
    print(f"  Test 1 (unconstrained): PASSED, solution = {result[0]}")
    
    # Test 2: Constrained problem (the original test)
    G = np.array([[1., 0.], [0., 1.]])
    a = np.array([0., 0.])
    # Constraints: x >= -2, x <= 2, y >= -2, y <= 2
    # In matrix form: -x <= 2, x <= 2, -y <= 2, y <= 2
    C = np.array([[-1., 0.], [1., 0.], [0., -1.], [0., 1.]]).T
    b = np.array([-2., -2., -2., -2.])
    
    result = quadprog_wrapper.solve_qp(G, a, C, b)
    print(f"  Test 2 (constrained): PASSED, solution = {result[0]}")
    
except Exception as e:
    import traceback
    print(f"✗ quadprog_wrapper: {e}")
    traceback.print_exc()

print("\nAll other dependencies:")

try:
    import scipy
    print(f"✓ SciPy {scipy.__version__}")
except ImportError as e:
    print(f"✗ SciPy: {e}")

try:
    import matplotlib
    print(f"✓ Matplotlib {matplotlib.__version__}")
except ImportError as e:
    print(f"✗ Matplotlib: {e}")

try:
    import casadi
    print(f"✓ CasADi {casadi.__version__}")
except ImportError as e:
    print(f"✗ CasADi: {e}")

try:
    import pandas
    print(f"✓ Pandas {pandas.__version__}")
except ImportError as e:
    print(f"✗ Pandas: {e}")

print("\nSetup test complete!")
