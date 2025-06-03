#!/usr/bin/env python3
"""Test the setup and dependencies"""

import sys
print("Python version:", sys.version)

try:
    import numpy as np
    print(f"✓ NumPy {np.__version__}")
except ImportError as e:
    print(f"✗ NumPy: {e}")

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
    import quadprog
    print("✓ quadprog")
    # Test quadprog functionality
    import numpy as np
    G = np.eye(2)
    a = np.array([0., 0.])
    C = np.array([[1., 0.], [0., 1.], [-1., 0.], [0., -1.]])
    b = np.array([2., 2., 2., 2.])
    result = quadprog.solve_qp(G, a, C.T, b)
    print("  quadprog test: PASSED")
except Exception as e:
    print(f"✗ quadprog: {e}")

try:
    import trajectory_planning_helpers
    print(f"✓ trajectory_planning_helpers {trajectory_planning_helpers.__version__}")
except ImportError as e:
    print(f"✗ trajectory_planning_helpers: {e}")

print("\nSetup test complete!")
