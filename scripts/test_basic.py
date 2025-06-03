#!/usr/bin/env python3
"""Basic test to ensure environment is working"""

print("Testing basic imports...")

try:
    import numpy as np
    print(f"✓ NumPy {np.__version__}")
    
    import scipy
    print(f"✓ SciPy {scipy.__version__}")
    
    import matplotlib
    print(f"✓ Matplotlib {matplotlib.__version__}")
    
    import casadi
    print(f"✓ CasADi {casadi.__version__}")
    
    import cvxopt
    print(f"✓ cvxopt {cvxopt.__version__}")
    
    import pandas as pd
    print(f"✓ Pandas {pd.__version__}")
    
    import yaml
    print("✓ PyYAML")
    
    print("\nAll basic imports successful!")
    
except Exception as e:
    print(f"Error: {e}")
