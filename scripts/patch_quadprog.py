#!/usr/bin/env python3
"""
Patch TUMFTM code to use our quadprog wrapper
"""

import os
import re

def patch_file(filepath):
    """Replace quadprog imports with our wrapper"""
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Replace quadprog imports
    replacements = [
        (r'import quadprog', 'import sys\nsys.path.insert(0, "/app/src")\nimport quadprog_wrapper as quadprog'),
        (r'from quadprog import solve_qp', 'import sys\nsys.path.insert(0, "/app/src")\nfrom quadprog_wrapper import solve_qp'),
    ]
    
    modified = False
    for pattern, replacement in replacements:
        if re.search(pattern, content):
            content = re.sub(pattern, replacement, content)
            modified = True
    
    if modified:
        with open(filepath, 'w') as f:
            f.write(content)
        print(f"Patched: {filepath}")

# Find and patch all Python files that use quadprog
for root, dirs, files in os.walk('/app/global_racetrajectory_optimization'):
    for file in files:
        if file.endswith('.py'):
            filepath = os.path.join(root, file)
            patch_file(filepath)

print("Patching complete!")
