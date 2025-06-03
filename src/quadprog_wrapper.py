"""
Wrapper to replace quadprog with cvxopt
This provides the same interface as quadprog.solve_qp but uses cvxopt
"""

import numpy as np
from cvxopt import matrix, solvers

# Silence cvxopt output
solvers.options['show_progress'] = False
solvers.options['feastol'] = 1e-9
solvers.options['abstol'] = 1e-9
solvers.options['reltol'] = 1e-9

def solve_qp(G, a, C=None, b=None, meq=0):
    """
    Solve a quadratic program using cvxopt
    
    This mimics the interface of quadprog.solve_qp
    
    minimize: 0.5 * x^T * G * x + a^T * x
    subject to: C^T * x >= b (inequality constraints)
                C^T * x = b (equality constraints, first meq rows)
    """
    n = G.shape[0]
    
    # Ensure G is positive definite by adding small regularization if needed
    G_reg = G + np.eye(n) * 1e-8
    
    # Convert to cvxopt format
    P = matrix(G_reg.astype(float))
    q = matrix(a.astype(float))
    
    if C is not None and b is not None:
        # quadprog uses opposite sign convention for inequalities
        # quadprog: C^T * x >= b
        # cvxopt: G * x <= h
        
        if meq > 0:
            # Equality constraints: A * x = b
            A_eq = matrix(C[:, :meq].T.astype(float))
            b_eq = matrix(b[:meq].astype(float))
            
            # Inequality constraints
            if C.shape[1] > meq:
                G_ineq = matrix(-C[:, meq:].T.astype(float))
                h_ineq = matrix(-b[meq:].astype(float))
            else:
                G_ineq = None
                h_ineq = None
        else:
            # Only inequality constraints
            A_eq = None
            b_eq = None
            G_ineq = matrix(-C.T.astype(float))
            h_ineq = matrix(-b.astype(float))
    else:
        A_eq = None
        b_eq = None
        G_ineq = None
        h_ineq = None
    
    try:
        # Solve
        sol = solvers.qp(P, q, G_ineq, h_ineq, A_eq, b_eq)
        
        if sol['status'] != 'optimal':
            # Try with relaxed tolerances
            solvers.options['feastol'] = 1e-6
            solvers.options['abstol'] = 1e-6
            solvers.options['reltol'] = 1e-6
            sol = solvers.qp(P, q, G_ineq, h_ineq, A_eq, b_eq)
        
        # Return in quadprog format
        x = np.array(sol['x']).flatten()
        f = float(sol['primal objective'])
        xu = np.zeros(0)  # Lagrange multipliers for bounds (not used)
        iterations = sol['iterations']
        
        # Handle lagrangian multipliers
        if sol['y'] is not None:
            lagrangian = np.array(sol['y']).flatten()
        else:
            lagrangian = np.zeros(0)
            
        iact = np.zeros(0, dtype=int)  # Active constraints (not implemented)
        
        return x, f, xu, iterations, lagrangian, iact
        
    except Exception as e:
        # Fallback for problematic cases
        # Return a feasible solution if possible
        if G_ineq is None and A_eq is None:
            # Unconstrained problem
            x = -np.linalg.solve(G_reg, a)
            f = 0.5 * x.T @ G @ x + a.T @ x
            return x, float(f), np.zeros(0), 1, np.zeros(0), np.zeros(0, dtype=int)
        else:
            raise ValueError(f"QP solve failed: {e}")

# Create module-level alias for compatibility
import sys
sys.modules['quadprog'] = sys.modules[__name__]
