"""
Wrapper to replace quadprog with cvxopt
This provides the same interface as quadprog.solve_qp but uses cvxopt
"""

import numpy as np
from cvxopt import matrix, solvers
import warnings

# Silence cvxopt output
solvers.options['show_progress'] = False
solvers.options['feastol'] = 1e-9
solvers.options['abstol'] = 1e-9
solvers.options['reltol'] = 1e-9

def solve_qp(G, a, C=None, b=None, meq=0, factorized=True):
    """
    Solve a quadratic program using cvxopt
    
    This mimics the interface of quadprog.solve_qp:
    
    minimize: 0.5 * x^T * G * x - a^T * x
    subject to: C^T * x >= b
    
    Where the first 'meq' constraints are equality constraints.
    
    Parameters:
    -----------
    G : 2D array, shape (n, n)
        Positive definite quadratic part of objective function
    a : 1D array, shape (n,)
        Linear part of objective function (note the negative sign)
    C : 2D array, shape (n, m), optional
        Constraint matrix
    b : 1D array, shape (m,), optional
        Constraint bounds
    meq : int, optional
        Number of equality constraints (first meq rows of C^T x = b[0:meq])
    factorized : bool, optional
        If True, G is already factorized. Ignored in this implementation.
    
    Returns:
    --------
    x : 1D array, shape (n,)
        Solution
    f : float
        Objective value at solution
    xu : 1D array
        Lagrange multipliers for bounds (not used, returns empty array)
    iterations : int
        Number of iterations
    lagrangian : 1D array
        Lagrange multipliers for constraints
    iact : 1D array
        Active constraints (not used, returns empty array)
    """
    
    # Ensure inputs are numpy arrays and proper shape
    G = np.asarray(G, dtype=np.float64)
    a = np.asarray(a, dtype=np.float64).flatten()
    
    n = G.shape[0]
    
    # Make G positive definite
    G = 0.5 * (G + G.T)  # Ensure symmetry
    min_eig = np.min(np.real(np.linalg.eigvals(G)))
    if min_eig < 1e-8:
        G = G + np.eye(n) * (1e-8 - min_eig)
    
    # Convert to cvxopt format
    # Note: quadprog minimizes 0.5*x'*G*x - a'*x
    # cvxopt minimizes 0.5*x'*P*x + q'*x
    # So we need q = -a
    P = matrix(G)
    q = matrix(-a)  # Note the sign change!
    
    # Handle constraints
    if C is not None and b is not None:
        C = np.asarray(C, dtype=np.float64)
        b = np.asarray(b, dtype=np.float64).flatten()
        
        # Ensure C is 2D
        if C.ndim == 1:
            C = C.reshape(-1, 1)
        
        # quadprog: C^T * x >= b
        # cvxopt: G * x <= h
        # So we need G = -C^T and h = -b
        
        # But we need to handle equality constraints separately
        if meq > 0:
            # First meq constraints are equalities
            A_eq = matrix(C[:, :meq].T)  # Transpose for cvxopt
            b_eq = matrix(b[:meq])
            
            # Remaining are inequalities
            if C.shape[1] > meq:
                G_ineq = matrix(-C[:, meq:].T)  # Transpose and negate
                h_ineq = matrix(-b[meq:])
            else:
                G_ineq = None
                h_ineq = None
        else:
            # All constraints are inequalities
            A_eq = None
            b_eq = None
            G_ineq = matrix(-C.T)  # Transpose and negate
            h_ineq = matrix(-b)
    else:
        # No constraints
        A_eq = None
        b_eq = None
        G_ineq = None
        h_ineq = None
    
    # Solve with cvxopt
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            sol = solvers.qp(P, q, G_ineq, h_ineq, A_eq, b_eq)
        
        if sol['status'] != 'optimal':
            # Try with relaxed tolerances
            solvers.options['feastol'] = 1e-6
            solvers.options['abstol'] = 1e-6
            solvers.options['reltol'] = 1e-6
            sol = solvers.qp(P, q, G_ineq, h_ineq, A_eq, b_eq)
        
        # Extract solution
        x = np.array(sol['x']).flatten()
        
        # Calculate objective value (using quadprog's sign convention)
        f = 0.5 * x.T @ G @ x - a.T @ x
        
        # Extract Lagrange multipliers
        lagrangian = []
        if meq > 0 and sol['y'] is not None:
            lagrangian.extend(np.array(sol['y']).flatten())
        if G_ineq is not None and sol['z'] is not None:
            # Note: cvxopt multipliers have opposite sign
            lagrangian.extend(-np.array(sol['z']).flatten())
        lagrangian = np.array(lagrangian) if lagrangian else np.zeros(0)
        
        # Return in quadprog format
        return (x, 
                float(f), 
                np.zeros(0),  # xu - not used
                sol['iterations'], 
                lagrangian,
                np.zeros(0, dtype=int))  # iact - not used
        
    except Exception as e:
        # If optimization fails, try to provide useful error info
        if 'sol' in locals():
            raise ValueError(f"QP solve failed with status: {sol['status']}")
        else:
            raise ValueError(f"QP solve failed: {str(e)}")

# For backward compatibility
solve_qp_old = solve_qp