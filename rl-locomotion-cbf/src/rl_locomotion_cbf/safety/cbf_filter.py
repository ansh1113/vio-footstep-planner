import numpy as np
import osqp
from scipy import sparse

class CBFSafetyFilter:
    """Control Barrier Function safety filter for RL policies."""
    
    def __init__(self, alpha=1.0, slack_penalty=1000.0):
        self.alpha = alpha
        self.slack_penalty = slack_penalty
    
    def filter(self, state, action_raw, dynamics):
        """Filter unsafe actions through CBF constraints."""
        n_actions = len(action_raw)
        
        # Setup QP: minimize ||u - u_raw||^2
        P = sparse.eye(n_actions)
        q = -action_raw
        
        # Collect CBF constraints
        A_list = []
        b_list = []
        
        # Stability constraint
        A_stab, b_stab = self.stability_constraint(state, dynamics)
        A_list.append(A_stab)
        b_list.append(b_stab)
        
        # Stack constraints
        A = np.vstack(A_list)
        b = np.hstack(b_list)
        
        # Solve QP
        prob = osqp.OSQP()
        prob.setup(P=P, q=q, A=sparse.csr_matrix(A), l=-np.inf*np.ones(len(b)), u=b, verbose=False)
        result = prob.solve()
        
        if result.info.status == 'solved':
            return result.x
        else:
            return np.zeros(n_actions)  # Fallback: stop
    
    def stability_constraint(self, state, dynamics):
        """Generate stability CBF constraint."""
        # Simplified - compute h(x) and constraints
        h = self.compute_stability_cbf(state)
        
        # Linearize dynamics
        A = np.eye(len(state))  # Simplified
        b = self.alpha * h
        
        return A[:1], np.array([b])
    
    def compute_stability_cbf(self, state):
        """Compute barrier function value."""
        # Example: ensure CoM height > threshold
        com_height = state[2] if len(state) > 2 else 0.3
        return com_height - 0.2  # h > 0 means safe
