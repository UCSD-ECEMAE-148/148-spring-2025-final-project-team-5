#!/usr/bin/env python3
"""
Minimal working example of the optimizer
This tests basic functionality without the full TUMFTM integration
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import yaml
import os

class MinimalOptimizer:
    def __init__(self, config_path="config/optimizer_config.yaml"):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        print("Minimal optimizer initialized")
        
    def create_test_track(self):
        """Create a simple oval test track"""
        # Create oval track
        theta = np.linspace(0, 2*np.pi, 100)
        x = 50 * np.cos(theta)
        y = 30 * np.sin(theta)
        
        # Track width
        width_right = np.ones_like(x) * 3.0
        width_left = np.ones_like(x) * 3.0
        
        return np.column_stack([x, y, width_right, width_left])
        
    def optimize_simple_path(self, track):
        """Simple centerline optimization"""
        # For now, just return the centerline
        return track[:, :2]
        
    def visualize(self, track, path):
        """Visualize track and path"""
        plt.figure(figsize=(10, 6))
        
        # Plot track boundaries
        x, y = track[:, 0], track[:, 1]
        normals = self._compute_normals(x, y)
        
        # Right boundary
        right_x = x - normals[:, 0] * track[:, 2]
        right_y = y - normals[:, 1] * track[:, 2]
        
        # Left boundary  
        left_x = x + normals[:, 0] * track[:, 3]
        left_y = y + normals[:, 1] * track[:, 3]
        
        plt.plot(right_x, right_y, 'k-', label='Track boundaries')
        plt.plot(left_x, left_y, 'k-')
        
        # Plot optimized path
        plt.plot(path[:, 0], path[:, 1], 'r-', linewidth=2, label='Optimized path')
        
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.title('Simple Track Optimization Test')
        
        # Save plot
        os.makedirs('outputs', exist_ok=True)
        plt.savefig('outputs/test_track.png')
        print("Saved visualization to outputs/test_track.png")
        
    def _compute_normals(self, x, y):
        """Compute normal vectors for the track"""
        dx = np.gradient(x)
        dy = np.gradient(y)
        
        # Normal vector (perpendicular to tangent)
        normals = np.column_stack([-dy, dx])
        
        # Normalize
        norms = np.sqrt(normals[:, 0]**2 + normals[:, 1]**2)
        normals = normals / norms[:, np.newaxis]
        
        return normals

if __name__ == "__main__":
    print("Running minimal optimizer test...")
    
    # Initialize
    optimizer = MinimalOptimizer("/app/config/optimizer_config.yaml")
    
    # Create test track
    track = optimizer.create_test_track()
    print(f"Created test track with {len(track)} points")
    
    # Optimize
    path = optimizer.optimize_simple_path(track)
    print("Optimization complete")
    
    # Visualize
    optimizer.visualize(track, path)
    
    print("Test complete!")
