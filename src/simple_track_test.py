#!/usr/bin/env python3
"""
Simple track test without TUMFTM dependencies
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def load_track_simple(filepath):
    """Load track from CSV file"""
    # Read CSV
    data = pd.read_csv(filepath, comment='#')
    
    # Expected columns: x_m, y_m, w_tr_right_m, w_tr_left_m
    x = data.iloc[:, 0].values
    y = data.iloc[:, 1].values
    w_right = data.iloc[:, 2].values if data.shape[1] > 2 else np.ones_like(x) * 5.0
    w_left = data.iloc[:, 3].values if data.shape[1] > 3 else np.ones_like(x) * 5.0
    
    return np.column_stack([x, y, w_right, w_left])

def create_simple_raceline(track):
    """Create a simple racing line"""
    # For now, just use centerline
    return track[:, :2]

def visualize_track(track, raceline=None):
    """Visualize track and racing line"""
    plt.figure(figsize=(12, 8))
    
    # Plot centerline
    plt.plot(track[:, 0], track[:, 1], 'b--', alpha=0.5, label='Centerline')
    
    # Plot boundaries (simplified)
    # Just offset by track width for visualization
    angles = np.arctan2(np.gradient(track[:, 1]), np.gradient(track[:, 0]))
    
    # Right boundary
    x_right = track[:, 0] - track[:, 2] * np.sin(angles)
    y_right = track[:, 1] + track[:, 2] * np.cos(angles)
    
    # Left boundary  
    x_left = track[:, 0] + track[:, 3] * np.sin(angles)
    y_left = track[:, 1] - track[:, 3] * np.cos(angles)
    
    plt.plot(x_right, y_right, 'k-', linewidth=1)
    plt.plot(x_left, y_left, 'k-', linewidth=1, label='Track boundaries')
    
    # Plot racing line if provided
    if raceline is not None:
        plt.plot(raceline[:, 0], raceline[:, 1], 'r-', linewidth=2, label='Racing line')
    
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.title('Berlin 2018 Track')
    
    plt.savefig('/app/outputs/simple_track_test.png', dpi=150, bbox_inches='tight')
    print("Saved visualization to /app/outputs/simple_track_test.png")

if __name__ == "__main__":
    print("Simple track test")
    
    # Load track
    track = load_track_simple('/app/inputs/tracks/berlin_2018.csv')
    print(f"Loaded track with {len(track)} points")
    print(f"Track bounds: X=[{track[:,0].min():.1f}, {track[:,0].max():.1f}], Y=[{track[:,1].min():.1f}, {track[:,1].max():.1f}]")
    
    # Create simple racing line
    raceline = create_simple_raceline(track)
    
    # Visualize
    visualize_track(track, raceline)
    
    print("Done!")
