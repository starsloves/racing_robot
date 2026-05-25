#!/usr/bin/env python3
"""Visualize trajectory from CSV log file."""

import argparse
import csv
import os
import sys

import matplotlib.pyplot as plt
import numpy as np


def load_trajectory(csv_path):
    """Load trajectory data from CSV file."""
    data = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append(row)
    
    # Check format (sim vs real)
    fieldnames = data[0].keys() if data else []
    is_sim_format = 'phase' in fieldnames and 'segment' not in fieldnames
    
    # Convert to numpy arrays
    trajectory = {
        'time': np.array([float(row['time']) for row in data]),
        'x': np.array([float(row['x']) for row in data]),
        'y': np.array([float(row['y']) for row in data]),
        'yaw_deg': np.array([float(row['yaw_deg']) for row in data]),
        'linear_x': np.array([float(row['linear_x']) for row in data]),
        'angular_z': np.array([float(row['angular_z']) for row in data]),
    }
    
    if is_sim_format:
        # Sim format
        trajectory['segment'] = ['sim'] * len(data)
        trajectory['segment_type'] = ['move'] * len(data)
        trajectory['detour_phase'] = [row.get('phase', 'none') for row in data]
        trajectory['avoidance_active'] = [row.get('phase', 'none') not in ('none', 'follow') for row in data]
    else:
        # Real format
        trajectory['segment'] = [row.get('segment', 'none') for row in data]
        trajectory['segment_type'] = [row.get('segment_type', 'none') for row in data]
        trajectory['detour_phase'] = [row.get('detour_phase', 'none') for row in data]
        if 'avoidance_active' in fieldnames:
            trajectory['avoidance_active'] = [row.get('avoidance_active', 'False') == 'True' for row in data]
        else:
            trajectory['avoidance_active'] = [
                row.get('detour_phase', 'none') not in ('none', 'follow')
                for row in data
            ]
    
    trajectory['nearest_obstacle'] = np.array([
        float(row.get('nearest_obstacle', 'inf')) if row.get('nearest_obstacle', 'inf') != 'inf' else np.inf
        for row in data
    ])
    trajectory['front_obstacle'] = np.array([
        float(row.get('front_obstacle', 'inf')) if row.get('front_obstacle', 'inf') != 'inf' else np.inf
        for row in data
    ])
    
    return trajectory


def plot_trajectory(trajectory, output_path=None):
    """Plot trajectory with obstacle information."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Robot Trajectory with Obstacle Avoidance', fontsize=16)
    
    # Plot 1: XY trajectory
    ax1 = axes[0, 0]
    ax1.plot(trajectory['x'], trajectory['y'], 'b-', linewidth=2, label='Trajectory')
    ax1.plot(trajectory['x'][0], trajectory['y'][0], 'go', markersize=10, label='Start')
    ax1.plot(trajectory['x'][-1], trajectory['y'][-1], 'ro', markersize=10, label='End')
    
    # Color points by detour phase
    detour_colors = {'none': 'blue', 'out': 'orange', 'pass': 'yellow', 'rejoin': 'green'}
    for phase in detour_colors:
        mask = np.array([p == phase for p in trajectory['detour_phase']])
        if np.any(mask):
            ax1.scatter(trajectory['x'][mask], trajectory['y'][mask], 
                       c=detour_colors[phase], s=20, alpha=0.6, label=f'Detour: {phase}')
    
    # Color points by corridor avoidance
    avoid_mask = trajectory['avoidance_active']
    if np.any(avoid_mask):
        ax1.scatter(trajectory['x'][avoid_mask], trajectory['y'][avoid_mask],
                   c='red', s=30, marker='x', alpha=0.8, label='Avoidance active')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('XY Trajectory')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # Plot 2: Velocity over time
    ax2 = axes[0, 1]
    ax2.plot(trajectory['time'], trajectory['linear_x'], 'b-', label='Linear X')
    ax2.plot(trajectory['time'], trajectory['angular_z'], 'r-', label='Angular Z')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity')
    ax2.set_title('Velocity over Time')
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Obstacle distances over time
    ax3 = axes[1, 0]
    valid_nearest = trajectory['nearest_obstacle'] < np.inf
    valid_front = trajectory['front_obstacle'] < np.inf
    
    if np.any(valid_nearest):
        ax3.plot(trajectory['time'][valid_nearest], trajectory['nearest_obstacle'][valid_nearest],
                'b-', label='Nearest Obstacle')
    if np.any(valid_front):
        ax3.plot(trajectory['time'][valid_front], trajectory['front_obstacle'][valid_front],
                'r-', label='Front Obstacle')
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Distance (m)')
    ax3.set_title('Obstacle Distances over Time')
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: Yaw over time
    ax4 = axes[1, 1]
    ax4.plot(trajectory['time'], trajectory['yaw_deg'], 'g-')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Yaw (deg)')
    ax4.set_title('Yaw over Time')
    ax4.grid(True)
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Trajectory plot saved to: {output_path}')
    else:
        plt.show()
    
    plt.close()


def print_statistics(trajectory):
    """Print trajectory statistics."""
    print('\n=== Trajectory Statistics ===')
    print(f'Total duration: {trajectory["time"][-1] - trajectory["time"][0]:.2f} s')
    print(f'Total distance: {np.sum(np.sqrt(np.diff(trajectory["x"])**2 + np.diff(trajectory["y"])**2)):.2f} m')
    print(f'Average linear speed: {np.mean(np.abs(trajectory["linear_x"])):.3f} m/s')
    print(f'Max linear speed: {np.max(np.abs(trajectory["linear_x"])):.3f} m/s')
    print(f'Average angular speed: {np.mean(np.abs(trajectory["angular_z"])):.3f} rad/s')
    print(f'Max angular speed: {np.max(np.abs(trajectory["angular_z"])):.3f} rad/s')
    
    # Count detour phases
    phase_counts = {}
    for phase in trajectory['detour_phase']:
        phase_counts[phase] = phase_counts.get(phase, 0) + 1
    print(f'\nDetour phase distribution:')
    for phase, count in phase_counts.items():
        print(f'  {phase}: {count} points ({count/len(trajectory["detour_phase"])*100:.1f}%)')
    
    # Count avoidance active samples
    avoid_count = np.sum(trajectory['avoidance_active'])
    print(
        f'\nAvoidance active: {avoid_count} points '
        f'({avoid_count/len(trajectory["avoidance_active"])*100:.1f}%)'
    )
    
    # Obstacle statistics
    valid_nearest = trajectory['nearest_obstacle'] < np.inf
    if np.any(valid_nearest):
        print(f'\nNearest obstacle distance:')
        print(f'  Min: {np.min(trajectory["nearest_obstacle"][valid_nearest]):.3f} m')
        print(f'  Mean: {np.mean(trajectory["nearest_obstacle"][valid_nearest]):.3f} m')
        print(f'  Max: {np.max(trajectory["nearest_obstacle"][valid_nearest]):.3f} m')


def main():
    parser = argparse.ArgumentParser(description='Visualize trajectory from CSV log')
    parser.add_argument('csv_path', help='Path to trajectory CSV file')
    parser.add_argument('--output', '-o', help='Output image path (default: show interactively)')
    args = parser.parse_args()
    
    if not os.path.exists(args.csv_path):
        print(f'Error: CSV file not found: {args.csv_path}')
        sys.exit(1)
    
    print(f'Loading trajectory from: {args.csv_path}')
    trajectory = load_trajectory(args.csv_path)
    print(f'Loaded {len(trajectory["time"])} trajectory points')
    
    print_statistics(trajectory)
    plot_trajectory(trajectory, args.output)


if __name__ == '__main__':
    main()
