#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import sys
import os
from matplotlib.widgets import Button
import numpy as np

# Configuration: Arena size (n x n), plot limits will be -n/2 to n/2
ARENA_SIZE = 14

class TrajectoryViewer:
    def __init__(self, trajectory_files, arena_size=ARENA_SIZE):
        self.trajectory_files = trajectory_files
        self.arena_size = arena_size
        self.current_index = 0
        self.trajectories_per_page = 4  # Show 4 trajectories at once
        
        self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 12))
        self.axes = self.axes.flatten()
        
        # Add navigation buttons
        self.setup_buttons()
        self.update_display()
        
    def setup_buttons(self):
        # Create button axes
        ax_prev = plt.axes([0.1, 0.02, 0.1, 0.04])
        ax_next = plt.axes([0.25, 0.02, 0.1, 0.04])
        ax_overlay = plt.axes([0.4, 0.02, 0.15, 0.04])
        ax_single = plt.axes([0.6, 0.02, 0.15, 0.04])
        
        # Create buttons
        self.btn_prev = Button(ax_prev, 'Previous')
        self.btn_next = Button(ax_next, 'Next')
        self.btn_overlay = Button(ax_overlay, 'Show All Overlay')
        self.btn_single = Button(ax_single, 'Single View')
        
        # Connect button events
        self.btn_prev.on_clicked(self.prev_page)
        self.btn_next.on_clicked(self.next_page)
        self.btn_overlay.on_clicked(self.show_overlay)
        self.btn_single.on_clicked(self.show_single)
        
    def prev_page(self, event):
        if self.current_index > 0:
            self.current_index = max(0, self.current_index - self.trajectories_per_page)
            self.update_display()
            
    def next_page(self, event):
        if self.current_index + self.trajectories_per_page < len(self.trajectory_files):
            self.current_index += self.trajectories_per_page
            self.update_display()
            
    def show_overlay(self, event):
        self.plot_all_overlay()
        
    def show_single(self, event):
        self.plot_single_large()
        
    def update_display(self):
        # Clear all axes
        for ax in self.axes:
            ax.clear()
            
        # Plot current page of trajectories
        for i, ax in enumerate(self.axes):
            file_idx = self.current_index + i
            if file_idx < len(self.trajectory_files):
                filename = self.trajectory_files[file_idx]
                self.plot_trajectory_on_axis(ax, filename)
            else:
                ax.set_visible(False)
                
        # Update title
        start_idx = self.current_index + 1
        end_idx = min(self.current_index + self.trajectories_per_page, len(self.trajectory_files))
        self.fig.suptitle(f'Trajectories {start_idx}-{end_idx} of {len(self.trajectory_files)}')
        
        plt.tight_layout()
        plt.draw()
        
    def plot_trajectory_on_axis(self, ax, filename):
        filepath = os.path.join("trajectory_data", filename)
        if not os.path.exists(filepath):
            return
            
        df = pd.read_csv(filepath)
        
        # Find split point based on target position if available
        if 'target_x' in df.columns and 'target_y' in df.columns:
            target_x = df['target_x'].iloc[0]
            target_y = df['target_y'].iloc[0]
            
            # Find closest point to target
            distances = np.sqrt((df['x'] - target_x)**2 + (df['y'] - target_y)**2)
            split_point = distances.idxmin()
        else:
            # Fallback to midpoint if no target info
            split_point = len(df) // 2
        
        # Plot first part (to target) in blue, second part (from target) in red
        ax.plot(df['x'].iloc[:split_point+1], df['y'].iloc[:split_point+1], 'b-', linewidth=1.5, alpha=0.8, label='To target')
        ax.plot(df['x'].iloc[split_point:], df['y'].iloc[split_point:], 'r-', linewidth=1.5, alpha=0.8, label='From target')
        
        ax.plot(df['x'].iloc[0], df['y'].iloc[0], 'go', markersize=8, label='Start')
        ax.plot(df['x'].iloc[-1], df['y'].iloc[-1], 'ko', markersize=8, label='End')
        
        # Mark the target position if available
        if 'target_x' in df.columns and 'target_y' in df.columns:
            ax.plot(target_x, target_y, 'y*', markersize=12, label='Target')
        
        ax.set_title(f'{filename}\n{len(df)} points', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
        
        # Set fixed plot limits
        limit = self.arena_size / 2
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_aspect('equal')
        
    def plot_all_overlay(self):
        """Show all trajectories overlaid on a single plot"""
        fig, ax = plt.subplots(figsize=(12, 10))
        
        colors = plt.cm.tab20(np.linspace(0, 1, len(self.trajectory_files)))
        
        for i, filename in enumerate(self.trajectory_files):
            filepath = os.path.join("trajectory_data", filename)
            if os.path.exists(filepath):
                df = pd.read_csv(filepath)
                
                # Find split point based on target position if available
                if 'target_x' in df.columns and 'target_y' in df.columns:
                    target_x = df['target_x'].iloc[0]
                    target_y = df['target_y'].iloc[0]
                    
                    # Find closest point to target
                    distances = np.sqrt((df['x'] - target_x)**2 + (df['y'] - target_y)**2)
                    split_point = distances.idxmin()
                else:
                    split_point = len(df) // 2
                
                # Plot first part in one color, second part in another
                ax.plot(df['x'].iloc[:split_point+1], df['y'].iloc[:split_point+1], color=colors[i], linewidth=1, alpha=0.7, linestyle='-')
                ax.plot(df['x'].iloc[split_point:], df['y'].iloc[split_point:], color=colors[i], linewidth=1, alpha=0.4, linestyle='--')
                ax.plot(df['x'].iloc[0], df['y'].iloc[0], 'o', color=colors[i], markersize=4)
                
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title(f'All Trajectories Overlay ({len(self.trajectory_files)} trajectories)')
        ax.grid(True, alpha=0.3)
        
        # Set fixed plot limits
        limit = self.arena_size / 2
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_aspect('equal')
        
        # Add legend with scrollable option
        if len(self.trajectory_files) <= 20:
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=8)
        
        plt.tight_layout()
        plt.show()
        
    def plot_single_large(self):
        """Show one trajectory at a time in a large view"""
        SingleTrajectoryViewer(self.trajectory_files, self.arena_size)

class SingleTrajectoryViewer:
    def __init__(self, trajectory_files, arena_size):
        self.trajectory_files = trajectory_files
        self.arena_size = arena_size
        self.current_index = 0
        
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        
        # Add navigation buttons
        ax_prev = plt.axes([0.2, 0.02, 0.1, 0.04])
        ax_next = plt.axes([0.35, 0.02, 0.1, 0.04])
        ax_info = plt.axes([0.5, 0.02, 0.3, 0.04])
        
        self.btn_prev = Button(ax_prev, 'Previous')
        self.btn_next = Button(ax_next, 'Next')
        self.info_text = ax_info.text(0.5, 0.5, '', ha='center', va='center', transform=ax_info.transAxes)
        ax_info.set_xticks([])
        ax_info.set_yticks([])
        
        self.btn_prev.on_clicked(self.prev_trajectory)
        self.btn_next.on_clicked(self.next_trajectory)
        
        self.update_display()
        plt.show()
        
    def prev_trajectory(self, event):
        self.current_index = (self.current_index - 1) % len(self.trajectory_files)
        self.update_display()
        
    def next_trajectory(self, event):
        self.current_index = (self.current_index + 1) % len(self.trajectory_files)
        self.update_display()
        
    def update_display(self):
        self.ax.clear()
        
        filename = self.trajectory_files[self.current_index]
        filepath = os.path.join("trajectory_data", filename)
        
        if os.path.exists(filepath):
            df = pd.read_csv(filepath)
            
            # Find split point based on target position if available
            if 'target_x' in df.columns and 'target_y' in df.columns:
                target_x = df['target_x'].iloc[0]
                target_y = df['target_y'].iloc[0]
                
                # Find closest point to target
                distances = np.sqrt((df['x'] - target_x)**2 + (df['y'] - target_y)**2)
                split_point = distances.idxmin()
            else:
                split_point = len(df) // 2
            
            # Plot first part (to target) in blue, second part (from target) in red
            self.ax.plot(df['x'].iloc[:split_point+1], df['y'].iloc[:split_point+1], 'b-', linewidth=2, alpha=0.8, label='To target')
            self.ax.plot(df['x'].iloc[split_point:], df['y'].iloc[split_point:], 'r-', linewidth=2, alpha=0.8, label='From target')
            
            self.ax.plot(df['x'].iloc[0], df['y'].iloc[0], 'go', markersize=10, label='Start')
            self.ax.plot(df['x'].iloc[-1], df['y'].iloc[-1], 'ko', markersize=10, label='End')
            
            # Mark the target position if available
            if 'target_x' in df.columns and 'target_y' in df.columns:
                self.ax.plot(target_x, target_y, 'y*', markersize=15, label='Target')
            
            self.ax.set_xlabel('X Position')
            self.ax.set_ylabel('Y Position')
            self.ax.set_title(f'{filename}\n{len(df)} recorded positions')
            self.ax.grid(True, alpha=0.3)
            self.ax.legend()
            
            # Set fixed plot limits
            limit = self.arena_size / 2
            self.ax.set_xlim(-limit, limit)
            self.ax.set_ylim(-limit, limit)
            self.ax.set_aspect('equal')
            
        # Update info text
        self.info_text.set_text(f'Trajectory {self.current_index + 1} of {len(self.trajectory_files)}')
        
        plt.draw()

def plot_trajectory(filename, arena_size=ARENA_SIZE):
    """Plot a single trajectory file"""
    if not os.path.exists(filename):
        print(f"File {filename} does not exist!")
        return
    
    # Read the trajectory data
    df = pd.read_csv(filename)
    
    # Create the plot
    plt.figure(figsize=(10, 8))
    
    # Find split point based on target position if available
    if 'target_x' in df.columns and 'target_y' in df.columns:
        target_x = df['target_x'].iloc[0]
        target_y = df['target_y'].iloc[0]
        
        # Find closest point to target
        distances = np.sqrt((df['x'] - target_x)**2 + (df['y'] - target_y)**2)
        split_point = distances.idxmin()
    else:
        split_point = len(df) // 2
    
    # Plot first part (to target) in blue, second part (from target) in red
    plt.plot(df['x'].iloc[:split_point+1], df['y'].iloc[:split_point+1], 'b-', linewidth=2, alpha=0.8, label='To target')
    plt.plot(df['x'].iloc[split_point:], df['y'].iloc[split_point:], 'r-', linewidth=2, alpha=0.8, label='From target')
    
    plt.plot(df['x'].iloc[0], df['y'].iloc[0], 'go', markersize=10, label='Start')
    plt.plot(df['x'].iloc[-1], df['y'].iloc[-1], 'ko', markersize=10, label='End')
    
    # Mark the target position if available
    if 'target_x' in df.columns and 'target_y' in df.columns:
        plt.plot(target_x, target_y, 'y*', markersize=15, label='Target')
    
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title(f'Robot Trajectory: {os.path.basename(filename)}\n{len(df)} recorded positions')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Set fixed plot limits based on arena size
    limit = arena_size / 2
    plt.xlim(-limit, limit)
    plt.ylim(-limit, limit)
    plt.axis('equal')
    
    # Show the plot
    plt.tight_layout()
    plt.show()

def plot_all_trajectories(arena_size=ARENA_SIZE):
    """Launch interactive trajectory viewer"""
    trajectory_dir = "trajectory_data"
    if not os.path.exists(trajectory_dir):
        print("trajectory_data directory does not exist!")
        return
    
    csv_files = [f for f in os.listdir(trajectory_dir) if f.endswith('.csv')]
    if not csv_files:
        print("No trajectory files found!")
        return
    
    print(f"Found {len(csv_files)} trajectory files")
    print("Interactive controls:")
    print("- Previous/Next: Navigate through pages")
    print("- Show All Overlay: View all trajectories on one plot")
    print("- Single View: View one trajectory at a time in large format")
    
    viewer = TrajectoryViewer(sorted(csv_files), arena_size)
    plt.show()

def print_usage():
    """Print usage instructions"""
    print("Usage:")
    print("  python3 plot_trajectory.py [options] [trajectory_file]")
    print("")
    print("Options:")
    print("  --arena=N, -a=N    Set arena size to N (plot limits: -N/2 to N/2)")
    print("  --help, -h         Show this help message")
    print("")
    print("Examples:")
    print("  python3 plot_trajectory.py                           # Interactive viewer for all trajectories")
    print("  python3 plot_trajectory.py --arena=16                # Interactive viewer with 16x16 arena")
    print("  python3 plot_trajectory.py trajectory_data/F20_trajectory_155.0.csv  # Plot specific file")
    print("  python3 plot_trajectory.py -a=12 F20_trajectory_155.0.csv            # Plot specific file with custom arena size")
    print("")
    print(f"Default arena size: {ARENA_SIZE}")
    print("")
    print("Interactive Features:")
    print("- Navigate through trajectories with Previous/Next buttons")
    print("- View all trajectories overlaid on a single plot")
    print("- Single trajectory view for detailed examination")

if __name__ == "__main__":
    # Check for arena size argument
    arena_size = ARENA_SIZE
    filename_arg = None
    
    # Parse command line arguments
    for i, arg in enumerate(sys.argv[1:], 1):
        if arg in ['--help', '-h']:
            print_usage()
            sys.exit(0)
        elif arg.startswith('--arena=') or arg.startswith('-a='):
            arena_size = float(arg.split('=')[1])
        elif arg.startswith('--arena') or arg == '-a':
            if i + 1 < len(sys.argv):
                arena_size = float(sys.argv[i + 1])
        elif not arg.startswith('-') and filename_arg is None:
            filename_arg = arg
    
    print(f"Using arena size: {arena_size} (plot limits: {-arena_size/2} to {arena_size/2})")
    
    if filename_arg:
        # Plot specific file
        plot_trajectory(filename_arg, arena_size)
    else:
        # Launch interactive viewer
        plot_all_trajectories(arena_size)