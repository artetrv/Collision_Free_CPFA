#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import sys
import os
from matplotlib.widgets import Button
import numpy as np

# Configuration: Arena size (n x n), plot limits will be -n/2 to n/2
ARENA_SIZE = 14

class TrajectoryViewer3Per:
    def __init__(self, trajectory_files, arena_size=ARENA_SIZE):
        self.trajectory_files = trajectory_files
        self.arena_size = arena_size
        self.current_index = 0
        self.trajectories_per_page = 3  # Show 3 trajectories on one plot
        
        self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 10))
        
        # Add navigation buttons
        self.setup_buttons()
        self.update_display()
        
    def setup_buttons(self):
        # Create button axes
        ax_prev = plt.axes([0.1, 0.02, 0.1, 0.04])
        ax_next = plt.axes([0.25, 0.02, 0.1, 0.04])
        ax_info = plt.axes([0.4, 0.02, 0.3, 0.04])
        ax_single = plt.axes([0.75, 0.02, 0.15, 0.04])
        
        # Create buttons
        self.btn_prev = Button(ax_prev, 'Previous')
        self.btn_next = Button(ax_next, 'Next')
        self.btn_single = Button(ax_single, 'Single View')
        
        # Info text area
        self.info_text = ax_info.text(0.5, 0.5, '', ha='center', va='center', transform=ax_info.transAxes, fontsize=10)
        ax_info.set_xticks([])
        ax_info.set_yticks([])
        
        # Connect button events
        self.btn_prev.on_clicked(self.prev_page)
        self.btn_next.on_clicked(self.next_page)
        self.btn_single.on_clicked(self.show_single)
        
    def prev_page(self, event):
        if self.current_index > 0:
            self.current_index = max(0, self.current_index - self.trajectories_per_page)
            self.update_display()
            
    def next_page(self, event):
        if self.current_index + self.trajectories_per_page < len(self.trajectory_files):
            self.current_index += self.trajectories_per_page
            self.update_display()
            
    def show_single(self, event):
        """Show one trajectory at a time in a large view"""
        SingleTrajectoryViewer(self.trajectory_files, self.arena_size)
        
    def update_display(self):
        # Clear the axis
        self.ax.clear()
        
        # Define colors for the 3 trajectories
        colors = ['blue', 'red', 'green']
        
        # Plot current page of trajectories (up to 3)
        trajectories_plotted = 0
        for i in range(self.trajectories_per_page):
            file_idx = self.current_index + i
            if file_idx < len(self.trajectory_files):
                filename = self.trajectory_files[file_idx]
                color = colors[i % len(colors)]
                self.plot_trajectory_on_axis(self.ax, filename, color, label_prefix=f"Traj {file_idx+1}")
                trajectories_plotted += 1
                
        # Set up the plot
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        
        # Update title and info
        start_idx = self.current_index + 1
        end_idx = min(self.current_index + trajectories_plotted, len(self.trajectory_files))
        self.ax.set_title(f'Robot Trajectories {start_idx}-{end_idx} of {len(self.trajectory_files)}', fontsize=14)
        
        # Set fixed plot limits
        limit = self.arena_size / 2
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect('equal')
        
        # Add grid lines that reflect the 14x14 arena structure
        self.ax.grid(True, alpha=0.3, linewidth=0.5)
        # Add major grid lines at integer coordinates
        self.ax.grid(True, which='major', alpha=0.6, linewidth=1.0)
        # Set ticks at integer positions to show the arena structure
        self.ax.set_xticks(range(int(-limit), int(limit) + 1, 1))
        self.ax.set_yticks(range(int(-limit), int(limit) + 1, 1))
        # Add minor ticks for finer resolution
        self.ax.set_xticks(np.arange(-limit, limit + 0.5, 0.5), minor=True)
        self.ax.set_yticks(np.arange(-limit, limit + 0.5, 0.5), minor=True)
        self.ax.grid(True, which='minor', alpha=0.2, linewidth=0.3)
        
        if trajectories_plotted > 0:
            self.ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            
        # Update info text
        self.info_text.set_text(f'Page {self.current_index//self.trajectories_per_page + 1} of {(len(self.trajectory_files)-1)//self.trajectories_per_page + 1}')
        
        plt.tight_layout()
        plt.draw()
        
    def plot_trajectory_on_axis(self, ax, filename, color, label_prefix=""):
        filepath = os.path.join("trajectory_data", filename)
        if not os.path.exists(filepath):
            return
            
        df = pd.read_csv(filepath)
        
        # Separate trajectory points and center points if available
        if 'point_type' in df.columns:
            traj_df = df[df['point_type'] == 0]
            center_df = df[df['point_type'] == 1]
        else:
            traj_df = df
            center_df = pd.DataFrame()
        
        if traj_df.empty:
            return

        # Find split point based on target position if available
        if 'target_x' in traj_df.columns and 'target_y' in traj_df.columns:
            target_x = traj_df['target_x'].iloc[0]
            target_y = traj_df['target_y'].iloc[0]
            
            # Find closest point to target
            distances = np.sqrt((traj_df['x'] - target_x)**2 + (traj_df['y'] - target_y)**2)
            split_point = distances.idxmin()
            # Adjust split_point to be relative to the start of traj_df
            # idxmin returns the index label, we need integer location for iloc
            # But since we filtered, indices might be non-contiguous.
            # Let's reset index for easier handling
            traj_df = traj_df.reset_index(drop=True)
            distances = np.sqrt((traj_df['x'] - target_x)**2 + (traj_df['y'] - target_y)**2)
            split_point = distances.idxmin()
        else:
            # Fallback to midpoint if no target info
            traj_df = traj_df.reset_index(drop=True)
            split_point = len(traj_df) // 2
        
        # Create distinct line styles for to/from target
        line_style_to = '-'
        line_style_from = '--'
        alpha_to = 0.8
        alpha_from = 0.6
        
        # Plot first part (to target) and second part (from target)
        ax.plot(traj_df['x'].iloc[:split_point+1], traj_df['y'].iloc[:split_point+1], 
                color=color, linestyle=line_style_to, linewidth=2, alpha=alpha_to, 
                label=f'{label_prefix} - To target')
        ax.plot(traj_df['x'].iloc[split_point:], traj_df['y'].iloc[split_point:], 
                color=color, linestyle=line_style_from, linewidth=2, alpha=alpha_from, 
                label=f'{label_prefix} - From target')
        
        # Mark start, end, and target with unique markers
        ax.plot(traj_df['x'].iloc[0], traj_df['y'].iloc[0], 'o', color=color, markersize=8, 
                markeredgecolor='black', markeredgewidth=1, label=f'{label_prefix} - Start')
        ax.plot(traj_df['x'].iloc[-1], traj_df['y'].iloc[-1], 's', color=color, markersize=8, 
                markeredgecolor='black', markeredgewidth=1, label=f'{label_prefix} - End')
        
        # Plot center points if available
        if not center_df.empty:
            ax.plot(center_df['x'], center_df['y'], 'D', color=color, markersize=6,
                    markeredgecolor='black', markeredgewidth=1, fillstyle='none', 
                    label=f'{label_prefix} - Centers')

        # Mark the target position if available
        if 'target_x' in traj_df.columns and 'target_y' in traj_df.columns:
            ax.plot(target_x, target_y, '*', color=color, markersize=12, 
                    markeredgecolor='black', markeredgewidth=1, label=f'{label_prefix} - Target')

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
            
            # Separate trajectory points and center points if available
            if 'point_type' in df.columns:
                traj_df = df[df['point_type'] == 0]
                center_df = df[df['point_type'] == 1]
            else:
                traj_df = df
                center_df = pd.DataFrame()
            
            if traj_df.empty:
                return

            # Find split point based on target position if available
            if 'target_x' in traj_df.columns and 'target_y' in traj_df.columns:
                target_x = traj_df['target_x'].iloc[0]
                target_y = traj_df['target_y'].iloc[0]
                
                # Find closest point to target
                # Reset index to ensure iloc works correctly
                traj_df = traj_df.reset_index(drop=True)
                distances = np.sqrt((traj_df['x'] - target_x)**2 + (traj_df['y'] - target_y)**2)
                split_point = distances.idxmin()
            else:
                traj_df = traj_df.reset_index(drop=True)
                split_point = len(traj_df) // 2
            
            # Plot first part (to target) in blue, second part (from target) in red
            self.ax.plot(traj_df['x'].iloc[:split_point+1], traj_df['y'].iloc[:split_point+1], 'b-', linewidth=2, alpha=0.8, label='To target')
            self.ax.plot(traj_df['x'].iloc[split_point:], traj_df['y'].iloc[split_point:], 'r-', linewidth=2, alpha=0.8, label='From target')
            
            self.ax.plot(traj_df['x'].iloc[0], traj_df['y'].iloc[0], 'go', markersize=10, label='Start')
            self.ax.plot(traj_df['x'].iloc[-1], traj_df['y'].iloc[-1], 'ko', markersize=10, label='End')
            
            # Plot center points if available
            if not center_df.empty:
                self.ax.plot(center_df['x'], center_df['y'], 'mD', markersize=8,
                        markeredgecolor='black', markeredgewidth=1, fillstyle='none', 
                        label='Spiral Centers')

            # Mark the target position if available
            if 'target_x' in traj_df.columns and 'target_y' in traj_df.columns:
                self.ax.plot(target_x, target_y, 'y*', markersize=15, label='Target')
            
            self.ax.set_xlabel('X Position')
            self.ax.set_ylabel('Y Position')
            self.ax.set_title(f'{filename}\n{len(traj_df)} recorded positions')
            
            # Add grid lines that reflect the arena structure
            self.ax.grid(True, alpha=0.3, linewidth=0.5)
            # Add major grid lines at integer coordinates
            self.ax.grid(True, which='major', alpha=0.6, linewidth=1.0)
            # Set ticks at integer positions to show the arena structure
            limit = self.arena_size / 2
            self.ax.set_xticks(range(int(-limit), int(limit) + 1, 1))
            self.ax.set_yticks(range(int(-limit), int(limit) + 1, 1))
            # Add minor ticks for finer resolution
            self.ax.set_xticks(np.arange(-limit, limit + 0.5, 0.5), minor=True)
            self.ax.set_yticks(np.arange(-limit, limit + 0.5, 0.5), minor=True)
            self.ax.grid(True, which='minor', alpha=0.2, linewidth=0.3)
            
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
    
    # Separate trajectory points and center points if available
    if 'point_type' in df.columns:
        traj_df = df[df['point_type'] == 0]
        center_df = df[df['point_type'] == 1]
    else:
        traj_df = df
        center_df = pd.DataFrame()
    
    if traj_df.empty:
        print("No trajectory points found in file")
        return

    # Create the plot
    plt.figure(figsize=(10, 8))
    
    # Find split point based on target position if available
    if 'target_x' in traj_df.columns and 'target_y' in traj_df.columns:
        target_x = traj_df['target_x'].iloc[0]
        target_y = traj_df['target_y'].iloc[0]
        
        # Find closest point to target
        traj_df = traj_df.reset_index(drop=True)
        distances = np.sqrt((traj_df['x'] - target_x)**2 + (traj_df['y'] - target_y)**2)
        split_point = distances.idxmin()
    else:
        traj_df = traj_df.reset_index(drop=True)
        split_point = len(traj_df) // 2
    
    # Plot first part (to target) in blue, second part (from target) in red
    plt.plot(traj_df['x'].iloc[:split_point+1], traj_df['y'].iloc[:split_point+1], 'b-', linewidth=2, alpha=0.8, label='To target')
    plt.plot(traj_df['x'].iloc[split_point:], traj_df['y'].iloc[split_point:], 'r-', linewidth=2, alpha=0.8, label='From target')
    
    plt.plot(traj_df['x'].iloc[0], traj_df['y'].iloc[0], 'go', markersize=10, label='Start')
    plt.plot(traj_df['x'].iloc[-1], traj_df['y'].iloc[-1], 'ko', markersize=10, label='End')
    
    # Plot center points if available
    if not center_df.empty:
        plt.plot(center_df['x'], center_df['y'], 'mD', markersize=8,
                markeredgecolor='black', markeredgewidth=1, fillstyle='none', 
                label='Spiral Centers')

    # Mark the target position if available
    if 'target_x' in traj_df.columns and 'target_y' in traj_df.columns:
        plt.plot(target_x, target_y, 'y*', markersize=15, label='Target')
    
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title(f'Robot Trajectory: {os.path.basename(filename)}\n{len(traj_df)} recorded positions')
    plt.legend()
    
    # Add grid lines that reflect the arena structure
    plt.grid(True, alpha=0.3, linewidth=0.5)
    # Add major grid lines at integer coordinates
    plt.grid(True, which='major', alpha=0.6, linewidth=1.0)
    # Set ticks at integer positions to show the arena structure
    limit = arena_size / 2
    plt.xticks(range(int(-limit), int(limit) + 1, 1))
    plt.yticks(range(int(-limit), int(limit) + 1, 1))
    # Add minor ticks for finer resolution
    plt.xticks(np.arange(-limit, limit + 0.5, 0.5), minor=True)
    plt.yticks(np.arange(-limit, limit + 0.5, 0.5), minor=True)
    plt.grid(True, which='minor', alpha=0.2, linewidth=0.3)
    
    # Set fixed plot limits based on arena size
    limit = arena_size / 2
    plt.xlim(-limit, limit)
    plt.ylim(-limit, limit)
    plt.axis('equal')
    
    # Show the plot
    plt.tight_layout()
    plt.show()

def plot_all_trajectories(arena_size=ARENA_SIZE):
    """Launch interactive trajectory viewer with 3 trajectories per plot"""
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
    print("- Previous/Next: Navigate through pages (3 trajectories per page)")
    print("- Single View: View one trajectory at a time in large format")
    print("- Each page shows 3 trajectories overlaid on the same plot")
    
    viewer = TrajectoryViewer3Per(sorted(csv_files), arena_size)
    plt.show()

def print_usage():
    """Print usage instructions"""
    print("Usage:")
    print("  python3 plot_trajectory_3per.py [options] [trajectory_file]")
    print("")
    print("Options:")
    print("  --arena=N, -a=N    Set arena size to N (plot limits: -N/2 to N/2)")
    print("  --help, -h         Show this help message")
    print("")
    print("Examples:")
    print("  python3 plot_trajectory_3per.py                      # Interactive viewer (3 trajectories per plot)")
    print("  python3 plot_trajectory_3per.py --arena=16           # Interactive viewer with 16x16 arena")
    print("  python3 plot_trajectory_3per.py trajectory_data/F20_trajectory_155.0.csv  # Plot specific file")
    print("  python3 plot_trajectory_3per.py -a=12 F20_trajectory_155.0.csv            # Plot specific file with custom arena size")
    print("")
    print(f"Default arena size: {ARENA_SIZE}")
    print("")
    print("Features:")
    print("- 3 trajectories displayed on each plot page")
    print("- Different colors for each trajectory (blue, red, green)")
    print("- Solid lines for 'to target', dashed lines for 'from target'")
    print("- Unique markers for start (circle), end (square), target (star)")
    print("- Navigate through trajectory sets with Previous/Next buttons")
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