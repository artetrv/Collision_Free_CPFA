#!/usr/bin/env python3
"""
Static Heatmap Generator for CPFA Grid Data
Reads the latest CSV file and generates a single heatmap image
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import glob
import os
import sys

def find_latest_csv(pattern="heatmap_data/grid_heatmap_*.csv"):
    """Find the most recent CSV file matching the pattern"""
    files = glob.glob(pattern)
    if not files:
        return None
    
    # Sort by modification time, get the latest
    latest_file = max(files, key=os.path.getmtime)
    return latest_file

def load_grid_data(filename):
    """Load grid data from CSV file"""
    try:
        # Read the CSV file, skipping header comments
        df = pd.read_csv(filename, comment='#', index_col=0)
        
        # Convert to numpy array for heatmap
        grid_array = df.values.astype(float)
        
        # Extract metadata from comments
        metadata = {}
        with open(filename, 'r') as f:
            for line in f:
                if line.startswith('#'):
                    if 'Simulation Time:' in line:
                        metadata['sim_time'] = line.split(':')[1].strip().split()[0]
                    elif 'Grid Dimensions:' in line:
                        metadata['dimensions'] = line.split(':')[1].strip()
                    elif 'Cell Size:' in line:
                        metadata['cell_size'] = line.split(':')[1].strip().split()[0]
                else:
                    break
        
        return grid_array, metadata
        
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None, {}

def create_heatmap(grid_data, metadata, output_file=None):
    """Create and display/save heatmap"""
    
    # Create figure
    plt.figure(figsize=(12, 8))
    
    # Create custom colormap (blue to red)
    colors_list = ['darkblue', 'blue', 'lightblue', 'white', 'yellow', 'orange', 'red', 'darkred']
    cmap = colors.LinearSegmentedColormap.from_list('visit_count', colors_list)
    
    # Get grid dimensions
    height, width = grid_data.shape
    
    # For 8x8 arena: world extends from -4 to +4, grid lines at integers
    # Cell centers are at half-integers: -3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5
    arena_half_size = width // 2  # 4 for 8x8 grid
    
    # Create the heatmap with extent from -4 to +4
    im = plt.imshow(grid_data, cmap=cmap, interpolation='nearest', origin='lower',
                   extent=[-arena_half_size, arena_half_size,
                          -arena_half_size, arena_half_size])
    
    # Add colorbar
    cbar = plt.colorbar(im)
    cbar.set_label('Visit Count', rotation=270, labelpad=20)
    
    # Add titles and labels
    sim_time = metadata.get('sim_time', 'Unknown')
    dimensions = metadata.get('dimensions', 'Unknown')
    cell_size = metadata.get('cell_size', 'Unknown')
    
    plt.title(f'Robot Visit Count Heatmap\n'
             f'Time: {sim_time}s | Grid: {dimensions} | Cell: {cell_size}m', 
             fontsize=14)
    plt.xlabel('World X Coordinate (meters)', fontsize=12)
    plt.ylabel('World Y Coordinate (meters)', fontsize=12)
    
    # Grid lines at integer positions: -4, -3, -2, -1, 0, 1, 2, 3, 4
    grid_lines = np.arange(-arena_half_size, arena_half_size + 1, 1)
    
    # Draw grid lines
    for x in grid_lines:
        plt.axvline(x, color='gray', alpha=0.3, linewidth=0.5)
    for y in grid_lines:
        plt.axhline(y, color='gray', alpha=0.3, linewidth=0.5)
    
    # Set ticks at grid line positions
    plt.xticks(grid_lines)
    plt.yticks(grid_lines)
    
    # Add visit count text inside each cell
    for i in range(height):
        for j in range(width):
            visit_count = int(grid_data[i, j])
            # Calculate world coordinates for cell centers
            # The extent goes from -arena_half_size to +arena_half_size
            # Cell centers should be at the middle of each cell
            x_pos = -arena_half_size + (j + 0.5) * (2 * arena_half_size / width)  # Cell center X
            y_pos = -arena_half_size + (i + 0.5) * (2 * arena_half_size / height)  # Cell center Y
            
            # Choose text color based on visit count for better visibility
            text_color = 'white' if visit_count > np.max(grid_data) * 0.5 else 'black'
            
            plt.text(x_pos, y_pos, str(visit_count), 
                   ha='center', va='center', 
                   fontsize=10, fontweight='bold',
                   color=text_color)
    
    # Add statistics
    max_visits = np.max(grid_data)
    total_visits = np.sum(grid_data)
    visited_cells = np.count_nonzero(grid_data)
    total_cells = grid_data.size
    
    stats_text = f'Max visits: {max_visits:.0f}\n'
    stats_text += f'Total visits: {total_visits:.0f}\n'
    stats_text += f'Visited cells: {visited_cells}/{total_cells}'
    
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes,
            verticalalignment='top', bbox=dict(boxstyle='round', 
            facecolor='white', alpha=0.8), fontsize=10)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Heatmap saved to: {output_file}")
    else:
        plt.show()

def main():
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = find_latest_csv()
        
    if csv_file is None:
        print("No CSV files found matching pattern 'heatmap_data/grid_heatmap_*.csv'")
        print("Make sure the simulation is running and generating CSV files.")
        return
    
    print(f"Loading data from: {csv_file}")
    
    grid_data, metadata = load_grid_data(csv_file)
    
    if grid_data is None:
        print("Failed to load grid data")
        return
    
    print(f"Loaded grid: {grid_data.shape}")
    print(f"Simulation time: {metadata.get('sim_time', 'Unknown')} seconds")
    print(f"Max visit count: {np.max(grid_data)}")
    
    # Generate output filename based on input
    base_name = os.path.splitext(csv_file)[0]
    output_file = f"{base_name}_heatmap.png"
    
    create_heatmap(grid_data, metadata, output_file)

if __name__ == "__main__":
    main()