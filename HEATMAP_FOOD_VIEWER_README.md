# Live Heatmap and Food Viewer

This file provides a comprehensive live visualization tool that combines robot visit count heatmaps with food location data.

## Overview

The `live_heatmap_food_viewer.py` script is an enhanced version of the original heatmap viewer that also displays:
- **Robot visit count heatmap**: Shows how many times robots have visited each grid cell
- **Food locations**: Shows the positions of all food items with their current status (available/collected)
- **Nest location**: Shows the nest as a green circle
- **Real-time updates**: Automatically refreshes every 2 seconds with new data

## Features

### Visual Elements
- **Heatmap**: Color-coded grid showing robot visitation frequency (white = unvisited, red shades = visited)
- **Food items**: Yellow circles with black outlines showing food positions
- **Food radius**: Orange circles showing the collection radius around each food item
- **Nest**: Green circle at the origin (0,0) showing the nest location
- **Visit counts**: Numbers displayed in each grid cell showing exact visit counts
- **Grid lines**: Gray lines at 1-meter intervals for spatial reference

### Data Sources
The viewer monitors two types of CSV files:
1. **Heatmap data**: `heatmap_data/grid_heatmap_*.csv` (robot visit counts)
2. **Food data**: `food_data/food_locations_*.csv` (food positions and status)

### Information Display
- Current simulation time
- Grid dimensions and cell size
- Number of food items
- Visit statistics (max visits, total visits, visited cells)
- Food collection status

## Usage

### Basic Usage
```bash
python3 live_heatmap_food_viewer.py
```

### Advanced Options
```bash
# Custom file patterns
python3 live_heatmap_food_viewer.py --heatmap-pattern "custom_heatmap/*.csv" --food-pattern "custom_food/*.csv"

# Different update interval (in milliseconds)
python3 live_heatmap_food_viewer.py --interval 5000  # Update every 5 seconds
```

### Command Line Arguments
- `--heatmap-pattern`: Pattern for heatmap CSV files (default: `heatmap_data/grid_heatmap_*.csv`)
- `--food-pattern`: Pattern for food CSV files (default: `food_data/food_locations_*.csv`)
- `--interval`: Update interval in milliseconds (default: 2000)

## Data Export Configuration

The food location export functionality has been added to the CPFA loop functions:

### C++ Implementation
- **Function**: `exportFoodLocationsToCSV()` in `CPFA_loop_functions.cpp`
- **Export frequency**: Every 10 seconds (same as heatmap and dotplot data)
- **Directory**: `food_data/` (automatically created)
- **Filename format**: `food_locations_[simulation_time].csv`

### CSV Format
```csv
# Food Locations Export - Simulation Time: 10 seconds
# Total Food Items: 48
# Food Distribution: 0
# Food Radius: 0.05
# Random Seed: 789012
# === FOOD LOCATIONS DATA ===
X,Y,Status
-0.188175,2.80731,available
3.18499,-1.99865,collected
...
```

## Food Distribution Support

The viewer works with all food distribution types:

### Random Distribution (FoodDistribution = 0)
- Food items randomly placed throughout the arena
- Complete position information available from `FoodList`

### Clustered Distribution (FoodDistribution = 1)
- Food arranged in rectangular clusters
- Cluster center information included in CSV headers

### Power-law Distribution (FoodDistribution = 2)
- Hierarchical cluster structure
- Position data exported same as other distributions

## Requirements

- Python 3.x
- matplotlib
- pandas
- numpy
- Running CPFA simulation that exports heatmap and food data

## Integration with Simulation

To use this viewer:

1. **Run a simulation**: Start your CPFA simulation with the updated loop functions
2. **Wait for data**: The simulation will create `heatmap_data/` and `food_data/` directories
3. **Launch viewer**: Run the Python script to see live updates
4. **Monitor progress**: Watch as robots explore and collect food in real-time

## Notes

- The viewer automatically handles missing data files gracefully
- Food status (available/collected) is determined by comparing with the `CollectedFoodList`
- The display updates only when new data is available
- Arena coordinates range from -4 to +4 meters (8x8 arena)
- Grid resolution is determined by the simulation's grid cell size setting

## Troubleshooting

### No data appearing
- Check that the simulation is running and generating CSV files
- Verify that `heatmap_data/` and `food_data/` directories exist
- Ensure file patterns match the actual CSV filenames

### Performance issues
- Increase the update interval with `--interval` for slower updates
- Check available system memory if handling large datasets

### Visualization problems
- Ensure matplotlib backend supports interactive displays
- Try different Python environments if display issues occur