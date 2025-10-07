# Live Heatmap Visualization for CPFA Grid System

This directory contains tools for visualizing the robot exploration grid as live heatmaps.

## Setup

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Usage

### Option 1: Live Heatmap (Recommended)
Run the live viewer that automatically updates as new CSV files are generated:

```bash
python3 live_heatmap_viewer.py
```

Optional parameters:
- `--pattern "heatmap_data/grid_heatmap_*.csv"` - Change the file pattern to monitor
- `--interval 2000` - Update interval in milliseconds

### Option 2: Static Heatmap
Generate a single heatmap from the latest CSV file:

```bash
python3 static_heatmap_generator.py
```

Or specify a specific file:
```bash
python3 static_heatmap_generator.py heatmap_data/grid_heatmap_120.csv
```

## How It Works

1. **C++ Simulation**: The CPFA simulation exports grid data to CSV files every 10 seconds in the `heatmap_data/` directory
2. **CSV Format**: Each file contains the visit count for each grid cell
3. **Python Visualization**: The Python scripts read these CSV files and create heatmaps

## Features

- **Live Updates**: Real-time visualization as the simulation runs
- **Color Coding**: Blue (unvisited) to Red (heavily visited)
- **Statistics**: Shows max visits, total visits, and coverage
- **Grid Overlay**: Visual grid lines for better cell identification
- **Metadata Display**: Shows simulation time, grid dimensions, and cell size

## File Organization

CSV files are stored in the `heatmap_data/` directory and named: `grid_heatmap_[simulation_time].csv`

For example: `heatmap_data/grid_heatmap_120.csv` contains the grid state at 120 seconds of simulation time.

## Directory Structure
```
Collision_Free_CPFA/
├── heatmap_data/           # Generated CSV files
│   ├── grid_heatmap_10.csv
│   ├── grid_heatmap_20.csv
│   └── ...
├── live_heatmap_viewer.py  # Live visualization
├── static_heatmap_generator.py # Static heatmap generation
└── requirements.txt        # Python dependencies
```