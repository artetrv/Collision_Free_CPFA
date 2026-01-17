#!/usr/bin/env python3
"""
Resource Collection Milestone Analyzer

This script processes milestone data from multiple simulation runs and creates:
1. A CSV file with averaged milestone times
2. Bar charts showing the time to collect each 10% of resources

Usage: python3 analyze_resource_milestones.py
"""

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import glob

def load_milestone_data(milestone_dir="milestone_data"):
    """
    Load all milestone CSV files from the specified directory.
    
    Returns:
        DataFrame: Combined milestone data from all files
    """
    if not os.path.exists(milestone_dir):
        print(f"Error: Directory '{milestone_dir}' not found!")
        print("Make sure to run the batch simulation first to generate milestone data.")
        return None
    
    csv_files = glob.glob(os.path.join(milestone_dir, "*.csv"))
    
    if not csv_files:
        print(f"Error: No CSV files found in '{milestone_dir}' directory!")
        print("Make sure to run the batch simulation first to generate milestone data.")
        return None
    
    print(f"Found {len(csv_files)} milestone files to process...")
    
    all_data = []
    
    for csv_file in csv_files:
        try:
            # Read the CSV file, skipping comment lines that start with #
            df = pd.read_csv(csv_file, comment='#')
            
            # Extract metadata from filename (random seed)
            filename = os.path.basename(csv_file)
            if 'resource_milestones_' in filename:
                seed = filename.replace('resource_milestones_', '').replace('.csv', '')
                df['random_seed'] = seed
            
            # Extract metadata from comment lines
            with open(csv_file, 'r') as f:
                lines = f.readlines()
                for line in lines:
                    if line.startswith('# Food Distribution:'):
                        df['food_distribution'] = int(line.split(':')[1].strip())
                    elif line.startswith('# Total Food Items:'):
                        df['total_food'] = int(line.split(':')[1].strip())
                    elif line.startswith('# Algorithm Mode:'):
                        df['algorithm_mode'] = int(line.split(':')[1].strip())
                    elif line.startswith('# Number of Robots:'):
                        df['num_robots'] = int(line.split(':')[1].strip())
            
            all_data.append(df)
            
        except Exception as e:
            print(f"Error reading {csv_file}: {e}")
    
    if not all_data:
        print("Error: No valid milestone data could be loaded!")
        return None
    
    # Combine all data
    combined_df = pd.concat(all_data, ignore_index=True)
    print(f"Loaded milestone data from {len(csv_files)} simulation runs")
    print(f"Total milestones: {len(combined_df)}")
    
    return combined_df

def calculate_time_intervals(df):
    """
    Calculate the time intervals between milestones for each simulation run.
    
    Returns:
        DataFrame: Time intervals for each 10% increment
    """
    intervals_data = []
    
    # Group by simulation run (random_seed)
    for seed, group in df.groupby('random_seed'):
        # Sort by milestone percentage
        group = group.sort_values('milestone_percent')
        
        # Calculate time intervals
        prev_time = 0  # Start time is 0
        for _, row in group.iterrows():
            interval = row['time_seconds'] - prev_time
            intervals_data.append({
                'random_seed': seed,
                'milestone_percent': row['milestone_percent'],
                'time_interval': interval,
                'cumulative_time': row['time_seconds'],
                'food_distribution': row.get('food_distribution', 'Unknown'),
                'algorithm_mode': row.get('algorithm_mode', 'Unknown'),
                'num_robots': row.get('num_robots', 'Unknown'),
                'total_food': row.get('total_food', 'Unknown')
            })
            prev_time = row['time_seconds']
    
    return pd.DataFrame(intervals_data)

def create_summary_statistics(intervals_df):
    """
    Calculate summary statistics for time intervals.
    
    Returns:
        DataFrame: Summary statistics by milestone percentage
    """
    summary = intervals_df.groupby('milestone_percent')['time_interval'].agg([
        'count', 'mean', 'std', 'median', 'min', 'max'
    ]).round(3)
    
    summary.columns = ['num_simulations', 'mean_time', 'std_time', 'median_time', 'min_time', 'max_time']
    summary = summary.reset_index()
    
    return summary

def save_results_to_csv(summary_df, intervals_df, output_file="resource_collection_analysis.csv"):
    """
    Save the analysis results to CSV files.
    """
    # Save summary statistics
    summary_file = output_file.replace('.csv', '_summary.csv')
    summary_df.to_csv(summary_file, index=False)
    print(f"Summary statistics saved to: {summary_file}")
    
    # Save detailed intervals data
    details_file = output_file.replace('.csv', '_details.csv')
    intervals_df.to_csv(details_file, index=False)
    print(f"Detailed intervals data saved to: {details_file}")
    
    return summary_file, details_file

def create_bar_chart(summary_df, intervals_df, output_file="resource_collection_chart.png"):
    """
    Create bar charts showing time to collect each 10% of resources.
    """
    # Set up the plotting style
    plt.style.use('default')
    sns.set_palette("husl")
    
    # Create figure with subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Chart 1: Average time intervals with error bars
    milestones = summary_df['milestone_percent']
    means = summary_df['mean_time']
    stds = summary_df['std_time']
    
    bars1 = ax1.bar(milestones, means, yerr=stds, capsize=5, alpha=0.8, color='skyblue', edgecolor='navy')
    ax1.set_xlabel('Resource Collection Milestone (%)')
    ax1.set_ylabel('Average Time Interval (seconds)')
    ax1.set_title('Average Time to Collect Each 10% of Resources\n(Error bars show standard deviation)')
    ax1.grid(axis='y', alpha=0.3)
    
    # Add value labels on bars
    for bar, mean_val, std_val in zip(bars1, means, stds):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height + std_val + 1,
                f'{mean_val:.1f}s', ha='center', va='bottom', fontsize=9)
    
    # Chart 2: Box plot showing distribution of intervals
    milestone_data = []
    milestone_labels = []
    for milestone in sorted(intervals_df['milestone_percent'].unique()):
        data = intervals_df[intervals_df['milestone_percent'] == milestone]['time_interval']
        milestone_data.append(data)
        milestone_labels.append(f'{int(milestone)}%')
    
    box_plot = ax2.boxplot(milestone_data, labels=milestone_labels, patch_artist=True)
    
    # Color the boxes
    colors = plt.cm.Set3(np.linspace(0, 1, len(box_plot['boxes'])))
    for patch, color in zip(box_plot['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax2.set_xlabel('Resource Collection Milestone (%)')
    ax2.set_ylabel('Time Interval (seconds)')
    ax2.set_title('Distribution of Time Intervals Across All Simulation Runs')
    ax2.grid(axis='y', alpha=0.3)
    
    # Add metadata text
    if not intervals_df.empty:
        sample_row = intervals_df.iloc[0]
        metadata_text = f"""Simulation Parameters:
        Food Distribution: {sample_row.get('food_distribution', 'N/A')}
        Algorithm Mode: {sample_row.get('algorithm_mode', 'N/A')}
        Number of Robots: {sample_row.get('num_robots', 'N/A')}
        Total Food Items: {sample_row.get('total_food', 'N/A')}
        Number of Runs: {len(intervals_df['random_seed'].unique())}"""
        
        fig.text(0.02, 0.02, metadata_text, fontsize=9, verticalalignment='bottom',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.15)
    
    # Save the chart
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Bar chart saved to: {output_file}")
    
    # Also create a simple average bar chart
    simple_fig, simple_ax = plt.subplots(figsize=(10, 6))
    bars = simple_ax.bar(milestones, means, alpha=0.8, color='lightcoral', edgecolor='darkred')
    simple_ax.set_xlabel('Resource Collection Milestone (%)')
    simple_ax.set_ylabel('Average Time Interval (seconds)')
    simple_ax.set_title('Average Time to Collect Each 10% of Resources')
    simple_ax.grid(axis='y', alpha=0.3)
    
    # Add value labels on bars
    for bar, mean_val in zip(bars, means):
        height = bar.get_height()
        simple_ax.text(bar.get_x() + bar.get_width()/2., height + 0.5,
                      f'{mean_val:.1f}s', ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    simple_output = output_file.replace('.png', '_simple.png')
    plt.savefig(simple_output, dpi=300, bbox_inches='tight')
    print(f"Simple bar chart saved to: {simple_output}")
    
    plt.show()

def main():
    """
    Main function to run the milestone analysis.
    """
    print("Resource Collection Milestone Analyzer")
    print("=" * 50)
    
    # Load milestone data
    milestone_df = load_milestone_data()
    if milestone_df is None:
        return
    
    print(f"\nMilestone data loaded successfully!")
    print(f"Milestones found: {sorted(milestone_df['milestone_percent'].unique())}")
    print(f"Random seeds: {len(milestone_df['random_seed'].unique())} unique runs")
    
    # Calculate time intervals
    print("\nCalculating time intervals between milestones...")
    intervals_df = calculate_time_intervals(milestone_df)
    
    # Create summary statistics
    print("Creating summary statistics...")
    summary_df = create_summary_statistics(intervals_df)
    
    print("\nSummary Statistics:")
    print(summary_df)
    
    # Save results to CSV
    print("\nSaving results to CSV files...")
    save_results_to_csv(summary_df, intervals_df)
    
    # Create bar charts
    print("\nCreating bar charts...")
    create_bar_chart(summary_df, intervals_df)
    
    print("\n" + "=" * 50)
    print("Analysis complete!")
    print("\nFiles created:")
    print("- resource_collection_analysis_summary.csv (summary statistics)")
    print("- resource_collection_analysis_details.csv (detailed intervals)")
    print("- resource_collection_chart.png (detailed charts)")
    print("- resource_collection_chart_simple.png (simple bar chart)")

if __name__ == "__main__":
    main()