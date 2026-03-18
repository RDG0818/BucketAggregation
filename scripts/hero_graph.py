import pandas as pd
import matplotlib.pyplot as plt
import re
import os

def generate_hero_graph(file_path='data.csv'):
    """
    Generates a dual-axis 'Hero Graph' for academic publication.
    Visualizes the trade-off between queue dequeue speed and overall search bloat.
    """
    # 1. Load the data
    if not os.path.exists(file_path):
        print(f"Error: {file_path} not found in the current directory.")
        return

    df = pd.read_csv(file_path)

    # 2. Extract alpha (a) using regex from algorithm_description
    # Example: "A* with AggregatedTwoLevelBucketQueue (a=5, b=1)" -> 5
    extracted = df['algorithm_description'].str.extract(r'a=(\d+)')
    
    # Drop rows that don't match the regex (e.g., baseline algorithms like IndexedHeap)
    df['alpha'] = extracted[0]
    df = df.dropna(subset=['alpha'])
    df['alpha'] = df['alpha'].astype(int)

    # 3. Sort by alpha to ensure continuous line plotting
    df = df.sort_values('alpha')

    # 4. Set Academic Style
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, ax1 = plt.subplots(figsize=(10, 6))

    # --- Plotting Left Axis (ax1): Dequeue Speed ---
    # Blue dashed line with circular markers
    color_deq = '#0055AA' # Professional Navy Blue
    lns1 = ax1.plot(
        df['alpha'], 
        df['avg_time_dequeue_ns'], 
        color=color_deq, 
        linestyle='--', 
        marker='o', 
        markersize=6,
        label='Avg. Dequeue Time (ns)',
        alpha=0.8
    )
    
    ax1.set_xlabel('Aggregation Factor (α)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Average Dequeue Time (ns)', color=color_deq, fontsize=12, fontweight='bold')
    ax1.tick_params(axis='y', labelcolor=color_deq)
    
    # Ensure dequeue time starts at 0
    ax1.set_ylim(0, df['avg_time_dequeue_ns'].max() * 1.1)

    # --- Plotting Right Axis (ax2): Total Search Bloat ---
    # Thick solid red line with square markers
    ax2 = ax1.twinx()
    color_node = '#CC0000' # Professional Academic Red
    lns2 = ax2.plot(
        df['alpha'], 
        df['avg_time_per_node_ns'], 
        color=color_node, 
        linewidth=3, 
        marker='s', 
        markersize=6,
        label='Avg. Time Per Node (ns)'
    )
    
    ax2.set_ylabel('Average Time Per Node (ns)', color=color_node, fontsize=12, fontweight='bold')
    ax2.tick_params(axis='y', labelcolor=color_node)
    
    # Adjust ax2 limit to highlight the U-curve (Search Bloat trade-off)
    # We allow it to be slightly above 0 if the values are high, but start near 0
    ax2.set_ylim(df['avg_time_per_node_ns'].min() * 0.95, df['avg_time_per_node_ns'].max() * 1.05)

    # 5. Legend and Title
    plt.title('The Aggregation Trade-off: Queue Speed vs. Search Bloat', fontsize=14, fontweight='bold', pad=20)
    
    # Combine legends from both axes into a single consolidated box
    lns = lns1 + lns2
    labs = [l.get_label() for l in lns]
    ax1.legend(lns, labs, loc='upper center', frameon=True, shadow=True, fontsize=10, bbox_to_anchor=(0.5, 0.98))

    # Grid customization
    ax1.grid(True, which='both', linestyle=':', alpha=0.5)
    ax2.grid(False) # Prevent overlapping grid lines

    # Final Layout Adjustment
    plt.tight_layout()

    # Save high-resolution figure
    output_name = 'hero_graph.png'
    plt.savefig(output_name, dpi=300, bbox_inches='tight')
    print(f"Success: High-resolution plot saved as '{output_name}'")

if __name__ == "__main__":
    # Assumes data.csv is in the project root if run from root
    generate_hero_graph('means_korf_data_anastar.csv')
