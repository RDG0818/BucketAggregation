import pandas as pd
import matplotlib.pyplot as plt
import re
import os

def extract_alpha(desc):
    match = re.search(r'a=(\d+)', desc)
    if match:
        return int(match.group(1))
    return None

def plot_frontier(csv_path, ax, title):
    if not os.path.exists(csv_path):
        print(f"File {csv_path} not found.")
        return
    
    df = pd.read_csv(csv_path)
    
    # Filter for entries with alpha (RealBucketHeap or AggregatedTwoLevelBucketQueue)
    df['alpha'] = df['algorithm_description'].apply(extract_alpha)
    df = df.dropna(subset=['alpha'])
    
    # Calculate X: Priority Queue Time per Node Expanded (ms/node)
    df['x'] = df['total_overhead_ms'] / df['nodes_expanded']
    # Y: Total Nodes Expanded
    df['y'] = df['nodes_expanded']
    
    # Sort by alpha for better visualization if needed, but scatter plot doesn't strictly need it
    df = df.sort_values('alpha')
    
    # Scatter plot
    scatter = ax.scatter(df['x'], df['y'], c=df['alpha'], cmap='viridis', s=100, edgecolors='black', alpha=0.8)
    
    # Annotate points with alpha values
    for i, row in df.iterrows():
        ax.annotate(f"α={int(row['alpha'])}", (row['x'], row['y']), 
                     textcoords="offset points", xytext=(5,5), ha='left', fontsize=9)
    
    ax.set_title(title, fontsize=12)
    ax.set_xlabel('Queue Time per Node (ms/node)', fontsize=10)
    ax.set_ylabel('Total Nodes Expanded', fontsize=10)
    ax.grid(True, linestyle='--', alpha=0.6)
    return scatter

def main():
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # ANA* Grid
    s1 = plot_frontier('means_csv/means_msa_data.csv', axes[0], 'Efficiency Frontier: ANA* MSA')
    
    # ANA* Pancake
    s2 = plot_frontier('means_csv/means_astar_msa_data.csv', axes[1], 'Efficiency Frontier: A* MSA')
    
    if s1 or s2:
        cbar = fig.colorbar(s1 or s2, ax=axes, orientation='vertical', fraction=0.02, pad=0.04)
        cbar.set_label('Aggregation (α)', fontsize=11)
    
    plt.tight_layout(rect=[0, 0, 0.95, 1])
    
    output_png = 'efficiency_frontier.png'
    plt.savefig(output_png, dpi=300)
    print(f"Graph saved to {output_png}")

if __name__ == "__main__":
    main()
