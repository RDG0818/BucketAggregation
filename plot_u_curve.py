import pandas as pd
import matplotlib.pyplot as plt
import re
import os

def extract_alpha(desc):
    match = re.search(r'a=(\d+)', desc)
    if match:
        return int(match.group(1))
    return None

def plot_u_curve(csv_path, title, output_name):
    if not os.path.exists(csv_path):
        print(f"File {csv_path} not found.")
        return
    
    df = pd.read_csv(csv_path)
    
    # Filter and extract alpha
    df['alpha'] = df['algorithm_description'].apply(extract_alpha)
    df = df.dropna(subset=['alpha'])
    
    # Sort by alpha
    df = df.sort_values('alpha')
    
    # Metrics
    # Average PQ Overhead per Node (in microseconds for better scale)
    df['avg_pq_overhead_us'] = (df['total_overhead_ms'] * 1000) / df['nodes_expanded']
    # Total Nodes Expanded
    df['nodes'] = df['nodes_expanded']
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    # X-axis: Alpha
    x = [str(int(a)) for a in df['alpha']]
    
    # Left Y-Axis: PQ Overhead
    color_pq = 'tab:blue'
    ax1.set_xlabel('Aggregation Parameter (α)', fontsize=12)
    ax1.set_ylabel('Avg PQ Overhead per Node (μs)', color=color_pq, fontsize=12)
    line1 = ax1.plot(x, df['avg_pq_overhead_us'], color=color_pq, marker='o', label='PQ Overhead (μs/node)', linewidth=2)
    ax1.tick_params(axis='y', labelcolor=color_pq)
    ax1.grid(True, axis='x', linestyle='--', alpha=0.5)
    
    # Right Y-Axis: Node Count
    ax2 = ax1.twinx()
    color_nodes = 'tab:red'
    ax2.set_ylabel('Total Nodes Expanded', color=color_nodes, fontsize=12)
    line2 = ax2.plot(x, df['nodes'], color=color_nodes, marker='s', label='Nodes Expanded', linewidth=2)
    ax2.tick_params(axis='y', labelcolor=color_nodes)
    
    # Add Legend
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper center', frameon=True)
    
    plt.title(title, fontsize=14, pad=20)
    fig.tight_layout()
    
    plt.savefig(output_name, dpi=300)
    print(f"U-Curve plot saved to {output_name}")

def main():
    # Plot for Grid
    plot_u_curve('means_csv/means_astar_msa_data.csv', 
                 'Dual-Axis U-Curve: A* MSA Trade-off', 
                 'u_curve_grid.png')
    
    # Plot for Pancake
    plot_u_curve('means_csv/means_msa_data.csv', 
                 'Dual-Axis U-Curve: ANA* MSA Trade-off', 
                 'u_curve_pancake.png')

if __name__ == "__main__":
    main()
