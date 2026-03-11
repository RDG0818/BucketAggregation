import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re
import argparse
import sys

def get_data_for_env(csv_path):
    """
    Reads CSV file, extracts data for heaps, and finds the alpha=1 and optimal alpha!=1.
    """
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading {csv_path}: {e}")
        sys.exit(1)
        
    df.columns = df.columns.str.strip()
    
    # Identify alpha=1
    alpha_1_mask = df['algorithm_description'].str.contains(r'a=1[,)]', regex=True)
    alpha_1_data = None
    if alpha_1_mask.any():
        alpha_1_data = df[alpha_1_mask].iloc[0]

    # Identify optimal alpha != 1
    others_mask = ~alpha_1_mask
    others_df = df[others_mask]
    
    optimal_alpha_data = None
    optimal_alpha_val = None
    if not others_df.empty:
        # Sort by total_overhead_ns to find the minimum
        optimal_alpha_data = others_df.loc[others_df['total_overhead_ns'].idxmin()]
        desc = optimal_alpha_data['algorithm_description']
        match = re.search(r'a=(\d+)', desc)
        if match:
            optimal_alpha_val = match.group(1)
            
    return {
        'Alpha=1': alpha_1_data,
        'Optimal Alpha': optimal_alpha_data,
        'Optimal Alpha Value': optimal_alpha_val
    }

def extract_plot_data(data):
    """ Extracts and converts time components to milliseconds. """
    if data is None:
        return {'enqueue': 0, 'dequeue': 0, 'change_key': 0, 'rebuild': 0}
    
    ns_to_ms = 1_000_000.0
    enqueue = data['time_enqueue_ns'] / ns_to_ms
    # Note: dequeue time in these CSVs often includes 'wasted_time_ns' (which is change_key time)
    # If wasted_time_ns is not explicitly separated in the CSV for some rows, it's 0.
    dequeue = (data['time_dequeue_ns'] - data.get('wasted_time_ns', 0)) / ns_to_ms
    change_key = data.get('wasted_time_ns', 0) / ns_to_ms
    rebuild = data.get('time_rebuild_ns', 0) / ns_to_ms
    return {'enqueue': enqueue, 'dequeue': dequeue, 'change_key': change_key, 'rebuild': rebuild}

def main():
    parser = argparse.ArgumentParser(description='Generate priority queue overhead graphs from CSV files comparing alpha=1 to optimal alpha.')
    parser.add_argument('inputs', nargs='+', help='Pairs of <csv_path> <title> (e.g., astar_grid.csv "Grid" pancake.csv "Pancake")')
    parser.add_argument('--output', default='alpha_comparison.png', help='Output filename (default: alpha_comparison.png)')
    args = parser.parse_args()

    if len(args.inputs) % 2 != 0:
        print("Error: Inputs must be provided in pairs of <csv_path> <title>")
        sys.exit(1)

    csv_inputs = []
    for i in range(0, len(args.inputs), 2):
        csv_inputs.append({'path': args.inputs[i], 'title': args.inputs[i+1]})

    # Set hatch line properties globally
    plt.rcParams['hatch.color'] = 'white'
    plt.rcParams['hatch.linewidth'] = 0.7

    plot_data = { 'Alpha=1': [], 'Optimal Alpha': [] }
    all_env_data = []
    
    for item in csv_inputs:
        env_data = get_data_for_env(item['path'])
        all_env_data.append(env_data)
        for alg_name in plot_data.keys():
            plot_data[alg_name].append(extract_plot_data(env_data[alg_name]))

    num_envs = len(csv_inputs)
    # Adjust figure width based on number of subplots
    fig, axes = plt.subplots(1, num_envs, figsize=(4 * num_envs, 7), sharey=False, squeeze=False)
    axes = axes[0] # Handle squeeze=False result

    algorithms = ['Alpha=1', 'Optimal Alpha']
    x_pos = np.arange(len(algorithms))

    components = ['enqueue', 'dequeue', 'change_key', 'rebuild']
    component_labels = ['Enqueue', 'Dequeue', 'Change Key', 'Reorder']
    
    colors = {'enqueue': '#E0E0E0', 'dequeue': '#A0A0A0', 'change_key': '#606060', 'rebuild': '#202020'}
    hatches = {'enqueue': '\\\\', 'dequeue': '/', 'change_key': '|', 'rebuild': ''}
    plot_colors = [colors[comp] for comp in components]
    plot_hatches = [hatches[comp] for comp in components]

    for i, item in enumerate(csv_inputs):
        ax = axes[i]
        env_name = item['title']
        ax.set_title(env_name, fontsize=16, weight='bold')
        
        for j, alg in enumerate(algorithms):
            bottom = 0
            for k, comp in enumerate(components):
                value = plot_data[alg][i][comp]
                ax.bar(j, value, bottom=bottom, color=plot_colors[k], hatch=plot_hatches[k],
                       edgecolor='black', linewidth=1)
                bottom += value
        
        labels = ['α=1', 'Optimal α']
        opt_alpha = all_env_data[i]['Optimal Alpha Value']
        if opt_alpha:
            labels[1] = f"Optimal α\n(α={opt_alpha})"

        ax.set_xticks(x_pos)
        ax.set_xticklabels(labels, fontsize=12)
        ax.grid(axis='y', linestyle='--', alpha=0.7)

    axes[0].set_ylabel('Priority Queue Overhead (ms)', fontsize=14)
    
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=colors[comp], edgecolor='black', hatch=hatches[comp], label=label) 
                       for comp, label in zip(components, component_labels)]
    
    # Place legend on the far right
    fig.legend(handles=legend_elements, fontsize=12, loc='upper right', bbox_to_anchor=(0.99, 0.98))

    fig.tight_layout(rect=[0, 0, 0.9, 1])
    plt.savefig(args.output)
    plt.close()

    print(f"Generated graph: {args.output}")

if __name__ == '__main__':
    main()
