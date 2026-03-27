import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import re
import argparse
import sys

# Constants
NS_TO_MS = 1_000_000.0

def get_best_in_group(df, binary_pattern, alpha_1_pattern, opt_alpha_pattern):
    """
    Finds the Binary, Alpha=1, and Optimal Alpha rows in a dataframe.
    """
    # Binary
    binary_df = df[df['algorithm_description'].str.contains(binary_pattern, regex=True, na=False)]
    binary_data = binary_df.sort_values('total_overhead_ns').iloc[0] if not binary_df.empty else None

    # Alpha = 1
    # Matches a=1 specifically or the base BucketHeap
    a1_df = df[df['algorithm_description'].str.contains(alpha_1_pattern, regex=True, na=False)]
    alpha_1_data = a1_df.sort_values('total_overhead_ns').iloc[0] if not a1_df.empty else None

    # Optimal Alpha != 1
    # Matches a=X where X != 1
    opt_df = df[df['algorithm_description'].str.contains(opt_alpha_pattern, regex=True, na=False)]
    # Filter out a=1 from the opt candidates just in case
    opt_df = opt_df[~opt_df['algorithm_description'].str.contains(r'a=1[,)]', regex=True, na=False)]
    
    optimal_alpha_data = None
    optimal_alpha_val = None
    if not opt_df.empty:
        optimal_alpha_data = opt_df.loc[opt_df['total_overhead_ns'].idxmin()]
        match = re.search(r'a=(\d+)', optimal_alpha_data['algorithm_description'])
        if match:
            optimal_alpha_val = match.group(1)
            
    return {
        'Binary': binary_data,
        'Alpha=1': alpha_1_data,
        'Optimal Alpha': optimal_alpha_data,
        'Optimal Alpha Value': optimal_alpha_val
    }

def extract_plot_data(data):
    res = {'enqueue': 0, 'dequeue': 0, 'wasted': 0, 'rebuild': 0}
    if data is None: return res
    
    desc = data.get('algorithm_description', '')
    is_indexed = 'IndexedHeap' in desc

    res['enqueue'] = data.get('time_enqueue_ns', 0) / NS_TO_MS
    res['rebuild'] = data.get('time_rebuild_ns', 0) / NS_TO_MS

    if is_indexed:
        res['wasted'] = data.get('time_decrease_key_ns', 0) / NS_TO_MS
        res['dequeue'] = data.get('time_dequeue_ns', 0) / NS_TO_MS
    else:
        res['wasted'] = data.get('wasted_time_ns', 0) / NS_TO_MS
        res['dequeue'] = (data.get('time_dequeue_ns', 0) - data.get('wasted_time_ns', 0)) / NS_TO_MS
        
    return res

def plot_results(all_env_data, output_filename, is_astar=False):
    if not all_env_data:
        return

    num_envs = len(all_env_data)
    fig, axes = plt.subplots(1, num_envs, figsize=(5 * num_envs, 7), sharey=False, squeeze=False)
    axes = axes[0]

    algorithms = ['Binary', 'Alpha=1', 'Optimal Alpha']
    x_pos = np.arange(len(algorithms))
    
    components = ['enqueue', 'dequeue', 'wasted', 'rebuild']
    component_labels = ['Enqueue', 'Dequeue', 'Change Key', 'Reorder/Rebuild']

    if is_astar:
        components = ['enqueue', 'dequeue', 'wasted']
        component_labels = ['Enqueue', 'Dequeue', 'Change Key']

    colors = {'enqueue': '#E0E0E0', 'dequeue': '#A0A0A0', 'wasted': '#606060', 'rebuild': '#202020'}
    hatches = {'enqueue': '\\\\', 'dequeue': '/', 'wasted': '|', 'rebuild': ''}

    for i, env_entry in enumerate(all_env_data):
        ax = axes[i]
        env_name = env_entry['title']
        data = env_entry['data']
        ax.set_title(env_name, fontsize=19, weight='bold')
        
        for j, alg_key in enumerate(algorithms):
            p_data = extract_plot_data(data[alg_key])
            bottom = 0
            for comp in components:
                val = p_data[comp]
                if val > 0:
                    ax.bar(j, val, bottom=bottom, color=colors[comp], hatch=hatches[comp], edgecolor='black', linewidth=0.8)
                    bottom += val
        
        labels = ['Binary\nHeap', 'Bucket\nα=1', 'Bucket\nOpt α']
        if data['Optimal Alpha Value']:
            labels[2] = f"Bucket\nOpt α={data['Optimal Alpha Value']}"
        
        ax.set_xticks(x_pos)
        ax.set_xticklabels(labels, fontsize=14)
        ax.tick_params(axis='y', labelsize=12)
        ax.grid(axis='y', linestyle='--', alpha=0.5)
        if i == 0:
            ax.set_ylabel('Priority Queue Overhead (ms)', fontsize=16)

    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor=colors[comp], edgecolor='black', hatch=hatches[comp], label=label) 
                       for comp, label in zip(components, component_labels)]
    
    # Place legend in the upper right corner of the leftmost subplot
    axes[0].legend(handles=legend_elements, loc='upper right', 
                   fontsize=14, ncol=1, frameon=True, facecolor='white', framealpha=0.8)

    fig.tight_layout()
    plt.savefig(output_filename, dpi=300)
    plt.close()
    print(f"Generated: {output_filename}")

def main():
    parser = argparse.ArgumentParser(description='Generate PQ overhead graphs.')
    parser.add_argument('inputs', nargs='+', help='Pairs of <csv_path> <title>')
    parser.add_argument('--output-astar', default='astar_comparison.png', help='Output for A*')
    parser.add_argument('--output-anastar', default='anastar_comparison.png', help='Output for ANA*')
    args = parser.parse_args()

    if len(args.inputs) % 2 != 0:
        print("Error: Inputs must be pairs of <csv_path> <title>")
        sys.exit(1)

    # Group dataframes by title
    data_by_title = {}
    titles_order = []
    for i in range(0, len(args.inputs), 2):
        path = args.inputs[i]
        title = args.inputs[i+1]
        try:
            df = pd.read_csv(path)
            df.columns = df.columns.str.strip()
            if title not in data_by_title:
                data_by_title[title] = []
                titles_order.append(title)
            data_by_title[title].append(df)
        except Exception as e:
            print(f"Error processing {path}: {e}")

    astar_all = []
    anastar_all = []

    for title in titles_order:
        combined_df = pd.concat(data_by_title[title], ignore_index=True)
        
        # Extract A* Data
        astar_df = combined_df[combined_df['algorithm_description'].str.startswith('A*')]
        if not astar_df.empty:
            res = get_best_in_group(astar_df, 
                                    binary_pattern='IndexedHeap', 
                                    alpha_1_pattern=r'a=1[,)]|BucketHeap \(D=', 
                                    opt_alpha_pattern=r'a=\d+')
            astar_all.append({'title': title, 'data': res})

        # Extract ANA* Data
        anastar_df = combined_df[combined_df['algorithm_description'].str.contains('ANA|Anytime')]
        if not anastar_df.empty:
            res = get_best_in_group(anastar_df, 
                                    binary_pattern='IndexedHeap', 
                                    alpha_1_pattern=r'a=1[,)]|BucketHeap \(D=', 
                                    opt_alpha_pattern=r'a=\d+')
            anastar_all.append({'title': title, 'data': res})

    plot_results(astar_all, args.output_astar, is_astar=True)
    plot_results(anastar_all, args.output_anastar, is_astar=False)

if __name__ == '__main__':
    main()
