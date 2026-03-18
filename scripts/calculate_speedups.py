import pandas as pd
import sys

def calculate_speedups(csv_path):
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    # Find the baseline (Binary Heap)
    baseline_mask = df['algorithm_description'].str.contains("IndexedHeap", case=False, na=False)
    if not baseline_mask.any():
        print("Error: Could not find baseline 'A* with IndexedHeap (D=2)' in the CSV.")
        return
    
    baseline = df[baseline_mask].iloc[0]
    baseline_total = baseline['total_time_ms']
    baseline_overhead = baseline['total_overhead_ms']

    results = []
    for _, row in df.iterrows():
        total_speedup = baseline_total / row['total_time_ms']
        overhead_speedup = baseline_overhead / row['total_overhead_ms']
        
        results.append({
            'Algorithm': row['algorithm_description'],
            'Total Speedup': f"{total_speedup:.2f}x",
            'Overhead Speedup': f"{overhead_speedup:.2f}x"
        })

    # Display results
    print(f"{'Algorithm':<60} | {'Total Speedup':<15} | {'Overhead Speedup':<15}")
    print("-" * 95)
    for res in results:
        print(f"{res['Algorithm']:<60} | {res['Total Speedup']:<15} | {res['Overhead Speedup']:<15}")

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "astar_means_random_grid.csv"
    calculate_speedups(path)
