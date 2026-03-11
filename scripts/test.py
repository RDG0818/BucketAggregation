import pandas as pd
import matplotlib.pyplot as plt

# 1. Load your compiled data
#input_files = ['csv/compiled_korf100_data.csv', 'csv/compiled_pancake40_data.csv', 'csv/compiled_grid_data.csv']
input_files = ['astar_csv/compiled_astar_korf_data.csv', 'astar_csv/compiled_astar_pancake_data.csv', 'astar_csv/compiled_astar_heavy_korf_data.csv', 'astar_csv/compiled_astar_heavy_pancake_data.csv', 'astar_csv/compiled_astar_grid_data.csv']
for input_file in input_files:
  df = pd.read_csv(input_file)
  df['total_overhead_ms'] = df['total_overhead_ns'] / 1_000_000

  # 2. Define the columns we want to average
  time_columns = [
      'total_time_ms', 
      'time_enqueue_ns', 
      'time_dequeue_ns', 
      'time_rebuild_ns', 
      'wasted_time_ns', 
      'total_overhead_ns',
      'total_overhead_ms'
  ]

  # 3. Group by the algorithm and calculate the mean
  # We use numeric_only=True just in case there are non-numeric bits hiding 
  analysis_df = df.groupby('algorithm_description')[time_columns].mean()

  # 4. Optional: Reset the index so 'algorithm_description' becomes a normal column
  analysis_df = analysis_df.reset_index()

  # 5. Save or Print the results
  print("--- Algorithm Performance Means ---")
  print(analysis_df)

  analysis_df.to_csv(input_file + '_algorithm_means_report.csv', index=False)