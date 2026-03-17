import pandas as pd
import matplotlib.pyplot as plt

# 1. Load your compiled data
input_files = ['final_csv/compiled_csv/msa.csv']
for input_file in input_files:
  df = pd.read_csv(input_file)
  df['total_overhead_ms'] = df['total_overhead_ns'] / 1_000_000

  # 2. Define the columns we want to average
  time_columns = [
      'total_time_ms', 
      'time_enqueue_ns', 
      'time_dequeue_ns', 
      'time_decrease_key_ns',
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

  analysis_df.to_csv('final_csv/mean_csv/msa.csv', index=False)