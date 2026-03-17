import pandas as pd
import glob

# 1. Setup paths and find all files matching the pattern
input_pattern = 'final_csv/korf_*.csv'
output_file = 'final_csv/compiled_csv/compiled_korf_data.csv'
files = glob.glob(input_pattern)

# List to hold the dataframes
data_frames = []

# 2. Process each file
for file in sorted(files):
    df = pd.read_csv(file)
    
    data_frames.append(df)

# 3. Combine all files into one
combined_df = pd.concat(data_frames, ignore_index=True)

# 4. Renumber the "instance_number" column
# This creates a unique ID for every unique set found in the combined data
# If you just want a sequence from 1 to N:
combined_df['instance_number'] = range(1, len(combined_df) + 1)

# 5. Save the result
combined_df.to_csv(output_file, index=False)

print(f"Successfully compiled {len(files)} files into {output_file}")