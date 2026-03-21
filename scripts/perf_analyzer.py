import re
import pandas as pd

def parse_perf_file(file_path):
    with open(file_path, 'r') as f:
        content = f.read()

    # Split by double newline to separate runs
    sections = re.split(r'\n\n(?=[A-Z])', content)
    
    results = []
    
    for section in sections:
        if not section.strip():
            continue
            
        title_match = re.match(r'^(.*?)\n', section)
        if not title_match:
            continue
        title = title_match.group(1).strip()
        
        # Regex for metrics
        metrics = {
            'cycles': r'([\d,]+)\s+cycles',
            'instructions': r'([\d,]+)\s+instructions',
            'branches': r'([\d,]+)\s+branches',
            'branch_misses': r'([\d,]+)\s+branch-misses',
            'l1_misses': r'([\d,]+)\s+L1-dcache-load-misses',
            'llc_misses': r'([\d,]+)\s+LLC-load-misses',
            'time': r'([\d.]+)\s+seconds time elapsed'
        }
        
        row = {'name': title}
        
        # Determine Domain and Algorithm Category
        if 'MSA' in title:
            row['domain'] = 'MSA'
        elif 'Grid' in title:
            row['domain'] = 'Grid'
        else:
            row['domain'] = 'Unknown'
            
        if 'ANA*' in title:
            row['algo_type'] = 'ANA*'
        else:
            row['algo_type'] = 'A*'
            
        # Determine Implementation
        if 'Aggregated' in title:
            row['impl'] = 'Aggregated Bucket'
        elif 'Bucket' in title:
            row['impl'] = 'Bucket'
        else:
            row['impl'] = 'Binary Heap'

        for key, pattern in metrics.items():
            match = re.search(pattern, section)
            if match:
                val = match.group(1).replace(',', '')
                row[key] = float(val)
            else:
                row[key] = None
                
        results.append(row)
        
    return pd.DataFrame(results)

def generate_analysis_table(df):
    # Calculate IPC
    df['IPC'] = df['instructions'] / df['cycles']
    
    # Analyze by Group (Domain + Algo Type)
    analysis_rows = []
    
    groups = df.groupby(['domain', 'algo_type'])
    
    for (domain, algo_type), group in groups:
        # Baseline is Binary Heap
        baseline = group[group['impl'] == 'Binary Heap']
        if baseline.empty:
            continue
        baseline = baseline.iloc[0]
        
        for _, row in group.iterrows():
            speedup = baseline['time'] / row['time']
            l1_reduction = (baseline['l1_misses'] - row['l1_misses']) / baseline['l1_misses'] * 100 if row['l1_misses'] and baseline['l1_misses'] else 0
            llc_reduction = (baseline['llc_misses'] - row['llc_misses']) / baseline['llc_misses'] * 100 if row['llc_misses'] and baseline['llc_misses'] else 0
            
            analysis_rows.append({
                'Domain': domain,
                'Search': algo_type,
                'Implementation': row['impl'],
                'Time (s)': f"{row['time']:.2f}",
                'Speedup': f"{speedup:.2f}x",
                'IPC': f"{row['IPC']:.2f}",
                'L1 Miss Red.': f"{l1_reduction:.1f}%",
                'LLC Miss Red.': f"{llc_reduction:.1f}%"
            })
            
    return pd.DataFrame(analysis_rows)

def main():
    df_raw = parse_perf_file('perf.txt')
    df_analysis = generate_analysis_table(df_raw)
    
    # Sort for better presentation
    df_analysis = df_analysis.sort_values(['Domain', 'Search', 'Implementation'], ascending=[True, True, True])
    
    print("\n### Performance Analysis Table (Bucket vs Binary Baseline)\n")
    print(df_analysis.to_markdown(index=False))
    
    # LaTeX format for the professor
    print("\n### LaTeX Table Snippet\n")
    print(df_analysis.to_latex(index=False))

if __name__ == "__main__":
    main()
