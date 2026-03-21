import sys

# Hardcoded data extracted directly from your perf stat outputs
DATA = {
    "Grid 10k x 10k (ANA*)": {
        "Binary Heap": {"Time (s)": 137.23, "Page Faults": 10672, "Branch Misses": 1474039403, "LLC Misses": 120154272},
        "Unaggregated Bucket": {"Time (s)": 97.02, "Page Faults": 234324, "Branch Misses": 1629479396, "LLC Misses": 109365755},
        "Aggregated Bucket": {"Time (s)": 47.22, "Page Faults": 8427, "Branch Misses": 494041156, "LLC Misses": 40121016}
    },
    "MSA 5-Sequence (ANA*)": {
        "Binary Heap": {"Time (s)": 59.36, "Page Faults": 11621, "Branch Misses": 109553093, "LLC Misses": 1095032212},
        "Unaggregated Bucket": {"Time (s)": 60.60, "Page Faults": 1246011, "Branch Misses": 142518058, "LLC Misses": 1138129341},
        "Aggregated Bucket": {"Time (s)": 49.90, "Page Faults": 55760, "Branch Misses": 90971218, "LLC Misses": 948634528}
    },
    "Grid 10k x 10k (A*)": {
        "Binary Heap": {"Time (s)": 3.15, "Page Faults": 10803, "Branch Misses": 57039141, "LLC Misses": 8251568},
        "Unaggregated Bucket": {"Time (s)": 2.05, "Page Faults": 8868, "Branch Misses": 32673386, "LLC Misses": 1218323},
        "Aggregated Bucket": {"Time (s)": 2.01, "Page Faults": 6846, "Branch Misses": 27870323, "LLC Misses": 1202008}
    },
    "MSA 5-Sequence (A*)": {
        "Binary Heap": {"Time (s)": 41.84, "Page Faults": 11859, "Branch Misses": 104816089, "LLC Misses": 1030494267},
        "Unaggregated Bucket": {"Time (s)": 42.26, "Page Faults": 1392964, "Branch Misses": 107929752, "LLC Misses": 1036571478},
        "Aggregated Bucket": {"Time (s)": 36.12, "Page Faults": 36484, "Branch Misses": 70552960, "LLC Misses": 942214208}
    }
}

def calculate_reduction(baseline_val, new_val):
    """Calculates the percentage reduction (or increase if negative)."""
    if baseline_val == 0:
        return 0.0
    return ((baseline_val - new_val) / baseline_val) * 100

def print_markdown_table(domain, baseline_name):
    target_name = "Aggregated Bucket"
    domain_data = DATA[domain]
    
    baseline = domain_data[baseline_name]
    target = domain_data[target_name]
    metrics = ["Time (s)", "Page Faults", "Branch Misses", "LLC Misses"]
    
    print(f"\n### Hardware Profiling: {domain}")
    print(f"**Baseline:** {baseline_name} | **Target:** {target_name}\n")
    
    # Table Header
    print(f"| Metric | {baseline_name} | {target_name} | % Improvement |")
    print(f"|---|---|---|---|")
    
    # Table Rows
    for metric in metrics:
        base_val = baseline[metric]
        tgt_val = target[metric]
        reduction = calculate_reduction(base_val, tgt_val)
        
        # Format large numbers with commas, floats with 2 decimals
        if isinstance(base_val, float):
            base_str = f"{base_val:.2f}"
            tgt_str = f"{tgt_val:.2f}"
        else:
            base_str = f"{base_val:,}"
            tgt_str = f"{tgt_val:,}"
            
        # Format percentage (show + if it got worse, though ours all improved)
        sign = "+" if reduction < 0 else "-"
        pct_str = f"{sign}{abs(reduction):.1f}%"
        
        print(f"| **{metric}** | {base_str} | {tgt_str} | **{pct_str}** |")
    print("\n")

def main():
    print("--- Hardware Profiling Table Generator ---")
    
    # 1. Choose Environment
    domains = list(DATA.keys())
    print("\nSelect the Environment:")
    for i, d in enumerate(domains):
        print(f"[{i+1}] {d}")
        
    try:
        domain_idx = int(input("Choice (1-4): ")) - 1
        selected_domain = domains[domain_idx]
    except (ValueError, IndexError):
        print("Invalid choice. Exiting.")
        sys.exit(1)

    # 2. Choose Baseline
    print("\nSelect the Baseline to compare Aggregation against:")
    print("[1] Unaggregated Bucket Heap (To prove aggregation fixes the sparsity/memory scatter)")
    print("[2] Binary Heap (To prove total overall speedup/efficiency)")
    
    try:
        base_idx = int(input("Choice (1-2): "))
        selected_baseline = "Unaggregated Bucket" if base_idx == 1 else "Binary Heap"
    except ValueError:
        print("Invalid choice. Exiting.")
        sys.exit(1)

    # Generate the table
    print_markdown_table(selected_domain, selected_baseline)

if __name__ == "__main__":
    main()