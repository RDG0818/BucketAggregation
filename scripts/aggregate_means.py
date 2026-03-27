#!/usr/bin/env python3
"""
Compile a glob of CSVs into one file and compute per-algorithm means.

Usage:
    python aggregate_means.py <input_glob> <output_file>

Example:
    python aggregate_means.py 'results/*.csv' results/means.csv
"""

import sys
import glob
import pandas as pd


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input_glob> <output_file>")
        sys.exit(1)

    input_glob, output_file = sys.argv[1], sys.argv[2]

    files = sorted(glob.glob(input_glob))
    if not files:
        print(f"No files matched: {input_glob}")
        sys.exit(1)

    print(f"Found {len(files)} file(s)")
    combined = pd.concat([pd.read_csv(f) for f in files], ignore_index=True)

    numeric_cols = combined.select_dtypes(include="number").columns.tolist()
    means = (
        combined.groupby("algorithm_description")[numeric_cols]
        .mean()
        .reset_index()
    )

    means.to_csv(output_file, index=False)
    print(f"Wrote means for {len(means)} algorithm(s) to {output_file}")
    print(means[["algorithm_description"] + [c for c in numeric_cols if "time" in c or "nodes" in c]].to_string(index=False))


if __name__ == "__main__":
    main()
