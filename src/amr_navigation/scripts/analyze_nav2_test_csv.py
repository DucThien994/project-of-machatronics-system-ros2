#!/usr/bin/env python3
"""
analyze_nav2_test_csv.py — Tinh mean/max/min tu file CSV do
nav2_random_pose_test.py tao ra.

Cach chay (khong can ROS2, chi can python3 thuan):

    python3 analyze_nav2_test_csv.py ~/nav2_test_results.csv

So sanh nhieu file (vi du: tolerance 0.05 vs 0.15) cung luc:

    python3 analyze_nav2_test_csv.py ~/nav2_test_tol_005.csv ~/nav2_test_tol_015.csv

Chi tinh tren cac dong co status == SUCCEEDED va co error_xy_m hop le,
dung cong thuc giong het ham _print_summary() trong nav2_random_pose_test.py.
"""
import argparse
import csv
from collections import Counter


def parse_args():
    p = argparse.ArgumentParser(
        description="Tinh mean/max/min sai so tu file CSV cua nav2_random_pose_test.py.")
    p.add_argument('csv_files', nargs='+', help="Duong dan (1 hoac nhieu) file .csv can phan tich.")
    return p.parse_args()


def analyze(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))

    status_counts = Counter(r['status'] for r in rows)
    n_total = len(rows)

    succeeded = [r for r in rows if r['status'] == 'SUCCEEDED' and r['error_xy_m'] != '']
    n_ok = len(succeeded)

    stats = {
        'path': path,
        'n_total': n_total,
        'n_ok': n_ok,
        'success_rate': (n_ok / n_total * 100.0) if n_total else 0.0,
        'status_counts': status_counts,
    }

    if n_ok > 0:
        err_xy = [float(r['error_xy_m']) for r in succeeded]
        err_yaw = [abs(float(r['error_yaw_deg'])) for r in succeeded]
        dur = [float(r['duration_sec']) for r in succeeded]

        stats.update({
            'mean_xy_m': sum(err_xy) / n_ok,
            'max_xy_m': max(err_xy),
            'min_xy_m': min(err_xy),
            'mean_yaw_deg': sum(err_yaw) / n_ok,
            'max_yaw_deg': max(err_yaw),
            'min_yaw_deg': min(err_yaw),
            'mean_duration_s': sum(dur) / n_ok,
            'max_duration_s': max(dur),
            'min_duration_s': min(dur),
        })
    return stats


def print_stats(stats):
    print(f"\n===== {stats['path']} =====")
    print(f"Tong so mau: {stats['n_total']}  |  SUCCEEDED: {stats['n_ok']} "
          f"({stats['success_rate']:.1f}%)")
    for status, count in stats['status_counts'].most_common():
        print(f"  - {status}: {count}")

    if stats['n_ok'] == 0:
        print("Khong co mau SUCCEEDED nao co du lieu sai so hop le.")
        return

    print(f"error_xy (m):   mean={stats['mean_xy_m']:.4f}  "
          f"max={stats['max_xy_m']:.4f}  min={stats['min_xy_m']:.4f}")
    print(f"error_yaw (deg,abs): mean={stats['mean_yaw_deg']:.2f}  "
          f"max={stats['max_yaw_deg']:.2f}  min={stats['min_yaw_deg']:.2f}")
    print(f"duration (s):   mean={stats['mean_duration_s']:.2f}  "
          f"max={stats['max_duration_s']:.2f}  min={stats['min_duration_s']:.2f}")


def print_comparison(all_stats):
    print("\n===== SO SANH =====")
    header = f"{'File':<40}{'Success%':>10}{'mean_xy(m)':>12}{'max_xy(m)':>11}{'mean_yaw(deg)':>15}"
    print(header)
    print('-' * len(header))
    for s in all_stats:
        name = s['path']
        mean_xy = f"{s['mean_xy_m']:.4f}" if s['n_ok'] else 'N/A'
        max_xy = f"{s['max_xy_m']:.4f}" if s['n_ok'] else 'N/A'
        mean_yaw = f"{s['mean_yaw_deg']:.2f}" if s['n_ok'] else 'N/A'
        print(f"{name:<40}{s['success_rate']:>9.1f}%{mean_xy:>12}{max_xy:>11}{mean_yaw:>15}")


def main():
    args = parse_args()
    all_stats = [analyze(path) for path in args.csv_files]

    for stats in all_stats:
        print_stats(stats)

    if len(all_stats) > 1:
        print_comparison(all_stats)


if __name__ == '__main__':
    main()
