#!/usr/bin/env python3
"""
csv_to_summary.py
─────────────────────────────────────────────────────────────────────────────
Đọc file CSV từ nav_data_collector → in bảng tổng kết thống kê.

Cách chạy:
  python3 tools/csv_to_summary.py ~/nav_experiment_data.csv
  python3 tools/csv_to_summary.py ~/nav_experiment_data.csv --latex   # in LaTeX table
"""

import argparse
import csv
import math
import os
import sys


def load_csv(path: str):
    with open(path, 'r') as f:
        return list(csv.DictReader(f))


def stats(values):
    if not values:
        return 0, 0, 0, 0
    n   = len(values)
    avg = sum(values) / n
    mn  = min(values)
    mx  = max(values)
    var = sum((v - avg)**2 for v in values) / n
    std = math.sqrt(var)
    return avg, std, mn, mx


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file', help='Path to CSV file')
    parser.add_argument('--latex', action='store_true', help='Print LaTeX table')
    args = parser.parse_args()

    if not os.path.isfile(args.csv_file):
        print(f'File không tồn tại: {args.csv_file}')
        sys.exit(1)

    rows = load_csv(args.csv_file)
    if not rows:
        print('CSV rỗng.')
        sys.exit(0)

    succeeded = [r for r in rows if r['result'] == 'SUCCEEDED']
    failed    = [r for r in rows if r['result'] != 'SUCCEEDED']

    ep = [float(r['error_pos_m'])  * 100 for r in succeeded]   # cm
    ey = [float(r['error_yaw_deg'])       for r in succeeded]   # deg
    dt = [float(r['duration_s'])          for r in succeeded]

    ep_avg, ep_std, ep_min, ep_max = stats(ep)
    ey_avg, ey_std, ey_min, ey_max = stats(ey)
    dt_avg, dt_std, dt_min, dt_max = stats(dt)

    # ── Bảng chi tiết ─────────────────────────────────────────────────────────
    print(f'\n{"="*80}')
    print(f'  Kết quả thực nghiệm — {args.csv_file}')
    print(f'{"="*80}')
    print(f'  Tổng số lần thử: {len(rows)}')
    print(f'  Thành công:      {len(succeeded)}')
    print(f'  Thất bại:        {len(failed)}')
    print()

    # Header
    hdr = f"{'#':>3} {'Goal(x,y,yaw°)':>22} {'Actual(x,y,yaw°)':>24} {'e_pos(cm)':>9} {'e_yaw(°)':>8} {'t(s)':>6} {'Result':>10}"
    print(hdr)
    print('-' * len(hdr))

    for r in rows:
        goal_str   = f"({float(r['goal_x_m']):5.2f},{float(r['goal_y_m']):5.2f},{float(r['goal_yaw_deg']):6.1f})"
        actual_str = f"({float(r['actual_x_m']):5.2f},{float(r['actual_y_m']):5.2f},{float(r['actual_yaw_deg']):6.1f})"
        ep_cm = float(r['error_pos_m']) * 100
        print(f"  {r['trial']:>2}  {goal_str:>22}  {actual_str:>24}  "
              f"{ep_cm:>8.1f}  {float(r['error_yaw_deg']):>7.2f}  "
              f"{float(r['duration_s']):>5.1f}  {r['result']:>10}")

    # ── Tổng kết ──────────────────────────────────────────────────────────────
    if succeeded:
        print(f'\n{"─"*60}')
        print(f'  Thống kê (chỉ các lần SUCCEEDED):')
        print(f'{"─"*60}')
        print(f'  Sai số vị trí:')
        print(f'    Trung bình:  {ep_avg:6.1f} cm')
        print(f'    Std dev:     {ep_std:6.1f} cm')
        print(f'    Min / Max:   {ep_min:6.1f} / {ep_max:6.1f} cm')
        print(f'  Sai số góc quay:')
        print(f'    Trung bình:  {ey_avg:6.2f}°')
        print(f'    Std dev:     {ey_std:6.2f}°')
        print(f'    Min / Max:   {ey_min:6.2f}° / {ey_max:6.2f}°')
        print(f'  Thời gian điều hướng:')
        print(f'    Trung bình:  {dt_avg:6.1f} s')
        print(f'    Min / Max:   {dt_min:6.1f} / {dt_max:6.1f} s')

    # ── LaTeX table ───────────────────────────────────────────────────────────
    if args.latex:
        print(f'\n{"─"*60}')
        print('  LaTeX table:')
        print('\\begin{table}[h]')
        print('\\centering')
        print('\\caption{Kết quả thực nghiệm điều hướng AMR}')
        print('\\begin{tabular}{|c|c|c|c|c|c|c|}')
        print('\\hline')
        print('Lần & Goal $(x,y,\\theta)$ & Thực tế $(x,y,\\theta)$ & '
              '$e_{pos}$ (cm) & $e_{\\theta}$ (°) & $t$ (s) & Kết quả \\\\')
        print('\\hline')
        for r in rows:
            gx, gy, gy_ = r['goal_x_m'], r['goal_y_m'], r['goal_yaw_deg']
            ax, ay, ay_ = r['actual_x_m'], r['actual_y_m'], r['actual_yaw_deg']
            ep_cm = float(r['error_pos_m']) * 100
            print(f"{r['trial']} & ({float(gx):.2f},{float(gy):.2f},{float(gy_):.1f}) & "
                  f"({float(ax):.2f},{float(ay):.2f},{float(ay_):.1f}) & "
                  f"{ep_cm:.1f} & {float(r['error_yaw_deg']):.2f} & "
                  f"{float(r['duration_s']):.1f} & {r['result']} \\\\")
        print('\\hline')
        if succeeded:
            print(f'\\multicolumn{{4}}{{|l|}}{{Trung bình}} & '
                  f'{ep_avg:.1f} & {ey_avg:.2f} & {dt_avg:.1f} & -- \\\\')
        print('\\hline')
        print('\\end{tabular}')
        print('\\end{table}')

    print()


if __name__ == '__main__':
    main()
