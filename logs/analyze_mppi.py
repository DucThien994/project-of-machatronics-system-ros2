#!/usr/bin/env python3
"""
Phân tích velocity log — Thí nghiệm 4.3
So sánh mức sử dụng vy giữa 2 kịch bản PreferForward ON/OFF
"""
import re, statistics, os

def parse_velocity_log(filepath):
    """Trả về list (vx, vy, wz) từ file log ros2 topic echo"""
    vx_list, vy_list, wz_list = [], [], []
    with open(filepath, 'r') as f:
        content = f.read()

    # Tìm từng block velocity
    blocks = content.split('---')
    for block in blocks:
        # Tìm linear.x, linear.y, angular.z
        vx_m = re.search(r'linear:\s*\n\s*x:\s*([-\d.e]+)', block)
        vy_m = re.search(r'linear:\s*\n\s*x:.*\n\s*y:\s*([-\d.e]+)', block)
        wz_m = re.search(r'angular:\s*\n\s*x:.*\n.*\n\s*z:\s*([-\d.e]+)', block)
        if vx_m and vy_m:
            vx_list.append(float(vx_m.group(1)))
            vy_list.append(float(vy_m.group(1)))
        if wz_m:
            wz_list.append(float(wz_m.group(1)))

    return vx_list, vy_list, wz_list

def summarize(label, vx_list, vy_list, wz_list):
    if not vx_list:
        print(f"  [!] Không có dữ liệu trong file {label}")
        return
    print(f"\n  [{label}]")
    print(f"  Số mẫu velocity   : {len(vx_list)}")
    print(f"  vx trung bình     : {statistics.mean(vx_list):+.4f} m/s")
    print(f"  |vy| trung bình   : {statistics.mean(abs(v) for v in vy_list):.4f} m/s")
    print(f"  |vy| tối đa       : {max(abs(v) for v in vy_list):.4f} m/s")
    print(f"  |wz| trung bình   : {statistics.mean(abs(v) for v in wz_list):.4f} rad/s")
    n_lateral = sum(1 for v in vy_list if abs(v) > 0.02)
    pct = n_lateral / len(vy_list) * 100 if vy_list else 0
    print(f"  Tỷ lệ có vy>0.02  : {pct:.1f}%  ({n_lateral}/{len(vy_list)} mẫu)")

LOG_DIR = os.path.expanduser('~/final-ros2/amr_ver6.0/logs')

print("=" * 60)
print("  PHÂN TÍCH MPPI VELOCITY — THÍ NGHIỆM 4.3")
print("=" * 60)

for fname, label in [('cmdvel_prefer_off.txt', 'PreferForward TẮT'),
                      ('cmdvel_prefer_on.txt',  'PreferForward BẬT')]:
    fpath = os.path.join(LOG_DIR, fname)
    if os.path.exists(fpath):
        vx, vy, wz = parse_velocity_log(fpath)
        summarize(label, vx, vy, wz)
    else:
        print(f"\n  [!] Chưa có file {fname}")

print("\n" + "=" * 60)