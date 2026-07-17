#!/usr/bin/env python3
"""
generate_keepout_mask.py — Sinh keepout_mask.pgm + keepout_mask.yaml de nap
vao Nav2 KeepoutFilter, đánh dấu 8 vùng "để hàng hóa" (quanh ShelfF va cum
ShelfE/D) trong world warehouse_v6_double.world (32x48m, gấp đôi 2 trục).

TAI SAO CAN SCRIPT NAY (khong the tao san file .pgm trong lan chinh sua nay):
Toa do 8 vung keepout la co dinh (list KEEPOUT_ZONES ben duoi, khop chinh
xac voi 8 model "keepout_zone_*" trong warehouse_v6_double.world), NHUNG
resolution/origin/kich thuoc pixel THAT SU cua map chi co sau khi ban chay
SLAM tren world moi va luu bang map_saver_cli — slam_toolbox dung map_size
dang hinh vuong noi bo, map that xuat ra co the bi crop khac. Script nay doc
truc tiep file map.yaml THAT (--reference-map) de lay dung resolution/origin/
kich thuoc, dam bao mask can chinh xac tung pixel voi map, khong doan mo.

Cach dung (chay tren may Linux co Python, SAU KHI da co map.yaml that):
    cd ~/new_map
    python3 src/amr_navigation/scripts/generate_keepout_mask.py \\
        --reference-map maps/warehouse_v6_double_map.yaml \\
        --output maps/keepout_mask

Rồi launch voi:
    ros2 launch amr_bringup bringup.launch.py \\
        map:=$HOME/new_map/maps/warehouse_v6_double_map.yaml \\
        keepout_mask:=$HOME/new_map/maps/keepout_mask.yaml

Phu thuoc: pip3 install pillow pyyaml --break-system-packages (neu chua co)
"""
import argparse
import os

import yaml
from PIL import Image

# 8 vung keepout (center_x, center_y, width, height) — met, trong khung 'map'.
# Khop CHINH XAC voi 8 model "keepout_zone_*" trong warehouse_v6_double.world.
KEEPOUT_ZONES = [
    ("shelfF_Q1",  -13.795143, -12.956635, 2.5, 2.5),
    ("shelfF_Q2",  -13.795143,  11.043365, 2.5, 2.5),
    ("shelfF_Q3",    2.204857, -12.956635, 2.5, 2.5),
    ("shelfF_Q4",    2.204857,  11.043365, 2.5, 2.5),
    ("shelfED_Q1",   -3.25,    -16.05,     3.1, 11.3),
    ("shelfED_Q2",   -3.25,      7.95,     3.1, 11.3),
    ("shelfED_Q3",   12.75,    -16.05,     3.1, 11.3),
    ("shelfED_Q4",   12.75,      7.95,     3.1, 11.3),
]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--reference-map', required=True,
                   help="Path toi map.yaml THAT (da chay SLAM + map_saver_cli). "
                        "Script lay resolution/origin/width/height tu day.")
    p.add_argument('--output', default='keepout_mask',
                   help="Duong dan output KHONG co duoi (se tao .pgm + .yaml)")
    p.add_argument('--margin', type=float, default=0.0,
                   help="Cong them vao moi canh vung keepout (m), phong khi robot to.")
    return p.parse_args()


def main():
    args = parse_args()

    ref_dir = os.path.dirname(os.path.abspath(args.reference_map))
    with open(args.reference_map) as f:
        ref = yaml.safe_load(f)

    resolution = float(ref['resolution'])
    origin_x, origin_y = float(ref['origin'][0]), float(ref['origin'][1])
    image_name = ref['image']
    image_path = image_name if os.path.isabs(image_name) else os.path.join(ref_dir, image_name)

    with Image.open(image_path) as ref_img:
        width_px, height_px = ref_img.size

    print(f"Reference map: {args.reference_map}")
    print(f"  resolution={resolution}  origin=({origin_x},{origin_y})  "
          f"size={width_px}x{height_px}px")

    # Nen = FREE (trang, 254) -> khong cam. Vung keepout = OCCUPIED (den, 0).
    mask = Image.new('L', (width_px, height_px), color=254)
    pixels = mask.load()

    def world_to_px(x, y):
        col = int(round((x - origin_x) / resolution))
        # PGM: hang 0 = TREN anh = y LON NHAT cua map (map_server quy uoc)
        row = int(round((height_px - 1) - (y - origin_y) / resolution))
        return col, row

    for name, cx, cy, w, h in KEEPOUT_ZONES:
        w += 2 * args.margin
        h += 2 * args.margin
        x0, y0 = cx - w / 2, cy - h / 2
        x1, y1 = cx + w / 2, cy + h / 2
        col0, row1 = world_to_px(x0, y0)   # goc duoi-trai world -> row lon hon
        col1, row0 = world_to_px(x1, y1)   # goc tren-phai world -> row nho hon
        col0, col1 = sorted((max(0, col0), min(width_px - 1, col1)))
        row0, row1 = sorted((max(0, row0), min(height_px - 1, row1)))
        n_px = 0
        for row in range(row0, row1 + 1):
            for col in range(col0, col1 + 1):
                pixels[col, row] = 0
                n_px += 1
        print(f"  vung '{name}': world=({x0:.2f},{y0:.2f})-({x1:.2f},{y1:.2f})  "
              f"pixel=({col0},{row0})-({col1},{row1})  {n_px} px")

    pgm_path = args.output + '.pgm'
    yaml_path = args.output + '.yaml'
    mask.save(pgm_path)

    with open(yaml_path, 'w') as f:
        f.write(
            f"image: {os.path.basename(pgm_path)}\n"
            f"mode: trinary\n"
            f"resolution: {resolution}\n"
            f"origin: [{origin_x}, {origin_y}, 0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        )

    print(f"\nDa tao: {pgm_path}, {yaml_path}")
    print("Dung: ros2 launch amr_bringup bringup.launch.py "
          f"map:=<map.yaml> keepout_mask:={os.path.abspath(yaml_path)}")


if __name__ == '__main__':
    main()
