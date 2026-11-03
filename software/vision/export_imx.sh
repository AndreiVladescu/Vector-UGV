#!/usr/bin/env bash
# YOLO for the AI camera's IMX500, built on an x86 Linux PC (Python 3.12, Java 17+), packaged on the Pi.
#   software/vision/export_imx.sh                         # yolo11n-seg, 640x320 input, prototypes pooled 2x
#   MODEL=yolo11n IMGSZ=320 software/vision/export_imx.sh # detection only, 320x320
#   PI=pi@vector.local software/vision/export_imx.sh      # also package and install it on that Pi
# IMGSZ is "size" or "height,width", with the camera frame's shape (2:1: the band the camera service crops).
# PROTO_POOL shrinks the mask prototypes inside the network: the Pi reads every value of the
# output through Python, about 30 ms per 216k values, so 480x640 with full prototypes (626k)
# gives 6 results/s and with them pooled 2x (165k) the sensor's own rate.
# CALIB_IMAGES: INT8 calibration on that many COCO val2017 images (downloaded once, 780 MB);
# 0 = Ultralytics' coco128. The quantizer keeps about 60 MB of RAM per image: 200 peaks at
# 13.5 GB, 500 runs a 32 GB PC out of memory.
# Only YOLOv8n and YOLO11n export to the IMX500 (Ultralytics, 2026); YOLO26 doesn't yet. The tool
# chain pins clash with each other, so the versions below are the ones that work together.
# On the Pi the result is /usr/local/share/imx500-models/<model>-<size>.rpk with its labels next to
# it as .txt; set CAMERA_NET to that path in /etc/vector.env and restart vector-camera.
set -euo pipefail

model=${MODEL:-yolo11n-seg}
size=${IMGSZ:-320,640}
pool=${PROTO_POOL:-2}
calib=${CALIB_IMAGES:-200}
venv=${VENV:-$HOME/.cache/vector-imx-venv}
work=${WORK:-$HOME/.cache/vector-imx}

if [[ ! -x $venv/bin/python ]]; then
    python3 -m venv "$venv"
    "$venv/bin/pip" install -q "torch>=2.9,<2.12" torchvision --index-url https://download.pytorch.org/whl/cpu
    # the converter first: resolved together with the rest, pip gives up
    "$venv/bin/pip" install -q "imx500-converter[pt]==3.19.2"
    "$venv/bin/pip" install -q "ultralytics==8.4.198" "model-compression-toolkit>=2.4.1,<2.6" "edge-mdt-tpc>=1.2.0" \
        "mct-quantizers~=1.6.0" "protobuf==4.25.5" "tensorboard<2.19" "pydantic<2.12" "pulp<3"
    "$venv/bin/pip" check
fi

mkdir -p "$work"
cd "$work"
if (( calib > 0 )); then
    if [[ ! -d val2017 ]]; then
        curl -fL -o val2017.zip http://images.cocodataset.org/zips/val2017.zip
        unzip -q val2017.zip && rm val2017.zip
    fi
    rm -rf calib && mkdir -p calib/images/val
    imgs=("$work"/val2017/*.jpg)  # sorted by the glob; no head in a pipe, which pipefail would trip on
    ln -s "${imgs[@]:0:$calib}" calib/images/val/
fi
YOLO_AUTOINSTALL=false "$venv/bin/python" - "$model" "$size" "$pool" "$calib" <<'EOF'
import os
import sys
import torch.nn.functional as F
import ultralytics.utils.export.imx as imx
from ultralytics import YOLO
model, size, pool, calib = sys.argv[1], [int(v) for v in sys.argv[2].split(',')], int(sys.argv[3]), int(sys.argv[4])
seg_forward = imx.segment_forward
def pooled(self, x):  # looked up when the export traces the model
    *head, mc, proto = seg_forward(self, x)
    return *head, mc, F.avg_pool2d(proto, pool)
if pool > 1:
    imx.segment_forward = pooled
yolo = YOLO(f'{model}.pt')
if calib:  # images only: calibration needs no labels
    data = 'calib.yaml'
    with open(data, 'w') as f:
        f.write(f'path: {os.path.abspath("calib")}\ntrain: images/val\nval: images/val\nnames: {dict(yolo.names)}\n')
else:
    data = 'coco128-seg.yaml' if model.endswith('-seg') else 'coco128.yaml'
yolo.export(format='imx', imgsz=size[0] if len(size) == 1 else size, data=data)
EOF
out=$work/${model}_imx_model
echo "exported: $out/packerOut.zip, $out/labels.txt"

if [[ -n ${PI:-} ]]; then
    name=$model-${size/,/x}$([[ $pool -gt 1 && $model == *-seg ]] && echo -p$pool)
    scp -q "$out/packerOut.zip" "$PI:/tmp/$name.zip"
    scp -q "$out/labels.txt" "$PI:/tmp/$name.txt"
    ssh -t "$PI" "rm -rf /tmp/$name && imx500-package -i /tmp/$name.zip -o /tmp/$name \
        && sudo install -D -m 644 /tmp/$name/network.rpk /usr/local/share/imx500-models/$name.rpk \
        && sudo install -m 644 /tmp/$name.txt /usr/local/share/imx500-models/$name.txt \
        && echo installed: /usr/local/share/imx500-models/$name.rpk"
fi
