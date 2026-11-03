#!/usr/bin/env python3
"""The Pi AI Camera with a network running on the IMX500, in place of rpicam-vid.

  imx500_camera.py NETWORK [--width 1280] [--height 960] [--fps 15] [--port 9000] [--dets 127.0.0.1:9001]

NETWORK is an .rpk, or a name from /usr/share/imx500-models (ssd_mobilenetv2_fpnlite_320x320_pp).
Two kinds work: SSD-style detectors with post-processing (_pp), and Ultralytics YOLO exports
(format=imx, packaged with imx500-package), detection or segmentation, with their labels.txt next
to the .rpk as <name>.txt.

Serves raw I420 frames on tcp://127.0.0.1:PORT to one reader at a time, like
`rpicam-vid --codec yuv420 --listen`, and before each frame sends what the sensor found in it as
one UDP datagram of JSON:
  {"seq": n, "w": W, "h": H, "new": bool, "dets": [[x1, y1, x2, y2, score, label], ...],
   "masks": {"rect": [x, y, w, h], "shape": [mh, mw], "objs": [[cx1, cy1, cx2, cy2, base64], ...]}}
in frame pixels. "masks" (segmentation only) holds each object's soft mask (probability x 255,
uint8) on the network's prototype grid (mh x mw over rect in the frame), cropped to the cells
around its box, in the order of dets: the reader scales it up and thresholds it at full size, so
the outline stays smooth. Masks stop where the datagram would pass MAX_MASK_BYTES; those objects
keep their boxes. The Pi's CPU only copies frames and works out the masks. Loading a network onto the sensor takes
about 35 s the first time, a few seconds after that.
"""
import argparse
import base64
import json
import os
import socket
import threading

import numpy as np
from picamera2 import Picamera2
from picamera2.devices import IMX500

MODELS = '/usr/share/imx500-models'
MAX_MASK_BYTES = 48000  # base64 of all masks: a UDP datagram holds 65507 bytes


class Server(threading.Thread):
    """One TCP reader at a time; a new one replaces the old."""

    def __init__(self, port):
        super().__init__(daemon=True)
        self.srv = socket.create_server(('127.0.0.1', port), reuse_port=True)
        self.client = None

    def run(self):
        while True:
            c, _ = self.srv.accept()
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            c.settimeout(1.0)
            old, self.client = self.client, c
            if old:
                old.close()

    def send(self, data):
        c = self.client
        if c is None:
            return
        try:
            c.sendall(data)
        except OSError:
            if self.client is c:
                self.client = None
            c.close()


def resolve(network):
    if os.path.exists(network):
        return network
    return os.path.join(MODELS, f'imx500_network_{network}.rpk')


def yolo_masks(boxes, mc, proto, size, budget=MAX_MASK_BYTES):
    """Soft masks from YOLO-seg outputs: [[cx1, cy1, cx2, cy2, base64 of uint8 probability x 255], ...].

    boxes are xyxy in input pixels, size is the input (width, height). Each mask covers its box on
    the prototype grid plus a cell all round, so scaling up has the neighbours it needs.
    """
    nm, mh, mw = proto.shape
    sx, sy = mw / size[0], mh / size[1]
    objs, used = [], 0
    for (x1, y1, x2, y2), coef in zip(boxes, mc):
        cx1, cy1 = max(int(x1 * sx) - 1, 0), max(int(y1 * sy) - 1, 0)
        cx2, cy2 = min(int(np.ceil(x2 * sx)) + 1, mw), min(int(np.ceil(y2 * sy)) + 1, mh)
        p = 0.5 + 0.5 * np.tanh(0.5 * np.tensordot(coef, proto[:, cy1:cy2, cx1:cx2], 1))  # sigmoid, no overflow
        data = base64.b64encode(np.round(p * 255).astype(np.uint8).tobytes()).decode()
        used += len(data)
        if used > budget:
            break
        objs.append([cx1, cy1, cx2, cy2, data])
    return objs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('network')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=960)
    ap.add_argument('--fps', type=float, default=15)
    ap.add_argument('--port', type=int, default=9000)
    ap.add_argument('--dets', default='127.0.0.1:9001')
    ap.add_argument('--conf', type=float, default=0.35)
    a = ap.parse_args()

    rpk = resolve(a.network)
    imx = IMX500(rpk)  # uploads the network to the sensor
    labels = (imx.network_intrinsics.labels if imx.network_intrinsics else None) or []
    txt = os.path.splitext(rpk)[0] + '.txt'
    if not labels and os.path.exists(txt):
        labels = [ln.strip() for ln in open(txt) if ln.strip()]
    size = np.array(imx.get_input_size(), float)  # (width, height)
    cam = Picamera2(imx.camera_num)
    cam.configure(cam.create_video_configuration(main={'size': (a.width, a.height), 'format': 'YUV420'},
                                                 controls={'FrameRate': a.fps}, buffer_count=6))
    if cam.camera_configuration()['main']['stride'] != a.width:
        raise SystemExit(f'width {a.width}: rows are padded, use a multiple of 64')
    cam.start()
    host, port = a.dets.split(':')
    udp, dest = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, int(port))
    server = Server(a.port)
    server.start()
    print(f'{rpk}: {a.width}x{a.height} at {a.fps} fps on :{a.port}, detections to {a.dets}', flush=True)

    seq, dets, mask = 0, [], None
    while True:
        req = cam.capture_request()
        try:
            md = req.get_metadata()
            out = imx.get_outputs(md, add_batch=True)
            if out is not None:  # no new result for this frame: the last one still stands
                dets, mask = [], None
                if len(out) == 5:  # YOLO seg: boxes xyxy in input pixels, scores, labels, coefficients, protos
                    keep = out[1][0] >= a.conf
                    boxes, scores, classes = out[0][0][keep], out[1][0][keep], out[2][0][keep]
                    order = np.argsort(-scores)
                    boxes, scores, classes = boxes[order], scores[order], classes[order]
                    proto = out[4][0]
                    x, y, w, h = imx.convert_inference_coords((0, 0, 1, 1), md, cam)
                    mask = {'rect': [x, y, w, h], 'shape': list(proto.shape[1:]),
                            'objs': yolo_masks(boxes, out[3][0][keep][order], proto, size)}
                    norm = boxes[:, [1, 0, 3, 2]] / size[[1, 0, 1, 0]]  # to (y0, x0, y1, x1) relative
                elif out[0].shape[-1] == 4 and out[0][0].max() > 1.5:  # YOLO detect: xyxy in input pixels
                    keep = out[1][0] >= a.conf
                    norm = out[0][0][keep][:, [1, 0, 3, 2]] / size[[1, 0, 1, 0]]
                    scores, classes = out[1][0][keep], out[2][0][keep]
                else:  # SSD-style: (y0, x0, y1, x1) relative
                    keep = out[1][0] >= a.conf
                    norm, scores, classes = out[0][0][keep], out[1][0][keep], out[2][0][keep]
                for box, s, c in zip(norm, scores, classes):
                    x, y, w, h = imx.convert_inference_coords(box, md, cam)
                    c = int(c)
                    dets.append([x, y, x + w, y + h, round(float(s), 3), labels[c] if c < len(labels) else str(c)])
            msg = {'seq': seq, 'w': a.width, 'h': a.height, 'dets': dets, 'new': out is not None}
            if mask:
                msg['masks'] = mask
            udp.sendto(json.dumps(msg).encode(), dest)
            frame = req.make_buffer('main')
        finally:
            req.release()
        server.send(frame)
        seq += 1


if __name__ == '__main__':
    main()
