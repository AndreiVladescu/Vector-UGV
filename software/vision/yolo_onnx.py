"""YOLO detection / segmentation from an Ultralytics ONNX export, with ONNX Runtime and NumPy
only, so PyTorch isn't needed on the robot.

Export once (the export stage of software/vision/Dockerfile):
  yolo export model=yolo26n-seg.pt format=onnx imgsz=320 simplify=True
Output layout of the export: output0 (1, 4 + classes [+ 32 mask coefficients], anchors), box as
cx, cy, w, h in input pixels; segmentation adds output1, the (1, 32, H/4, W/4) mask prototypes.
"""
import ast

import cv2
import numpy as np
import onnxruntime as ort


class Yolo:
    def __init__(self, path, conf=0.35, iou=0.5, threads=4):
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = threads
        self.session = ort.InferenceSession(path, opts, providers=['CPUExecutionProvider'])
        meta = self.session.get_modelmeta().custom_metadata_map
        self.names = ast.literal_eval(meta['names'])
        self.task = meta.get('task', 'detect')
        self.size = self.session.get_inputs()[0].shape[2]
        self.conf, self.iou = conf, iou

    def __call__(self, img):
        """img: BGR. Returns boxes (n, 4) x1 y1 x2 y2, scores (n,), classes (n,), masks (n, h, w) bool or None."""
        h, w = img.shape[:2]
        r = self.size / max(h, w)
        nh, nw = round(h * r), round(w * r)
        top, left = (self.size - nh) // 2, (self.size - nw) // 2
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        canvas[top:top + nh, left:left + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0

        outputs = self.session.run(None, {self.session.get_inputs()[0].name: blob})
        pred = outputs[0][0].T  # (anchors, 4 + classes [+ 32])
        nc = len(self.names)
        scores_all = pred[:, 4:4 + nc]
        classes = scores_all.argmax(1)
        scores = scores_all[np.arange(len(pred)), classes]
        keep = scores > self.conf
        pred, classes, scores = pred[keep], classes[keep], scores[keep]
        empty = (np.zeros((0, 4)), np.zeros(0), np.zeros(0, int), None if self.task == 'detect' else np.zeros((0, h, w), bool))
        if not len(pred):
            return empty

        cx, cy, bw, bh = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
        xywh = np.stack([cx - bw / 2, cy - bh / 2, bw, bh], 1)
        idx = cv2.dnn.NMSBoxesBatched(xywh.tolist(), scores.tolist(), classes.tolist(), self.conf, self.iou)
        idx = np.array(idx, int).reshape(-1)[:100]
        if not len(idx):
            return empty
        pred, classes, scores, xywh = pred[idx], classes[idx], scores[idx], xywh[idx]

        boxes = np.stack([xywh[:, 0], xywh[:, 1], xywh[:, 0] + xywh[:, 2], xywh[:, 1] + xywh[:, 3]], 1)
        boxes_img = (boxes - [left, top, left, top]) / r
        boxes_img = np.clip(boxes_img, 0, [w, h, w, h])

        masks = None
        if self.task == 'segment':
            protos = outputs[1][0]  # (32, mh, mw)
            mh, mw = protos.shape[1:]
            m = 1 / (1 + np.exp(-(pred[:, 4 + nc:] @ protos.reshape(32, -1))))
            m = m.reshape(-1, mh, mw)
            # crop each mask to its box (prototype scale), then back to the image
            s = mh / self.size
            ys, xs = np.arange(mh)[None, :, None], np.arange(mw)[None, None, :]
            b = boxes * s
            inside = (xs >= b[:, 0, None, None]) & (xs < b[:, 2, None, None]) & \
                     (ys >= b[:, 1, None, None]) & (ys < b[:, 3, None, None])
            m = m * inside
            t, l = int(top * s), int(left * s)
            m = m[:, t:t + int(nh * s), l:l + int(nw * s)]
            masks = np.stack([cv2.resize(x, (w, h), interpolation=cv2.INTER_LINEAR) > 0.5 for x in m])
        return boxes_img, scores, classes, masks


def draw(img, boxes, scores, classes, masks, names, dists=None):
    """Boxes, labels (with the ToF distance when there is one) and a translucent mask per object."""
    out = img.copy()
    rng = np.random.default_rng(7)
    colors = rng.integers(60, 255, (len(names), 3))
    if masks is not None and len(masks):
        overlay = out.copy()
        for m, c in zip(masks, classes):
            overlay[m] = colors[c]
        out = cv2.addWeighted(overlay, 0.4, out, 0.6, 0)
    dists = dists or [None] * len(boxes)
    for (x1, y1, x2, y2), s, c, d in zip(boxes.astype(int), scores, classes, dists):
        color = tuple(int(v) for v in colors[c])
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        label = f'{names[int(c)]} {s:.2f}' + (f' {d:.1f} m' if d is not None else '')
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(out, (x1, y1 - th - 4), (x1 + tw + 2, y1), color, -1)
        cv2.putText(out, label, (x1 + 1, y1 - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return out
