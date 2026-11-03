"""Keeps detections steady from frame to frame.

Detections pair up with tracks by box overlap (IoU) within a class. A paired track's box is
smoothed; a new object shows after `confirm` frames in a row, or at once when its score reaches
`sure`; an object the detector misses stays, with its last box and mask, for `hold` frames.
At 15 fps the defaults hide one-frame flukes and bridge a 200 ms dropout.
"""
import itertools

import numpy as np


def iou(a, b):
    """IoU of every box in a (n, 4) with every box in b (m, 4), xyxy."""
    if not len(a) or not len(b):
        return np.zeros((len(a), len(b)))
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area = lambda r: (r[:, 2] - r[:, 0]) * (r[:, 3] - r[:, 1])  # noqa: E731
    return inter / (area(a)[:, None] + area(b)[None, :] - inter + 1e-9)


class Track:
    __slots__ = ('id', 'box', 'score', 'cls', 'mask', 'hits', 'misses')

    def __init__(self, id_, box, score, cls, mask):
        self.id, self.box, self.score, self.cls, self.mask = id_, box.astype(float), score, cls, mask
        self.hits, self.misses = 1, 0


class Tracker:
    def __init__(self, iou_min=0.3, hold=3, confirm=2, sure=0.5, smooth=0.5):
        self.iou_min, self.hold, self.confirm, self.sure, self.smooth = iou_min, hold, confirm, sure, smooth
        self.tracks, self.ids = [], itertools.count(1)

    def update(self, boxes, scores, classes, masks):
        """One frame of detections in, the steady set out: boxes, scores, classes, masks, ids.

        masks is None for a detector without masks, and then stays None.
        """
        overlap = iou(np.array([t.box for t in self.tracks]).reshape(-1, 4), np.asarray(boxes).reshape(-1, 4))
        for ti, t in enumerate(self.tracks):
            overlap[ti, np.asarray(classes) != t.cls] = 0
        paired_t, paired_d = set(), set()
        for ti, di in zip(*np.unravel_index(np.argsort(-overlap, axis=None), overlap.shape)):
            if overlap[ti, di] < self.iou_min:
                break
            if ti in paired_t or di in paired_d:
                continue
            t = self.tracks[ti]
            t.box = self.smooth * t.box + (1 - self.smooth) * boxes[di]
            t.score, t.hits, t.misses = float(scores[di]), t.hits + 1, 0
            t.mask = None if masks is None else masks[di]
            paired_t.add(ti)
            paired_d.add(di)
        for ti, t in enumerate(self.tracks):
            if ti not in paired_t:
                t.misses += 1
        self.tracks = [t for t in self.tracks if t.misses <= self.hold]
        for di in range(len(boxes)):
            if di not in paired_d:
                self.tracks.append(Track(next(self.ids), np.asarray(boxes[di]), float(scores[di]), int(classes[di]),
                                         None if masks is None else masks[di]))

        shown = [t for t in self.tracks if t.hits >= self.confirm or t.score >= self.sure]
        out_masks = None
        if masks is not None:
            shape = masks.shape[1:]
            out_masks = np.stack([t.mask if t.mask is not None else np.zeros(shape, bool) for t in shown]) \
                if shown else np.zeros((0, *shape), bool)
        return (np.array([t.box for t in shown]).reshape(-1, 4), np.array([t.score for t in shown]),
                np.array([t.cls for t in shown], int), out_masks, [t.id for t in shown])
