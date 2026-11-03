import numpy as np

from track import Tracker, iou


def frame(*dets):
    """dets: (x1, y1, x2, y2, score, cls) -> the arrays the tracker takes, with masks."""
    boxes = np.array([d[:4] for d in dets], float).reshape(-1, 4)
    masks = np.zeros((len(dets), 48, 64), bool)
    for i, d in enumerate(dets):
        masks[i, int(d[1]) // 10:int(d[3]) // 10, int(d[0]) // 10:int(d[2]) // 10] = True
    return boxes, np.array([d[4] for d in dets], float), np.array([d[5] for d in dets], int), masks


def test_iou():
    a = np.array([[0, 0, 10, 10]], float)
    b = np.array([[0, 0, 10, 10], [5, 0, 15, 10], [20, 20, 30, 30]], float)
    assert iou(a, b).round(3).tolist() == [[1, 0.333, 0]]


def test_a_fluke_needs_two_frames_a_sure_one_shows_at_once():
    t = Tracker()
    assert len(t.update(*frame((100, 100, 200, 200, 0.4, 0)))[0]) == 0
    assert len(t.update(*frame((102, 100, 202, 200, 0.4, 0)))[0]) == 1
    t = Tracker()
    assert len(t.update(*frame((100, 100, 200, 200, 0.8, 0)))[0]) == 1


def test_a_missed_object_is_held_then_dropped():
    t = Tracker(hold=3)
    t.update(*frame((100, 100, 200, 200, 0.8, 0)))
    for _ in range(3):
        boxes, _, _, masks, ids = t.update(*frame())
        assert ids == [1] and masks[0].any()  # last box and mask kept
    assert t.update(*frame())[4] == []


def test_same_object_keeps_its_id_and_its_box_is_smoothed():
    t = Tracker(smooth=0.5)
    t.update(*frame((100, 100, 200, 200, 0.8, 0)))
    boxes, _, _, _, ids = t.update(*frame((120, 100, 220, 200, 0.8, 0)))
    assert ids == [1] and boxes[0].tolist() == [110, 100, 210, 200]


def test_classes_and_far_boxes_make_new_tracks():
    t = Tracker()
    t.update(*frame((100, 100, 200, 200, 0.8, 0)))
    ids = t.update(*frame((100, 100, 200, 200, 0.8, 1), (400, 300, 500, 400, 0.8, 0)))[4]
    assert sorted(ids) == [1, 2, 3]  # 1 held, 2 and 3 new


def test_without_masks():
    t = Tracker()
    b, s, c, _ = frame((100, 100, 200, 200, 0.8, 0))
    assert t.update(b, s, c, None)[3] is None
