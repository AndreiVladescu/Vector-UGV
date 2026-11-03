#!/usr/bin/env python3
"""YOLO on the camera, into ROS.

The camera stays on the host (libcamera is native there):
  rpicam-vid -t 0 -n --width 640 --height 480 --framerate 15 --codec yuv420 --listen -o tcp://127.0.0.1:9000
This node reads those raw I420 frames, keeps only the newest, and publishes
  ~/detections   vision_msgs/Detection2DArray
  ~/mask         sensor_msgs/Image mono8, class id + 1 per pixel, 0 = nothing (segmentation models),
                 at mask_scale of the frame (0.25: 160x120 from 640x480)
  ~/detections_3d  vision_msgs/Detection3DArray, the boxes the nose ToF (tof_front) has a distance
                 for: centre in frame nose_tof (x forward, y left, z up); the distance also goes
                 on the stream's labels
  ~/debug/compressed  annotated JPEG at debug_rate, for Foxglove
With stream set (e.g. rtsp://127.0.0.1:8554/yolo) every annotated frame also goes out as H.264 to
MediaMTX, which serves it as RTSP (VLC) and WebRTC (a browser, port 8889).
With the AI camera running a network on the sensor (software/host/imx500_camera.py), its detections
arrive on UDP (sensor, 127.0.0.1:9001) and stand in for the model while they keep coming, with the
object masks (soft, scaled up here) when the sensor runs a segmentation network, else the mask is
the boxes filled in.
Without them the ONNX model runs on the CPU.
Parameters: model (an Ultralytics ONNX export, see yolo_onnx.py), rate, source, sensor, stream, stream_kbps.
No PyTorch here: ONNX Runtime and NumPy only.
"""
import base64
import json
import math
import socket
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose

from fusion import box_distance, box_point
from track import Tracker
from yolo_onnx import Yolo, draw


class Camera(threading.Thread):
    """Raw I420 frames over TCP; the newest one wins."""

    def __init__(self, host, port, width, height):
        super().__init__(daemon=True)
        self.addr, self.w, self.h = (host, port), width, height
        self.size = width * height * 3 // 2
        self.lock = threading.Lock()
        self.frame, self.stamp, self.count = None, 0.0, 0

    def run(self):
        while True:
            try:
                s = socket.create_connection(self.addr, timeout=5)
            except OSError:
                time.sleep(1)
                continue
            buf = bytearray()
            while True:
                try:
                    chunk = s.recv(1 << 20)
                except OSError:  # 5 s without a frame: the camera service restarted, connect again
                    break
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= self.size:
                    raw = bytes(buf[:self.size])
                    del buf[:self.size]
                    with self.lock:
                        self.frame, self.stamp = raw, time.monotonic()
                        self.count += 1
            s.close()

    def latest(self):
        with self.lock:
            raw, stamp = self.frame, self.stamp
            self.frame = None
        if raw is None:
            return None, 0.0
        yuv = np.frombuffer(raw, np.uint8).reshape(self.h * 3 // 2, self.w)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420), stamp


class SensorDetections(threading.Thread):
    """Detections the camera's own network sends per frame; the newest wins."""

    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.lock = threading.Lock()
        self.msg, self.stamp = None, 0.0

    def run(self):
        while True:
            msg = json.loads(self.sock.recv(65536))
            with self.lock:
                self.msg, self.stamp = msg, time.monotonic()

    def latest(self, max_age=0.5):
        """The newest message (dets, maybe mask), or None when the camera hasn't sent any lately."""
        with self.lock:
            return self.msg if time.monotonic() - self.stamp < max_age else None


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo')
        model = self.declare_parameter('model', 'yolo26n-seg-320.onnx').value
        sensor = self.declare_parameter('sensor', '127.0.0.1:9001').value
        conf = self.declare_parameter('conf', 0.35).value
        rate = self.declare_parameter('rate', 15.0).value
        self.debug_rate = self.declare_parameter('debug_rate', 2.0).value
        host, port = self.declare_parameter('source', '127.0.0.1:9000').value.split(':')
        self.stream = self.declare_parameter('stream', '').value
        self.stream_kbps = self.declare_parameter('stream_kbps', 2500).value
        self.encoder = None
        self.rate = rate
        w = self.declare_parameter('width', 640).value
        h = self.declare_parameter('height', 480).value
        # Pi AI Camera (IMX500, 4.74 mm) across the sensor's full width; the frame is a centred band of
        # it, so the vertical angle follows from the frame's shape. The VL53L8CX sees 45 x 45 deg.
        self.cam_hfov = math.radians(self.declare_parameter('cam_hfov_deg', 66.0).value)
        self.tof_fov = math.radians(self.declare_parameter('tof_fov_deg', 45.0).value)
        self.depth, self.depth_t = None, 0.0
        self.mask_scale = self.declare_parameter('mask_scale', 0.25).value

        self.model = Yolo(model, conf=conf)
        self.seg = self.model.task == 'segment'
        self.names = dict(self.model.names)  # sensor labels the model doesn't know get new ids
        self.ids = {n: i for i, n in self.names.items()}
        self.sensor = None
        if sensor:
            shost, sport = sensor.split(':')
            self.sensor = SensorDetections(shost, int(sport))
            self.sensor.start()
        self.on_sensor = False
        self.tracker = Tracker()  # steady boxes, no one-frame flukes, short dropouts bridged
        self.cam = Camera(host, int(port), w, h)
        self.cam.start()
        self.det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self.mask_pub = self.create_publisher(Image, '~/mask', 10)
        self.dbg_pub = self.create_publisher(CompressedImage, '~/debug/compressed', 2)
        self.det3_pub = self.create_publisher(Detection3DArray, '~/detections_3d', 10)
        self.create_subscription(Image, '/nose/tof/depth', self.on_depth, qos_profile_sensor_data)
        self.stats = {'n': 0, 'infer': 0.0, 'lat': 0.0, 't0': time.monotonic(), 'cam0': 0}
        self.last_dbg = 0.0
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'{model} ({self.model.task}) at {self.model.size} px, up to {rate} Hz')

    def fov(self, w, h):
        """(horizontal, vertical) view of a w x h frame, square pixels."""
        return self.cam_hfov, 2 * math.atan(math.tan(self.cam_hfov / 2) * h / w)

    def on_depth(self, msg):
        if msg.encoding == '32FC1' and msg.height == msg.width:
            self.depth = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.width)
            self.depth_t = time.monotonic()

    def distances(self, img, boxes):
        if self.depth is None or time.monotonic() - self.depth_t > 0.3:
            return [None] * len(boxes)
        h, w = img.shape[:2]
        return [box_distance(b, w, h, self.depth, *self.fov(w, h), self.tof_fov) for b in boxes.tolist()]

    def tick(self):
        img, stamp = self.cam.latest()
        if img is None:
            return
        t = time.monotonic()
        dets = self.sensor.latest() if self.sensor else None
        if dets is not None:
            boxes, scores, classes, masks = self.from_sensor(dets, img.shape)
        else:
            boxes, scores, classes, masks = self.model(img)
        if (dets is not None) != self.on_sensor:
            self.on_sensor = dets is not None
            self.tracker = Tracker()
            self.get_logger().info("detections from the camera's network" if self.on_sensor else
                                   f'detections from {self.model.task} model on the CPU')
        boxes, scores, classes, masks, ids = self.tracker.update(boxes, scores, classes, masks)
        done = time.monotonic()
        header = self.get_clock().now().to_msg()
        header = type(Detection2DArray().header)(stamp=header, frame_id='camera')

        arr = Detection2DArray(header=header)
        for (x1, y1, x2, y2), c, conf, tid in zip(boxes.tolist(), classes.tolist(), scores.tolist(), ids):
            d = Detection2D(header=header, id=str(tid))
            d.bbox.center.position.x, d.bbox.center.position.y = (x1 + x2) / 2, (y1 + y2) / 2
            d.bbox.size_x, d.bbox.size_y = x2 - x1, y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id, hyp.hypothesis.score = self.names[int(c)], float(conf)
            d.results.append(hyp)
            arr.detections.append(d)
        self.det_pub.publish(arr)

        dists = self.distances(img, boxes)
        arr3 = Detection3DArray(header=type(header)(stamp=header.stamp, frame_id='nose_tof'))
        for d2, box, dist in zip(arr.detections, boxes.tolist(), dists):
            if dist is None:
                continue
            d3 = Detection3D(header=arr3.header, results=d2.results)
            p = d3.bbox.center.position
            p.x, p.y, p.z = box_point(box, img.shape[1], img.shape[0], *self.fov(img.shape[1], img.shape[0]), dist)
            arr3.detections.append(d3)
        if arr3.detections:
            self.det3_pub.publish(arr3)

        if masks is not None or self.on_sensor:
            mask = np.zeros(img.shape[:2], np.uint8)
            if masks is not None:
                for m, c in zip(masks, classes.tolist()):
                    mask[m] = int(c) + 1
            else:  # boxes filled in: their bottom edge is where the object meets the ground
                for (x1, y1, x2, y2), c in zip(boxes.astype(int).tolist(), classes.tolist()):
                    mask[max(y1, 0):y2, max(x1, 0):x2] = int(c) + 1
            if self.mask_scale != 1.0:
                mask = cv2.resize(mask, None, fx=self.mask_scale, fy=self.mask_scale, interpolation=cv2.INTER_NEAREST)
            self.mask_pub.publish(Image(header=header, height=mask.shape[0], width=mask.shape[1],
                                        encoding='mono8', step=mask.shape[1], data=mask.tobytes()))

        annotated = None
        if self.stream:
            annotated = draw(img, boxes, scores, classes, masks, self.names, dists)
            self.send(annotated)

        if done - self.last_dbg > 1.0 / self.debug_rate:
            self.last_dbg = done
            if annotated is None:
                annotated = draw(img, boxes, scores, classes, masks, self.names, dists)
            ok, jpg = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self.dbg_pub.publish(CompressedImage(header=header, format='jpeg', data=jpg.tobytes()))

        s = self.stats
        s['n'] += 1
        s['infer'] += done - t
        s['lat'] += done - stamp
        if done - s['t0'] > 5:
            cam_fps = (self.cam.count - s['cam0']) / (done - s['t0'])
            self.get_logger().info(
                f'{s["n"] / (done - s["t0"]):.1f} fps, inference {1000 * s["infer"] / s["n"]:.0f} ms, '
                f'frame to result {1000 * s["lat"] / s["n"]:.0f} ms, camera {cam_fps:.1f} fps, {len(arr.detections)} objects')
            self.stats = {'n': 0, 'infer': 0.0, 'lat': 0.0, 't0': done, 'cam0': self.cam.count}

    def from_sensor(self, msg, shape):
        dets = msg['dets']
        for d in dets:
            if d[5] not in self.ids:
                self.ids[d[5]] = len(self.names)
                self.names[len(self.names)] = d[5]
        boxes = np.array([d[:4] for d in dets], float).reshape(-1, 4)
        scores = np.array([d[4] for d in dets], float)
        classes = np.array([self.ids[d[5]] for d in dets], int)
        masks = None
        if 'masks' in msg:
            masks = self.sensor_masks(msg['masks'], boxes, shape[:2])
        return boxes, scores, classes, masks

    @staticmethod
    def sensor_masks(m, boxes, shape):
        """Per-object frame masks from the camera's soft masks: scaled up around each box, then thresholded."""
        x, y, w, h = m['rect']
        mh, mw = m['shape']
        sx, sy = mw / w, mh / h  # grid cells per frame pixel
        masks = np.zeros((len(boxes), *shape), bool)
        for i, ((x1, y1, x2, y2), (cx1, cy1, cx2, cy2, data)) in enumerate(zip(boxes, m['objs'])):
            soft = np.frombuffer(base64.b64decode(data), np.uint8).reshape(cy2 - cy1, cx2 - cx1)
            ox, oy = x + round(cx1 / sx), y + round(cy1 / sy)
            pw, ph = round((cx2 - cx1) / sx), round((cy2 - cy1) / sy)
            if pw <= 0 or ph <= 0:
                continue
            up = cv2.resize(soft, (pw, ph), interpolation=cv2.INTER_LINEAR) > 127
            fx1, fy1 = max(int(x1), ox, 0), max(int(y1), oy, 0)
            fx2, fy2 = min(int(np.ceil(x2)), ox + pw, shape[1]), min(int(np.ceil(y2)), oy + ph, shape[0])
            if fx2 > fx1 and fy2 > fy1:
                masks[i, fy1:fy2, fx1:fx2] = up[fy1 - oy:fy2 - oy, fx1 - ox:fx2 - ox]
        return masks

    def send(self, frame):
        """Annotated frames to ffmpeg -> H.264 baseline (what browsers' WebRTC takes) -> RTSP."""
        if self.encoder is None or self.encoder.poll() is not None:
            h, w = frame.shape[:2]
            self.encoder = subprocess.Popen(
                ['ffmpeg', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{w}x{h}',
                 '-r', str(int(self.rate)), '-i', '-', '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
                 '-profile:v', 'baseline', '-pix_fmt', 'yuv420p', '-g', str(int(self.rate)),
                 '-b:v', f'{self.stream_kbps}k', '-maxrate', f'{self.stream_kbps}k', '-bufsize', f'{self.stream_kbps}k',
                 '-f', 'rtsp', '-rtsp_transport', 'tcp', self.stream], stdin=subprocess.PIPE)
        try:
            self.encoder.stdin.write(frame.tobytes())
        except (BrokenPipeError, OSError):
            self.encoder = None  # MediaMTX restarted: reconnect with the next frame


def main():
    rclpy.init()
    rclpy.spin(YoloNode())


if __name__ == '__main__':
    main()
