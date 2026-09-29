#!/usr/bin/env python3
"""YOLO on the camera, into ROS.

The camera stays on the host (libcamera is native there):
  rpicam-vid -t 0 -n --width 640 --height 480 --framerate 15 --codec yuv420 --listen -o tcp://127.0.0.1:9000
This node reads those raw I420 frames, keeps only the newest, and publishes
  ~/detections   vision_msgs/Detection2DArray
  ~/mask         sensor_msgs/Image mono8, class id + 1 per pixel, 0 = nothing (segmentation models)
  ~/debug/compressed  annotated JPEG at debug_rate, for Foxglove
With stream set (e.g. rtsp://127.0.0.1:8554/yolo) every annotated frame also goes out as H.264 to
MediaMTX, which serves it as RTSP (VLC) and WebRTC (a browser, port 8889).
Parameters: model (an Ultralytics ONNX export, see yolo_onnx.py), rate, source, stream.
No PyTorch here: ONNX Runtime and NumPy only.
"""
import socket
import subprocess
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

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
                chunk = s.recv(1 << 20)
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


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo')
        model = self.declare_parameter('model', 'yolo26n-seg-320.onnx').value
        conf = self.declare_parameter('conf', 0.35).value
        rate = self.declare_parameter('rate', 10.0).value
        self.debug_rate = self.declare_parameter('debug_rate', 2.0).value
        host, port = self.declare_parameter('source', '127.0.0.1:9000').value.split(':')
        self.stream = self.declare_parameter('stream', '').value
        self.encoder = None
        self.rate = rate
        w = self.declare_parameter('width', 640).value
        h = self.declare_parameter('height', 480).value

        self.model = Yolo(model, conf=conf)
        self.seg = self.model.task == 'segment'
        self.cam = Camera(host, int(port), w, h)
        self.cam.start()
        self.det_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self.mask_pub = self.create_publisher(Image, '~/mask', 10)
        self.dbg_pub = self.create_publisher(CompressedImage, '~/debug/compressed', 2)
        self.stats = {'n': 0, 'infer': 0.0, 'lat': 0.0, 't0': time.monotonic(), 'cam0': 0}
        self.last_dbg = 0.0
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f'{model} ({self.model.task}) at {self.model.size} px, up to {rate} Hz')

    def tick(self):
        img, stamp = self.cam.latest()
        if img is None:
            return
        t = time.monotonic()
        boxes, scores, classes, masks = self.model(img)
        done = time.monotonic()
        header = self.get_clock().now().to_msg()
        header = type(Detection2DArray().header)(stamp=header, frame_id='camera')

        arr = Detection2DArray(header=header)
        for (x1, y1, x2, y2), c, conf in zip(boxes.tolist(), classes.tolist(), scores.tolist()):
            d = Detection2D(header=header)
            d.bbox.center.position.x, d.bbox.center.position.y = (x1 + x2) / 2, (y1 + y2) / 2
            d.bbox.size_x, d.bbox.size_y = x2 - x1, y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id, hyp.hypothesis.score = self.model.names[int(c)], float(conf)
            d.results.append(hyp)
            arr.detections.append(d)
        self.det_pub.publish(arr)

        if self.seg:
            mask = np.zeros(img.shape[:2], np.uint8)
            for m, c in zip(masks, classes.tolist()):
                mask[m] = int(c) + 1
            self.mask_pub.publish(Image(header=header, height=mask.shape[0], width=mask.shape[1],
                                        encoding='mono8', step=mask.shape[1], data=mask.tobytes()))

        annotated = None
        if self.stream:
            annotated = draw(img, boxes, scores, classes, masks, self.model.names)
            self.send(annotated)

        if done - self.last_dbg > 1.0 / self.debug_rate:
            self.last_dbg = done
            if annotated is None:
                annotated = draw(img, boxes, scores, classes, masks, self.model.names)
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


    def send(self, frame):
        """Annotated frames to ffmpeg -> H.264 baseline (what browsers' WebRTC takes) -> RTSP."""
        if self.encoder is None or self.encoder.poll() is not None:
            h, w = frame.shape[:2]
            self.encoder = subprocess.Popen(
                ['ffmpeg', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{w}x{h}',
                 '-r', str(int(self.rate)), '-i', '-', '-c:v', 'libx264', '-preset', 'ultrafast', '-tune', 'zerolatency',
                 '-profile:v', 'baseline', '-pix_fmt', 'yuv420p', '-g', str(int(self.rate)), '-b:v', '800k',
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
