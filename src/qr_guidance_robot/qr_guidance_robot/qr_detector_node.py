import json
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def qr_side_px(pts):
    """Apparent QR side (px) from its 4 corners: mean length of the 2 most vertical edges.

    A horizontal pan of the view (robot not square to the QR) shortens the horizontal
    edges by cos(angle) but leaves the vertical ones, so the distance k / w_px stays right.
    """
    p = np.asarray(pts, dtype=float).reshape(4, 2)
    edges = [p[(i + 1) % 4] - p[i] for i in range(4)]
    edges.sort(key=lambda e: abs(e[0]) - abs(e[1]))       # most vertical first
    return float(np.mean([np.hypot(e[0], e[1]) for e in edges[:2]]))


SENSOR_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)


class QRDetectorNode(Node):
    """
    Subscribes to a raw camera image, detects QR codes, publishes what it finds.

    /qr/detections (String, JSON):
        {"id": "<decoded string>", "cx": <pixel x>, "cy": <pixel y>,
         "img_w": <image width>, "img_h": <image height>,
         "w_px": <apparent QR side in pixels>, "stamp": <capture time of the frame, s>}

    w_px is what mission_node / localization_node use for the distance (k / w_px) and the
    "close enough" gate. It is measured on the two QR edges that are vertical in the image,
    so it does not shrink when the QR is seen from the side (a horizontal width would, by
    cos(angle), and the robot would think it is further away than it is).
    stamp is the camera header stamp: decoding takes 50-300 ms on the Pi, so the consumers
    look up where the robot and the camera servo were at that instant, not at reception.
    -1 for w_px means "no corners returned by OpenCV" (consumers then ignore the size).

    /qr/debug_image: annotated frame, published from the camera callback at
    debug_max_hz, independent of how long decoding takes.

    Decoding runs in a separate worker thread and always works on the newest
    frame, so a slow decode never slows the video down.
    """

    def __init__(self):
        super().__init__('qr_detector_node')

        self.declare_parameter('image_topic', '/camera/image')   # Gazebo camera
        self.declare_parameter('debug_max_hz', 15.0)
        self.declare_parameter('debug_scale', 0.5)
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.debug_period = 1.0 / max(
            self.get_parameter('debug_max_hz').get_parameter_value().double_value, 0.1)
        self.debug_scale = self.get_parameter('debug_scale').get_parameter_value().double_value
        self.last_debug = 0.0

        # The Aruco-based detector (OpenCV >= 4.8) is far better on real cameras.
        if hasattr(cv2, 'QRCodeDetectorAruco'):
            self.qr_decoder = cv2.QRCodeDetectorAruco()
            self.get_logger().info('Using QRCodeDetectorAruco')
        else:
            self.qr_decoder = cv2.QRCodeDetector()
            self.get_logger().warn('QRCodeDetectorAruco not available, using the basic detector')

        self.mirror_warned = False
        # One attempt per frame: (upscale, mirrored). Cheap settings first.
        self.attempts = [(1, False), (2, False), (1, True)]
        self.declare_parameter('multi', True)        # decode every QR in view, keep the nearest
        self.use_multi = bool(self.get_parameter('multi').value)
        self.attempt_idx = 0
        self._self_test()

        # Shared state between the camera callback and the decode worker
        self._lock = threading.Lock()
        self._latest = None          # (newest BGR frame, its capture stamp in s)
        self._new_frame = threading.Event()
        self._stop = threading.Event()
        self._last_det = None        # (pts, text, time) for the debug overlay

        # Stats
        self._frames = 0
        self._decode_ms = 0.0
        self._decode_count = 0

        self.sub = self.create_subscription(Image, image_topic, self.image_callback, SENSOR_QOS)
        self.detections_pub = self.create_publisher(String, '/qr/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/qr/debug_image', SENSOR_QOS)

        self.got_image = False
        self.image_topic = image_topic
        self.create_timer(5.0, self._report)

        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
        self.get_logger().info(f'qr_detector_node started, listening on {image_topic}')

    # encoding -> (bytes per pixel, OpenCV conversion code to BGR or None if already BGR)
    _ENCODINGS = {
        'bgr8': (3, None),
        'rgb8': (3, cv2.COLOR_RGB2BGR),
        'bgra8': (4, cv2.COLOR_BGRA2BGR),
        'rgba8': (4, cv2.COLOR_RGBA2BGR),
        'mono8': (1, cv2.COLOR_GRAY2BGR),
        'yuv422_yuy2': (2, cv2.COLOR_YUV2BGR_YUY2),
        'yuyv': (2, cv2.COLOR_YUV2BGR_YUY2),
        'yuv422': (2, cv2.COLOR_YUV2BGR_UYVY),
        'uyvy': (2, cv2.COLOR_YUV2BGR_UYVY),
    }

    @staticmethod
    def _msg_to_bgr(msg: Image):
        """Image msg -> BGR numpy array, without cv_bridge."""
        if msg.encoding not in QRDetectorNode._ENCODINGS:
            raise ValueError(f'unsupported image encoding: {msg.encoding}')
        bpp, code = QRDetectorNode._ENCODINGS[msg.encoding]
        arr = np.frombuffer(msg.data, dtype=np.uint8)   # no extra bytes() copy
        arr = arr.reshape(msg.height, msg.step)[:, :msg.width * bpp]
        frame = arr.reshape(msg.height, msg.width, bpp) if bpp > 1 else arr
        if code is not None:
            frame = cv2.cvtColor(np.ascontiguousarray(frame), code)
        return np.ascontiguousarray(frame)

    @staticmethod
    def _bgr_to_msg(frame, header):
        out = Image()
        out.header = header
        out.height, out.width = frame.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = np.ascontiguousarray(frame).tobytes()
        return out

    def _self_test(self):
        """Make sure this OpenCV build can really decode QR codes."""
        try:
            enc = cv2.QRCodeEncoder.create()
            qr = enc.encode('selftest')
            qr = cv2.resize(qr, None, fx=8, fy=8, interpolation=cv2.INTER_NEAREST)
            qr = cv2.copyMakeBorder(qr, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
            if qr.ndim == 3:
                qr = cv2.cvtColor(qr, cv2.COLOR_BGR2GRAY)
            ok = self.qr_decoder.detectAndDecode(qr)[0] == 'selftest'
        except Exception as exc:
            self.get_logger().warn(f'QR self-test skipped: {exc}')
            return
        if ok:
            self.get_logger().info(f'OpenCV {cv2.__version__}: QR decoding self-test OK')
        else:
            self.get_logger().error(
                f'OpenCV {cv2.__version__} cannot decode QR codes. '
                'Install a full build: pip3 install "numpy<2" opencv-python-headless==4.8.1.78')

    def _report(self):
        if not self.got_image:
            self.get_logger().warn(
                f'No image received yet. Check: ros2 topic hz {self.image_topic}')
            return
        fps = self._frames / 5.0
        avg = self._decode_ms / self._decode_count if self._decode_count else 0.0
        self.get_logger().info(
            f'camera ~{fps:.1f} fps | decode avg {avg:.0f} ms over {self._decode_count} frames')
        self._frames = 0
        self._decode_ms = 0.0
        self._decode_count = 0

    # ---------------- camera callback: must stay fast ----------------
    def image_callback(self, msg: Image):
        if not self.got_image:
            self.got_image = True
            self.get_logger().info(f'First image received: {msg.width}x{msg.height} {msg.encoding}')
        self._frames += 1

        frame = self._msg_to_bgr(msg)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:             # camera driver without stamps: use reception time
            stamp = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._latest = (frame, stamp)
        self._new_frame.set()

        now = time.monotonic()
        if now - self.last_debug < self.debug_period:
            return
        self.last_debug = now

        s = self.debug_scale if 0.0 < self.debug_scale < 1.0 else 1.0
        debug = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s != 1.0 else frame.copy()
        det = self._last_det
        if det is not None and now - det[2] < 0.5:      # show the last hit for half a second
            pts, text, _ = det
            if pts is not None:
                cv2.polylines(debug, [(pts * s).astype(int)], True, (0, 255, 0), 2)
            cv2.putText(debug, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        self.debug_pub.publish(self._bgr_to_msg(debug, msg.header))

    # ---------------- decode worker: can be slow, nobody waits ----------------
    def _worker_loop(self):
        while not self._stop.is_set():
            if not self._new_frame.wait(timeout=0.5):
                continue
            self._new_frame.clear()
            with self._lock:
                latest = self._latest
            if latest is None:
                continue
            frame, stamp = latest

            t0 = time.monotonic()
            try:
                data, pts = self._decode(frame)
            except Exception as exc:
                self.get_logger().error(f'decode error: {exc}')
                continue
            self._decode_ms += (time.monotonic() - t0) * 1000.0
            self._decode_count += 1

            if not data:
                continue
            h, w = frame.shape[:2]
            if pts is not None and len(pts) == 4:
                cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
                w_px = qr_side_px(pts)
            else:
                cx, cy, w_px = -1.0, -1.0, -1.0
            self._last_det = (pts, data, time.monotonic())

            out = String()
            out.data = json.dumps({'id': data, 'cx': cx, 'cy': cy, 'img_w': w, 'img_h': h,
                                   'w_px': w_px, 'stamp': stamp})
            self.detections_pub.publish(out)
            self.get_logger().info(f'QR detected: {data} at ({cx:.0f}, {cy:.0f}) w={w_px:.0f}px',
                                   throttle_duration_sec=0.5)

    def _decode(self, frame):
        """One attempt per call, on grayscale. Returns (data, pts in original pixels)."""
        scale, mirrored = self.attempts[self.attempt_idx]
        work = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if mirrored:
            work = cv2.flip(work, 1)
        if scale != 1:
            work = cv2.resize(work, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)

        data, points = '', None
        # Several QRs can be in view (Gazebo world, or a room with many codes): the single-QR
        # decoder then fails at some headings, so read them all and keep the biggest (= nearest).
        if self.use_multi:
            try:
                ok, infos, pts_all, _ = self.qr_decoder.detectAndDecodeMulti(work)
                best_w = -1.0
                if ok and pts_all is not None:
                    for info, q in zip(infos, pts_all):
                        q = np.asarray(q, dtype=float).reshape(-1, 2)
                        if not info or len(q) != 4:
                            continue
                        w = qr_side_px(q)
                        if w > best_w:
                            best_w, data, points = w, info, q
            except (AttributeError, cv2.error):
                self.use_multi = False
                self.get_logger().warn('detectAndDecodeMulti not available: single-QR decoding')
        if not data:
            data, points, _ = self.qr_decoder.detectAndDecode(work)
        if not data:
            self.attempt_idx = (self.attempt_idx + 1) % len(self.attempts)
            return '', None

        pts = np.asarray(points, dtype=float).reshape(-1, 2) / scale if points is not None else None
        if mirrored:
            if pts is not None:
                pts = pts.copy()
                pts[:, 0] = (frame.shape[1] - 1) - pts[:, 0]
            if not self.mirror_warned:
                self.get_logger().warn('QR only decodes when mirrored -> the print/texture is mirrored.')
                self.mirror_warned = True
        return data, pts

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
