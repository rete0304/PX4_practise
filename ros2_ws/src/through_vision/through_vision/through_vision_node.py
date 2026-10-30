"""THROUGH vision node: HSV color tracking of the ring (THROUGH_MODE_SPEC.md 12).

Subscribes to the gz camera (through ros_gz_bridge), finds the largest blob in
the configured color range and publishes px4_msgs/VisionTarget on
/fmu/in/vision_target for every image:

    pan_error  = atan((u - cx) / fx)   + = target right of the image center
    tilt_error = atan((v - cy) / fy)   + = target below the image center
    target_size = 2 * radius / image width

The operator lock (uncage) is set with the debug window keys ('l' lock, 'u'
unlock, 'q' quit) or the ~/set_lock service (std_srvs/SetBool). Locking is
only accepted while the target is detected.

Timestamps (VERIFICATION.md D2, requires use_sim_time and UXRCE_DDS_SYNCT=0):
    timestamp_sample = image header stamp (capture time, PX4 clock)
    timestamp        = publish time
"""

import math
import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import VisionTarget
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import SetBool

WINDOW_NAME = 'through_vision'


def stamp_to_us(stamp):
    return stamp.sec * 1_000_000 + stamp.nanosec // 1000


class ThroughVision(Node):

    def __init__(self):
        super().__init__('through_vision')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        # red wraps around H = 0 in OpenCV HSV (H in 0..179): two ranges, merged
        self.declare_parameter('hsv_lower_1', [0, 120, 70])
        self.declare_parameter('hsv_upper_1', [10, 255, 255])
        self.declare_parameter('hsv_lower_2', [170, 120, 70])
        self.declare_parameter('hsv_upper_2', [179, 255, 255])
        self.declare_parameter('min_area_px', 30)
        # Opening removes speckle noise but also erases thin lines: at 25-30 m the 0.15 m ring
        # tube is only ~2 px wide, and any opening >= 2 px deletes it. The simulated image has
        # no noise, so opening is off by default (1 = no-op); blobs below min_area_px are dropped.
        self.declare_parameter('open_kernel_px', 1)
        self.declare_parameter('close_kernel_px', 5)
        self.declare_parameter('center_method', 'circle')  # 'circle' | 'moments'
        self.declare_parameter('hfov', math.radians(90.0))  # used only without CameraInfo
        self.declare_parameter('show_window', True)
        self.declare_parameter('auto_unlock_s', 0.0)  # 0 = disabled
        self.declare_parameter('publish_debug_image', True)

        self._bridge = CvBridge()
        self._camera_info = None
        self._uncage = False
        self._seq = 0
        self._last_valid_time = None
        self._fps = 0.0
        self._last_frame_wall = None
        self._debug_frame = None

        # PX4 DataReader is BEST_EFFORT / VOLATILE / KEEP_LAST (VERIFICATION.md V7)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._target_pub = self.create_publisher(VisionTarget, '/fmu/in/vision_target', px4_qos)
        self._debug_pub = self.create_publisher(Image, '~/debug_image', qos_profile_sensor_data)

        self.create_subscription(Image, self.get_parameter('image_topic').value, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._on_camera_info, qos_profile_sensor_data)
        self.create_service(SetBool, '~/set_lock', self._on_set_lock)

        self._show_window = bool(self.get_parameter('show_window').value)
        if self._show_window and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            self.get_logger().warn('no display available, disabling the debug window')
            self._show_window = False
        if self._show_window:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
            # GUI updates and key handling, independent of the image rate
            self.create_timer(1.0 / 30.0, self._on_gui_timer)

        self.get_logger().info(
            f'listening on {self.get_parameter("image_topic").value}, '
            f'use_sim_time={self.get_parameter("use_sim_time").value}, window={self._show_window}')

    # --- lock -----------------------------------------------------------------------------

    def _set_lock(self, lock, target_valid):
        if lock and not target_valid:
            return False, 'target not detected, lock refused'
        if lock != self._uncage:
            self._uncage = lock
            self.get_logger().info('target LOCKED' if lock else 'target UNLOCKED')
        return True, 'locked' if lock else 'unlocked'

    def _on_set_lock(self, request, response):
        valid = self._last_valid_time is not None and \
            (self.get_clock().now() - self._last_valid_time).nanoseconds < 0.5e9
        response.success, response.message = self._set_lock(request.data, valid)
        return response

    # --- image processing -----------------------------------------------------------------

    def _on_camera_info(self, msg):
        if self._camera_info is None:
            self.get_logger().info(
                f'camera info: {msg.width}x{msg.height}, fx={msg.k[0]:.2f} fy={msg.k[4]:.2f} '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}')
        self._camera_info = msg

    def _intrinsics(self, width, height):
        if self._camera_info is not None and self._camera_info.k[0] > 0.0:
            k = self._camera_info.k
            return k[0], k[4], k[2], k[5]
        fx = (width / 2.0) / math.tan(self.get_parameter('hfov').value / 2.0)
        return fx, fx, width / 2.0, height / 2.0

    def _mask(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        get = self.get_parameter
        mask = cv2.inRange(hsv, np.array(get('hsv_lower_1').value, np.uint8),
                           np.array(get('hsv_upper_1').value, np.uint8))
        mask |= cv2.inRange(hsv, np.array(get('hsv_lower_2').value, np.uint8),
                            np.array(get('hsv_upper_2').value, np.uint8))
        k_open = int(get('open_kernel_px').value)
        if k_open > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_open, k_open)))
        k_close = int(get('close_kernel_px').value)
        if k_close > 1:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_close, k_close)))
        return mask

    def _detect(self, mask):
        """Return (contour, center (u, v), radius) of the largest blob, or None."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < self.get_parameter('min_area_px').value:
            return None
        (u, v), radius = cv2.minEnclosingCircle(contour)
        if self.get_parameter('center_method').value == 'moments':
            m = cv2.moments(contour)
            if m['m00'] > 0.0:
                u, v = m['m10'] / m['m00'], m['m01'] / m['m00']
        return contour, (u, v), radius

    def _on_image(self, msg):
        now = self.get_clock().now()
        wall = time.monotonic()
        if self._last_frame_wall is not None:
            dt = wall - self._last_frame_wall
            if dt > 0.0:
                self._fps = 0.9 * self._fps + 0.1 / dt if self._fps > 0.0 else 1.0 / dt
        self._last_frame_wall = wall

        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = bgr.shape[:2]
        fx, fy, cx, cy = self._intrinsics(width, height)

        detection = self._detect(self._mask(bgr))
        target = VisionTarget()
        target.confidence = 1.0
        if detection is not None:
            _, (u, v), radius = detection
            target.pan_error = math.atan((u - cx) / fx)
            target.tilt_error = math.atan((v - cy) / fy)
            target.target_size = 2.0 * radius / width
            target.target_valid = True
            self._last_valid_time = now
        else:
            target.pan_error = float('nan')
            target.tilt_error = float('nan')
            target.target_size = 0.0
            target.target_valid = False

        auto_unlock_s = self.get_parameter('auto_unlock_s').value
        if self._uncage and auto_unlock_s > 0.0 and self._last_valid_time is not None and \
                (now - self._last_valid_time).nanoseconds * 1e-9 > auto_unlock_s:
            self.get_logger().warn(f'target lost for more than {auto_unlock_s:.1f} s, auto unlock')
            self._uncage = False

        target.uncage = self._uncage
        target.seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        stamp_us = stamp_to_us(msg.header.stamp)
        now_us = now.nanoseconds // 1000
        target.timestamp_sample = stamp_us if stamp_us > 0 else now_us
        target.timestamp = now_us
        self._target_pub.publish(target)

        if self._show_window or (self.get_parameter('publish_debug_image').value and
                                 self._debug_pub.get_subscription_count() > 0):
            debug = self._draw(bgr, detection, target, (cx, cy))
            self._debug_frame = debug
            if self._debug_pub.get_subscription_count() > 0:
                out = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
                out.header = msg.header
                self._debug_pub.publish(out)

    def _draw(self, bgr, detection, target, center):
        img = bgr.copy()
        h, w = img.shape[:2]
        cx, cy = int(round(center[0])), int(round(center[1]))
        cv2.line(img, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 1)
        cv2.line(img, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 1)
        if detection is not None:
            contour, (u, v), radius = detection
            cv2.drawContours(img, [contour], -1, (0, 255, 0), 1)
            cv2.circle(img, (int(u), int(v)), int(radius), (255, 255, 0), 1)
            cv2.circle(img, (int(u), int(v)), 3, (0, 255, 255), -1)
            text = (f'pan {math.degrees(target.pan_error):+6.2f} deg  '
                    f'tilt {math.degrees(target.tilt_error):+6.2f} deg  size {target.target_size:.3f}')
        else:
            text = 'no target'
        lock_text = 'LOCKED' if self._uncage else 'unlocked'
        cv2.putText(img, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f'{lock_text}  {self._fps:4.1f} fps  [l]ock [u]nlock [q]uit', (8, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        if self._uncage:
            cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 0, 255), 6)
        return img

    # --- GUI ------------------------------------------------------------------------------

    def _on_gui_timer(self):
        if self._debug_frame is not None:
            cv2.imshow(WINDOW_NAME, self._debug_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('l'):
            ok, message = self._set_lock(True, self._last_valid_time is not None and
                                         (self.get_clock().now() - self._last_valid_time).nanoseconds < 0.5e9)
            if not ok:
                self.get_logger().warn(message)
        elif key == ord('u'):
            self._set_lock(False, False)
        elif key == ord('q'):
            self.get_logger().info('quit requested')
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ThroughVision()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
