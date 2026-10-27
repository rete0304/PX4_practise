"""Publish synthetic px4_msgs/VisionTarget without a camera (THROUGH spec 12.6).

Used for M1/M2 to test the DDS link, mode registration and condition one.

Parameters (all can be changed at runtime with ``ros2 param set``):
    rate_hz        publish rate [Hz]
    mode           'fixed' or 'sine'
    pan_deg        pan error offset [deg] (+ = target right of center)
    tilt_deg       tilt error offset [deg] (+ = target below center)
    amp_pan_deg    sine amplitude of pan [deg]
    amp_tilt_deg   sine amplitude of tilt [deg]
    freq_hz        sine frequency [Hz]
    target_size    enclosing-circle diameter / image width [-]
    confidence     detection quality [0..1]
    uncage         operator lock state
    target_valid   target detected flag
    latency_ms     simulated capture-to-publish latency [ms]

Services:
    ~/set_lock     std_srvs/SetBool, sets ``uncage``
    ~/set_valid    std_srvs/SetBool, sets ``target_valid``

Run with ``use_sim_time:=true`` and a /clock bridge so that timestamps are
directly in the PX4 SITL clock (requires UXRCE_DDS_SYNCT=0, see VERIFICATION.md D2).
"""

import math

import rclpy
from px4_msgs.msg import VisionTarget
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool


class FakeVisionTarget(Node):

    def __init__(self):
        super().__init__('fake_vision_target')

        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('mode', 'fixed')
        self.declare_parameter('pan_deg', 0.0)
        self.declare_parameter('tilt_deg', 0.0)
        self.declare_parameter('amp_pan_deg', 5.0)
        self.declare_parameter('amp_tilt_deg', 5.0)
        self.declare_parameter('freq_hz', 0.2)
        self.declare_parameter('target_size', 0.1)
        self.declare_parameter('confidence', 1.0)
        self.declare_parameter('uncage', False)
        self.declare_parameter('target_valid', True)
        self.declare_parameter('latency_ms', 20.0)
        self.declare_parameter('topic', '/fmu/in/vision_target')

        # PX4 DataReader is BEST_EFFORT / VOLATILE / KEEP_LAST (VERIFICATION.md V7)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        topic = self.get_parameter('topic').value
        self._pub = self.create_publisher(VisionTarget, topic, qos)

        self.create_service(SetBool, '~/set_lock', self._on_set_lock)
        self.create_service(SetBool, '~/set_valid', self._on_set_valid)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._seq = 0
        self._t0_us = None
        self._waiting_for_clock_logged = False
        self._timer = self.create_timer(1.0 / self.get_parameter('rate_hz').value, self._on_timer)

        self.get_logger().info(
            f'publishing {topic} at {self.get_parameter("rate_hz").value:.1f} Hz, '
            f'use_sim_time={self.get_parameter("use_sim_time").value}')

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == 'rate_hz':
                if p.value <= 0.0:
                    return SetParametersResult(successful=False, reason='rate_hz must be > 0')
                self._timer.cancel()
                self._timer = self.create_timer(1.0 / p.value, self._on_timer)
            elif p.name == 'mode' and p.value not in ('fixed', 'sine'):
                return SetParametersResult(successful=False, reason="mode must be 'fixed' or 'sine'")
        return SetParametersResult(successful=True)

    def _set_bool_param(self, name, value):
        self.set_parameters([Parameter(name, Parameter.Type.BOOL, value)])
        self.get_logger().info(f'{name} -> {value}')

    def _on_set_lock(self, request, response):
        self._set_bool_param('uncage', request.data)
        response.success = True
        response.message = 'locked' if request.data else 'unlocked'
        return response

    def _on_set_valid(self, request, response):
        self._set_bool_param('target_valid', request.data)
        response.success = True
        response.message = 'valid' if request.data else 'invalid'
        return response

    def _on_timer(self):
        now_us = self.get_clock().now().nanoseconds // 1000

        if now_us == 0:
            # use_sim_time without /clock yet: PX4 would replace 0 by its receive time
            if not self._waiting_for_clock_logged:
                self.get_logger().warn('clock is 0 (no /clock yet?), not publishing')
                self._waiting_for_clock_logged = True
            return

        if self._t0_us is None:
            self._t0_us = now_us

        get = self.get_parameter
        pan = math.radians(get('pan_deg').value)
        tilt = math.radians(get('tilt_deg').value)

        if get('mode').value == 'sine':
            phase = 2.0 * math.pi * get('freq_hz').value * (now_us - self._t0_us) * 1e-6
            pan += math.radians(get('amp_pan_deg').value) * math.sin(phase)
            tilt += math.radians(get('amp_tilt_deg').value) * math.cos(phase)

        latency_us = int(get('latency_ms').value * 1000.0)

        msg = VisionTarget()
        msg.timestamp = now_us
        msg.timestamp_sample = max(now_us - latency_us, 1)
        msg.pan_error = pan
        msg.tilt_error = tilt
        msg.target_size = float(get('target_size').value)
        msg.confidence = float(get('confidence').value)
        msg.uncage = bool(get('uncage').value)
        msg.target_valid = bool(get('target_valid').value)
        msg.seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFFFFFF

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVisionTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
