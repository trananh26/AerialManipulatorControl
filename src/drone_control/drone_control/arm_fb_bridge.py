"""
Bridge ESP32 ADC feedback (USB serial) → ROS 2 JointState.

Expected line formats from ESP32 (newline-terminated):
  S:<shoulder_deg>,E:<elbow_deg>
  or raw ADC for bring-up:
  A0:<raw>,A1:<raw>

Close PlatformIO / serial monitor before running — only one process may own the port.

Example:
  ros2 run drone_control arm_fb_bridge --ros-args \\
    -p port:=/dev/ttyUSB0 -p baud:=115200
"""

from __future__ import annotations

import math
import re
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        'pyserial is required: pip3 install pyserial '
        '(or sudo apt install python3-serial)'
    ) from exc


_DEG_RE = re.compile(
    r'S:\s*(-?\d+(?:\.\d+)?)\s*,\s*E:\s*(-?\d+(?:\.\d+)?)',
    re.IGNORECASE,
)
_ADC_RE = re.compile(
    r'A0:\s*(-?\d+)\s*,\s*A1:\s*(-?\d+)',
    re.IGNORECASE,
)


class ArmFbBridge(Node):
    def __init__(self):
        super().__init__('arm_fb_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'arm_base_link')
        self.declare_parameter('publish_hz', 50.0)
        # Optional linear map when ESP32 sends raw ADC: deg = raw * scale + offset
        self.declare_parameter('a0_scale', 1.0)
        self.declare_parameter('a0_offset', 0.0)
        self.declare_parameter('a1_scale', 1.0)
        self.declare_parameter('a1_offset', 0.0)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.frame_id = self.get_parameter('frame_id').value
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.a0_scale = float(self.get_parameter('a0_scale').value)
        self.a0_offset = float(self.get_parameter('a0_offset').value)
        self.a1_scale = float(self.get_parameter('a1_scale').value)
        self.a1_offset = float(self.get_parameter('a1_offset').value)

        self._lock = threading.Lock()
        self._ser_lock = threading.Lock()
        self._shoulder_rad = 0.0
        self._elbow_rad = 0.0
        self._have_sample = False
        self.ser = None
        self._stop = False

        self.pub = self.create_publisher(JointState, '/arm/joint_states', 10)

        if not self._open_serial():
            raise SystemExit(f'Cannot open {self.port}')

        self.get_logger().info(
            f'Listening on {self.port} @ {self.baud}, publishing /arm/joint_states '
            '(close pio device monitor / other serial tools first)'
        )

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        period = 1.0 / max(publish_hz, 1.0)
        self.create_timer(period, self._publish_cb)

    def _open_serial(self) -> bool:
        """Open UART; disable DTR/RTS where possible to reduce ESP32 auto-reset glitches."""
        with self._ser_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            try:
                self.ser = serial.Serial()
                self.ser.port = self.port
                self.ser.baudrate = self.baud
                self.ser.timeout = 0.2
                # Avoid toggling lines that reset many ESP32 USB-UART bridges
                self.ser.dtr = False
                self.ser.rts = False
                self.ser.open()
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
                time.sleep(0.3)  # allow board settle if it still reset
                return True
            except serial.SerialException as exc:
                self.get_logger().error(f'Cannot open {self.port}: {exc}')
                self.ser = None
                return False

    def _read_loop(self):
        while rclpy.ok() and not self._stop:
            with self._ser_lock:
                ser = self.ser
            if ser is None or not ser.is_open:
                self.get_logger().warn(
                    f'Reconnecting {self.port}…', throttle_duration_sec=2.0)
                if not self._open_serial():
                    time.sleep(1.0)
                continue

            try:
                raw = ser.readline()
            except serial.SerialException as exc:
                self.get_logger().error(
                    f'Serial read error: {exc} — will reconnect '
                    '(is another program using the port?)')
                with self._ser_lock:
                    try:
                        if self.ser is not None:
                            self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(0.5)
                continue

            if not raw:
                continue

            try:
                line = raw.decode('utf-8', errors='ignore').strip()
            except Exception:
                continue

            if not line or line.startswith('#'):
                continue

            shoulder_deg = None
            elbow_deg = None

            m = _DEG_RE.search(line)
            if m:
                shoulder_deg = float(m.group(1))
                elbow_deg = float(m.group(2))
            else:
                m = _ADC_RE.search(line)
                if m:
                    a0 = float(m.group(1))
                    a1 = float(m.group(2))
                    shoulder_deg = a0 * self.a0_scale + self.a0_offset
                    elbow_deg = a1 * self.a1_scale + self.a1_offset
                else:
                    self.get_logger().warn(
                        f'Unparsed line: {line!r}', throttle_duration_sec=2.0)
                    continue

            with self._lock:
                self._shoulder_rad = math.radians(shoulder_deg)
                self._elbow_rad = math.radians(elbow_deg)
                self._have_sample = True

    def _publish_cb(self):
        with self._lock:
            if not self._have_sample:
                return
            shoulder = self._shoulder_rad
            elbow = self._elbow_rad

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.name = ['shoulder_joint', 'elbow_joint']
        msg.position = [shoulder, elbow]
        self.pub.publish(msg)

    def destroy_node(self):
        self._stop = True
        with self._ser_lock:
            if self.ser is not None and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmFbBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
