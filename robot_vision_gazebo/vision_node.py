"""
vision_node.py
Nodo de visión y control para el proyecto Robot-Vision (ROS2 + Gazebo + OpenCV).

Pipeline:
  /camera/image_raw --> cv_bridge --> resize 320x240 --> Partición ROI (60% sup / 40% inf)
      ROI superior -> detección de obstáculo rojo (trigger FSM)
      ROI inferior -> segmentación HSV de línea blanca -> centroide (Cx) -> error
  error --> PID --> angular_z
  FSM (si obstáculo) --> override cinemático ciego (lazo abierto)
  --> publica geometry_msgs/Twist en /cmd_vel
  --> publica std_msgs/Float32 en /vision_controller/error, /vision_controller/fps
  --> publica std_msgs/Bool en /vision_controller/obstacle
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from .fsm_rebase import EvasionFSM
from .pid_controller import PIDController


class VisionControlNode(Node):
    def __init__(self):
        super().__init__('vision_control_node')

        # --- Parámetros dinámicos (ajustables sin recompilar) ---
        self.declare_parameter('Kp', 0.005)
        self.declare_parameter('Kd', 0.002)
        self.declare_parameter('Ki', 0.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('roi_split', 0.6)          # 60% superior / 40% inferior
        self.declare_parameter('red_area_threshold', 1500)
        self.declare_parameter('resize_width', 320)
        self.declare_parameter('resize_height', 240)
        self.declare_parameter('publish_fps', True)
        self.declare_parameter('fps_smoothing', 0.85)
        self.declare_parameter('line_presence_threshold', 0.005)
        self.declare_parameter('hsv_line_low', [0, 0, 200])
        self.declare_parameter('hsv_line_high', [180, 30, 255])
        self.declare_parameter('hsv_red_low1', [0, 120, 70])
        self.declare_parameter('hsv_red_high1', [10, 255, 255])
        self.declare_parameter('hsv_red_low2', [170, 120, 70])
        self.declare_parameter('hsv_red_high2', [180, 255, 255])

        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.roi_split = self.get_parameter('roi_split').value
        self.red_area_threshold = self.get_parameter('red_area_threshold').value
        self.resize_width = self.get_parameter('resize_width').value
        self.resize_height = self.get_parameter('resize_height').value
        self.publish_fps = self.get_parameter('publish_fps').value
        self.fps_smoothing = self.get_parameter('fps_smoothing').value
        self.line_presence_threshold = self.get_parameter('line_presence_threshold').value

        self.bridge = CvBridge()
        self.pid = PIDController(
            kp=self.get_parameter('Kp').value,
            kd=self.get_parameter('Kd').value,
            ki=self.get_parameter('Ki').value,
        )
        self.fsm = EvasionFSM(linear_speed=self.linear_speed, turn_duration=2.0, advance_duration=2.0, return_max_duration=6.0)
        self.fsm.on_state_change = self._log_fsm_transition
        self._reacquire_count = 0
        self._reacquire_needed = 5

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.error_pub = self.create_publisher(Float32, '/vision_controller/error', 10)
        self.fps_pub = self.create_publisher(Float32, '/vision_controller/fps', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/vision_controller/obstacle', 10)

        self.last_time = self.get_clock().now()
        self.smoothed_fps = 0.0

        self.get_logger().info('🟢 Nodo de control de visión inicializado.')

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error en cv_bridge: {e}')
            return

        frame = cv2.resize(frame, (self.resize_width, self.resize_height), interpolation=cv2.INTER_AREA)
        h, w, _ = frame.shape
        split = int(self.roi_split * h)
        roi_evasion = frame[0:split, :]     # 60% superior -> obstáculo
        roi_lines = frame[split:, :]        # 40% inferior -> línea

        obstacle_detected = self._detect_obstacle(roi_evasion)
        self.obstacle_pub.publish(Bool(data=obstacle_detected))
        self.fsm.trigger(obstacle_detected)
        if obstacle_detected:
            self.get_logger().warn(f'🔴 Obstáculo detectado | Estado FSM: {self.fsm.state.name}')

        twist = Twist()
        dt = self._update_timing()

        if self.fsm.is_following():
            error = self._compute_line_error(roi_lines, w)
            if error is not None:
                angular_z = -self.pid.compute(float(error), dt=dt)
                angular_z = max(min(angular_z, self.max_angular), -self.max_angular)
                twist.linear.x = self.linear_speed
                twist.angular.z = angular_z
                self._publish_error(float(error))
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().warn('⚠️ Línea no detectada en ROI inferior.')
        else:
            raw_line_detected = self._line_exists(roi_lines)
            if raw_line_detected:
                self._reacquire_count += 1
            else:
                self._reacquire_count = 0
            line_detected = self._reacquire_count >= self._reacquire_needed
            linear_x, angular_z, _ = self.fsm.step(line_detected=line_detected, dt=dt)
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.pid.reset()

        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    def _update_timing(self) -> float:
        now = self.get_clock().now()
        dt = float((now - self.last_time).nanoseconds) * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 0.05  # valor de respaldo ~20Hz, evita que el timer de la FSM se congele

        fps = 1.0 / dt
        if self.smoothed_fps <= 0.0:
            self.smoothed_fps = fps
        else:
            self.smoothed_fps = (self.fps_smoothing * self.smoothed_fps) + ((1.0 - self.fps_smoothing) * fps)

        if self.publish_fps:
            self.fps_pub.publish(Float32(data=float(self.smoothed_fps)))

        return dt

    # ------------------------------------------------------------------
    def _compute_line_error(self, roi, frame_width: int):
        low = np.array(self.get_parameter('hsv_line_low').value, dtype=np.uint8)
        high = np.array(self.get_parameter('hsv_line_high').value, dtype=np.uint8)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low, high)
        
        # --- AQUI ESTA EL CAMBIO APLICADO ---
        mask = cv2.dilate(mask, None, iterations=1)
        mask = cv2.erode(mask, None, iterations=1)

        m = cv2.moments(mask)
        if m['m00'] > 0:
            cx = int(m['m10'] / m['m00'])
            return cx - (frame_width // 2)
        return None

    def _line_exists(self, roi) -> bool:
        low = np.array(self.get_parameter('hsv_line_low').value, dtype=np.uint8)
        high = np.array(self.get_parameter('hsv_line_high').value, dtype=np.uint8)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low, high)
        
        # --- AQUI ESTA EL CAMBIO APLICADO ---
        mask = cv2.dilate(mask, None, iterations=1)
        mask = cv2.erode(mask, None, iterations=1)

        count = int(cv2.countNonZero(mask))
        threshold = int(roi.shape[0] * roi.shape[1] * self.line_presence_threshold)
        return count > threshold

    def _detect_obstacle(self, roi) -> bool:
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        low1 = np.array(self.get_parameter('hsv_red_low1').value, dtype=np.uint8)
        high1 = np.array(self.get_parameter('hsv_red_high1').value, dtype=np.uint8)
        low2 = np.array(self.get_parameter('hsv_red_low2').value, dtype=np.uint8)
        high2 = np.array(self.get_parameter('hsv_red_high2').value, dtype=np.uint8)

        mask1 = cv2.inRange(hsv, low1, high1)
        mask2 = cv2.inRange(hsv, low2, high2)
        mask = cv2.bitwise_or(mask1, mask2)

        area = int(cv2.countNonZero(mask))
        return area > self.red_area_threshold

    def _log_fsm_transition(self, old_state, new_state):
        self.get_logger().info(f'FSM: {old_state.name} -> {new_state.name}')

    def _publish_error(self, error: float):
        msg = Float32()
        msg.data = error
        self.error_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
