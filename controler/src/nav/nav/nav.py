import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from enum import Enum, auto
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .vision import process

class State(Enum):
    STANDBY = auto()
    START = auto()
    STOP = auto()

class FiniteStateMachine:
    def __init__(self):
        self.current_state: State = State.STANDBY

    def get(self):
        return self.current_state

    def set(self, state: State):
        self.current_state = state

class Nav(Node):
    def __init__(self):
        super().__init__("nav")
        self.lateral_error = 0.0
        self.heading_error = 0.0
        self.fsm = FiniteStateMachine()

        self.declare_parameter("nav_hz", 10)
        self.declare_parameter("base_speed", 2.0)
        self.declare_parameter("turn_speed", 4.0)
        self.declare_parameter("error_threshold", 20.0)  # mismas unidades que lateral_error

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            qos_profile_sensor_data
        )

        self.subscription_state = self.create_subscription(
            String,
            "/state",
            self.state_cb,
            10
        )

        self.setpoints = self.create_publisher(
            Float32MultiArray,
            "arduino/setpoints",
            10
        )

        self.publisher_mask = self.create_publisher(
            Image,
            "/image_processed/mask",
            10
        )

        self.publisher_vis = self.create_publisher(
            Image,
            "/image_processed/vis",
            10
        )

        self.nav_timer = self.create_timer(
            1 / self.get_parameter("nav_hz").value,
            self.nav_cb
        )

    def state_cb(self, msg):
        command = msg.data.strip().lower()

        if command == "start":
            self.fsm.set(State.START)
        elif command == "stop":
            self.fsm.set(State.STOP)
        else:
            self.get_logger().warn(f"Unknown state command: '{msg.data}'")
            return

        self.get_logger().info(f"State -> {self.fsm.get().name}")

    def nav_cb(self):
        if self.fsm.get() == State.START:
            base_speed = self.get_parameter("base_speed").value
            turn_speed = self.get_parameter("turn_speed").value
            threshold = self.get_parameter("error_threshold").value

            left_wheel_setpoint = base_speed
            right_wheel_setpoint = base_speed

            if self.lateral_error < -threshold:
                # left
                left_wheel_setpoint = base_speed
                right_wheel_setpoint = turn_speed
            elif self.lateral_error > threshold:
                # right
                left_wheel_setpoint = turn_speed
                right_wheel_setpoint = base_speed
            else:
                # error is acceptable
                left_wheel_setpoint = base_speed
                right_wheel_setpoint = base_speed

            msg = Float32MultiArray()
            msg.data = [float(left_wheel_setpoint), float(right_wheel_setpoint)]
            self.setpoints.publish(msg)

        if self.fsm.get() == State.STOP:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0]
            self.setpoints.publish(msg)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        vis, mask, lateral_error, heading_error = process(frame)
        self.lateral_error = lateral_error
        self.heading_error = heading_error

        out = self.bridge.cv2_to_imgmsg(mask, "mono8")
        out.header = msg.header
        self.publisher_mask.publish(out)

        out = self.bridge.cv2_to_imgmsg(vis, "bgr8")
        out.header = msg.header
        self.publisher_vis.publish(out)

def main(args=None):
    rclpy.init(args=args)
    nodo = Nav()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
