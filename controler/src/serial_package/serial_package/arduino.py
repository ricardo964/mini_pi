import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

# Telemetry frame (Arduino -> ROS): 0xAA + 2 floats (8 bytes) + 0x55 = 10 bytes
# struct Telemetry { float left_wheel_velocity; float right_wheel_velocity; };
#
# Command (ROS -> Arduino): 2 floats (8 bytes), no header
# struct Command { float left_wheel_setpoint; float right_wheel_setpoint; };

HEADER = 0xAA
FOOTER = 0x55
FRAME_SIZE = 10


class ArduinoNode(Node):
    def __init__(self):
        super().__init__("arduino")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("read_hz", 100)

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value
        read_hz = self.get_parameter("read_hz").value

        try:
            self.serial = serial.Serial(
                port,
                baudrate=baudrate,
                timeout=0,
                write_timeout=0.1
            )
        except serial.SerialException as e:
            self.get_logger().fatal(f"fatal {port}: {e}")
            raise

        self.rx_buffer = bytearray()

        self.wheels_velocities = self.create_publisher(
            Float32MultiArray,
            "arduino/wheels_velocities",
            10
        )

        self.setpoints = self.create_subscription(
            Float32MultiArray,
            "arduino/setpoints",
            self.setpoints_cb,
            10
        )

        self.read_timer = self.create_timer(1 / read_hz, self.read_timer_cb)

    def read_timer_cb(self):
        try:
            data = self.serial.read(self.serial.in_waiting)
        except serial.SerialException as e:
            self.get_logger().error(f"serial read error: {e}")
            return

        if data:
            self.rx_buffer.extend(data)

        while True:
            start = self.rx_buffer.find(bytes([HEADER]))

            if start == -1:
                self.rx_buffer.clear()
                break

            # drop garbage before the header
            del self.rx_buffer[:start]

            if len(self.rx_buffer) < FRAME_SIZE:
                break  # incomplete frame, wait for the next tick

            if self.rx_buffer[FRAME_SIZE - 1] != FOOTER:
                # false header: drop one byte and resync
                del self.rx_buffer[0]
                continue

            frame = bytes(self.rx_buffer[:FRAME_SIZE])
            del self.rx_buffer[:FRAME_SIZE]

            left_wheel_velocity, right_wheel_velocity = struct.unpack('<ff', frame[1:9])

            msg = Float32MultiArray()
            msg.data = [left_wheel_velocity, right_wheel_velocity]
            self.wheels_velocities.publish(msg)

    def setpoints_cb(self, message: Float32MultiArray):
        if len(message.data) < 2:
            self.get_logger().warn("setpoints needs 2 values")
            return

        payload = struct.pack('<ff', message.data[0], message.data[1])

        try:
            self.serial.write(payload)
        except serial.SerialException as e:
            self.get_logger().error(f"serial write error: {e}")

    def destroy_node(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = ArduinoNode()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
