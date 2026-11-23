import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

# 8 bytes + 2 bytes
# struct Telemetry
# {
#   float left_wheel_velocity;
#   float right_wheel_velocity;
# };

# 8 bytes
# struct Command
# {
#   float left_wheel_setpoint;
#   float right_wheel_setpoint;
# };

class ArduinoNode(Node):
    def __init__(self):
        super().__init__("arduino")

        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baurate", 115200)
        self.declare_parameter("read_hz", 50)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baurate").value
        read_hx = self.get_parameter("read_hz").value

        try:
            self.serial = serial.Serial(
                port,
                baudrate=baud,
                timeout=0,
                write_timeout=0.1
            )
        except serial.SerialException as e:
            self.get_logger().fatal(f"fatal {port}: {e}")
            raise

        self.wheels_velocities = self.create_publisher(
            Float32MultiArray,
            "arduino/wheels_velocities",
            10
        )

        self.setpoints = self.create_subscription(
            Float32MultiArray,
            "arduino/setpoints",
            self.write_timer_cb,
            10
        )

        # timer
        self.read_timer = self.create_timer(1/read_hx, self.read_timer_cb)

    def read_timer_cb(self):
        try:
            # 0xAA 0x55
            while True:
                byte = self.serial.read(1)
                if byte == b'\xAA':
                    buff = byte
                    payload = self.serial.read(9)

                    if len(payload) != 9:
                        self.get_logger().error("Error invalid len")
                        continue

                    buff += payload

                    if buff[-1] == b'\x55':
                        self.get_logger().error("error")
                        continue

                    telemetry_data = buff[1:-1]

                    left_wheel_velocity = struct.unpack('f', telemetry_data[0:4])[0]
                    right_wheel_velocity = struct.unpack('f', telemetry_data[4:8])[0]

                    # self.get_logger().info(
                    #     f"left_wheel_velocity {left_wheel_velocity}, right_wheel_velocity {right_wheel_velocity}"
                    # )

                    msg = Float32MultiArray()
                    msg.data = [left_wheel_velocity, right_wheel_velocity]
                    self.wheels_velocities.publish(msg)

                    break

        except Exception as e:
            self.get_logger().error("except")
            raise

    def write_timer_cb(self, message: Float32MultiArray):
        left_wheel_setpoint = message.data[0]
        right_wheel_setpoint = message.data[1]
        message = struct.pack('<f', left_wheel_setpoint) + struct.pack('<f', right_wheel_setpoint)
        self.serial.write(message)

    def destroy_node(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.ser.close()
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
