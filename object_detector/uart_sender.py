import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray

"""
For the raspbi config (see https://www.pyserial.com/docs/raspberry-pi):
Don't forget to edit /boo/firmware/config.txt
Edit with the following content:
    enable_uart=1
    dtoverlay=disable-bt

Disable serial console:
    sudo systemctl disable serial-getty@ttyAMA0.service
    sudo systemctl disable serial-getty@serial0.service

And then reboot:
    sudo reboot

Finally check if port exist:
    ls -l /dev/serial0

If `Permission denied` error:
    Add user to the dialout group
    chmod 777 /dev/serial0
"""


class UARTSender(Node):
    """Receive the distance and the angle of a detected object and send it via UART interface

    Attributes:
        port: the UART port of the raspi (usually /dev/serial0)
        subscription: ROS2 subscription to the i2c_data topic for the object detected data
    """

    def __init__(self):
        super().__init__("uart_sender")

        self.declare_parameter("port", "/dev/serial0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("debug", False)

        self.debug = self.get_parameter("debug").value
        self.baudrate = self.get_parameter("baudrate").value
        self.port = self.get_parameter("port").value

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
            )
            self.get_logger().info(f"UART initialized on port {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize UART: {e}")
            return

        # Subscribe to the uart_data
        self.subscription = self.create_subscription(
            UInt16MultiArray, "/uart_data", self.data_callback, 10
        )

        self.get_logger().info("UART Sender ready!")

    def data_callback(self, msg):
        """Extract the value from the msg send by the uart_data topic when data are received

        Args:
            msg (std_msgs.msg.UInt16MultiArray): A specific message with the distance in millimeter and angle in degree inside array of data
        """

        # Extract values from array
        if len(msg.data) != 2:
            self.get_logger().warn(f"Expected 2 values, got {len(msg.data)}")
            return

        distance_mm = msg.data[0]
        angle_deg = msg.data[1]

        if self.debug:
            self.get_logger().info(
                f"Theses data will be sent: distance={distance_mm}mm, angle={angle_deg}°"
            )

        # Send the data using the UART interface
        try:
            self.uart_sender(distance_mm, angle_deg)
        except Exception as e:
            self.get_logger().error(f"UART transmission failed: {e}")

    def uart_sender(self, distance: int, angle: int):
        """Send the distance and angle using UART link
        Args:
            distance: Distance in millimeter
            angle: Angle in degree
        """

        message = f"{distance};{angle}\n"

        self.ser.write(message.encode("utf-8"))


def main(args=None):
    rclpy.init(args=args)
    sender = UARTSender()

    try:
        rclpy.spin(sender)
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
