import threading
import time

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
    """Receive the distance and the angle of a detected object and send it via UART interface when UART message is received and ask for it

    Attributes:
        port: The UART port of the raspi (usually /dev/serial0)
        debug: Bool to get debug output
        baudrate: Baudrate used by the Serial
        subscription: ROS2 subscription to the i2c_data topic for the object detected data
        distance_mm: Distance retrieved from uart_data
        angle_deg: Angle retrieved from uart_data
        last_msg_time: Time value from the last message coming from uart_data
        timer: ROS2 timer that call check_timeout to reset distance_mm and angle_deg after no new value
        lock: Lock for thread var
        uart_running: Bool to stop the uart_reader function
        uart_thread: UART thread to read incoming message
    """

    def __init__(self):
        super().__init__("uart_sender")

        # Program settings
        self.declare_parameter("port", "/dev/serial0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("debug", False)

        self.debug = self.get_parameter("debug").value
        self.baudrate = self.get_parameter("baudrate").value
        self.port = self.get_parameter("port").value

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
            )
            self.get_logger().info(f"UART initialized on port {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize UART: {e}")
            return

        # Data manipulated by uart_thread and the data_callback
        self.distance_mm = 0
        self.angle_deg = 0

        # Timer to check for timeout
        self.last_msg_time = time.time()
        self.timer = self.create_timer(0.1, self.check_timeout)

        # Lock for threading var
        self.lock = threading.Lock()

        # UART thread
        self.uart_running = True
        self.uart_thread = threading.Thread(target=self.uart_reader)
        self.uart_thread.daemon = True
        self.uart_thread.start()

        # Subscribe to the uart_data from object_detector.py
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

        with self.lock:
            self.distance_mm = msg.data[0]
            self.angle_deg = msg.data[1]
            self.last_msg_time = time.time()

            if self.debug:
                self.get_logger().info(
                    f"Theses data can be sent: distance={self.distance_mm}mm, angle={self.angle_deg}°"
                )

    def uart_reader(self):
        """Read message coming from the UART and immediatly send a response depending of the id"""

        if self.uart_running:
            data = self.serial.readline().decode("utf-8").strip()

            if self.debug:
                self.get_logger().info(f"Data coming from uart: {data}")

            if not data == "1;42;42" or not data == "2;42;42":
                self.get_logger().error("Wrong data coming from uart")
                return

            id = int(data[0])

            if id == 1:
                self.uart_sender(id)

            elif id == 2:
                with self.lock:
                    distance_mm = self.distance_mm
                    angle_mm = self.angle_deg
                    self.uart_sender(id, distance_mm, angle_mm)

            else:
                self.get_logger().error(f"Wrong id. Got: {id}, expecting 1 or 2")

    def uart_sender(self, id: int = 0, distance_mm: int = 0, angle_deg: int = 0):
        """Send a ping or the distance and angle using UART link

        Args:
            id: 1 for a ping, 2 to send data
            distance_mm: Distance in millimeter
            angle_deg: Angle in degree
        """

        if id != 1 and id != 2:
            self.get_logger().error(f"Wrong id. Got: {id}, expecting 1 or 2")

        elif id == 1:
            message = f"{id};42;42\n"
            self.get_logger().info(f"The following message will be send: {message}")
            self.serial.write(message.encode("utf-8"))

        elif id == 2:
            message = f"{id};{distance_mm};{angle_deg}\n"
            self.get_logger().info(f"The following message will be send: {message}")
            self.serial.write(message.encode("utf-8"))

    def check_timeout(self):
        """After a short time (no more detected object), reset the distance_mm and angle_deg to default value"""

        with self.lock:
            elapsed = time.time() - self.last_msg_time

            if elapsed > 1.0:
                self.distance_mm = 0
                self.angle_deg = 0

    def destroy_node(self):
        """Close the node, the thread and the Serial"""

        self.uart_running = False
        self.uart_thread.join()

        self.serial.close()

        super().destroy_node()


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
