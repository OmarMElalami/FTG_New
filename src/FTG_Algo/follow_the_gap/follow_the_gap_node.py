import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class StraightDriver(Node):
    """
    Stufe 2: Minimaler Autonomy-Tester (zeitbasiert)
    ARM (0 senden) -> DRIVE -> STOP (0 halten)
    """

    def __init__(self):
        super().__init__("straight_driver")

        # Topics / Rate
        self.declare_parameter("cmd_topic", "/autonomous/ackermann_cmd")
        self.declare_parameter("publish_rate_hz", 20.0)

        # Zeiten
        self.declare_parameter("arm_time_sec", 2.0)     # am Anfang 0 senden
        self.declare_parameter("start_delay_sec", 0.0)  # optional
        self.declare_parameter("drive_time_sec", 2.0)   # fahren
        self.declare_parameter("stop_time_sec", 5.0)    # stop halten

        # Fahrwerte
        self.declare_parameter("speed", 0.30)           # m/s (für erste Tests nicht zu klein!)
        self.declare_parameter("steering_angle", 0.0)   # rad

        # Verhalten
        self.declare_parameter("repeat", False)         # Sequenz wiederholen
        self.declare_parameter("exit_on_finish", False) # am Ende shutdown statt STOP halten

        # Parameter lesen
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.arm_time = float(self.get_parameter("arm_time_sec").value)
        self.start_delay = float(self.get_parameter("start_delay_sec").value)
        self.drive_time = float(self.get_parameter("drive_time_sec").value)
        self.stop_time = float(self.get_parameter("stop_time_sec").value)

        self.speed = float(self.get_parameter("speed").value)
        self.steering = float(self.get_parameter("steering_angle").value)

        self.repeat = bool(self.get_parameter("repeat").value)
        self.exit_on_finish = bool(self.get_parameter("exit_on_finish").value)

        # Publisher
        self.pub = self.create_publisher(AckermannDriveStamped, self.cmd_topic, 10)

        # State machine
        self.state = "ARM"
        self.state_t0 = self.get_clock().now()

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"StraightDriver started -> {self.cmd_topic} | "
            f"ARM={self.arm_time}s DELAY={self.start_delay}s DRIVE={self.drive_time}s STOP={self.stop_time}s | "
            f"speed={self.speed} steering={self.steering} | repeat={self.repeat} exit_on_finish={self.exit_on_finish}"
        )

    def publish_cmd(self, speed: float, steering: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.pub.publish(msg)

    def elapsed(self) -> float:
        return (self.get_clock().now() - self.state_t0).nanoseconds * 1e-9

    def switch(self, new_state: str):
        self.state = new_state
        self.state_t0 = self.get_clock().now()
        self.get_logger().info(f"STATE -> {new_state}")

    def on_timer(self):
        t = self.elapsed()

        if self.state == "ARM":
            self.publish_cmd(0.0, 0.0)
            if t >= self.arm_time:
                self.switch("START_DELAY")

        elif self.state == "START_DELAY":
            self.publish_cmd(0.0, 0.0)
            if t >= self.start_delay:
                self.switch("DRIVE")

        elif self.state == "DRIVE":
            self.publish_cmd(self.speed, self.steering)
            if t >= self.drive_time:
                self.switch("STOP")

        elif self.state == "STOP":
            self.publish_cmd(0.0, 0.0)
            if t >= self.stop_time:
                if self.repeat:
                    self.switch("ARM")
                else:
                    if self.exit_on_finish:
                        self.get_logger().info("Finished -> shutdown")
                        rclpy.shutdown()
                    # sonst: STOP halten

        else:
            # safety fallback
            self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = StraightDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_cmd(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()