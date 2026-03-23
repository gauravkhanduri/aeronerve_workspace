import subprocess
import time

from mavros_msgs.srv import SetMode
import rclpy
from rclpy.node import Node


class GCSWatchdog(Node):
    def __init__(self):
        super().__init__("gcs_watchdog")

        # Parameters
        self.declare_parameter("laptop_ip", "10.42.0.50")
        self.declare_parameter("timeout", 5.0)  # Seconds

        self.laptop_ip = self.get_parameter("laptop_ip").get_parameter_value().string_value
        self.timeout_limit = self.get_parameter("timeout").get_parameter_value().double_value

        # Client to change flight mode
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Timer to check connection every 1 second
        self.last_seen = time.time()
        self._loiter_triggered = False
        self.timer = self.create_timer(1.0, self.check_connection)

        self.get_logger().info(f"Watching GCS at {self.laptop_ip}...")

    def check_connection(self):
        # Ping the laptop (1 packet, 1 second timeout) — non-blocking
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "1", self.laptop_ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        if result.returncode == 0:
            self.last_seen = time.time()
            self._loiter_triggered = False
        else:
            elapsed = time.time() - self.last_seen
            if elapsed > self.timeout_limit and not self._loiter_triggered:
                self.get_logger().error(f"GCS LOST for {elapsed:.1f}s! Switching to LOITER.")
                self._loiter_triggered = True
                self.trigger_loiter()

    def trigger_loiter(self):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("MAVROS SetMode service not available!")
            return

        req = SetMode.Request()
        req.custom_mode = "LOITER"
        future = self.mode_client.call_async(req)
        future.add_done_callback(self._on_mode_set)

    def _on_mode_set(self, future):
        try:
            result = future.result()
            if result.mode_sent:
                self.get_logger().info("LOITER mode set successfully.")
            else:
                self.get_logger().error("LOITER mode request rejected by FCU.")
        except Exception as e:
            self.get_logger().error(f"SetMode service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GCSWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
