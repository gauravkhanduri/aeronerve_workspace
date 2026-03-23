#!/usr/bin/env python3


import threading
import time

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
import rclpy
from rclpy.node import Node


class TakeoffLand(Node):

    def __init__(self):
        super().__init__("takeoff_land")

        self.declare_parameter("altitude", 2.0)
        self.altitude = self.get_parameter("altitude").value

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")
        self.land_client = self.create_client(CommandTOL, "/mavros/cmd/land")

        self.get_logger().info("TakeoffLand node started. Launching sequence in 3 s...")
        threading.Thread(target=self._sequence, daemon=True).start()

    # ------------------------------------------------------------------
    # Blocking service call — safe to use from a non-executor thread
    # ------------------------------------------------------------------
    def _call(self, client, request, name):
        client.wait_for_service(timeout_sec=10.0)
        future = client.call_async(request)
        # Poll until done — avoids spin_until_future_complete deadlock
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        result = future.result()
        if result is None:
            self.get_logger().error(f"{name}: no response")
        else:
            self.get_logger().info(f"{name}: result={result}")
        return result

    # ------------------------------------------------------------------
    # Main sequence (runs in its own thread)
    # ------------------------------------------------------------------
    def _sequence(self):
        time.sleep(3.0)  # let MAVROS finish connecting

        # 1. Arm
        self.get_logger().info("Arming...")
        req = CommandBool.Request()
        req.value = True
        self._call(self.arming_client, req, "arming")

        time.sleep(2.0)  # ArduPilot needs a moment after arming

        # 2. Takeoff
        self.get_logger().info(f"Taking off to {self.altitude} m...")
        req = CommandTOL.Request()
        req.altitude = self.altitude
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        self._call(self.takeoff_client, req, "takeoff")

        # 3. Hover
        self.get_logger().info("Hovering for 5 seconds...")
        time.sleep(5.0)

        # 4. Land
        self.get_logger().info("Landing...")
        req = CommandTOL.Request()
        req.altitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        self._call(self.land_client, req, "land")

        self.get_logger().info("Done.")


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
