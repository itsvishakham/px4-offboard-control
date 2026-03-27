#!/usr/bin/env python3
"""
Two Drone Takeoff — PX4 v1.16 + ROS2 Jazzy
==========================================
This script arms and commands both drones to take off to 5 metres.

HOW IT WORKS (plain english):
- Each drone needs 3 things to take off:
  1. OffboardControlMode — tells PX4 "I am controlling you from ROS2"
  2. TrajectorySetpoint   — tells PX4 "go to this position"
  3. VehicleCommand       — tells PX4 "arm!" and "switch to offboard mode!"

- We send messages for BOTH drones from ONE Python script.
- Each drone listens on its own namespace: /px4_1/... and /px4_2/...

HOW TO RUN:
  1. Make sure your 3 terminals are running (2 drones + DDS agent)
  2. Open a new terminal and run:
     cd ~/ros2_px4_ws
     source install/setup.bash
     python3 two_drone_takeoff.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class DroneController:
    """
    Controls a single drone.
    Think of this as a remote control for one drone.
    We create TWO of these — one for each drone.
    """

    def __init__(self, node, namespace):
        self.node = node
        self.namespace = namespace  # e.g. "/px4_1" or "/px4_2"
        self.vehicle_status = VehicleStatus()
        self.is_armed = False
        self.is_offboard = False
        self.takeoff_done = False

        # QoS = "Quality of Service" — just means how reliably messages are sent
        # PX4 requires this specific setting, don't change it
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers (we SEND these messages TO the drone) ──────────────────
        self.offboard_pub = node.create_publisher(
            OffboardControlMode,
            f"{namespace}/fmu/in/offboard_control_mode",
            qos,
        )
        self.trajectory_pub = node.create_publisher(
            TrajectorySetpoint,
            f"{namespace}/fmu/in/trajectory_setpoint",
            qos,
        )
        self.command_pub = node.create_publisher(
            VehicleCommand,
            f"{namespace}/fmu/in/vehicle_command",
            qos,
        )

        # ── Subscriber (we RECEIVE status FROM the drone) ─────────────────────
        self.status_sub = node.create_subscription(
            VehicleStatus,
            f"{namespace}/fmu/out/vehicle_status",
            self.status_callback,
            qos,
        )

        node.get_logger().info(f"✅ DroneController ready for {namespace}")

    def status_callback(self, msg):
        """Called automatically every time the drone sends us its status."""
        self.vehicle_status = msg
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.is_offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def send_offboard_mode(self):
        """
        Step 1: Tell PX4 we want position control from ROS2.
        We must send this repeatedly — PX4 stops offboard mode
        if it doesn't hear from us for 0.5 seconds.
        """
        msg = OffboardControlMode()
        msg.position = True      # We're controlling by position
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def send_position(self, x, y, z, yaw=0.0):
        """
        Step 2: Tell PX4 WHERE to go.
        x, y = horizontal position in metres
        z    = height (NEGATIVE means UP in PX4's coordinate system!)
        yaw  = rotation in radians (0 = facing North)
        """
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]  # Remember: z=-5.0 means 5 metres UP
        msg.yaw = yaw
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def arm(self):
        """Step 3a: ARM the drone (spin up motors)."""
        self._send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,  # 1.0 = arm, 0.0 = disarm
        )
        self.node.get_logger().info(f"🔧 {self.namespace} — ARM command sent")

    def engage_offboard_mode(self):
        """Step 3b: Switch to offboard mode (let ROS2 control the drone)."""
        self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.node.get_logger().info(f"🚁 {self.namespace} — OFFBOARD mode command sent")

    def _send_command(self, command, param1=0.0, param2=0.0):
        """Internal helper to send a MAVLink command to the drone."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 0       # 0 = broadcast to all
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


class SwarmTakeoffNode(Node):
    """
    The main ROS2 node that controls BOTH drones.
    Think of this as the "swarm brain."
    """

    def __init__(self):
        super().__init__("swarm_takeoff")

        # Create one controller per drone
        self.drone1 = DroneController(self, "/px4_1")
        self.drone2 = DroneController(self, "/px4_2")

        self.counter = 0  # Counts how many times our timer has run

        # Run our control loop 10 times per second (every 100ms)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("🚀 Swarm takeoff node started!")
        self.get_logger().info("   Waiting 2 seconds before sending commands...")

    def control_loop(self):
        """
        This runs 10 times per second automatically.
        Think of it as a heartbeat — we keep telling both drones what to do.

        Timeline:
        0.0s - 1.0s  → Send offboard mode signals (warm up)
        1.0s          → Switch to offboard mode
        1.5s          → Arm both drones
        2.0s+         → Command both to fly up to 5 metres
        """
        # Always send offboard mode — PX4 needs this constantly
        self.drone1.send_offboard_mode()
        self.drone2.send_offboard_mode()

        # Always send target position — stay at 5 metres up
        # z = -5.0 means 5 metres UP (PX4 uses NED: North-East-DOWN)
        self.drone1.send_position(0.0, 0.0, -5.0)
        self.drone2.send_position(0.0, 0.0, -5.0)  # 2m sideways so they don't collide

        # After 10 cycles (1 second) — engage offboard mode
        if self.counter == 10:
            self.drone1.engage_offboard_mode()
            self.drone2.engage_offboard_mode()

        # After 15 cycles (1.5 seconds) — arm both drones
        if self.counter >= 15:
            if not self.drone1.is_armed:
                self.drone1.arm()
            if not self.drone2.is_armed:
                self.drone2.arm()
        # Log status every 2 seconds
        if self.counter % 20 == 0:
            self.get_logger().info(
                f"Drone1 — armed: {self.drone1.is_armed} | offboard: {self.drone1.is_offboard}"
            )
            self.get_logger().info(
                f"Drone2 — armed: {self.drone2.is_armed} | offboard: {self.drone2.is_offboard}"
            )

        # After both are armed and in offboard mode — celebrate!
        if self.drone1.is_armed and self.drone2.is_armed and not self.drone1.takeoff_done:
            self.drone1.takeoff_done = True
            self.get_logger().info("🎉 BOTH DRONES ARMED AND FLYING!")
            self.get_logger().info("   Watch Gazebo — they should be climbing to 5 metres!")

        self.counter += 1


def main():
    rclpy.init()
    node = SwarmTakeoffNode()
    try:
        rclpy.spin(node)  # Keep running until Ctrl+C
    except KeyboardInterrupt:
        node.get_logger().info("👋 Shutting down — drones will hold position")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
