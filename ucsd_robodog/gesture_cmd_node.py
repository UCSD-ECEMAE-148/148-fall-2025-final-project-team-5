#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist


class GestureCmdNode(Node):
    def __init__(self):
        super().__init__("gesture_cmd_node")

        # Current high-level state
        # IDLE, FOLLOW_HAND, CLEAR_ROOM, CLEAR_LEFT_CORNER, CLEAR_RIGHT_CORNER
        self.state = "IDLE"

        # Inputs from gesture detector
        self.current_gesture = "UNKNOWN"
        self.current_steering = 0.0
        self.current_throttle = 0.0

        # Mission timing
        self.mission_start_time = None
        self.mission_phase = 0
        self.mission_phase_start = 0.0

        # Subscribers
        self.create_subscription(String, "gesture", self.gesture_callback, 10)
        self.create_subscription(Float32, "steering", self.steering_callback, 10)
        self.create_subscription(Float32, "throttle", self.throttle_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timer for control loop
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        # Speed parameters (tune for your car)
        self.max_linear_speed = 0.15   # m/s for hand-following
        self.max_angular_speed = 1   # rad/s for hand-following

        # Scripted speeds for missions
        self.forward_speed = 0.20      # m/s
        self.reverse_speed = -0.20     # m/s
        self.turn_speed = 0.7          # rad/s

        self.get_logger().info("GestureCmdNode with mission scripts ready")

    # ----------------- Subscribers -----------------

    def gesture_callback(self, msg: String):
        g = msg.data
        self.current_gesture = g

        # FIST = abort / emergency stop from any state
        if g == "FIST":
            self.get_logger().info("FIST detected → aborting mission, going IDLE")
            self.go_idle()
            return

        # Only allow starting missions from IDLE
        if self.state == "IDLE":
            if g == "OPEN PALM":
                self.get_logger().info("Entering FOLLOW_HAND mode")
                self.state = "FOLLOW_HAND"
            elif g == "ONE":
                self.start_clear_left_corner()
            elif g == "TWO":
                self.start_clear_right_corner()
            elif g == "SHAKA":
                self.start_clear_room()
        # In non-IDLE states, ignore other gestures except FIST (handled above)

    def steering_callback(self, msg: Float32):
        self.current_steering = msg.data

    def throttle_callback(self, msg: Float32):
        self.current_throttle = msg.data

    # ----------------- State helpers -----------------

    def go_idle(self):
        self.state = "IDLE"
        self.mission_start_time = None
        self.mission_phase = 0
        self.mission_phase_start = 0.0
        self.current_steering = 0.0
        self.current_throttle = 0.0

    def start_mission(self, mission_name: str):
        self.state = mission_name
        self.mission_start_time = self.get_clock().now()
        self.mission_phase = 0
        self.mission_phase_start = 0.0
        self.get_logger().info(f"Starting mission: {mission_name}")

    def start_clear_room(self):
        self.start_mission("CLEAR_ROOM")

    def start_clear_left_corner(self):
        self.start_mission("CLEAR_LEFT_CORNER")

    def start_clear_right_corner(self):
        self.start_mission("CLEAR_RIGHT_CORNER")

    def phase_time(self, now_sec: float) -> float:
        return now_sec - self.mission_phase_start

    def next_phase(self, time_in_mission_sec: float):
        self.mission_phase += 1
        self.mission_phase_start = time_in_mission_sec

    def finish_mission(self):
        self.get_logger().info(f"Mission {self.state} complete → IDLE")
        self.go_idle()

    # ----------------- Mission scripts (no lidar, timed) -----------------

    def clear_room_step(self, dt: float) -> Twist:
        """
        CLEAR_ROOM script (ignoring lidar):
          Phase 0: drive forward into room
          Phase 1: spin in place
          Phase 2: reverse back out
        """
        twist = Twist()
        phase_t = self.phase_time(dt)

        if self.mission_phase == 0:
            # Drive forward into room
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            # e.g. x seconds forward
            if phase_t > 2.5:
                self.get_logger().info("CLEAR_ROOM: reached Phase 1 (spin)")
                self.next_phase(dt)

        elif self.mission_phase == 1:
            # Spin in place
            twist.linear.x = 0.2
            twist.angular.z = self.turn_speed
            # e.g. spin ~xs
            if phase_t > 8.8:
                self.get_logger().info("CLEAR_ROOM: reached Phase 2 (reverse out)")
                self.next_phase(dt)

        elif self.mission_phase == 2:
            # Reverse back
            twist.linear.x = self.reverse_speed
            twist.angular.z = 0.0
            # same distance back: 1.5s
            if phase_t > 2.5:
                self.finish_mission()

        return twist

    def clear_left_corner_step(self, dt: float) -> Twist:
        """
        CLEAR_LEFT_CORNER script (ignoring lidar):
          Phase 0: approach corner (forward)
          Phase 1: turn left while moving (round the corner)
          Phase 2: straight a bit down new hallway
          Phase 3: reverse straight back
          Phase 4: reverse while turning right (undo corner)
          Phase 5: reverse straight to original spot
        """
        twist = Twist()
        phase_t = self.phase_time(dt)

        if self.mission_phase == 0:
            # Approach corner
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            if phase_t > 1.0:
                self.get_logger().info("CLEAR_LEFT_CORNER: Phase 1 (turn left around corner)")
                self.next_phase(dt)

        elif self.mission_phase == 1:
            # Turn left around corner (moving arc)
            twist.linear.x = self.forward_speed
            twist.angular.z = self.turn_speed
            if phase_t > 2.0:
                self.get_logger().info("CLEAR_LEFT_CORNER: Phase 2 (down new hallway)")
                self.next_phase(dt)

        elif self.mission_phase == 2:
            # Straight a bit in new hallway
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            if phase_t > 1.0:
                self.get_logger().info("CLEAR_LEFT_CORNER: Phase 3 (reverse straight)")
                self.next_phase(dt)

        elif self.mission_phase == 3:
            # Reverse straight back
            twist.linear.x = self.reverse_speed
            twist.angular.z = 0.0
            if phase_t > 1.0:
                self.get_logger().info("CLEAR_LEFT_CORNER: Phase 4 (reverse turning right)")
                self.next_phase(dt)

        elif self.mission_phase == 4:
            # Reverse arc to undo corner (turn right while reversing)
            twist.linear.x = self.reverse_speed
            twist.angular.z = self.turn_speed
            if phase_t > 2.0:
                self.get_logger().info("CLEAR_LEFT_CORNER: Phase 5 (reverse to original)")
                self.next_phase(dt)

        elif self.mission_phase == 5:
            # Final straight reverse to original line
            twist.linear.x = self.reverse_speed
            twist.angular.z = 0.0
            if phase_t > 1.0:
                self.finish_mission()

        return twist

    def clear_right_corner_step(self, dt: float) -> Twist:
        """
        CLEAR_RIGHT_CORNER script (mirror of left):
          Phase 0: approach corner
          Phase 1: turn right while moving
          Phase 2: straight in new hallway
          Phase 3: reverse straight back
          Phase 4: reverse while turning left
          Phase 5: reverse straight to original
        """
        twist = Twist()
        phase_t = self.phase_time(dt)

        if self.mission_phase == 0:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            if phase_t > 1.0:
                self.get_logger().info("CLEAR_RIGHT_CORNER: Phase 1 (turn right around corner)")
                self.next_phase(dt)

        elif self.mission_phase == 1:
            twist.linear.x = self.forward_speed
            twist.angular.z = -self.turn_speed  # right turn
            if phase_t > 2.0:
                self.get_logger().info("CLEAR_RIGHT_CORNER: Phase 2 (down new hallway)")
                self.next_phase(dt)

        elif self.mission_phase == 2:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
            if phase_t > 1.0:
                self.get_logger().info("CLEAR_RIGHT_CORNER: Phase 3 (reverse straight)")
                self.next_phase(dt)

        elif self.mission_phase == 3:
            twist.linear.x = self.reverse_speed
            twist.angular.z = 0.0
            if phase_t > 1.0:
                self.get_logger().info("CLEAR_RIGHT_CORNER: Phase 4 (reverse turning left)")
                self.next_phase(dt)

        elif self.mission_phase == 4:
            # Reverse arc to undo corner (turn left while reversing)
            twist.linear.x = self.reverse_speed
            twist.angular.z = -self.turn_speed
            if phase_t > 2.0:
                self.get_logger().info("CLEAR_RIGHT_CORNER: Phase 5 (reverse to original)")
                self.next_phase(dt)

        elif self.mission_phase == 5:
            twist.linear.x = self.reverse_speed
            twist.angular.z = 0.0
            if phase_t > 1.0:
                self.finish_mission()

        return twist

    # ----------------- Main control loop -----------------

    def timer_callback(self):
        twist = Twist()

        # If we're IDLE: don't move
        if self.state == "IDLE":
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        elif self.state == "FOLLOW_HAND":
            # Use hand-following steering/throttle
            twist.linear.x = self.current_throttle * self.max_linear_speed
            twist.angular.z = self.current_steering * self.max_angular_speed

        else:
            # Mission states: CLEAR_ROOM / CLEAR_LEFT_CORNER / CLEAR_RIGHT_CORNER
            if self.mission_start_time is None:
                self.mission_start_time = self.get_clock().now()
                self.mission_phase = 0
                self.mission_phase_start = 0.0

            now = self.get_clock().now()
            dt = (now - self.mission_start_time).nanoseconds * 1e-9

            if self.state == "CLEAR_ROOM":
                twist = self.clear_room_step(dt)
            elif self.state == "CLEAR_LEFT_CORNER":
                twist = self.clear_left_corner_step(dt)
            elif self.state == "CLEAR_RIGHT_CORNER":
                twist = self.clear_right_corner_step(dt)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GestureCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down GestureCmdNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
