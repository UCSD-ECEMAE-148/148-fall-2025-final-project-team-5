#!/usr/bin/env python3
import time

import cv2
from roboflowoak import RoboflowOak

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool


class GunDetectionOakNode(Node):
    def __init__(self):
        super().__init__("gun_detection_node")

        # --- Publishers for other nodes ---
        self.people_pub = self.create_publisher(Int32, "people_count", 10)
        self.armed_pub = self.create_publisher(Int32, "armed_count", 10)
        self.sweep_done_pub = self.create_publisher(Bool, "sweep_done", 10)

        # --- State for sweep tracking ---
        self.first_armed_seen = False
        self.first_armed_time = None
        self.sweep_done_published = False

        # Saved totals during the 10-second window
        self.saved_max_people = 0
        self.saved_max_armed = 0

        # --- Roboflow + OAK-D setup ---
        self.get_logger().info("Initializing RoboflowOak...")
        self.rf = RoboflowOak(
            model="gun-detection-7eyjy",
            version="3",
            api_key="QxVNJFo7CSLpCNeYAvA1",
            confidence=0.67,   # adjust as needed
            overlap=0.5,
            rgb=True,
            depth=False,
            device=None,
            blocking=True,     # we drive the loop directly
        )
        self.get_logger().info("RoboflowOak initialized ✅")

        # Whether to show the OAK-D preview window
        self.show_preview = True

    def run(self):
        while rclpy.ok():
            t0 = time.time()

            # Run Roboflow + OAK-D inference
            try:
                result, frame, raw_frame, depth = self.rf.detect()
            except Exception as e:
                self.get_logger().error(f"RoboflowOak detect() failed: {e}")
                break

            predictions = result["predictions"]

            # --- Count classes per frame ---
            people_count = 0
            armed_count = 0

            for p in predictions:
                pj = p.json()
                cls = pj.get("class", "")

                if cls == "Person":
                    people_count += 1
                elif cls == "Person-Holding-Gun":
                    armed_count += 1

            # --- FPS / logging ---
            dt = time.time() - t0
            fps = 1.0 / dt if dt > 0 else 0.0

            self.get_logger().info(
                f"FPS: {fps:.1f} | People: {people_count} | Armed: {armed_count}"
            )

            # --- Publish counts every frame ---
            self.publish_counts(people_count, armed_count)

            # --- Sweep logic ---
            self.update_sweep_state(people_count, armed_count)

            # --- Optional preview window ---
            if self.show_preview and frame is not None:
                cv2.imshow("OAK-D Gun Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    self.get_logger().info("Q pressed, exiting loop")
                    break

        self.cleanup()

    def publish_counts(self, people_count: int, armed_count: int):
        """Publish current frame counts on people_count and armed_count topics."""
        people_msg = Int32()
        people_msg.data = people_count
        self.people_pub.publish(people_msg)

        armed_msg = Int32()
        armed_msg.data = armed_count
        self.armed_pub.publish(armed_msg)

    def update_sweep_state(self, people_count: int, armed_count: int):
        """
        Sweep behavior:
        - When the first armed person is seen (armed_count > 0 and never seen before),
          start a 10-second window and initialize saved totals.
        - During those 10 seconds, track the max people_count and armed_count observed.
        - After 10 seconds from first armed detection, publish sweep_done=True once,
          and log the saved totals.
        """

        now = time.time()

        # First armed person ever detected: start the sweep window
        if armed_count > 0 and not self.first_armed_seen:
            self.first_armed_seen = True
            self.first_armed_time = now
            self.sweep_done_published = False

            # Initialize saved maxima with current counts
            self.saved_max_people = people_count
            self.saved_max_armed = armed_count

            self.get_logger().info(
                "First armed person detected. "
                "Starting 10-second sweep window to track people/armed counts."
            )

        # If we're in the sweep window and have not yet published sweep_done
        if self.first_armed_seen and not self.sweep_done_published:
            # Update maxima during the 10-second window
            if people_count > self.saved_max_people:
                self.saved_max_people = people_count
            # Only update armed max if there is at least one armed detection
            if armed_count > self.saved_max_armed:
                self.saved_max_armed = armed_count

            # Check if 10 seconds have elapsed since the first armed detection
            if now - self.first_armed_time >= 10.0:
                sweep_msg = Bool()
                sweep_msg.data = True
                self.sweep_done_pub.publish(sweep_msg)
                self.sweep_done_published = True

                self.get_logger().info(
                    f"sweep_done: True (published after 10 seconds from first gun). "
                    f"Max people seen: {self.saved_max_people}, "
                    f"Max armed people seen: {self.saved_max_armed}"
                )

                # If you ever want to allow a new sweep later, you could reset:
                # self.first_armed_seen = False

    def cleanup(self):
        self.get_logger().info("Cleaning up GunDetectionOakNode")
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = GunDetectionOakNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
