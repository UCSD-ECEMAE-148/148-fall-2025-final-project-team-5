#!/usr/bin/env python3

import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool


class SweepAudioAnnouncer(Node):
    def __init__(self):
        super().__init__("sweep_audio_announcer")

        # Latest known sweep results
        self.people_count = 0
        self.armed_count = 0

        # Subscribers
        self.people_sub = self.create_subscription(
            Int32, "people_count", self.people_cb, 10
        )
        self.armed_sub = self.create_subscription(
            Int32, "armed_count", self.armed_cb, 10
        )
        self.done_sub = self.create_subscription(
            Bool, "sweep_done", self.sweep_done_cb, 10
        )

        # Timer: say what we see every 10 seconds
        self.timer_period = 10.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        self.get_logger().info(
            "SweepAudioAnnouncer ready "
            "(topics: people_count, armed_count, sweep_done), "
            f"periodic announcements every {self.timer_period} seconds."
        )

    # ----- Callbacks -----

    def people_cb(self, msg: Int32):
        self.people_count = int(msg.data)

    def armed_cb(self, msg: Int32):
        self.armed_count = int(msg.data)

    def sweep_done_cb(self, msg: Bool):
        # Optional: still speak immediately when sweep is reported done
        if msg.data:
            phrase = self.build_report_phrase()
            self.get_logger().info(f"[sweep_done] Speaking report: {phrase}")
            self.speak_async(phrase)

    def timer_cb(self):
        """
        Called every self.timer_period seconds.
        Announces what the node currently 'sees' based on the latest counts.
        """
        phrase = self.build_report_phrase()
        self.get_logger().info(f"[timer] Speaking periodic report: {phrase}")
        self.speak_async(phrase)

    # ----- Logic for building the spoken message -----

    def build_report_phrase(self) -> str:
        p = self.people_count
        a = self.armed_count

        if p == 0:
            return "Room clear. No people detected."
        elif p == 1 and a == 0:
            return "One person detected. No visible weapons."
        elif p > 0 and a == 0:
            return f"{p} people detected. No visible weapons."
        elif p == 1 and a == 1:
            return "One person detected. One visible weapon."
        elif p == 1 and a > 1:
            return f"One person detected. {a} visible weapons."
        elif p > 1 and a == 1:
            return f"{p} people detected. One visible weapon."
        elif p > 1 and a > 1:
            return f"{p} people detected. {a} visible weapons."
        else:
            return "Error in sweep report."

    # ----- Audio output -----

    def speak_async(self, text: str):
        """
        Run TTS in a background thread so we don't block callbacks.
        Assumes 'espeak' is installed on the system.
        """
        thread = threading.Thread(target=self._speak_blocking, args=(text,), daemon=True)
        thread.start()

    def _speak_blocking(self, text: str):
        try:
            subprocess.run(
                ["espeak", "-s", "150","-a","200", text],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.get_logger().error(f"Failed to speak text: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SweepAudioAnnouncer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down SweepAudioAnnouncer")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
