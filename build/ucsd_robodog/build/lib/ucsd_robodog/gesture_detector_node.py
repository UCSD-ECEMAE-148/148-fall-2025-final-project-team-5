#!/usr/bin/env python3

import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Finger landmark indices (from your code)
wrist = 0
thumb_tip = 4
thumb_ref = 3
index_tip = 8
index_ref = 6
middle_tip = 12
middle_ref = 10
ring_tip = 16
ring_ref = 14
pinky_tip = 20
pinky_ref = 18


def get_finger_states(hand_landmarks):
    finger_states = {}

    # Thumb
    def is_up(tip, ref):
        return hand_landmarks.landmark[tip].y > hand_landmarks.landmark[ref].y

    finger_states["index"] = is_up(index_tip, index_ref)
    finger_states["middle"] = is_up(middle_tip, middle_ref)
    finger_states["ring"] = is_up(ring_tip, ring_ref)
    finger_states["pinky"] = is_up(pinky_tip, pinky_ref)

    # thumb done by x (your original logic)
    finger_states["thumb"] = (
        hand_landmarks.landmark[thumb_tip].x < hand_landmarks.landmark[thumb_ref].x
    )
    return finger_states


def classify_gesture(hand_landmarks):
    finger_states = get_finger_states(hand_landmarks)

    if all(finger_states[finger] for finger in finger_states):
        return "OPEN PALM"
    if not any(finger_states[finger] for finger in finger_states):
        return "FIST"
    if (finger_states["index"] and not finger_states["middle"] and
        not finger_states["ring"] and not finger_states["pinky"]):
        return "ONE"
    if (finger_states["index"] and finger_states["middle"] and
        not finger_states["ring"] and not finger_states["pinky"]):
        return "TWO"
    if (finger_states["thumb"] and finger_states["pinky"] and
        not finger_states["middle"] and not finger_states["ring"]):
        return "SHAKA"

    return "UNKNOWN"


def get_hand_center(hand_landmarks):
    x_coords = [landmark.x for landmark in hand_landmarks.landmark]
    center_x = sum(x_coords) / len(x_coords)
    return center_x


def follow_control(hand_landmarks):
    # Directly from your code
    xgain = 1.0
    zgain = 1.0
    max_tilt = 0.2   # adjust based on testing
    zero_tilt = 0.11  # neutral tilt value
    deadzonez = 0.05  # to avoid jitter
    deadzonex = 0.09  # to avoid jitter

    cx = get_hand_center(hand_landmarks)
    # print("Hand center x:", cx)
    x_control = (cx - 0.5) * 2  # Normalize to [-1, 1]
    if abs(x_control) < deadzonex:
        x_control = 0.0

    z_tilt_raw = hand_landmarks.landmark[wrist].z - hand_landmarks.landmark[middle_tip].z
    # print("Z tilt raw:", z_tilt_raw)
    z_tilt = z_tilt_raw - zero_tilt
    # print("Z tilt adjusted:", z_tilt)
    if abs(z_tilt) < deadzonez:
        z_tilt = 0.0

    tilt_clamped = max(-max_tilt, min(max_tilt, z_tilt))
    z_control = -tilt_clamped / max_tilt  # Normalize to [-1, 1]
    # print("Z control:", z_control)

    steering = x_control * xgain
    throttle = z_control * zgain

    return steering, throttle


class GestureDetectorNode(Node):
    def __init__(self):
        super().__init__("gesture_detector_node")

        # Publishers
        self.gesture_pub = self.create_publisher(String, "gesture", 10)
        self.steering_pub = self.create_publisher(Float32, "steering", 10)
        self.throttle_pub = self.create_publisher(Float32, "throttle", 10)

        # Camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera 0")
        else:
            self.get_logger().info("Camera 0 opened")

        # MediaPipe Hands
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,        # only one hand
            model_complexity=0,     # FAST model
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        # Timer ~30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info("GestureDetectorNode initialized")

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame")
            return

        # Convert BGR (OpenCV) → RGB (MediaPipe)
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run MediaPipe Hands
        results = self.hands.process(image_rgb)

        gesture_str = "NO_HAND"
        steering = 0.0
        throttle = 0.0

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw landmarks
                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS
                )

                # Classify gesture (your logic)
                gesture_str = classify_gesture(hand_landmarks)

                if gesture_str == "OPEN PALM":
                    # Use your follow_control for steering/throttle
                    steering, throttle = follow_control(hand_landmarks)

                    cv2.putText(frame, f"Gesture: {gesture_str}", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    cv2.putText(frame, f"Steering: {steering:.2f}", (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    cv2.putText(frame, f"Throttle: {throttle:.2f}", (10, 75),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    cv2.putText(frame, gesture_str,
                                (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                1, (0, 255, 0), 2)

                # Just use the first hand
                break
        else:
            cv2.putText(frame, "No Hand",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 255), 2)

        # Publish ROS messages
        gesture_msg = String()
        gesture_msg.data = gesture_str
        self.gesture_pub.publish(gesture_msg)

        steering_msg = Float32()
        steering_msg.data = float(steering)
        self.steering_pub.publish(steering_msg)

        throttle_msg = Float32()
        throttle_msg.data = float(throttle)
        self.throttle_pub.publish(throttle_msg)

        # Show debug window (can turn off on headless Pi)
        cv2.imshow("RoboDog Gestures", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            self.get_logger().info("ESC pressed, shutting down")
            rclpy.shutdown()

    def destroy_node(self):
        self.get_logger().info("Cleaning up GestureDetectorNode")
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
