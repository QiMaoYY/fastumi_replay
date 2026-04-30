#!/usr/bin/env python3

import math
import sys
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


HANDS = ("left", "right")


def get_config_param(private_name, config_path, default):
    package_value = rospy.get_param(f"/fastumi_replay/{config_path}", default)
    return rospy.get_param(f"~{private_name}", package_value)


def get_float_list_param(private_name, config_path, default):
    value = get_config_param(private_name, config_path, default)
    return [float(item) for item in value]


class ClampState:
    def __init__(self, hand, thickness):
        self.hand = hand
        self.thickness = thickness
        self.position = (0.0, 0.0, 0.0)
        self.rpy = (0.0, 0.0, 0.0)
        self.distance_m = 0.0
        self.trail = deque()

    def update_pose(self, msg):
        self.position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        quat = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        self.rpy = euler_from_quaternion(quat)
        self.trail.append((msg.header.stamp, self.position))

    def update_distance(self, msg, unit_scale):
        distance_m = msg.data * unit_scale
        self.distance_m = max(0.0, distance_m)

    def joint_names(self):
        prefix = f"{self.hand}_clamp"
        return [
            f"{prefix}_pose_x",
            f"{prefix}_pose_y",
            f"{prefix}_pose_z",
            f"{prefix}_pose_yaw",
            f"{prefix}_pose_pitch",
            f"{prefix}_pose_roll",
            f"{self.hand}_left_finger_slide",
            f"{self.hand}_right_finger_slide",
        ]

    def joint_positions(self):
        roll, pitch, yaw = self.rpy
        x, y, z = self.position
        finger_offset = self.distance_m / 2.0 + self.thickness / 2.0
        return [
            x,
            y,
            z,
            normalize_angle(yaw),
            normalize_angle(pitch),
            normalize_angle(roll),
            finger_offset,
            finger_offset,
        ]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def unit_scale_from_param(value):
    value = str(value).strip().lower()
    if value in ("mm", "millimeter", "millimeters"):
        return 0.001
    if value in ("m", "meter", "meters"):
        return 1.0
    raise ValueError("~distance_unit 仅支持 'mm' 或 'm'")


class ClampVisualizer:
    def __init__(self):
        self.thickness = float(
            get_config_param("thickness", "visualizer/thickness", 0.005)
        )
        self.rate_hz = float(
            get_config_param("rate", "visualizer/publish_rate", 60.0)
        )
        self.unit_scale = unit_scale_from_param(
            get_config_param("distance_unit", "visualizer/distance_unit", "mm")
        )
        joint_states_topic = get_config_param(
            "joint_states_topic",
            "topics/joint_states",
            "/joint_states",
        )
        self.trail_enabled = bool(
            get_config_param("trail_enabled", "trail/enabled", True)
        )
        self.trail_duration = float(
            get_config_param("trail_duration", "trail/duration", 2.0)
        )
        self.trail_width = float(
            get_config_param("trail_width", "trail/width", 0.01)
        )
        self.trail_color = get_float_list_param(
            "trail_color",
            "trail/color",
            [1.0, 0.0, 0.0, 1.0],
        )
        self.trail_topic = get_config_param(
            "trail_topic",
            "trail/topic",
            "/clamp_tip_trails",
        )
        self.frame_ids = {
            "left": get_config_param("left_base_frame", "frames/left_base", "left_base"),
            "right": get_config_param("right_base_frame", "frames/right_base", "right_base"),
        }

        self.states = {
            hand: ClampState(hand, self.thickness)
            for hand in HANDS
        }
        self.joint_pub = rospy.Publisher(
            joint_states_topic,
            JointState,
            queue_size=10,
        )
        self.marker_pub = rospy.Publisher(
            self.trail_topic,
            MarkerArray,
            queue_size=10,
        )

        self.subscribers = []
        for hand in HANDS:
            pose_topic = get_config_param(
                f"{hand}_pose_topic",
                f"topics/{hand}_pose",
                f"/{hand}_clamp/pose",
            )
            distance_topic = get_config_param(
                f"{hand}_distance_topic",
                f"topics/{hand}_distance",
                f"/{hand}_clamp/distance",
            )
            self.subscribers.append(
                rospy.Subscriber(
                    pose_topic,
                    PoseStamped,
                    self._pose_callback,
                    callback_args=hand,
                    queue_size=10,
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    distance_topic,
                    Float32,
                    self._distance_callback,
                    callback_args=hand,
                    queue_size=10,
                )
            )

    def _pose_callback(self, msg, hand):
        self.states[hand].update_pose(msg)

    def _distance_callback(self, msg, hand):
        self.states[hand].update_distance(msg, self.unit_scale)

    def _prune_trail(self, state, now):
        cutoff = now - rospy.Duration.from_sec(self.trail_duration)
        while state.trail and state.trail[0][0] < cutoff:
            state.trail.popleft()

    def _build_trail_marker(self, hand, state, now, marker_id):
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.frame_ids[hand]
        marker.ns = "clamp_tip_trail"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.trail_width
        marker.color.r = self.trail_color[0]
        marker.color.g = self.trail_color[1]
        marker.color.b = self.trail_color[2]
        marker.color.a = self.trail_color[3]
        marker.lifetime = rospy.Duration.from_sec(0.25)
        marker.points = [
            Point(x=position[0], y=position[1], z=position[2])
            for _, position in state.trail
        ]
        return marker

    def publish_trails(self, now):
        if not self.trail_enabled:
            return

        markers = MarkerArray()
        for marker_id, hand in enumerate(HANDS):
            state = self.states[hand]
            self._prune_trail(state, now)
            markers.markers.append(
                self._build_trail_marker(hand, state, now, marker_id)
            )
        self.marker_pub.publish(markers)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        for hand in HANDS:
            state = self.states[hand]
            msg.name.extend(state.joint_names())
            msg.position.extend(state.joint_positions())
            msg.velocity.extend([0.0] * len(state.joint_names()))
            msg.effort.extend([0.0] * len(state.joint_names()))
        self.joint_pub.publish(msg)
        self.publish_trails(msg.header.stamp)

    def spin(self):
        rospy.loginfo(
            "Publishing clamp visualization joint states at %.1f Hz "
            "(distance unit scale=%.4f)",
            self.rate_hz,
            self.unit_scale,
        )
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish_joint_state()
            rate.sleep()


def main():
    rospy.init_node("fastumi_clamp_visualizer")
    try:
        visualizer = ClampVisualizer()
    except Exception as exc:
        rospy.logfatal("Failed to initialize clamp visualizer: %s", exc)
        return 1

    visualizer.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
