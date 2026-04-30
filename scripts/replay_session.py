#!/usr/bin/env python3

import argparse
import csv
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path


DEFAULT_DATA_ROOT = Path("/home/qimao/fastumi/DATA")
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "replay.yaml"

DATE_DIR_RE = re.compile(r"^multi_session_\d{8}$")
SESSION_DIR_RE = re.compile(r"^session_\d{6}$")
EXIT_REQUESTED = object()


@dataclass(frozen=True)
class ReplayEvent:
    timestamp: float
    priority: int
    hand: str
    kind: str
    payload: tuple


@dataclass(frozen=True)
class HandPaths:
    hand: str
    root: Path
    video: Path
    image_timestamps: Path
    trajectory: Path
    clamp: Path
    frame_id: str


@dataclass(frozen=True)
class SessionData:
    session_dir: Path
    events: list
    hand_paths: dict
    start_time: float
    end_time: float


class SessionValidationError(Exception):
    pass


def load_yaml_config(config_path):
    if config_path is None:
        return {}
    config_path = Path(config_path).expanduser()
    if not config_path.exists():
        return {}
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("读取 --config 需要 PyYAML。") from exc
    with config_path.open("r") as file_obj:
        return yaml.safe_load(file_obj) or {}


def config_get(config, keys, default):
    current = config
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def parse_args():
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    pre_args, _ = pre_parser.parse_known_args()
    config = load_yaml_config(pre_args.config)

    parser = argparse.ArgumentParser(
        description="Replay FastUMI dual-hand data as ROS1 topics."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=pre_args.config,
        help="YAML config file used for default replay parameters.",
    )
    parser.add_argument(
        "--data-root",
        type=Path,
        default=Path(config_get(config, ("data_root",), DEFAULT_DATA_ROOT)),
        help="Root directory containing multi_session_YYYYMMDD folders.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=float(config_get(config, ("playback_rate",), 1.0)),
        help="Replay speed multiplier. Use 2.0 for 2x speed.",
    )
    parser.add_argument(
        "--queue-size",
        type=int,
        default=int(config_get(config, ("queue_size",), 10)),
        help="ROS publisher queue size.",
    )
    parser.add_argument(
        "--left-image-topic",
        default=config_get(config, ("topics", "left_image"), "/left_rgb/image_raw"),
        help="ROS topic for left RGB images.",
    )
    parser.add_argument(
        "--right-image-topic",
        default=config_get(config, ("topics", "right_image"), "/right_rgb/image_raw"),
        help="ROS topic for right RGB images.",
    )
    parser.add_argument(
        "--left-pose-topic",
        default=config_get(config, ("topics", "left_pose"), "/left_clamp/pose"),
        help="ROS topic for left PoseStamped messages.",
    )
    parser.add_argument(
        "--right-pose-topic",
        default=config_get(config, ("topics", "right_pose"), "/right_clamp/pose"),
        help="ROS topic for right PoseStamped messages.",
    )
    parser.add_argument(
        "--left-clamp-topic",
        default=config_get(
            config,
            ("topics", "left_distance"),
            "/left_clamp/distance",
        ),
        help="ROS topic for left clamp distance Float32 messages.",
    )
    parser.add_argument(
        "--right-clamp-topic",
        default=config_get(
            config,
            ("topics", "right_distance"),
            "/right_clamp/distance",
        ),
        help="ROS topic for right clamp distance Float32 messages.",
    )
    parser.add_argument(
        "--left-base-frame",
        default=config_get(config, ("frames", "left_base"), "left_base"),
        help="Frame id used by left PoseStamped/Image messages.",
    )
    parser.add_argument(
        "--right-base-frame",
        default=config_get(config, ("frames", "right_base"), "right_base"),
        help="Frame id used by right PoseStamped/Image messages.",
    )
    parser.add_argument(
        "--check-session",
        type=Path,
        help="Validate and summarize one session directory without starting ROS.",
    )
    args, unknown = parser.parse_known_args()
    if unknown:
        print(f"忽略 ROS/launch 追加参数: {' '.join(unknown)}")
    return args


def list_matching_dirs(parent, pattern):
    if not parent.exists():
        return []
    return sorted(
        path
        for path in parent.iterdir()
        if path.is_dir() and pattern.match(path.name)
    )


def prompt_choice(title, options, allow_back=False, allow_exit=True):
    while True:
        print("")
        print(title)
        if allow_back:
            print("  0) 返回上一级")
        elif allow_exit:
            print("  0) 退出")
        for index, path in enumerate(options, start=1):
            print(f"  {index}) {path.name}")
        if allow_back and allow_exit:
            print(f"  {len(options) + 1}) 退出")

        try:
            choice = input("请输入编号: ").strip()
        except EOFError:
            print("")
            return EXIT_REQUESTED

        if allow_back and choice == "0":
            return None
        if allow_exit:
            exit_choices = {"q", "quit", "exit"}
            if allow_back:
                exit_choices.add(str(len(options) + 1))
            else:
                exit_choices.add("0")
            if choice.lower() in exit_choices:
                return EXIT_REQUESTED
        if not choice.isdigit():
            print("请输入数字编号。")
            continue

        index = int(choice)
        if 1 <= index <= len(options):
            return options[index - 1]
        print("编号超出范围，请重新输入。")


def find_hand_dir(session_dir, hand):
    matches = sorted(
        path
        for path in session_dir.iterdir()
        if path.is_dir() and path.name.startswith(f"{hand}_")
    )
    if len(matches) != 1:
        raise SessionValidationError(
            f"{session_dir} 中期望找到 1 个 {hand}_* 目录，实际找到 {len(matches)} 个。"
        )
    return matches[0]


def build_hand_paths(session_dir, hand, frame_ids=None):
    hand_root = find_hand_dir(session_dir, hand)
    frame_ids = frame_ids or {}
    default_frame = "left_base" if hand == "left" else "right_base"
    frame_id = frame_ids.get(hand, default_frame)
    paths = HandPaths(
        hand=hand,
        root=hand_root,
        video=hand_root / "RGB_Images" / "video.mp4",
        image_timestamps=hand_root / "RGB_Images" / "timestamps.csv",
        trajectory=hand_root / "Merged_Trajectory" / "merged_trajectory.txt",
        clamp=hand_root / "Clamp_Data" / "clamp_data_tum.txt",
        frame_id=frame_id,
    )

    missing = [
        path
        for path in (
            paths.video,
            paths.image_timestamps,
            paths.trajectory,
            paths.clamp,
        )
        if not path.exists()
    ]
    if missing:
        missing_text = "\n".join(f"  - {path}" for path in missing)
        raise SessionValidationError(f"{hand} 手缺少必要文件:\n{missing_text}")
    return paths


def read_image_events(hand_paths):
    events = []
    with hand_paths.image_timestamps.open("r", newline="") as file_obj:
        reader = csv.DictReader(file_obj)
        required = {"frame_index", "header_stamp"}
        if not required.issubset(reader.fieldnames or []):
            raise SessionValidationError(
                f"{hand_paths.image_timestamps} 缺少列: frame_index/header_stamp"
            )

        for row_number, row in enumerate(reader, start=2):
            try:
                timestamp = float(row["header_stamp"])
                frame_index = int(row["frame_index"])
            except (TypeError, ValueError) as exc:
                raise SessionValidationError(
                    f"{hand_paths.image_timestamps}:{row_number} 图像时间戳格式错误。"
                ) from exc

            events.append(
                ReplayEvent(
                    timestamp=timestamp,
                    priority=1,
                    hand=hand_paths.hand,
                    kind="image",
                    payload=(frame_index,),
                )
            )
    return events


def read_pose_events(hand_paths):
    events = []
    with hand_paths.trajectory.open("r") as file_obj:
        for row_number, line in enumerate(file_obj, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            parts = stripped.split()
            if len(parts) != 8:
                raise SessionValidationError(
                    f"{hand_paths.trajectory}:{row_number} 期望 8 列，实际 {len(parts)} 列。"
                )
            try:
                values = tuple(float(part) for part in parts)
            except ValueError as exc:
                raise SessionValidationError(
                    f"{hand_paths.trajectory}:{row_number} 位姿数据格式错误。"
                ) from exc

            events.append(
                ReplayEvent(
                    timestamp=values[0],
                    priority=0,
                    hand=hand_paths.hand,
                    kind="pose",
                    payload=values[1:],
                )
            )
    return events


def read_clamp_events(hand_paths):
    events = []
    with hand_paths.clamp.open("r") as file_obj:
        for row_number, line in enumerate(file_obj, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            parts = stripped.split()
            if len(parts) != 2:
                raise SessionValidationError(
                    f"{hand_paths.clamp}:{row_number} 期望 2 列，实际 {len(parts)} 列。"
                )
            try:
                timestamp = float(parts[0])
                distance = float(parts[1])
            except ValueError as exc:
                raise SessionValidationError(
                    f"{hand_paths.clamp}:{row_number} 夹爪数据格式错误。"
                ) from exc

            events.append(
                ReplayEvent(
                    timestamp=timestamp,
                    priority=2,
                    hand=hand_paths.hand,
                    kind="clamp",
                    payload=(distance,),
                )
            )
    return events


def load_session(session_dir, frame_ids=None):
    if not session_dir.is_dir() or not SESSION_DIR_RE.match(session_dir.name):
        raise SessionValidationError(f"{session_dir} 不是有效的 session_HHMMSS 目录。")

    hand_paths = {
        "left": build_hand_paths(session_dir, "left", frame_ids),
        "right": build_hand_paths(session_dir, "right", frame_ids),
    }
    events = []
    for paths in hand_paths.values():
        events.extend(read_pose_events(paths))
        events.extend(read_image_events(paths))
        events.extend(read_clamp_events(paths))

    if not events:
        raise SessionValidationError(f"{session_dir} 没有可回放的数据。")

    events.sort(key=lambda event: (event.timestamp, event.priority, event.hand))
    return SessionData(
        session_dir=session_dir,
        events=events,
        hand_paths=hand_paths,
        start_time=events[0].timestamp,
        end_time=events[-1].timestamp,
    )


def summarize_session(session_data):
    counts = {}
    for event in session_data.events:
        key = (event.hand, event.kind)
        counts[key] = counts.get(key, 0) + 1

    duration = session_data.end_time - session_data.start_time
    print(f"Session: {session_data.session_dir}")
    print(f"时间范围: {session_data.start_time:.6f} -> {session_data.end_time:.6f}")
    print(f"时长: {duration:.3f}s")
    for hand in ("left", "right"):
        print(
            f"{hand}: "
            f"pose={counts.get((hand, 'pose'), 0)}, "
            f"image={counts.get((hand, 'image'), 0)}, "
            f"clamp={counts.get((hand, 'clamp'), 0)}"
        )


class VideoReader:
    def __init__(self, video_path):
        import cv2

        self.video_path = video_path
        self.capture = cv2.VideoCapture(str(video_path))
        if not self.capture.isOpened():
            raise RuntimeError(f"无法打开视频文件: {video_path}")
        self.next_index = 0
        self.exhausted = False
        self.warned_exhausted = False

    def read_frame(self, frame_index):
        if self.exhausted:
            return None
        if frame_index < self.next_index:
            raise RuntimeError(
                f"{self.video_path} 请求倒退读取帧 {frame_index}，当前已到 {self.next_index}。"
            )

        frame = None
        while self.next_index <= frame_index:
            ok, frame = self.capture.read()
            if not ok:
                self.exhausted = True
                if not self.warned_exhausted:
                    print(
                        f"警告: {self.video_path} 在帧 {self.next_index} 后结束，"
                        "后续图像时间戳将跳过。"
                    )
                    self.warned_exhausted = True
                return None
            self.next_index += 1
        return frame

    def close(self):
        self.capture.release()


class RosReplayPublisher:
    def __init__(self, args, session_data):
        import rospy
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import Image
        from std_msgs.msg import Float32

        self.rospy = rospy
        self.PoseStamped = PoseStamped
        self.Image = Image
        self.Float32 = Float32
        self.args = args
        self.session_data = session_data

        self.image_publishers = {
            "left": rospy.Publisher(args.left_image_topic, Image, queue_size=args.queue_size),
            "right": rospy.Publisher(args.right_image_topic, Image, queue_size=args.queue_size),
        }
        self.pose_publishers = {
            "left": rospy.Publisher(args.left_pose_topic, PoseStamped, queue_size=args.queue_size),
            "right": rospy.Publisher(args.right_pose_topic, PoseStamped, queue_size=args.queue_size),
        }
        self.clamp_publishers = {
            "left": rospy.Publisher(args.left_clamp_topic, Float32, queue_size=args.queue_size),
            "right": rospy.Publisher(args.right_clamp_topic, Float32, queue_size=args.queue_size),
        }
        self.video_readers = {}

    def _stamp_for(self, replay_start_stamp, relative_time):
        return replay_start_stamp + self.rospy.Duration.from_sec(relative_time)

    def _build_image_msg(self, frame, stamp, frame_id):
        msg = self.Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = frame.tobytes()
        return msg

    def _build_pose_msg(self, payload, stamp, frame_id):
        x, y, z, qx, qy, qz, qw = payload
        msg = self.PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _publish_event(self, event, stamp):
        hand_paths = self.session_data.hand_paths[event.hand]
        if event.kind == "image":
            reader = self.video_readers[event.hand]
            frame = reader.read_frame(event.payload[0])
            if frame is None:
                return
            msg = self._build_image_msg(frame, stamp, hand_paths.frame_id)
            self.image_publishers[event.hand].publish(msg)
        elif event.kind == "pose":
            msg = self._build_pose_msg(event.payload, stamp, hand_paths.frame_id)
            self.pose_publishers[event.hand].publish(msg)
        elif event.kind == "clamp":
            msg = self.Float32()
            msg.data = event.payload[0]
            self.clamp_publishers[event.hand].publish(msg)

    def replay(self):
        if self.args.rate <= 0:
            raise ValueError("--rate 必须大于 0。")

        self.video_readers = {
            hand: VideoReader(paths.video)
            for hand, paths in self.session_data.hand_paths.items()
        }
        try:
            print(f"开始回放: {self.session_data.session_dir}")
            summarize_session(self.session_data)
            wall_start = time.monotonic()
            replay_start_stamp = self.rospy.Time.now()
            source_start = self.session_data.start_time

            for index, event in enumerate(self.session_data.events, start=1):
                if self.rospy.is_shutdown():
                    break

                relative_time = event.timestamp - source_start
                target_wall_time = relative_time / self.args.rate
                wait_time = wall_start + target_wall_time - time.monotonic()
                if wait_time > 0:
                    self.rospy.sleep(wait_time)

                stamp = self._stamp_for(replay_start_stamp, relative_time)
                self._publish_event(event, stamp)

                if index % 1000 == 0:
                    print(f"已发布 {index}/{len(self.session_data.events)} 条消息")

            print("回放结束。")
        finally:
            for reader in self.video_readers.values():
                reader.close()
            self.video_readers = {}


def run_interactive(args):
    import rospy

    rospy.init_node("fastumi_data_replay", anonymous=True)
    data_root = args.data_root.expanduser().resolve()
    frame_ids = {
        "left": args.left_base_frame,
        "right": args.right_base_frame,
    }
    date_dirs = list_matching_dirs(data_root, DATE_DIR_RE)
    if not date_dirs:
        print(f"未找到日期目录: {data_root}/multi_session_YYYYMMDD")
        return 1

    while not rospy.is_shutdown():
        date_dir = prompt_choice("请选择日期目录:", date_dirs)
        if date_dir is EXIT_REQUESTED:
            print("退出。")
            return 0
        session_dirs = list_matching_dirs(date_dir, SESSION_DIR_RE)
        if not session_dirs:
            print(f"{date_dir} 下没有 session_HHMMSS 目录。")
            continue

        while not rospy.is_shutdown():
            session_dir = prompt_choice(
                f"请选择 {date_dir.name} 下的 session:", session_dirs, allow_back=True
            )
            if session_dir is EXIT_REQUESTED:
                print("退出。")
                return 0
            if session_dir is None:
                break

            try:
                session_data = load_session(session_dir, frame_ids)
                publisher = RosReplayPublisher(args, session_data)
                publisher.replay()
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                print(f"无法回放 {session_dir}: {exc}")

    return 0


def main():
    args = parse_args()
    if args.check_session:
        try:
            frame_ids = {
                "left": args.left_base_frame,
                "right": args.right_base_frame,
            }
            session_data = load_session(
                args.check_session.expanduser().resolve(),
                frame_ids,
            )
            summarize_session(session_data)
        except SessionValidationError as exc:
            print(f"检查失败: {exc}", file=sys.stderr)
            return 1
        return 0

    try:
        return run_interactive(args)
    except KeyboardInterrupt:
        print("\n收到中断，退出。")
        return 130
    except ImportError as exc:
        print(
            "缺少运行依赖。请先执行 use_ros1，并确认 rospy、sensor_msgs、geometry_msgs、"
            f"std_msgs、opencv-python 可用。原始错误: {exc}",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    sys.exit(main())
