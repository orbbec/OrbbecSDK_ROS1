#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
function:
  Monitor subscriber-side frame rates, end-to-end delays, CPU and RAM usage,
  and estimated frame loss for Orbbec camera topics.

usage examples:
  # Run for 2 minutes, save results to a CSV file
  1. rosrun orbbec_camera common_benchmark_node.py _run_time:=2m _csv_file:=/path/to/log.csv

  # Run for 30 seconds
  2. rosrun orbbec_camera common_benchmark_node.py --run_time 30s

  # Monitor multiple cameras and write one CSV row per timestamp
  3. rosrun orbbec_camera common_benchmark_node.py --camera_names camera,camera01

  # Select streams or full compressed-image/point-cloud topics
  4. rosrun orbbec_camera common_benchmark_node.py --topics color,depth

Parameters:
  --run_time / _run_time: Set run time duration. Supports formats:
      "10s" (10 seconds), "5m" (5 minutes), "1h" (1 hour), "2d" (2 days).

  --csv_file / _csv_file: Path to save the benchmark log in CSV format.
      Default: ./multi_service_results.csv
"""

import argparse
import rospy
import psutil
import time
import csv
import os
from collections import defaultdict
from orbbec_camera.msg import DeviceStatus
from sensor_msgs.msg import CompressedImage, Image, PointCloud2

from tabulate import tabulate

CAMERA_NODE_NAMES = ["component_container", "orbbec_camera_node", "nodelet"]
MONITORED_STREAMS = (
    "color",
    "depth",
    "ir",
    "left_ir",
    "right_ir",
    "left_color",
    "right_color",
)
DISCOVERY_INTERVAL_SECONDS = 0.1
DISCOVERY_DURATION_SECONDS = 1.0

# ----------------tool functions----------------
def parse_duration(s):
    # Parse duration strings like "10s", "5m", "1h", "2d" into seconds.
    if isinstance(s, (int, float)):
        return float(s)

    s = str(s).strip().lower()
    if s.endswith("s"):
        return float(s[:-1])
    elif s.endswith("m"):
        return float(s[:-1]) * 60
    elif s.endswith("h"):
        return float(s[:-1]) * 3600
    elif s.endswith("d"):
        return float(s[:-1]) * 86400
    else:
        return float(s)

def format_duration(seconds):
    # Format seconds into a human-readable string like "1h 2m 3s".
    seconds = int(seconds)
    days, seconds = divmod(seconds, 86400)
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)

    parts = []
    if days > 0:
        parts.append(f"{days}d")
    if hours > 0:
        parts.append(f"{hours}h")
    if minutes > 0:
        parts.append(f"{minutes}m")
    if seconds > 0 or not parts:
        parts.append(f"{seconds}s")
    return " ".join(parts)

def parse_camera_names(camera_names):
    if isinstance(camera_names, str):
        names = camera_names.replace(";", ",").split(",")
    else:
        names = camera_names or []

    parsed_names = []
    for name in names:
        normalized = str(name).strip().strip("/")
        if normalized and normalized not in parsed_names:
            parsed_names.append(normalized)
    return parsed_names or ["camera"]

def make_stat():
    return {"cur": 0.0, "avg": 0.0, "min": float("inf"), "max": float("-inf"), "count": 0, "sum": 0.0}

def estimate_dropped_frames(dt, expected_interval):
    if expected_interval <= 0 or dt <= 1.5 * expected_interval:
        return 0
    return max(1, int(dt / expected_interval) - 1)
# ----------------------------------------------

class TopicTracker:
    def __init__(self, sample_start_time=None):
        self.received = 0
        self.sample_start_time = sample_start_time or time.monotonic()
        self.last_sample_time = self.sample_start_time
        self.last_sample_received = 0
        self.last_header_stamp = None
        self.estimated_interval = None
        self.drop_frames = 0

    def on_msg(self, header, ros_receive_time, ideal_fps):
        """Record a received message and return its age in milliseconds."""
        header_stamp = header.stamp.to_sec()
        self.received += 1
        delay_ms = None

        if header_stamp > 0:
            delay_ms = (ros_receive_time - header_stamp) * 1000.0
            if self.last_header_stamp is not None:
                header_dt = header_stamp - self.last_header_stamp
                if header_dt > 0:
                    expected_interval = (
                        1.0 / ideal_fps
                        if ideal_fps and ideal_fps > 0.0
                        else self.estimated_interval
                    )
                    if expected_interval is not None:
                        self.drop_frames += estimate_dropped_frames(
                            header_dt, expected_interval
                        )
                    self.update_estimated_interval(header_dt)
            self.last_header_stamp = header_stamp

        return delay_ms

    def sample_fps(self, sample_time):
        window_elapsed = sample_time - self.last_sample_time
        total_elapsed = sample_time - self.sample_start_time
        if window_elapsed <= 0.0 or total_elapsed <= 0.0:
            return None

        window_received = self.received - self.last_sample_received
        current_fps = window_received / window_elapsed
        average_fps = self.received / total_elapsed
        self.last_sample_time = sample_time
        self.last_sample_received = self.received
        return current_fps, average_fps

    def update_estimated_interval(self, interval):
        if self.estimated_interval is None or interval < 0.75 * self.estimated_interval:
            self.estimated_interval = interval
        elif interval <= 1.5 * self.estimated_interval:
            self.estimated_interval = 0.9 * self.estimated_interval + 0.1 * interval

    def frames_loss_rate(self):
        total = self.received + self.drop_frames
        if total == 0:
            return 0.0
        return float(self.drop_frames) / total

class CameraMonitorNode:
    def __init__(
        self,
        run_time,
        csv_file="camera_monitor_log.csv",
        camera_names=None,
        ideal_fps=0.0,
        topics=None,
    ):
        self.run_time = run_time
        self.discovery_start_time = time.time()
        self.start_time = None
        self.process = psutil.Process(os.getpid())
        self.first_data_collected = False
        self.camera_names = parse_camera_names(camera_names)
        self.node_names = {camera_name: "Not Found" for camera_name in self.camera_names}
        self.total_node_name = "Not Found"
        self.ideal_fps = float(ideal_fps) if ideal_fps is not None else 0.0

        self.cameras = {}
        for camera_name in self.camera_names:
            self.cameras[camera_name] = {
                "connection_type": None,
                "disconnect_count": 0,
                "prev_online": True,
                "data_collected": False,
                "stats": defaultdict(make_stat),
                "cpu_stats": make_stat(),
                "ram_stats": make_stat(),
                "trackers": {},
            }

        self.total_cpu_stats = make_stat()
        self.total_ram_stats = make_stat()
        self.status_subscriptions = []
        self.topic_subscriptions = {}
        self.topic_configs = {}
        self.requested_streams = self.parse_requested_topics(topics)

        # CSV
        self.csv_file = csv_file
        self.csv_fh = open(self.csv_file, "w", newline="")
        self.csv_writer = csv.writer(self.csv_fh)

        for camera_name in self.camera_names:
            ns = self.camera_namespace(camera_name)
            self.status_subscriptions.append(
                rospy.Subscriber(
                    f"{ns}/device_status",
                    DeviceStatus,
                    self.status_callback,
                    callback_args=camera_name,
                )
            )

        selected_topics = self.requested_streams or self.discover_published_image_streams()
        self.finish_topic_discovery(selected_topics)

    def camera_namespace(self, camera_name):
        return "/" + camera_name.strip("/")

    def image_topic_name(self, camera_name, stream):
        return f"{self.camera_namespace(camera_name)}/{stream}/image_raw"

    def make_raw_image_config(self, camera_name, stream):
        return {
            "camera_name": camera_name,
            "topic_id": stream,
            "topic_name": self.image_topic_name(camera_name, stream),
            "msg_type": Image,
        }

    def parse_full_topic(self, topic_name):
        normalized_topic = "/" + topic_name.strip("/")
        for camera_name in self.camera_names:
            namespace_prefix = self.camera_namespace(camera_name) + "/"
            if not normalized_topic.startswith(namespace_prefix):
                continue

            relative_name = normalized_topic[len(namespace_prefix):]
            for stream in MONITORED_STREAMS:
                raw_name = f"{stream}/image_raw"
                if relative_name == raw_name:
                    return self.make_raw_image_config(camera_name, stream)
                if relative_name == f"{raw_name}/compressed":
                    return {
                        "camera_name": camera_name,
                        "topic_id": f"{stream}_compressed",
                        "topic_name": normalized_topic,
                        "msg_type": CompressedImage,
                    }
                if relative_name == f"{raw_name}/compressedDepth":
                    return {
                        "camera_name": camera_name,
                        "topic_id": f"{stream}_compressed_depth",
                        "topic_name": normalized_topic,
                        "msg_type": CompressedImage,
                    }

            point_cloud_ids = {
                "depth/points": "depth_points",
                "depth_registered/points": "depth_registered_points",
            }
            if relative_name in point_cloud_ids:
                return {
                    "camera_name": camera_name,
                    "topic_id": point_cloud_ids[relative_name],
                    "topic_name": normalized_topic,
                    "msg_type": PointCloud2,
                }
        return None

    def parse_requested_topics(self, topics):
        if not topics:
            return {}

        selected = {}
        for raw_value in str(topics).replace(";", ",").split(","):
            value = raw_value.strip()
            if not value:
                continue
            if value in MONITORED_STREAMS:
                for camera_name in self.camera_names:
                    config = self.make_raw_image_config(camera_name, value)
                    selected[(camera_name, config["topic_id"])] = config
                continue

            config = self.parse_full_topic(value)
            if config is None:
                raise ValueError(
                    f"Unsupported topic '{value}'. Specify a raw or compressed image topic, "
                    "or a depth/points or depth_registered/points topic under a configured "
                    "camera namespace."
                )
            selected[(config["camera_name"], config["topic_id"])] = config
        return selected

    def find_published_image_streams(self):
        published_topic_names = {name for name, _ in rospy.get_published_topics()}
        published_streams = {}
        for camera_name in self.camera_names:
            for stream in MONITORED_STREAMS:
                config = self.make_raw_image_config(camera_name, stream)
                if config["topic_name"] in published_topic_names:
                    published_streams[(camera_name, stream)] = config
        return published_streams

    def discover_published_image_streams(self):
        discovered_streams = {}
        discovery_start = time.monotonic()
        while not rospy.is_shutdown():
            discovered_streams.update(self.find_published_image_streams())
            if time.monotonic() - discovery_start >= DISCOVERY_DURATION_SECONDS:
                break
            rospy.sleep(DISCOVERY_INTERVAL_SECONDS)
        return discovered_streams

    def finish_topic_discovery(self, selected_streams):
        self.topic_configs = dict(selected_streams)
        sample_start_time = time.monotonic()
        for key, config in self.topic_configs.items():
            camera_name = config["camera_name"]
            topic_id = config["topic_id"]
            self.cameras[camera_name]["trackers"][topic_id] = TopicTracker(sample_start_time)
            self.topic_subscriptions[key] = rospy.Subscriber(
                config["topic_name"],
                config["msg_type"],
                self.topic_callback,
                callback_args=(camera_name, topic_id),
            )

        self.csv_writer.writerow(self.build_csv_header())
        self.csv_fh.flush()
        self.start_time = time.time()

    def topics_for_camera(self, camera_name):
        return [
            config
            for config in self.topic_configs.values()
            if config["camera_name"] == camera_name
        ]

    def update_topic_fps_stats(self):
        sample_time = time.monotonic()
        for config in self.topic_configs.values():
            camera = self.cameras[config["camera_name"]]
            topic_id = config["topic_id"]
            fps_sample = camera["trackers"][topic_id].sample_fps(sample_time)
            if fps_sample is None:
                continue
            current_fps, average_fps = fps_sample
            self.update_sample_stat(
                camera["stats"][f"{topic_id}_fps"],
                current_fps,
                average=average_fps,
            )

    def cmdline_has_camera_namespace(self, cmdline_args, camera_name):
        ns = self.camera_namespace(camera_name)
        candidates = [
            f"__ns:={ns}",
            f"__ns:={camera_name}",
            f"namespace:={ns}",
            f"namespace:={camera_name}",
            f"/{camera_name}/camera",
            f"/{camera_name}/{camera_name}",
        ]
        if any(arg in candidates for arg in cmdline_args):
            return True
        return any(arg.startswith(f"{ns}/") or f":={ns}/" in arg for arg in cmdline_args)

    def find_camera_nodes(self):
        found = {camera_name: [] for camera_name in self.camera_names}
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline_args = proc.info.get('cmdline') or []
                cmdline = " ".join(cmdline_args)
                if not any(name.lower() in cmdline.lower() for name in CAMERA_NODE_NAMES):
                    continue
                for camera_name in self.camera_names:
                    if self.cmdline_has_camera_namespace(cmdline_args, camera_name):
                        found[camera_name].append(proc)
            except Exception:
                continue
        return found

    def get_camera_stats(self):
        found = self.find_camera_nodes()
        camera_stats = {}
        total_cpu = 0.0
        total_ram = 0.0
        total_proc_count = 0

        for camera_name, root_procs in found.items():
            if not root_procs:
                camera_stats[camera_name] = (0.0, 0.0, "Not Found")
                continue

            seen_pids = set()
            procs = []
            for proc in root_procs:
                try:
                    proc_group = [proc] + proc.children(recursive=True)
                    for p in proc_group:
                        if p.pid not in seen_pids:
                            seen_pids.add(p.pid)
                            procs.append(p)
                except Exception:
                    continue

            try:
                cpu = sum((p.cpu_percent(interval=None) for p in procs)) / max(1, psutil.cpu_count())
                mem_bytes = sum((p.memory_info().rss for p in procs))
                mem_mb = mem_bytes / (1024 * 1024)
                root_names = ", ".join(f"{p.name()}[{p.pid}]" for p in root_procs)
                if len(procs) > len(root_procs):
                    root_names = f"{root_names} + {len(procs) - len(root_procs)} child"
                camera_stats[camera_name] = (cpu, mem_mb, root_names)
                total_cpu += cpu
                total_ram += mem_mb
                total_proc_count += len(procs)
            except Exception:
                camera_stats[camera_name] = (0.0, 0.0, "Error")

        total_node_name = f"{total_proc_count} matched process(es)" if total_proc_count > 0 else "Not Found"
        return camera_stats, total_cpu, total_ram, total_node_name

    def status_callback(self, msg, camera_name):
        if not self.first_data_collected:
          self.first_data_collected = True

        camera = self.cameras[camera_name]
        camera["data_collected"] = True
        camera["connection_type"] = msg.connection_type
        if camera["prev_online"] and not msg.device_online:
            camera["disconnect_count"] += 1
            camera["prev_online"] = msg.device_online
            return

        camera["prev_online"] = msg.device_online

    def topic_callback(self, msg, callback_args):
        camera_name, topic_id = callback_args
        self.first_data_collected = True
        camera = self.cameras[camera_name]
        camera["data_collected"] = True
        tracker = camera["trackers"][topic_id]
        delay_ms = tracker.on_msg(
            msg.header,
            rospy.Time.now().to_sec(),
            self.ideal_fps,
        )
        if delay_ms is not None:
            self.update_sample_stat(camera["stats"][f"{topic_id}_delay"], delay_ms)

    def update_sample_stat(self, stat, value, average=None):
        stat["cur"] = value
        stat["count"] += 1
        stat["sum"] += value
        stat["avg"] = average if average is not None else stat["sum"] / stat["count"]
        stat["min"] = min(stat["min"], value)
        stat["max"] = max(stat["max"], value)

    def update_sys_stat(self, stat_dict, value, online=True):
        stat_dict["cur"] = value

        if value is None or value <= 0.0 or not online:
            return

        stat_dict["count"] += 1
        stat_dict["sum"] += value
        stat_dict["avg"] = stat_dict["sum"] / stat_dict["count"] if stat_dict["count"] > 0 else 0.0
        stat_dict["min"] = min(stat_dict["min"], value)
        stat_dict["max"] = max(stat_dict["max"], value)

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            elapsed = time.time() - self.start_time
            if elapsed > self.run_time:
                break

            self.update_topic_fps_stats()
            camera_sys_stats, total_cpu, total_ram, self.total_node_name = self.get_camera_stats()
            for camera_name in self.camera_names:
                camera = self.cameras[camera_name]
                cpu, ram, node_name = camera_sys_stats.get(camera_name, (0.0, 0.0, "Not Found"))
                self.node_names[camera_name] = node_name
                self.update_sys_stat(camera["cpu_stats"], cpu, camera["prev_online"])
                self.update_sys_stat(camera["ram_stats"], ram, camera["prev_online"])

            self.update_sys_stat(self.total_cpu_stats, total_cpu, True)
            self.update_sys_stat(self.total_ram_stats, total_ram, True)

            if self.first_data_collected:
                self.log_to_csv(elapsed)
                self.print_status()

            rate.sleep()

        self.csv_fh.close()
        elapsed = time.time() - self.start_time
        rospy.loginfo("Monitoring finished, it takes time: %s", format_duration(elapsed))
        rospy.loginfo("CSV data is saved to %s", self.csv_file)

    def log_to_csv(self, elapsed):
        row = [round(elapsed, 2)]
        for camera_name in self.camera_names:
            row.extend(self.build_camera_csv_values(camera_name, self.cameras[camera_name]))

        row.extend([
            round(self.total_cpu_stats["cur"], 2), round(self.total_cpu_stats["avg"], 2),
            self.format_csv_number(self.total_cpu_stats["min"]), self.format_csv_number(self.total_cpu_stats["max"]),
            round(self.total_ram_stats["cur"], 2), round(self.total_ram_stats["avg"], 2),
            self.format_csv_number(self.total_ram_stats["min"]), self.format_csv_number(self.total_ram_stats["max"]),
        ])
        self.csv_writer.writerow(row)

    def build_csv_header(self):
        header = ["time(s)"]
        for camera_name in self.camera_names:
            header.extend(
                [
                    f"{camera_name}_connection_type",
                    f"{camera_name}_status_online",
                    f"{camera_name}_disconnects",
                ]
            )
            for config in self.topics_for_camera(camera_name):
                topic_id = config["topic_id"]
                header.extend(
                    [
                        f"{camera_name}_{topic_id}_fps_cur",
                        f"{camera_name}_{topic_id}_fps_avg",
                        f"{camera_name}_{topic_id}_fps_min",
                        f"{camera_name}_{topic_id}_fps_max",
                        f"{camera_name}_{topic_id}_delay_cur",
                        f"{camera_name}_{topic_id}_delay_avg",
                        f"{camera_name}_{topic_id}_delay_min",
                        f"{camera_name}_{topic_id}_delay_max",
                        f"{camera_name}_{topic_id}_sub_lost_count",
                        f"{camera_name}_{topic_id}_sub_lost_rate(%)",
                    ]
                )
            header.extend(
                [
                    f"{camera_name}_cpu_cur",
                    f"{camera_name}_cpu_avg",
                    f"{camera_name}_cpu_min",
                    f"{camera_name}_cpu_max",
                    f"{camera_name}_ram_cur",
                    f"{camera_name}_ram_avg",
                    f"{camera_name}_ram_min",
                    f"{camera_name}_ram_max",
                ]
            )

        header.extend([
            "total_cpu_cur", "total_cpu_avg", "total_cpu_min", "total_cpu_max",
            "total_ram_cur", "total_ram_avg", "total_ram_min", "total_ram_max",
        ])
        return header

    def build_camera_csv_values(self, camera_name, camera):
        def safe(k):
            v = camera["stats"].get(k, {})
            return (
                round(v.get("cur", 0.0), 2),
                round(v.get("avg", 0.0), 2),
                self.format_csv_number(v.get("min", 0.0)),
                self.format_csv_number(v.get("max", 0.0)),
            )

        values = [
            camera["connection_type"],
            camera["prev_online"],
            camera["disconnect_count"],
        ]
        for config in self.topics_for_camera(camera_name):
            topic_id = config["topic_id"]
            tracker = camera["trackers"][topic_id]
            if camera["prev_online"]:
                values.extend(safe(f"{topic_id}_fps"))
                values.extend(safe(f"{topic_id}_delay"))
            else:
                values.extend(["N/A"] * 8)
            values.extend(
                [
                    tracker.drop_frames,
                    round(tracker.frames_loss_rate() * 100.0, 3),
                ]
            )

        if camera["prev_online"]:
            values.extend(
                [
                    round(camera["cpu_stats"]["cur"], 2),
                    round(camera["cpu_stats"]["avg"], 2),
                    self.format_csv_number(camera["cpu_stats"]["min"]),
                    self.format_csv_number(camera["cpu_stats"]["max"]),
                    round(camera["ram_stats"]["cur"], 2),
                    round(camera["ram_stats"]["avg"], 2),
                    self.format_csv_number(camera["ram_stats"]["min"]),
                    self.format_csv_number(camera["ram_stats"]["max"]),
                ]
            )
        else:
            values.extend(
                [
                    round(camera["cpu_stats"]["cur"], 2), "N/A", "N/A", "N/A",
                    round(camera["ram_stats"]["cur"], 2), "N/A", "N/A", "N/A",
                ]
            )
        return values

    def format_csv_number(self, value):
        if value == float("inf") or value == float("-inf"):
            return 0.0
        return round(value, 2)

    def print_status(self):
        """
        Print the Orbbec camera monitoring status, including FPS, Delay, CPU, and RAM usage.
        """
        def format_stats(s):
          if s["count"] <= 0:
              return "0.00", "0.00", "0.00", "0.00"
          return f"{s['cur']:.2f}", f"{s['avg']:.2f}", f"{s['min']:.2f}", f"{s['max']:.2f}"

        rows = []
        for camera_name in self.camera_names:
            camera = self.cameras[camera_name]
            for config in self.topics_for_camera(camera_name):
                topic_id = config["topic_id"]
                fps_key = f"{topic_id}_fps"
                delay_key = f"{topic_id}_delay"
                topic_name = config["topic_name"]

                if not camera["prev_online"]:
                    rows.append([camera_name, topic_name, *["N/A"] * 10])
                else:
                    fps_vals = format_stats(camera["stats"][fps_key])
                    delay_vals = format_stats(camera["stats"][delay_key])

                    tracker = camera["trackers"][topic_id]
                    frames_loss = tracker.drop_frames
                    frames_loss_rate = round(tracker.frames_loss_rate() * 100.0, 3)
                    rows.append([camera_name, topic_name, *fps_vals, *delay_vals, frames_loss, frames_loss_rate])

        header_bottom = [
            "Camera", "Topic", "fps_cur", "fps_avg", "fps_min", "fps_max",
            "delay_cur(ms)", "delay_avg(ms)", "delay_min(ms)", "delay_max(ms)",
            "Sub_lost_count", "Sub_lost_rate(%)",
        ]

        os.system("clear")
        print("Orbbec Camera Subscriber Benchmark\n")
        print(tabulate([header_bottom] + rows, tablefmt="fancy_grid"))

        sys_rows = []
        for camera_name in self.camera_names:
            camera = self.cameras[camera_name]
            if not camera["prev_online"]:
                cpu_vals = (f"{camera['cpu_stats']['cur']:.2f}", "N/A", "N/A", "N/A")
                ram_vals = (f"{camera['ram_stats']['cur']:.2f}", "N/A", "N/A", "N/A")
            else:
                cpu_vals = format_stats(camera["cpu_stats"])
                ram_vals = format_stats(camera["ram_stats"])

            sys_rows.append([camera_name, "CPU Usage (%)", *cpu_vals, self.node_names[camera_name]])
            sys_rows.append([camera_name, "RAM Usage (MB)", *ram_vals, self.node_names[camera_name]])

        sys_rows.append(["TOTAL", "CPU Usage (%)", *format_stats(self.total_cpu_stats), self.total_node_name])
        sys_rows.append(["TOTAL", "RAM Usage (MB)", *format_stats(self.total_ram_stats), self.total_node_name])

        print("\n\n(CPU & RAM)\n")
        print(tabulate(sys_rows, headers=["Camera", "Option", "cur", "avg", "min", "max", "Camera Node"], tablefmt="fancy_grid"))

        status_rows = []
        for camera_name in self.camera_names:
            camera = self.cameras[camera_name]
            status_rows.append([camera_name, camera["connection_type"], camera["prev_online"], camera["disconnect_count"]])
        print("\n")
        print(tabulate(status_rows, headers=["Camera", "connection_type", "status_online", "disconnect_count"], tablefmt="fancy_grid"))



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--run_time", type=str, default=None, help="Total run time for monitoring, e.g., 10s, 5m, 1h. Default is 10 seconds.")
    parser.add_argument("--csv_file", type=str, default=None)
    parser.add_argument("--camera_names", type=str, default=None, help="Comma-separated camera namespaces, e.g., camera,camera01,camera02.")
    parser.add_argument("--ideal_fps", type=float, default=None, help="Optional ideal frame rate for subscriber-side drop detection.")
    parser.add_argument(
        "--topics",
        type=str,
        default=None,
        help=(
            "Comma-separated raw stream names or full raw/compressed image and point cloud "
            "topics. Automatic discovery only selects raw image topics."
        ),
    )
    args, unknown = parser.parse_known_args()

    rospy.init_node("camera_monitor_node")

    run_time_param = args.run_time if args.run_time is not None else rospy.get_param("~run_time", "10s")
    run_time = parse_duration(run_time_param)
    csv_file = args.csv_file if args.csv_file is not None else rospy.get_param("~csv_file", "camera_monitor_log.csv")
    camera_names = args.camera_names if args.camera_names is not None else rospy.get_param("~camera_names", "camera")
    ideal_fps = args.ideal_fps if args.ideal_fps is not None else rospy.get_param("~ideal_fps", 0.0)
    topics = args.topics if args.topics is not None else rospy.get_param("~topics", "")

    monitor = CameraMonitorNode(
        run_time,
        csv_file,
        camera_names=camera_names,
        ideal_fps=ideal_fps,
        topics=topics,
    )
    monitor.run()
