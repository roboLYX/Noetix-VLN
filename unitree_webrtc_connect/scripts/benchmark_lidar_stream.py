from __future__ import annotations

import argparse
import asyncio
import logging
import sys
import time
from pathlib import Path
from typing import Any, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from unitree_webrtc_connect import RTC_TOPIC, UnitreeWebRTCConnection, WebRTCConnectionMethod
from unitree_webrtc_connect.utils.benchmark_metrics import LidarStreamMetrics


logging.basicConfig(level=logging.WARNING, format="[%(levelname)s] %(message)s")


LIDAR_TOPIC_OPTIONS = {
    "voxel_map": RTC_TOPIC["ULIDAR"],
    "voxel_map_compressed": RTC_TOPIC["ULIDAR_ARRAY"],
}


def _format_optional_number(value: Optional[float], digits: int = 3) -> str:
    if value is None:
        return "N/A"
    return f"{value:.{digits}f}"


def _extract_point_count(message: dict[str, Any]) -> tuple[bool, Optional[int], str]:
    data = message.get("data")
    if not isinstance(data, dict):
        return False, None, "message['data'] is not a dict"

    decoded_data = data.get("data")
    if not isinstance(decoded_data, dict):
        return False, None, "message['data']['data'] is not a dict"

    positions = decoded_data.get("positions")
    if positions is not None:
        if not hasattr(positions, "__len__"):
            return False, None, "positions is not a sized sequence"
        positions_len = len(positions)
        if positions_len % 3 != 0:
            return False, None, f"positions length {positions_len} is not divisible by 3"
        return True, positions_len // 3, ""

    points = decoded_data.get("points")
    if points is not None:
        shape = getattr(points, "shape", None)
        if shape is not None:
            if len(shape) != 2 or shape[1] != 3:
                return False, None, f"points shape {shape} is not Nx3"
            return True, int(shape[0]), ""
        if isinstance(points, list):
            if not all(isinstance(item, (list, tuple)) and len(item) == 3 for item in points):
                return False, None, "points list is not Nx3"
            return True, len(points), ""
        return False, None, "points is not an array-like Nx3 structure"

    return True, None, ""


def _get_requested_lidar_topics(args: argparse.Namespace) -> list[str]:
    if args.lidar_topic == "auto":
        return [
            LIDAR_TOPIC_OPTIONS["voxel_map_compressed"],
            LIDAR_TOPIC_OPTIONS["voxel_map"],
        ]
    return [LIDAR_TOPIC_OPTIONS[args.lidar_topic]]


def _build_lidar_summary_lines(
    metrics: LidarStreamMetrics,
    gap_threshold_s: float,
) -> list[str]:
    summary = metrics.summarize(gap_threshold_s)
    lines = [
        "[LIDAR SUMMARY]",
        f"total_frames={summary['total_frames']}",
        f"ok_frames={summary['ok_frames']}",
        f"failed_frames={summary['failed_frames']}",
        f"avg_hz={_format_optional_number(summary['avg_hz'])}",
        f"median_gap_s={_format_optional_number(summary['median_gap_s'], 4)}",
        f"p95_gap_s={_format_optional_number(summary['p95_gap_s'], 4)}",
        f"max_gap_s={_format_optional_number(summary['max_gap_s'], 4)}",
        f"gap_over_{gap_threshold_s:.3f}s={summary['gap_over_threshold_count']}",
        f"avg_point_count={_format_optional_number(summary['avg_point_count'], 1)}",
        "median_point_count="
        f"{_format_optional_number(summary['median_point_count'], 1)}",
    ]

    if summary["total_frames"] == 0:
        lines.append("[RESULT] LiDAR 不可用: 运行窗口内没有收到任何 LiDAR 帧")
    elif summary["ok_frames"] == 0:
        lines.append("[RESULT] LiDAR 不可用: 收到了 LiDAR topic 消息，但解码结果全部不可用")
    else:
        lines.append("[RESULT] LiDAR 可回传: 已收到可解码点云帧")

    return lines


def _print_and_save_lidar_summary(
    metrics: LidarStreamMetrics,
    gap_threshold_s: float,
    output_path: str,
) -> None:
    summary_text = "\n".join(_build_lidar_summary_lines(metrics, gap_threshold_s))
    print(f"\n{summary_text}")

    output_file = Path(output_path)
    output_file.write_text(f"{summary_text}\n", encoding="utf-8")
    print(f"[INFO] summary saved to {output_file}")


async def _run_lidar_benchmark(args: argparse.Namespace) -> LidarStreamMetrics:
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
    metrics = LidarStreamMetrics()
    requested_topics = _get_requested_lidar_topics(args)
    active_lidar_topic: Optional[str] = None

    def lidar_callback(message: dict[str, Any], topic: str) -> None:
        nonlocal active_lidar_topic
        if active_lidar_topic is not None and topic != active_lidar_topic:
            return

        arrival_monotonic_s = time.monotonic()
        try:
            decoded_ok, point_count, error = _extract_point_count(message)
        except Exception as exc:  # noqa: BLE001
            metrics.add_frame(arrival_monotonic_s, False, None, repr(exc))
            return
        if active_lidar_topic is None:
            active_lidar_topic = topic
            print(f"[INFO] using LiDAR topic: {active_lidar_topic}")
        metrics.add_frame(arrival_monotonic_s, decoded_ok, point_count, error)

    try:
        print("[INFO] connecting LocalAP WebRTC...")
        await asyncio.wait_for(conn.connect(), timeout=args.connect_timeout)
        print("[OK] WebRTC connected")

        await asyncio.wait_for(
            conn.datachannel.disableTrafficSaving(True),
            timeout=args.request_timeout,
        )
        conn.datachannel.set_decoder(decoder_type=args.decoder)
        for topic in requested_topics:
            conn.datachannel.pub_sub.subscribe(
                topic,
                lambda message, source_topic=topic: lidar_callback(message, source_topic),
            )
            print(f"[INFO] subscribed to LiDAR topic: {topic}")
        conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["ULIDAR_SWITCH"], "on")
        print(
            "[INFO] LiDAR subscribed: "
            f"topics={requested_topics}, duration={args.duration}s"
        )

        await asyncio.sleep(args.duration)
    finally:
        try:
            conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["ULIDAR_SWITCH"], "off")
        except Exception:
            pass
        await conn.disconnect()

    return metrics


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Benchmark Go2-W LiDAR stream over LocalAP WebRTC."
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--gap-threshold", type=float, default=0.5)
    parser.add_argument("--csv", type=str, default="")
    parser.add_argument("--output", type=str, default="benchmark_lidar_stream_output.txt")
    parser.add_argument("--decoder", choices=["libvoxel", "native"], default="libvoxel")
    parser.add_argument(
        "--lidar-topic",
        choices=["auto", "voxel_map", "voxel_map_compressed"],
        default="auto",
    )
    parser.add_argument("--connect-timeout", type=float, default=20.0)
    parser.add_argument("--request-timeout", type=float, default=5.0)
    return parser


def main() -> int:
    args = _build_parser().parse_args()

    try:
        metrics = asyncio.run(_run_lidar_benchmark(args))
    except KeyboardInterrupt:
        print("\n[INFO] interrupted by user")
        return 130

    if args.csv:
        metrics.write_csv(args.csv)
        print(f"[INFO] CSV saved to {args.csv}")

    _print_and_save_lidar_summary(metrics, args.gap_threshold, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
