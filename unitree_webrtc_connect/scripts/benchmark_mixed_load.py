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

from unitree_webrtc_connect import (
    RTC_TOPIC,
    SPORT_CMD,
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)
from unitree_webrtc_connect.utils.benchmark_metrics import (
    LidarStreamMetrics,
    RequestRttMetrics,
)


logging.basicConfig(level=logging.WARNING, format="[%(levelname)s] %(message)s")


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
    if positions is None:
        return True, None, ""
    if not isinstance(positions, list):
        return False, None, "positions is not a list"
    if len(positions) % 3 != 0:
        return False, None, f"positions length {len(positions)} is not divisible by 3"

    return True, len(positions) // 3, ""


async def _publish_sport_request(
    conn: UnitreeWebRTCConnection,
    api_id: int,
    parameter: Optional[dict[str, float]],
    timeout_s: float,
) -> dict[str, Any]:
    payload: dict[str, Any] = {"api_id": api_id}
    if parameter is not None:
        payload["parameter"] = parameter

    return await asyncio.wait_for(
        conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], payload),
        timeout=timeout_s,
    )


async def _control_loop(
    conn: UnitreeWebRTCConnection,
    args: argparse.Namespace,
    stop_event: asyncio.Event,
    rtt_metrics: RequestRttMetrics,
) -> None:
    yaw_sign = 1.0
    while not stop_event.is_set():
        if abs(args.yaw_pulse) > 0.0:
            move_payload = {"x": 0.0, "y": 0.0, "z": yaw_sign * abs(args.yaw_pulse)}
            started_s = time.monotonic()
            try:
                await _publish_sport_request(
                    conn,
                    SPORT_CMD["Move"],
                    move_payload,
                    args.request_timeout,
                )
                rtt_metrics.add_request(
                    time.monotonic(),
                    time.monotonic() - started_s,
                    True,
                    "MoveYaw",
                )
            except Exception as exc:  # noqa: BLE001
                rtt_metrics.add_request(
                    time.monotonic(),
                    time.monotonic() - started_s,
                    False,
                    "MoveYaw",
                    repr(exc),
                )

            try:
                await asyncio.wait_for(stop_event.wait(), timeout=args.yaw_duration)
            except asyncio.TimeoutError:
                pass

            yaw_sign *= -1.0

        started_s = time.monotonic()
        try:
            await _publish_sport_request(
                conn,
                SPORT_CMD["StopMove"],
                None,
                args.request_timeout,
            )
            rtt_metrics.add_request(
                time.monotonic(),
                time.monotonic() - started_s,
                True,
                "StopMove",
            )
        except Exception as exc:  # noqa: BLE001
            rtt_metrics.add_request(
                time.monotonic(),
                time.monotonic() - started_s,
                False,
                "StopMove",
                repr(exc),
            )

        try:
            await asyncio.wait_for(stop_event.wait(), timeout=args.control_interval)
        except asyncio.TimeoutError:
            pass


def _build_summary_lines(
    lidar_metrics: LidarStreamMetrics,
    rtt_metrics: RequestRttMetrics,
    low_state_count: int,
    lf_sport_state_count: int,
    gap_threshold_s: float,
) -> list[str]:
    lidar_summary = lidar_metrics.summarize(gap_threshold_s)
    rtt_summary = rtt_metrics.summarize()

    lines = [
        "[MIXED LOAD SUMMARY]",
        "[LiDAR] "
        f"frames={lidar_summary['total_frames']}, "
        f"ok={lidar_summary['ok_frames']}, "
        f"failed={lidar_summary['failed_frames']}, "
        f"avg_hz={_format_optional_number(lidar_summary['avg_hz'])}, "
        f"median_gap_s={_format_optional_number(lidar_summary['median_gap_s'], 4)}, "
        f"p95_gap_s={_format_optional_number(lidar_summary['p95_gap_s'], 4)}, "
        f"max_gap_s={_format_optional_number(lidar_summary['max_gap_s'], 4)}, "
        f"gap_over={lidar_summary['gap_over_threshold_count']}",
        "[CTRL] "
        f"requests={rtt_summary['total_requests']}, "
        f"failed={rtt_summary['failed_requests']}, "
        f"avg_rtt_ms={_format_optional_number(rtt_summary['avg_rtt_ms'], 2)}, "
        f"median_rtt_ms={_format_optional_number(rtt_summary['median_rtt_ms'], 2)}, "
        f"p95_rtt_ms={_format_optional_number(rtt_summary['p95_rtt_ms'], 2)}, "
        f"max_rtt_ms={_format_optional_number(rtt_summary['max_rtt_ms'], 2)}",
        "[STATE] "
        f"low_state_frames={low_state_count}, "
        f"lf_sport_mod_state_frames={lf_sport_state_count}",
    ]

    if lidar_summary["total_frames"] == 0:
        lines.append("[RESULT] LiDAR 不可用: 并发控制测试期间没有收到任何 LiDAR 帧")
    elif lidar_summary["ok_frames"] == 0:
        lines.append("[RESULT] LiDAR 不可用: 并发控制测试期间 LiDAR 帧解码全部失败")
    else:
        lines.append("[RESULT] LiDAR 可回传: 已在控制并发条件下收到可解码点云帧")

    return lines


def _print_and_save_summary(
    lidar_metrics: LidarStreamMetrics,
    rtt_metrics: RequestRttMetrics,
    low_state_count: int,
    lf_sport_state_count: int,
    gap_threshold_s: float,
    output_path: str,
) -> None:
    summary_text = "\n".join(
        _build_summary_lines(
            lidar_metrics,
            rtt_metrics,
            low_state_count,
            lf_sport_state_count,
            gap_threshold_s,
        )
    )
    print(f"\n{summary_text}")

    output_file = Path(output_path)
    output_file.write_text(f"{summary_text}\n", encoding="utf-8")
    print(f"[INFO] summary saved to {output_file}")


async def _run_mixed_benchmark(args: argparse.Namespace) -> tuple[
    LidarStreamMetrics,
    RequestRttMetrics,
    int,
    int,
]:
    conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalAP)
    lidar_metrics = LidarStreamMetrics()
    rtt_metrics = RequestRttMetrics()
    stop_event = asyncio.Event()
    low_state_count = 0
    lf_sport_state_count = 0

    def lidar_callback(message: dict[str, Any]) -> None:
        arrival_monotonic_s = time.monotonic()
        try:
            decoded_ok, point_count, error = _extract_point_count(message)
        except Exception as exc:  # noqa: BLE001
            lidar_metrics.add_frame(arrival_monotonic_s, False, None, repr(exc))
            return
        lidar_metrics.add_frame(arrival_monotonic_s, decoded_ok, point_count, error)

    def low_state_callback(_: dict[str, Any]) -> None:
        nonlocal low_state_count
        low_state_count += 1

    def lf_sport_state_callback(_: dict[str, Any]) -> None:
        nonlocal lf_sport_state_count
        lf_sport_state_count += 1

    control_task: Optional[asyncio.Task[None]] = None
    try:
        print("[INFO] connecting LocalAP WebRTC...")
        await asyncio.wait_for(conn.connect(), timeout=args.connect_timeout)
        print("[OK] WebRTC connected")

        await asyncio.wait_for(
            conn.datachannel.disableTrafficSaving(True),
            timeout=args.request_timeout,
        )
        conn.datachannel.set_decoder(decoder_type=args.decoder)
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["ULIDAR_ARRAY"], lidar_callback)
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], low_state_callback)
        conn.datachannel.pub_sub.subscribe(
            RTC_TOPIC["LF_SPORT_MOD_STATE"],
            lf_sport_state_callback,
        )
        conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["ULIDAR_SWITCH"], "on")

        if abs(args.yaw_pulse) > 0.0:
            print(
                "[WARN] yaw pulse enabled: "
                f"z={args.yaw_pulse}, duration={args.yaw_duration}s"
            )
        else:
            print("[INFO] control mode: periodic StopMove only")

        control_task = asyncio.create_task(
            _control_loop(conn, args, stop_event, rtt_metrics)
        )
        await asyncio.sleep(args.duration)
    finally:
        stop_event.set()
        if control_task is not None:
            try:
                await control_task
            except Exception:
                pass

        try:
            await _publish_sport_request(
                conn,
                SPORT_CMD["StopMove"],
                None,
                args.request_timeout,
            )
        except Exception:
            pass

        try:
            conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["ULIDAR_SWITCH"], "off")
        except Exception:
            pass

        await conn.disconnect()

    return lidar_metrics, rtt_metrics, low_state_count, lf_sport_state_count


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Benchmark Go2-W LiDAR under concurrent control traffic over LocalAP."
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--gap-threshold", type=float, default=0.5)
    parser.add_argument("--control-interval", type=float, default=1.0)
    parser.add_argument("--yaw-pulse", type=float, default=0.0)
    parser.add_argument("--yaw-duration", type=float, default=0.15)
    parser.add_argument("--output", type=str, default="benchmark_mixed_load_output.txt")
    parser.add_argument("--decoder", choices=["libvoxel", "native"], default="libvoxel")
    parser.add_argument("--connect-timeout", type=float, default=20.0)
    parser.add_argument("--request-timeout", type=float, default=5.0)
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    if abs(args.yaw_pulse) > 0.4:
        print("[ERROR] --yaw-pulse must be within [-0.4, 0.4] for safety")
        return 2
    if args.control_interval <= 0:
        print("[ERROR] --control-interval must be > 0")
        return 2
    if args.duration <= 0:
        print("[ERROR] --duration must be > 0")
        return 2

    try:
        lidar_metrics, rtt_metrics, low_state_count, lf_sport_state_count = asyncio.run(
            _run_mixed_benchmark(args)
        )
    except KeyboardInterrupt:
        print("\n[INFO] interrupted by user")
        return 130

    _print_and_save_summary(
        lidar_metrics,
        rtt_metrics,
        low_state_count,
        lf_sport_state_count,
        args.gap_threshold,
        args.output,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
