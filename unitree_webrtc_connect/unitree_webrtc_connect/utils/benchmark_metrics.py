from __future__ import annotations

import csv
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


def _percentile(sorted_values: list[float], percentile: float) -> Optional[float]:
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]

    rank = (len(sorted_values) - 1) * percentile / 100.0
    lower = int(rank)
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = rank - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


@dataclass
class LidarFrameRecord:
    arrival_monotonic_s: float
    decoded_ok: bool
    point_count: Optional[int] = None
    error: str = ""


@dataclass
class LidarStreamMetrics:
    records: list[LidarFrameRecord] = field(default_factory=list)

    def add_frame(
        self,
        arrival_monotonic_s: float,
        decoded_ok: bool,
        point_count: Optional[int] = None,
        error: str = "",
    ) -> None:
        self.records.append(
            LidarFrameRecord(
                arrival_monotonic_s=arrival_monotonic_s,
                decoded_ok=decoded_ok,
                point_count=point_count,
                error=error,
            )
        )

    def _intervals_s(self) -> list[float]:
        if len(self.records) < 2:
            return []
        return [
            current.arrival_monotonic_s - previous.arrival_monotonic_s
            for previous, current in zip(self.records[:-1], self.records[1:])
        ]

    def _point_counts(self) -> list[int]:
        return [
            record.point_count
            for record in self.records
            if record.decoded_ok and record.point_count is not None
        ]

    def summarize(self, gap_threshold_s: float) -> dict[str, Optional[float] | int]:
        intervals_s = self._intervals_s()
        sorted_intervals_s = sorted(intervals_s)
        point_counts = self._point_counts()

        total_frames = len(self.records)
        ok_frames = sum(1 for record in self.records if record.decoded_ok)
        failed_frames = total_frames - ok_frames

        if len(self.records) >= 2:
            duration_s = self.records[-1].arrival_monotonic_s - self.records[0].arrival_monotonic_s
            avg_hz = (len(self.records) - 1) / duration_s if duration_s > 0 else None
        else:
            avg_hz = None

        return {
            "total_frames": total_frames,
            "ok_frames": ok_frames,
            "failed_frames": failed_frames,
            "avg_hz": avg_hz,
            "median_gap_s": statistics.median(intervals_s) if intervals_s else None,
            "p95_gap_s": _percentile(sorted_intervals_s, 95.0),
            "max_gap_s": max(intervals_s) if intervals_s else None,
            "gap_over_threshold_count": sum(
                1 for interval_s in intervals_s if interval_s > gap_threshold_s
            ),
            "avg_point_count": statistics.fmean(point_counts) if point_counts else None,
            "median_point_count": statistics.median(point_counts) if point_counts else None,
        }

    def write_csv(self, csv_path: str | Path) -> None:
        output_path = Path(csv_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        intervals_s = [None, *self._intervals_s()]
        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "frame_index",
                    "arrival_monotonic_s",
                    "delta_s",
                    "decoded_ok",
                    "point_count",
                    "error",
                ]
            )
            for frame_index, (record, delta_s) in enumerate(zip(self.records, intervals_s)):
                writer.writerow(
                    [
                        frame_index,
                        f"{record.arrival_monotonic_s:.9f}",
                        "" if delta_s is None else f"{delta_s:.9f}",
                        int(record.decoded_ok),
                        "" if record.point_count is None else record.point_count,
                        record.error,
                    ]
                )


@dataclass
class RequestRttRecord:
    monotonic_s: float
    rtt_s: float
    ok: bool
    command_name: str
    error: str = ""


@dataclass
class RequestRttMetrics:
    records: list[RequestRttRecord] = field(default_factory=list)

    def add_request(
        self,
        monotonic_s: float,
        rtt_s: float,
        ok: bool,
        command_name: str,
        error: str = "",
    ) -> None:
        self.records.append(
            RequestRttRecord(
                monotonic_s=monotonic_s,
                rtt_s=rtt_s,
                ok=ok,
                command_name=command_name,
                error=error,
            )
        )

    def summarize(self) -> dict[str, Optional[float] | int]:
        success_rtts_s = [record.rtt_s for record in self.records if record.ok]
        sorted_success_rtts_s = sorted(success_rtts_s)
        total_requests = len(self.records)
        failed_requests = sum(1 for record in self.records if not record.ok)

        return {
            "total_requests": total_requests,
            "failed_requests": failed_requests,
            "avg_rtt_ms": (
                statistics.fmean(success_rtts_s) * 1000.0
                if success_rtts_s
                else None
            ),
            "median_rtt_ms": (
                statistics.median(success_rtts_s) * 1000.0
                if success_rtts_s
                else None
            ),
            "p95_rtt_ms": (
                _percentile(sorted_success_rtts_s, 95.0) * 1000.0
                if success_rtts_s
                else None
            ),
            "max_rtt_ms": (
                max(success_rtts_s) * 1000.0
                if success_rtts_s
                else None
            ),
        }
