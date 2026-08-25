"""CSV logging of motion samples and run summaries under data/logs/."""

import csv
import os
from typing import Optional, Sequence

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_LOG_DIR = os.path.join(PROJECT_ROOT, "data", "logs")


class CSVLogger:
    """Writes per-sample motion rows to a CSV file under data/logs/."""

    def __init__(self, log_dir: Optional[str] = None):
        self.log_dir = log_dir or DEFAULT_LOG_DIR
        os.makedirs(self.log_dir, exist_ok=True)
        self._file = None
        self._writer = None
        self._num_joints = None
        self.path = None

    def start_log(self, filename: str, num_joints: int = 5) -> str:
        """Open filename under log_dir and write the CSV header.

        Returns the full path of the opened log file.
        """
        if not filename.endswith(".csv"):
            filename += ".csv"
        self.path = os.path.join(self.log_dir, filename)
        self._num_joints = num_joints

        self._file = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)

        header = ["timestamp"]
        header += [f"target_j{i + 1}" for i in range(num_joints)]
        header += [f"actual_j{i + 1}" for i in range(num_joints)]
        header += [f"error_j{i + 1}" for i in range(num_joints)]
        self._writer.writerow(header)
        self._file.flush()
        return self.path

    def write_motion_sample(
        self,
        timestamp: float,
        target_deg: Sequence[float],
        actual_deg: Sequence[float],
        error_deg: Sequence[float],
        extra: Optional[dict] = None,
    ) -> None:
        """Append one motion sample row to the open log."""
        if self._writer is None:
            raise RuntimeError("start_log() must be called before write_motion_sample()")

        row = [timestamp]
        row += list(target_deg)
        row += list(actual_deg)
        row += list(error_deg)
        if extra:
            row += list(extra.values())
        self._writer.writerow(row)
        self._file.flush()

    def write_summary(self, summary_dict: dict) -> None:
        """Write a summary CSV alongside the motion log (same basename + _summary)."""
        if self.path is None:
            raise RuntimeError("start_log() must be called before write_summary()")

        summary_path = self.path[: -len(".csv")] + "_summary.csv"
        with open(summary_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["key", "value"])
            for key, value in summary_dict.items():
                writer.writerow([key, value])

    def close(self) -> None:
        """Close the open log file, if any."""
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
