"""
Block Telemetry Module.

Records actual vs predicted metrics per G-code block for model accuracy
monitoring and drift detection.
"""

import csv
import time
from collections import deque
from dataclasses import dataclass, fields
from typing import List, Optional


@dataclass
class BlockTelemetryRecord:
    """A single predicted-vs-actual comparison for one G-code block."""
    block_index: int
    gcode_line: str
    timestamp: float
    predicted_force_n: float
    actual_force_n: float
    predicted_temp_c: float
    actual_temp_c: float
    predicted_power_w: float
    actual_power_w: float
    predicted_wear_mm: float
    actual_wear_mm: float  # estimated from force signature
    force_error_pct: float  # (actual - predicted) / predicted * 100
    temp_error_pct: float
    power_error_pct: float


@dataclass
class DriftReport:
    """Summary of model drift over a sliding window of blocks."""
    window_size: int
    mean_force_error_pct: float
    mean_temp_error_pct: float
    mean_power_error_pct: float
    force_drift_trend: float  # slope of error over time (positive = model underpredicts)
    temp_drift_trend: float
    is_drifting: bool  # True if any trend exceeds threshold
    recalibration_suggested: bool
    blocks_since_calibration: int


def _error_pct(actual: float, predicted: float) -> float:
    """Compute signed error percentage; returns 0.0 when predicted is zero."""
    if abs(predicted) < 1e-12:
        return 0.0
    return (actual - predicted) / predicted * 100.0


def _linear_slope(values: List[float]) -> float:
    """Compute the slope of a simple linear regression (y on index).

    The slope is normalised to "per 100 blocks" to make thresholds
    independent of window size.  Returns 0.0 for fewer than 2 points.
    """
    n = len(values)
    if n < 2:
        return 0.0
    mean_x = (n - 1) / 2.0
    mean_y = sum(values) / n
    num = 0.0
    den = 0.0
    for i, y in enumerate(values):
        dx = i - mean_x
        num += dx * (y - mean_y)
        den += dx * dx
    if abs(den) < 1e-12:
        return 0.0
    slope_per_block = num / den
    return slope_per_block * 100.0  # normalise to per-100-blocks


class BlockTelemetryTracker:
    """Tracks actual vs predicted per-block metrics for model accuracy monitoring."""

    def __init__(self, max_history: int = 500, drift_threshold: float = 10.0):
        """
        Args:
            max_history: Maximum number of records to retain (bounded deque).
            drift_threshold: Threshold on |trend slope| (per 100 blocks) to
                flag drift.
        """
        self._max_history = max_history
        self._drift_threshold = drift_threshold
        self._records: deque[BlockTelemetryRecord] = deque(maxlen=max_history)
        self._blocks_since_calibration: int = 0
        self._consecutive_drifting: int = 0

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------

    def record(
        self,
        block_index: int,
        gcode_line: str,
        predicted: dict,
        actual: dict,
    ) -> BlockTelemetryRecord:
        """Record a predicted vs actual comparison for a block.

        Args:
            block_index: Sequential index of the block in the program.
            gcode_line: Raw G-code text of the block.
            predicted: Dict with keys ``force_n``, ``temp_c``, ``power_w``,
                ``wear_mm``.
            actual: Dict with the same keys containing measured values.

        Returns:
            The created ``BlockTelemetryRecord``.
        """
        rec = BlockTelemetryRecord(
            block_index=block_index,
            gcode_line=gcode_line,
            timestamp=time.time(),
            predicted_force_n=predicted.get('force_n', 0.0),
            actual_force_n=actual.get('force_n', 0.0),
            predicted_temp_c=predicted.get('temp_c', 0.0),
            actual_temp_c=actual.get('temp_c', 0.0),
            predicted_power_w=predicted.get('power_w', 0.0),
            actual_power_w=actual.get('power_w', 0.0),
            predicted_wear_mm=predicted.get('wear_mm', 0.0),
            actual_wear_mm=actual.get('wear_mm', 0.0),
            force_error_pct=_error_pct(
                actual.get('force_n', 0.0), predicted.get('force_n', 0.0)
            ),
            temp_error_pct=_error_pct(
                actual.get('temp_c', 0.0), predicted.get('temp_c', 0.0)
            ),
            power_error_pct=_error_pct(
                actual.get('power_w', 0.0), predicted.get('power_w', 0.0)
            ),
        )
        self._records.append(rec)
        self._blocks_since_calibration += 1
        return rec

    # ------------------------------------------------------------------
    # Drift analysis
    # ------------------------------------------------------------------

    def get_drift_report(self, window: int = 50) -> DriftReport:
        """Compute drift statistics over the last *window* blocks.

        Args:
            window: Number of most-recent records to analyse.

        Returns:
            A ``DriftReport`` summarising current model accuracy.
        """
        recent = list(self._records)[-window:] if self._records else []

        if not recent:
            return DriftReport(
                window_size=0,
                mean_force_error_pct=0.0,
                mean_temp_error_pct=0.0,
                mean_power_error_pct=0.0,
                force_drift_trend=0.0,
                temp_drift_trend=0.0,
                is_drifting=False,
                recalibration_suggested=False,
                blocks_since_calibration=self._blocks_since_calibration,
            )

        n = len(recent)
        mean_force = sum(r.force_error_pct for r in recent) / n
        mean_temp = sum(r.temp_error_pct for r in recent) / n
        mean_power = sum(r.power_error_pct for r in recent) / n

        force_trend = _linear_slope([r.force_error_pct for r in recent])
        temp_trend = _linear_slope([r.temp_error_pct for r in recent])

        is_drifting = (
            abs(force_trend) > self._drift_threshold
            or abs(temp_trend) > self._drift_threshold
        )

        if is_drifting:
            self._consecutive_drifting += 1
        else:
            self._consecutive_drifting = 0

        recalibration_suggested = self._consecutive_drifting > 50

        return DriftReport(
            window_size=n,
            mean_force_error_pct=mean_force,
            mean_temp_error_pct=mean_temp,
            mean_power_error_pct=mean_power,
            force_drift_trend=force_trend,
            temp_drift_trend=temp_trend,
            is_drifting=is_drifting,
            recalibration_suggested=recalibration_suggested,
            blocks_since_calibration=self._blocks_since_calibration,
        )

    # ------------------------------------------------------------------
    # History queries
    # ------------------------------------------------------------------

    def get_block_history(self, last_n: int = 100) -> List[BlockTelemetryRecord]:
        """Return the most recent *last_n* records."""
        records = list(self._records)
        return records[-last_n:]

    def get_worst_blocks(
        self, metric: str = 'force', top_n: int = 5
    ) -> List[BlockTelemetryRecord]:
        """Return blocks with the largest prediction error.

        Args:
            metric: One of ``'force'``, ``'temp'``, ``'power'``.
            top_n: How many blocks to return.

        Returns:
            List of records sorted by descending absolute error percentage.
        """
        key_map = {
            'force': 'force_error_pct',
            'temp': 'temp_error_pct',
            'power': 'power_error_pct',
        }
        attr = key_map.get(metric, 'force_error_pct')
        sorted_recs = sorted(
            self._records, key=lambda r: abs(getattr(r, attr)), reverse=True
        )
        return sorted_recs[:top_n]

    # ------------------------------------------------------------------
    # Management
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Clear all telemetry history and reset counters."""
        self._records.clear()
        self._blocks_since_calibration = 0
        self._consecutive_drifting = 0

    def export_csv(self, path: str) -> None:
        """Export telemetry history to a CSV file.

        Args:
            path: Filesystem path for the output CSV.
        """
        field_names = [f.name for f in fields(BlockTelemetryRecord)]
        with open(path, 'w', newline='') as fh:
            writer = csv.writer(fh)
            writer.writerow(field_names)
            for rec in self._records:
                writer.writerow([getattr(rec, fn) for fn in field_names])
