"""
Block Telemetry Module.

Records actual vs predicted metrics per G-code block for model accuracy
monitoring and drift detection.
"""

import csv
import time
from collections import deque
from dataclasses import dataclass, fields
from typing import List, Optional, Tuple


@dataclass
class CalibrationTrigger:
    """Result of analyzing telemetry for auto-calibration."""
    triggered: bool
    reason: str  # "force_drift", "temp_drift", "power_drift", "cumulative_error", "multi_metric"
    force_correction: float  # multiplier, e.g. 1.05 means increase predicted force by 5%
    temp_correction: float
    power_correction: float
    confidence: float  # 0.0-1.0 based on data quality
    blocks_analyzed: int
    drift_magnitude: float = 0.0  # how far off predictions are (max bias across metrics)
    suggested_adjustments: Optional[dict] = None  # e.g. {"force_scale": 1.05, ...}
    blocks_since_last_calibration: int = 0


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
        self._calibration_triggers: deque[CalibrationTrigger] = deque(maxlen=10)

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
    # Auto-calibration
    # ------------------------------------------------------------------

    @staticmethod
    def _compute_ratio_stats(
        records: List[BlockTelemetryRecord],
        actual_attr: str,
        predicted_attr: str,
    ) -> Tuple[float, float, float]:
        """Compute mean(actual/predicted), mean absolute deviation, and R² of error trend.

        Returns (mean_ratio, mad, r_squared) considering only records where
        the predicted value is non-negligible.
        """
        ratios: List[float] = []
        for r in records:
            pred = getattr(r, predicted_attr)
            act = getattr(r, actual_attr)
            if abs(pred) < 1e-12:
                continue
            ratios.append(act / pred)

        if not ratios:
            return 1.0, 0.0, 0.0

        n = len(ratios)
        mean_ratio = sum(ratios) / n
        mad = sum(abs(v - mean_ratio) for v in ratios) / n

        # R² of error trend (ratio vs index) — high R² means consistent trend
        if n < 2:
            return mean_ratio, mad, 0.0

        mean_x = (n - 1) / 2.0
        mean_y = mean_ratio  # already computed
        ss_tot = sum((v - mean_y) ** 2 for v in ratios)
        if ss_tot < 1e-12:
            # All ratios identical — perfectly consistent, R² = 1
            return mean_ratio, mad, 1.0

        num = 0.0
        den = 0.0
        for i, v in enumerate(ratios):
            dx = i - mean_x
            num += dx * (v - mean_y)
            den += dx * dx
        if abs(den) < 1e-12:
            return mean_ratio, mad, 0.0
        slope = num / den
        # Predicted values from linear fit
        ss_res = sum((ratios[i] - (mean_y + slope * (i - mean_x))) ** 2
                     for i in range(n))
        r_squared = max(0.0, 1.0 - ss_res / ss_tot)
        return mean_ratio, mad, r_squared

    def compute_calibration_trigger(self, min_blocks: int = 30) -> CalibrationTrigger:
        """Analyze recent telemetry and compute calibration corrections.

        If predictions consistently under/overpredict, compute a correction factor:
        correction = mean(actual/predicted) over the last min_blocks records.

        Only triggers if:
        - At least min_blocks records exist
        - Mean error exceeds 5% for any metric
        - Error trend is consistent (not oscillating)
        - Confidence > 0.6 (based on data quality)

        Args:
            min_blocks: Minimum number of records required to compute a trigger.

        Returns:
            A ``CalibrationTrigger`` with correction factors and metadata.
        """
        recent = list(self._records)[-min_blocks:]
        n = len(recent)

        no_trigger = CalibrationTrigger(
            triggered=False,
            reason="insufficient_data",
            force_correction=1.0,
            temp_correction=1.0,
            power_correction=1.0,
            confidence=0.0,
            blocks_analyzed=n,
            drift_magnitude=0.0,
            suggested_adjustments={'force_scale': 1.0, 'temp_scale': 1.0, 'power_scale': 1.0},
            blocks_since_last_calibration=self._blocks_since_calibration,
        )

        if n < min_blocks:
            return no_trigger

        # Compute ratio stats for each metric
        force_ratio, force_mad, force_r2 = self._compute_ratio_stats(
            recent, 'actual_force_n', 'predicted_force_n')
        temp_ratio, temp_mad, temp_r2 = self._compute_ratio_stats(
            recent, 'actual_temp_c', 'predicted_temp_c')
        power_ratio, power_mad, power_r2 = self._compute_ratio_stats(
            recent, 'actual_power_w', 'predicted_power_w')

        # Clamp corrections to [0.8, 1.2]
        force_correction = max(0.8, min(1.2, force_ratio))
        temp_correction = max(0.8, min(1.2, temp_ratio))
        power_correction = max(0.8, min(1.2, power_ratio))

        # Determine which metrics have >5% bias
        force_bias = abs(force_ratio - 1.0)
        temp_bias = abs(temp_ratio - 1.0)
        power_bias = abs(power_ratio - 1.0)

        drifting_metrics = []
        if force_bias > 0.05:
            drifting_metrics.append(('force_drift', force_bias))
        if temp_bias > 0.05:
            drifting_metrics.append(('temp_drift', temp_bias))
        if power_bias > 0.05:
            drifting_metrics.append(('power_drift', power_bias))

        if not drifting_metrics:
            return no_trigger

        # Determine reason
        if len(drifting_metrics) > 1:
            reason = "multi_metric"
        else:
            reason = drifting_metrics[0][0]

        # Confidence = 1.0 - mad/|mean_error| for the dominant metric, clamped [0, 1]
        # Use the metric with largest bias for confidence calculation
        dominant = max(drifting_metrics, key=lambda x: x[1])
        if dominant[0] == 'force_drift':
            mean_err = force_bias
            mad = force_mad
        elif dominant[0] == 'temp_drift':
            mean_err = temp_bias
            mad = temp_mad
        else:
            mean_err = power_bias
            mad = power_mad

        if mean_err < 1e-12:
            confidence = 0.0
        else:
            confidence = max(0.0, min(1.0, 1.0 - mad / mean_err))

        drift_magnitude = max(force_bias, temp_bias, power_bias)
        adjustments = self.compute_suggested_adjustments(window=n)

        if confidence <= 0.6:
            no_trigger_low_conf = CalibrationTrigger(
                triggered=False,
                reason=reason,
                force_correction=force_correction,
                temp_correction=temp_correction,
                power_correction=power_correction,
                confidence=confidence,
                blocks_analyzed=n,
                drift_magnitude=drift_magnitude,
                suggested_adjustments=adjustments,
                blocks_since_last_calibration=self._blocks_since_calibration,
            )
            return no_trigger_low_conf

        trigger = CalibrationTrigger(
            triggered=True,
            reason=reason,
            force_correction=force_correction,
            temp_correction=temp_correction,
            power_correction=power_correction,
            confidence=confidence,
            blocks_analyzed=n,
            drift_magnitude=drift_magnitude,
            suggested_adjustments=adjustments,
            blocks_since_last_calibration=self._blocks_since_calibration,
        )
        self._calibration_triggers.append(trigger)
        return trigger

    def check_calibration_needed(self, min_blocks: int = 30) -> CalibrationTrigger:
        """Evaluate whether model recalibration should be triggered.

        Triggers if any of the following conditions are met:
        - Force drift > 8% sustained for 30+ blocks
        - Temp drift > 10% sustained for 30+ blocks
        - Cumulative absolute error exceeds threshold
        - blocks_since_calibration > 200 (periodic recalibration)

        Args:
            min_blocks: Minimum number of recent records to consider.

        Returns:
            A ``CalibrationTrigger`` describing whether calibration is needed.
        """
        recent = list(self._records)[-max(min_blocks, 50):]
        n = len(recent)

        no_trigger = CalibrationTrigger(
            triggered=False,
            reason="no_drift",
            force_correction=1.0,
            temp_correction=1.0,
            power_correction=1.0,
            confidence=0.0,
            blocks_analyzed=n,
            drift_magnitude=0.0,
            suggested_adjustments={'force_scale': 1.0, 'temp_scale': 1.0, 'power_scale': 1.0},
            blocks_since_last_calibration=self._blocks_since_calibration,
        )

        if n < min_blocks:
            no_trigger.reason = "insufficient_data"
            return no_trigger

        # Compute ratio stats
        force_ratio, force_mad, _ = self._compute_ratio_stats(
            recent, 'actual_force_n', 'predicted_force_n')
        temp_ratio, temp_mad, _ = self._compute_ratio_stats(
            recent, 'actual_temp_c', 'predicted_temp_c')
        power_ratio, power_mad, _ = self._compute_ratio_stats(
            recent, 'actual_power_w', 'predicted_power_w')

        force_bias = abs(force_ratio - 1.0)
        temp_bias = abs(temp_ratio - 1.0)
        power_bias = abs(power_ratio - 1.0)

        adjustments = self.compute_suggested_adjustments(window=len(recent))

        # Check specific drift thresholds
        reasons: List[str] = []
        if force_bias > 0.08 and n >= 30:
            reasons.append("force_drift")
        if temp_bias > 0.10 and n >= 30:
            reasons.append("temp_drift")
        if power_bias > 0.08 and n >= 30:
            reasons.append("power_drift")

        # Cumulative absolute error check
        cum_force_err = sum(abs(r.force_error_pct) for r in recent)
        cum_temp_err = sum(abs(r.temp_error_pct) for r in recent)
        cum_power_err = sum(abs(r.power_error_pct) for r in recent)
        # Threshold: average absolute error > 6% across all blocks
        cum_threshold = n * 6.0
        if (cum_force_err > cum_threshold or cum_temp_err > cum_threshold
                or cum_power_err > cum_threshold):
            if "cumulative_error" not in reasons:
                reasons.append("cumulative_error")

        # Periodic trigger at 200 blocks since last calibration
        if self._blocks_since_calibration > 200:
            if not reasons:
                reasons.append("periodic")

        if not reasons:
            return no_trigger

        # Determine reason string
        if len(reasons) > 1:
            reason = "multi_metric" if "periodic" not in reasons else reasons[0]
            if len([r for r in reasons if r != "periodic"]) > 1:
                reason = "multi_metric"
        else:
            reason = reasons[0]

        drift_magnitude = max(force_bias, temp_bias, power_bias)

        # Confidence calculation
        dominant_mad = force_mad
        dominant_bias = force_bias
        if temp_bias > force_bias and temp_bias > power_bias:
            dominant_mad = temp_mad
            dominant_bias = temp_bias
        elif power_bias > force_bias:
            dominant_mad = power_mad
            dominant_bias = power_bias

        if dominant_bias < 1e-12:
            confidence = 0.0
        else:
            confidence = max(0.0, min(1.0, 1.0 - dominant_mad / dominant_bias))

        # Clamp corrections
        force_correction = max(0.8, min(1.2, force_ratio))
        temp_correction = max(0.8, min(1.2, temp_ratio))
        power_correction = max(0.8, min(1.2, power_ratio))

        trigger = CalibrationTrigger(
            triggered=True,
            reason=reason,
            force_correction=force_correction,
            temp_correction=temp_correction,
            power_correction=power_correction,
            confidence=confidence,
            blocks_analyzed=n,
            drift_magnitude=drift_magnitude,
            suggested_adjustments=adjustments,
            blocks_since_last_calibration=self._blocks_since_calibration,
        )
        self._calibration_triggers.append(trigger)
        return trigger

    def compute_suggested_adjustments(self, window: int = 50) -> dict:
        """Compute scaling factors to correct systematic bias.

        Returns dict like::

            {
                'force_scale': 1.05,  # multiply predicted force by this
                'temp_scale': 0.97,   # multiply predicted temp by this
                'power_scale': 1.02,  # multiply predicted power by this
            }

        Simple approach: scale = mean(actual/predicted) over the window,
        clamped to [0.8, 1.2].

        Args:
            window: Number of most-recent records to use.

        Returns:
            Dictionary with per-metric scaling factors.
        """
        recent = list(self._records)[-window:] if self._records else []
        if not recent:
            return {'force_scale': 1.0, 'temp_scale': 1.0, 'power_scale': 1.0}

        force_ratio, _, _ = self._compute_ratio_stats(
            recent, 'actual_force_n', 'predicted_force_n')
        temp_ratio, _, _ = self._compute_ratio_stats(
            recent, 'actual_temp_c', 'predicted_temp_c')
        power_ratio, _, _ = self._compute_ratio_stats(
            recent, 'actual_power_w', 'predicted_power_w')

        return {
            'force_scale': max(0.8, min(1.2, force_ratio)),
            'temp_scale': max(0.8, min(1.2, temp_ratio)),
            'power_scale': max(0.8, min(1.2, power_ratio)),
        }

    def mark_calibrated(self) -> None:
        """Record that calibration was performed, reset drift counters."""
        self._blocks_since_calibration = 0
        self._consecutive_drifting = 0

    def get_calibration_history(self) -> List[CalibrationTrigger]:
        """Return history of calibration triggers."""
        return list(self._calibration_triggers)

    # ------------------------------------------------------------------
    # Management
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Clear all telemetry history and reset counters."""
        self._records.clear()
        self._blocks_since_calibration = 0
        self._consecutive_drifting = 0
        self._calibration_triggers.clear()

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
