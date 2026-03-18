"""Tests for SpindleBearingMonitor logic (Python mirror of C# implementation).

Validates spindle bearing health monitoring through vibration and temperature
analysis, including health scoring, temperature trend detection, vibration
spectrum analysis, remaining life prediction, and recommendation generation.
"""

import math
import pytest
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from datetime import datetime, timezone


# ── Python mirror of C# data classes ─────────────────────────────────

@dataclass
class BearingReading:
    timestamp: str = ""
    temperature: float = 0.0       # degC
    vibrationRms: float = 0.0      # mm/s
    vibrationPeak: float = 0.0     # mm/s
    axialLoad: float = 0.0         # N
    radialLoad: float = 0.0        # N


@dataclass
class BearingHealthReport:
    overallHealth: float = 0.0               # 0-100
    temperatureStatus: str = "normal"        # normal | elevated | critical
    vibrationStatus: str = "normal"          # normal | elevated | critical
    lubricationStatus: str = "good"          # good | marginal | poor
    estimatedRemainingHours: float = 0.0
    recommendations: List[str] = field(default_factory=list)


@dataclass
class TemperatureTrend:
    average: float = 0.0
    slope: float = 0.0
    isRising: bool = False


@dataclass
class VibrationSpectrum:
    rmsAvg: float = 0.0
    peakAvg: float = 0.0
    crestFactor: float = 0.0


# ── SpindleBearingMonitor (Python mirror) ────────────────────────────

class SpindleBearingMonitor:
    """Monitors spindle bearing health through vibration and temperature analysis.

    Health scoring weights:
        Temperature  30%
        Vibration    40%
        Load         20%
        Trend        10%

    Thresholds:
        Temperature - normal < 50 degC, elevated < 70 degC, critical >= 70 degC
        Vibration   - normal < 2.5 mm/s, elevated < 5.0 mm/s, critical >= 5.0 mm/s
    """

    MAX_READINGS = 1000
    TREND_WINDOW = 50

    # Temperature thresholds (degC)
    TEMP_NORMAL_MAX = 50.0
    TEMP_ELEVATED_MAX = 70.0

    # Vibration RMS thresholds (mm/s)
    VIB_NORMAL_MAX = 2.5
    VIB_ELEVATED_MAX = 5.0

    # Scoring weights
    WEIGHT_TEMPERATURE = 0.30
    WEIGHT_VIBRATION = 0.40
    WEIGHT_LOAD = 0.20
    WEIGHT_TREND = 0.10

    # Rated loads for scoring
    RATED_AXIAL_LOAD = 5000.0   # N
    RATED_RADIAL_LOAD = 3000.0  # N

    def __init__(self):
        self._readings: List[BearingReading] = []

    @property
    def reading_count(self) -> int:
        return len(self._readings)

    # ── Recording ─────────────────────────────────────────────────

    def record_reading(self, reading: BearingReading) -> None:
        """Store a bearing reading. Buffer is capped at 1000 entries."""
        if reading is None:
            return
        self._readings.append(reading)
        if len(self._readings) > self.MAX_READINGS:
            self._readings.pop(0)

    # ── Health Report ─────────────────────────────────────────────

    def get_health_report(self) -> BearingHealthReport:
        """Analyse recent readings and produce a comprehensive health report."""
        report = BearingHealthReport()

        if len(self._readings) == 0:
            report.overallHealth = 100.0
            report.temperatureStatus = "normal"
            report.vibrationStatus = "normal"
            report.lubricationStatus = "good"
            report.estimatedRemainingHours = 0.0
            report.recommendations = ["No readings recorded yet."]
            return report

        # --- Temperature scoring (30%) ---
        avg_temp = sum(r.temperature for r in self._readings) / len(self._readings)
        temp_score = self._score_temperature(avg_temp)
        report.temperatureStatus = self._classify_temperature(avg_temp)

        # --- Vibration scoring (40%) ---
        spectrum = self.get_vibration_spectrum()
        vib_score = self._score_vibration(spectrum.rmsAvg)
        report.vibrationStatus = self._classify_vibration(spectrum.rmsAvg)

        # --- Load scoring (20%) ---
        avg_axial = sum(r.axialLoad for r in self._readings) / len(self._readings)
        avg_radial = sum(r.radialLoad for r in self._readings) / len(self._readings)
        load_score = self._score_load(avg_axial, avg_radial)

        # --- Trend scoring (10%) ---
        trend = self.get_temperature_trend()
        trend_score = self._score_trend(trend)

        # --- Overall health ---
        raw = (temp_score * self.WEIGHT_TEMPERATURE +
               vib_score * self.WEIGHT_VIBRATION +
               load_score * self.WEIGHT_LOAD +
               trend_score * self.WEIGHT_TREND)
        report.overallHealth = max(0.0, min(100.0, raw))

        # --- Lubrication status ---
        report.lubricationStatus = self._classify_lubrication(spectrum.crestFactor)

        # --- Remaining life estimate ---
        report.estimatedRemainingHours = self.predict_remaining_life(0.0, 20000.0)

        # --- Recommendations ---
        report.recommendations = []

        if report.temperatureStatus == "critical":
            report.recommendations.append(
                "Immediate inspection required: bearing temperature critical.")
        elif report.temperatureStatus == "elevated":
            report.recommendations.append(
                "Monitor bearing temperature closely; consider coolant check.")

        if report.vibrationStatus == "critical":
            report.recommendations.append(
                "Immediate inspection required: vibration levels critical.")
        elif report.vibrationStatus == "elevated":
            report.recommendations.append(
                "Schedule vibration analysis; check bearing pre-load.")

        if report.lubricationStatus == "poor":
            report.recommendations.append("Re-lubricate bearings immediately.")
        elif report.lubricationStatus == "marginal":
            report.recommendations.append(
                "Plan bearing lubrication at next maintenance window.")

        if trend.isRising and trend.slope > 0.5:
            report.recommendations.append(
                "Temperature trend is rising; investigate root cause.")

        if len(report.recommendations) == 0:
            report.recommendations.append(
                "Bearing health is within normal parameters.")

        return report

    # ── Temperature Trend ─────────────────────────────────────────

    def get_temperature_trend(self) -> TemperatureTrend:
        """Return (average, slope, isRising) from last 50 readings."""
        result = TemperatureTrend()

        if len(self._readings) == 0:
            return result

        window = (self._readings if len(self._readings) <= self.TREND_WINDOW
                  else self._readings[-self.TREND_WINDOW:])

        result.average = sum(r.temperature for r in window) / len(window)

        if len(window) >= 2:
            n = len(window)
            sum_x = 0.0
            sum_y = 0.0
            sum_xy = 0.0
            sum_x2 = 0.0
            for i, r in enumerate(window):
                x = float(i)
                y = r.temperature
                sum_x += x
                sum_y += y
                sum_xy += x * y
                sum_x2 += x * x
            denom = n * sum_x2 - sum_x * sum_x
            result.slope = (n * sum_xy - sum_x * sum_y) / denom if denom != 0 else 0.0

        result.isRising = result.slope > 0.0
        return result

    # ── Vibration Spectrum ────────────────────────────────────────

    def get_vibration_spectrum(self) -> VibrationSpectrum:
        """Return (rmsAvg, peakAvg, crestFactor) from recent readings."""
        result = VibrationSpectrum()

        if len(self._readings) == 0:
            return result

        result.rmsAvg = sum(r.vibrationRms for r in self._readings) / len(self._readings)
        result.peakAvg = sum(r.vibrationPeak for r in self._readings) / len(self._readings)
        result.crestFactor = (result.peakAvg / result.rmsAvg) if result.rmsAvg > 0 else 0.0

        return result

    # ── Remaining Life Prediction ─────────────────────────────────

    def predict_remaining_life(self, current_hours: float, max_hours: float) -> float:
        """Predict remaining bearing life based on health degradation rate."""
        if len(self._readings) == 0:
            return max_hours - current_hours

        avg_temp = sum(r.temperature for r in self._readings) / len(self._readings)
        avg_vib = sum(r.vibrationRms for r in self._readings) / len(self._readings)

        if avg_temp < self.TEMP_NORMAL_MAX:
            temp_factor = 1.0
        elif avg_temp < self.TEMP_ELEVATED_MAX:
            temp_factor = 0.7
        else:
            temp_factor = 0.3

        if avg_vib < self.VIB_NORMAL_MAX:
            vib_factor = 1.0
        elif avg_vib < self.VIB_ELEVATED_MAX:
            vib_factor = 0.6
        else:
            vib_factor = 0.2

        degradation_factor = (temp_factor + vib_factor) / 2.0
        remaining = (max_hours - current_hours) * degradation_factor
        return max(remaining, 0.0)

    # ── Scoring helpers ───────────────────────────────────────────

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * t

    def _score_temperature(self, avg_temp: float) -> float:
        if avg_temp < self.TEMP_NORMAL_MAX:
            return 100.0
        if avg_temp < self.TEMP_ELEVATED_MAX:
            t = (avg_temp - self.TEMP_NORMAL_MAX) / (self.TEMP_ELEVATED_MAX - self.TEMP_NORMAL_MAX)
            return self._lerp(100.0, 50.0, t)
        t = min(1.0, max(0.0, (avg_temp - self.TEMP_ELEVATED_MAX) / 30.0))
        return self._lerp(50.0, 0.0, t)

    def _score_vibration(self, rms_avg: float) -> float:
        if rms_avg < self.VIB_NORMAL_MAX:
            return 100.0
        if rms_avg < self.VIB_ELEVATED_MAX:
            t = (rms_avg - self.VIB_NORMAL_MAX) / (self.VIB_ELEVATED_MAX - self.VIB_NORMAL_MAX)
            return self._lerp(100.0, 50.0, t)
        t = min(1.0, max(0.0, (rms_avg - self.VIB_ELEVATED_MAX) / 5.0))
        return self._lerp(50.0, 0.0, t)

    def _score_load(self, axial: float, radial: float) -> float:
        axial_ratio = min(1.0, max(0.0, axial / self.RATED_AXIAL_LOAD))
        radial_ratio = min(1.0, max(0.0, radial / self.RATED_RADIAL_LOAD))
        load_ratio = max(axial_ratio, radial_ratio)
        return self._lerp(100.0, 0.0, load_ratio)

    def _score_trend(self, trend: TemperatureTrend) -> float:
        if not trend.isRising:
            return 100.0
        t = min(1.0, max(0.0, trend.slope))
        return self._lerp(100.0, 0.0, t)

    # ── Classification helpers ────────────────────────────────────

    def _classify_temperature(self, temp: float) -> str:
        if temp < self.TEMP_NORMAL_MAX:
            return "normal"
        if temp < self.TEMP_ELEVATED_MAX:
            return "elevated"
        return "critical"

    def _classify_vibration(self, rms: float) -> str:
        if rms < self.VIB_NORMAL_MAX:
            return "normal"
        if rms < self.VIB_ELEVATED_MAX:
            return "elevated"
        return "critical"

    @staticmethod
    def _classify_lubrication(crest_factor: float) -> str:
        if crest_factor < 3.0:
            return "good"
        if crest_factor < 5.0:
            return "marginal"
        return "poor"


# ── Helper to create readings quickly ─────────────────────────────────

def _make_reading(
    temp: float = 40.0,
    rms: float = 1.0,
    peak: float = 1.5,
    axial: float = 1000.0,
    radial: float = 500.0,
    ts: str = "",
) -> BearingReading:
    if not ts:
        ts = datetime.now(timezone.utc).isoformat()
    return BearingReading(
        timestamp=ts,
        temperature=temp,
        vibrationRms=rms,
        vibrationPeak=peak,
        axialLoad=axial,
        radialLoad=radial,
    )


# ── Fixtures ──────────────────────────────────────────────────────────

@pytest.fixture
def monitor():
    """Fresh SpindleBearingMonitor instance."""
    return SpindleBearingMonitor()


@pytest.fixture
def healthy_monitor(monitor):
    """Monitor pre-loaded with 100 healthy readings."""
    for _ in range(100):
        monitor.record_reading(_make_reading(temp=35.0, rms=1.0, peak=1.2,
                                              axial=800.0, radial=400.0))
    return monitor


@pytest.fixture
def critical_monitor(monitor):
    """Monitor pre-loaded with 100 critical readings."""
    for _ in range(100):
        monitor.record_reading(_make_reading(temp=80.0, rms=7.0, peak=42.0,
                                              axial=4500.0, radial=2800.0))
    return monitor


# ── Tests ─────────────────────────────────────────────────────────────

class TestRecordReading:
    """Tests for reading storage and buffer management."""

    def test_record_reading_increments_count(self, monitor):
        """Recording a reading increases the reading count."""
        assert monitor.reading_count == 0
        monitor.record_reading(_make_reading())
        assert monitor.reading_count == 1
        monitor.record_reading(_make_reading())
        assert monitor.reading_count == 2

    def test_buffer_capped_at_max_readings(self, monitor):
        """Buffer never exceeds MAX_READINGS (1000) entries."""
        for i in range(1100):
            monitor.record_reading(_make_reading(temp=20.0 + i * 0.01))
        assert monitor.reading_count == 1000


class TestHealthReport:
    """Tests for health report generation."""

    def test_empty_monitor_returns_full_health(self, monitor):
        """A monitor with no readings reports 100% health."""
        report = monitor.get_health_report()
        assert report.overallHealth == 100.0
        assert report.temperatureStatus == "normal"
        assert report.vibrationStatus == "normal"
        assert report.lubricationStatus == "good"
        assert "No readings recorded yet." in report.recommendations

    def test_healthy_readings_produce_high_health(self, healthy_monitor):
        """Normal operating conditions yield a high overall health score."""
        report = healthy_monitor.get_health_report()
        assert report.overallHealth > 80.0
        assert report.temperatureStatus == "normal"
        assert report.vibrationStatus == "normal"

    def test_critical_readings_produce_low_health(self, critical_monitor):
        """Critical temperature and vibration yield a low health score."""
        report = critical_monitor.get_health_report()
        assert report.overallHealth < 35.0
        assert report.temperatureStatus == "critical"
        assert report.vibrationStatus == "critical"

    def test_critical_report_has_recommendations(self, critical_monitor):
        """Critical conditions generate actionable recommendations."""
        report = critical_monitor.get_health_report()
        assert len(report.recommendations) > 0
        has_temp_rec = any("temperature" in r.lower() for r in report.recommendations)
        has_vib_rec = any("vibration" in r.lower() for r in report.recommendations)
        assert has_temp_rec, "Expected a temperature-related recommendation"
        assert has_vib_rec, "Expected a vibration-related recommendation"


class TestTemperatureTrend:
    """Tests for temperature trend analysis."""

    def test_empty_monitor_trend_is_zero(self, monitor):
        """No readings produces a zeroed trend."""
        trend = monitor.get_temperature_trend()
        assert trend.average == 0.0
        assert trend.slope == 0.0
        assert trend.isRising is False

    def test_rising_temperature_detected(self, monitor):
        """A steadily increasing temperature sequence is detected as rising."""
        for i in range(60):
            monitor.record_reading(_make_reading(temp=30.0 + i * 0.5))
        trend = monitor.get_temperature_trend()
        assert trend.isRising is True
        assert trend.slope > 0.0

    def test_stable_temperature_not_rising(self, monitor):
        """Constant temperature readings produce a near-zero, non-rising slope."""
        for _ in range(60):
            monitor.record_reading(_make_reading(temp=45.0))
        trend = monitor.get_temperature_trend()
        assert abs(trend.slope) < 1e-6
        assert trend.average == pytest.approx(45.0)


class TestVibrationSpectrum:
    """Tests for vibration spectrum analysis."""

    def test_empty_monitor_spectrum_is_zero(self, monitor):
        """No readings produces zeroed spectrum."""
        spectrum = monitor.get_vibration_spectrum()
        assert spectrum.rmsAvg == 0.0
        assert spectrum.peakAvg == 0.0
        assert spectrum.crestFactor == 0.0

    def test_crest_factor_calculation(self, monitor):
        """Crest factor is peakAvg / rmsAvg."""
        for _ in range(10):
            monitor.record_reading(_make_reading(rms=2.0, peak=6.0))
        spectrum = monitor.get_vibration_spectrum()
        assert spectrum.rmsAvg == pytest.approx(2.0)
        assert spectrum.peakAvg == pytest.approx(6.0)
        assert spectrum.crestFactor == pytest.approx(3.0)


class TestPredictRemainingLife:
    """Tests for bearing remaining life prediction."""

    def test_no_readings_returns_full_remaining(self, monitor):
        """With no sensor data, remaining life = maxHours - currentHours."""
        remaining = monitor.predict_remaining_life(1000.0, 20000.0)
        assert remaining == pytest.approx(19000.0)

    def test_healthy_conditions_high_remaining(self, healthy_monitor):
        """Healthy bearings retain most of their rated life."""
        remaining = healthy_monitor.predict_remaining_life(0.0, 20000.0)
        # temp < 50 -> factor 1.0, vib < 2.5 -> factor 1.0
        # degradation = (1.0 + 1.0)/2 = 1.0 -> remaining = 20000
        assert remaining == pytest.approx(20000.0)

    def test_critical_conditions_low_remaining(self, critical_monitor):
        """Critical bearings have severely reduced remaining life."""
        remaining = critical_monitor.predict_remaining_life(0.0, 20000.0)
        # temp >= 70 -> factor 0.3, vib >= 5.0 -> factor 0.2
        # degradation = (0.3 + 0.2)/2 = 0.25 -> remaining = 5000
        assert remaining == pytest.approx(5000.0)

    def test_remaining_life_never_negative(self, critical_monitor):
        """Remaining life is clamped to zero."""
        remaining = critical_monitor.predict_remaining_life(20000.0, 20000.0)
        assert remaining == 0.0


class TestClassification:
    """Tests for temperature, vibration, and lubrication classification."""

    def test_temperature_classification_boundaries(self, monitor):
        """Temperature is classified correctly at each boundary."""
        # normal < 50
        monitor.record_reading(_make_reading(temp=30.0))
        report = monitor.get_health_report()
        assert report.temperatureStatus == "normal"

    def test_elevated_temperature(self):
        """Temperature between 50 and 70 is elevated."""
        m = SpindleBearingMonitor()
        m.record_reading(_make_reading(temp=60.0))
        report = m.get_health_report()
        assert report.temperatureStatus == "elevated"

    def test_critical_temperature(self):
        """Temperature >= 70 is critical."""
        m = SpindleBearingMonitor()
        m.record_reading(_make_reading(temp=75.0))
        report = m.get_health_report()
        assert report.temperatureStatus == "critical"

    def test_vibration_classification(self):
        """Vibration RMS is classified at each threshold."""
        # normal
        m = SpindleBearingMonitor()
        m.record_reading(_make_reading(rms=1.0))
        assert m.get_health_report().vibrationStatus == "normal"

        # elevated
        m2 = SpindleBearingMonitor()
        m2.record_reading(_make_reading(rms=3.5))
        assert m2.get_health_report().vibrationStatus == "elevated"

        # critical
        m3 = SpindleBearingMonitor()
        m3.record_reading(_make_reading(rms=6.0))
        assert m3.get_health_report().vibrationStatus == "critical"

    def test_lubrication_classification_via_crest_factor(self):
        """Lubrication status depends on vibration crest factor."""
        # good: crest < 3
        m = SpindleBearingMonitor()
        m.record_reading(_make_reading(rms=2.0, peak=4.0))  # crest = 2.0
        assert m.get_health_report().lubricationStatus == "good"

        # marginal: 3 <= crest < 5
        m2 = SpindleBearingMonitor()
        m2.record_reading(_make_reading(rms=2.0, peak=8.0))  # crest = 4.0
        assert m2.get_health_report().lubricationStatus == "marginal"

        # poor: crest >= 5
        m3 = SpindleBearingMonitor()
        m3.record_reading(_make_reading(rms=2.0, peak=12.0))  # crest = 6.0
        assert m3.get_health_report().lubricationStatus == "poor"


class TestScoringWeights:
    """Tests for health scoring weight distribution."""

    def test_scoring_weights_sum_to_one(self):
        """The four scoring weights must sum to 1.0."""
        total = (SpindleBearingMonitor.WEIGHT_TEMPERATURE +
                 SpindleBearingMonitor.WEIGHT_VIBRATION +
                 SpindleBearingMonitor.WEIGHT_LOAD +
                 SpindleBearingMonitor.WEIGHT_TREND)
        assert total == pytest.approx(1.0)

    def test_elevated_vibration_lowers_health_more_than_elevated_temp(self):
        """Vibration has a higher weight (40%) than temperature (30%),
        so elevated vibration should reduce health more."""
        # Monitor with only elevated vibration (everything else normal)
        m_vib = SpindleBearingMonitor()
        for _ in range(20):
            m_vib.record_reading(_make_reading(temp=35.0, rms=4.0, peak=5.0,
                                                axial=500.0, radial=300.0))

        # Monitor with only elevated temperature (everything else normal)
        m_temp = SpindleBearingMonitor()
        for _ in range(20):
            m_temp.record_reading(_make_reading(temp=65.0, rms=1.0, peak=1.2,
                                                 axial=500.0, radial=300.0))

        health_vib = m_vib.get_health_report().overallHealth
        health_temp = m_temp.get_health_report().overallHealth

        # Both should be below 100, and vib should be lower
        assert health_vib < 100.0
        assert health_temp < 100.0
        assert health_vib < health_temp
