"""Tests for spindle vibration spectrum analysis.

Mirrors the C# SpindleVibrationAnalyzer logic in Python to validate:
- DFT computation and dominant frequency detection
- RMS calculation
- Chatter detection (tooth-passing harmonics vs non-harmonic peaks)
- Runout estimation from 1x RPM component
- Bearing health indicator (high-frequency energy ratio)
- Health report generation
"""

import math
import pytest

# ---------------------------------------------------------------------------
# Python mirror of SpindleVibrationAnalyzer (matches C# implementation)
# ---------------------------------------------------------------------------

class VibrationSpectrum:
    __slots__ = (
        "frequencies", "amplitudes", "phases",
        "dominantFrequencyHz", "dominantAmplitudeMM",
        "totalRMS", "timestamp",
    )

    def __init__(self):
        self.frequencies = []
        self.amplitudes = []
        self.phases = []
        self.dominantFrequencyHz = 0.0
        self.dominantAmplitudeMM = 0.0
        self.totalRMS = 0.0
        self.timestamp = 0.0


class ChatterDetectionResult:
    __slots__ = (
        "isChatter", "chatterFrequencyHz", "chatterAmplitudeMM",
        "toothPassingFrequencyHz", "confidence",
    )

    def __init__(self):
        self.isChatter = False
        self.chatterFrequencyHz = 0.0
        self.chatterAmplitudeMM = 0.0
        self.toothPassingFrequencyHz = 0.0
        self.confidence = 0.0


class SpindleHealthReport:
    __slots__ = (
        "vibrationRMS", "dominantFrequency", "chatterDetected",
        "runoutMM", "bearingHealth", "overallHealth", "recommendations",
    )

    def __init__(self):
        self.vibrationRMS = 0.0
        self.dominantFrequency = 0.0
        self.chatterDetected = False
        self.runoutMM = 0.0
        self.bearingHealth = 0.0
        self.overallHealth = 0.0
        self.recommendations = []


class SpindleVibrationAnalyzer:
    """Python mirror of the C# SpindleVibrationAnalyzer."""

    def __init__(self, capacity=4096, sample_rate=10000.0):
        self._buffer = [0.0] * capacity
        self._write_index = 0
        self._sample_count = 0
        self.sample_rate_hz = sample_rate

    @property
    def sample_count(self):
        return self._sample_count

    @property
    def capacity(self):
        return len(self._buffer)

    def add_sample(self, amplitude: float):
        self._buffer[self._write_index] = amplitude
        self._write_index = (self._write_index + 1) % len(self._buffer)
        if self._sample_count < len(self._buffer):
            self._sample_count += 1

    def _get_ordered_samples(self):
        if self._sample_count < len(self._buffer):
            return self._buffer[: self._sample_count]
        # Circular unwrap
        start = self._write_index
        return self._buffer[start:] + self._buffer[:start]

    def compute_spectrum(self) -> VibrationSpectrum:
        spec = VibrationSpectrum()
        if self._sample_count == 0:
            return spec

        samples = self._get_ordered_samples()
        N = len(samples)
        num_bins = N // 2
        freq_res = self.sample_rate_hz / N

        frequencies = [0.0] * num_bins
        amplitudes = [0.0] * num_bins
        phases = [0.0] * num_bins

        for k in range(num_bins):
            real_part = 0.0
            imag_part = 0.0
            for n in range(N):
                angle = 2.0 * math.pi * k * n / N
                real_part += samples[n] * math.cos(angle)
                imag_part -= samples[n] * math.sin(angle)

            mag = math.sqrt(real_part * real_part + imag_part * imag_part) / N
            if k > 0:
                mag *= 2.0

            frequencies[k] = k * freq_res
            amplitudes[k] = mag
            phases[k] = math.atan2(imag_part, real_part)

        # Dominant (skip DC)
        dom_freq = 0.0
        dom_amp = 0.0
        for k in range(1, num_bins):
            if amplitudes[k] > dom_amp:
                dom_amp = amplitudes[k]
                dom_freq = frequencies[k]

        # RMS
        sum_sq = sum(s * s for s in samples)
        rms = math.sqrt(sum_sq / N)

        spec.frequencies = frequencies
        spec.amplitudes = amplitudes
        spec.phases = phases
        spec.dominantFrequencyHz = dom_freq
        spec.dominantAmplitudeMM = dom_amp
        spec.totalRMS = rms
        return spec

    def detect_chatter_frequency(self, spindle_rpm: float, num_flutes: int) -> ChatterDetectionResult:
        spectrum = self.compute_spectrum()
        tooth_pass_freq = spindle_rpm * num_flutes / 60.0

        result = ChatterDetectionResult()
        result.toothPassingFrequencyHz = tooth_pass_freq

        if len(spectrum.amplitudes) < 2 or tooth_pass_freq <= 0:
            return result

        freq_res = spectrum.frequencies[1] - spectrum.frequencies[0]
        tolerance = freq_res * 1.5

        max_non_harmonic_amp = 0.0
        max_non_harmonic_freq = 0.0
        max_harmonic_amp = 0.0

        for k in range(1, len(spectrum.amplitudes)):
            freq = spectrum.frequencies[k]
            amp = spectrum.amplitudes[k]

            is_harmonic = False
            for h in range(1, 4):  # 1x, 2x, 3x
                if abs(freq - h * tooth_pass_freq) < tolerance:
                    is_harmonic = True
                    if amp > max_harmonic_amp:
                        max_harmonic_amp = amp
                    break

            if not is_harmonic and amp > max_non_harmonic_amp:
                max_non_harmonic_amp = amp
                max_non_harmonic_freq = freq

        threshold = max_harmonic_amp * 0.3
        if max_non_harmonic_amp > threshold and max_non_harmonic_amp > 1e-6:
            result.isChatter = True
            result.chatterFrequencyHz = max_non_harmonic_freq
            result.chatterAmplitudeMM = max_non_harmonic_amp
            ratio = max_non_harmonic_amp / max_harmonic_amp if max_harmonic_amp > 0 else 1.0
            result.confidence = max(0.0, min(1.0, ratio))

        return result

    def get_runout_estimate(self, spindle_rpm: float) -> float:
        if spindle_rpm <= 0 or self._sample_count == 0:
            return 0.0

        spectrum = self.compute_spectrum()
        once_per_rev_hz = spindle_rpm / 60.0

        if len(spectrum.frequencies) < 2:
            return 0.0

        freq_res = spectrum.frequencies[1] - spectrum.frequencies[0]
        if freq_res <= 0:
            return 0.0

        bin_idx = round(once_per_rev_hz / freq_res)
        if bin_idx < 0 or bin_idx >= len(spectrum.amplitudes):
            return 0.0

        return spectrum.amplitudes[bin_idx]

    def get_bearing_health_indicator(self) -> float:
        if self._sample_count == 0:
            return 1.0

        spectrum = self.compute_spectrum()
        if len(spectrum.amplitudes) < 2:
            return 1.0

        total_energy = 0.0
        high_freq_energy = 0.0
        high_freq_threshold = 2000.0

        for k in range(1, len(spectrum.amplitudes)):
            energy = spectrum.amplitudes[k] ** 2
            total_energy += energy
            if spectrum.frequencies[k] > high_freq_threshold:
                high_freq_energy += energy

        if total_energy < 1e-12:
            return 1.0

        high_freq_ratio = high_freq_energy / total_energy
        return max(0.0, min(1.0, 1.0 - high_freq_ratio))

    def generate_health_report(self, spindle_rpm: float, num_flutes: int) -> SpindleHealthReport:
        spectrum = self.compute_spectrum()
        chatter_result = self.detect_chatter_frequency(spindle_rpm, num_flutes)
        runout = self.get_runout_estimate(spindle_rpm)
        bearing_health = self.get_bearing_health_indicator()

        recommendations = []

        rms_score = max(0.0, min(1.0, 1.0 - spectrum.totalRMS / 0.1))
        chatter_score = (1.0 - chatter_result.confidence) if chatter_result.isChatter else 1.0
        runout_score = max(0.0, min(1.0, 1.0 - runout / 0.05))

        overall = (0.30 * bearing_health
                   + 0.30 * rms_score
                   + 0.25 * chatter_score
                   + 0.15 * runout_score)

        if chatter_result.isChatter:
            recommendations.append(
                f"Chatter detected at {chatter_result.chatterFrequencyHz:.0f} Hz. "
                "Consider adjusting RPM or depth of cut."
            )
        if runout > 0.025:
            recommendations.append(
                f"Runout {runout:.3f} mm exceeds 0.025 mm threshold. "
                "Inspect collet and tool holder."
            )
        if bearing_health < 0.7:
            recommendations.append(
                "Elevated high-frequency vibration. Schedule bearing inspection."
            )
        if spectrum.totalRMS > 0.05:
            recommendations.append(
                f"RMS vibration {spectrum.totalRMS:.3f} mm is elevated. "
                "Check tool condition and workholding."
            )
        if not recommendations:
            recommendations.append(
                "Spindle health is within normal operating parameters."
            )

        report = SpindleHealthReport()
        report.vibrationRMS = spectrum.totalRMS
        report.dominantFrequency = spectrum.dominantFrequencyHz
        report.chatterDetected = chatter_result.isChatter
        report.runoutMM = runout
        report.bearingHealth = bearing_health
        report.overallHealth = max(0.0, min(1.0, overall))
        report.recommendations = recommendations
        return report


# ---------------------------------------------------------------------------
# Helper to fill an analyzer with a synthetic signal
# ---------------------------------------------------------------------------

def _make_analyzer_with_signal(freqs_hz, amps_mm, capacity=1024, sample_rate=10000.0,
                               dc_offset=0.0):
    """Create an analyzer pre-loaded with a sum-of-sinusoids signal."""
    analyzer = SpindleVibrationAnalyzer(capacity=capacity, sample_rate=sample_rate)
    N = capacity
    dt = 1.0 / sample_rate
    for n in range(N):
        t = n * dt
        val = dc_offset
        for f, a in zip(freqs_hz, amps_mm):
            val += a * math.sin(2.0 * math.pi * f * t)
        analyzer.add_sample(val)
    return analyzer


# ===========================================================================
# Tests
# ===========================================================================


class TestDFTSingleSineWave:
    """DFT of a known single sine wave should produce the correct frequency peak."""

    def test_single_sine_peak_frequency(self):
        freq = 500.0
        amp = 0.05
        analyzer = _make_analyzer_with_signal([freq], [amp])
        spec = analyzer.compute_spectrum()
        assert abs(spec.dominantFrequencyHz - freq) < 15.0

    def test_single_sine_peak_amplitude(self):
        freq = 500.0
        amp = 0.05
        analyzer = _make_analyzer_with_signal([freq], [amp])
        spec = analyzer.compute_spectrum()
        assert abs(spec.dominantAmplitudeMM - amp) < 0.005

    def test_single_sine_frequency_250(self):
        freq = 250.0
        amp = 0.03
        analyzer = _make_analyzer_with_signal([freq], [amp])
        spec = analyzer.compute_spectrum()
        assert abs(spec.dominantFrequencyHz - freq) < 15.0


class TestDFTMultipleFrequencies:
    """Multiple frequency components — correct dominant identification."""

    def test_dominant_is_largest(self):
        analyzer = _make_analyzer_with_signal([200.0, 800.0], [0.01, 0.05])
        spec = analyzer.compute_spectrum()
        assert abs(spec.dominantFrequencyHz - 800.0) < 15.0

    def test_dominant_amplitude(self):
        analyzer = _make_analyzer_with_signal([200.0, 800.0], [0.01, 0.05])
        spec = analyzer.compute_spectrum()
        assert abs(spec.dominantAmplitudeMM - 0.05) < 0.005

    def test_three_components(self):
        analyzer = _make_analyzer_with_signal([100.0, 400.0, 1500.0], [0.02, 0.08, 0.01])
        spec = analyzer.compute_spectrum()
        assert abs(spec.dominantFrequencyHz - 400.0) < 15.0


class TestRMSCalculation:
    """RMS calculation accuracy."""

    def test_rms_single_sine(self):
        amp = 0.05
        analyzer = _make_analyzer_with_signal([500.0], [amp])
        spec = analyzer.compute_spectrum()
        expected_rms = amp / math.sqrt(2.0)
        assert abs(spec.totalRMS - expected_rms) < 0.002

    def test_rms_zero_signal(self):
        analyzer = SpindleVibrationAnalyzer(capacity=256)
        for _ in range(256):
            analyzer.add_sample(0.0)
        spec = analyzer.compute_spectrum()
        assert spec.totalRMS == pytest.approx(0.0, abs=1e-10)

    def test_rms_dc_only(self):
        dc = 0.1
        analyzer = SpindleVibrationAnalyzer(capacity=256, sample_rate=10000.0)
        for _ in range(256):
            analyzer.add_sample(dc)
        spec = analyzer.compute_spectrum()
        assert abs(spec.totalRMS - dc) < 0.001


class TestChatterDetection:
    """Chatter detection: tooth passing frequency only vs extra peaks."""

    def test_no_chatter_tooth_passing_only(self):
        """Signal at tooth passing freq harmonics only -> no chatter."""
        rpm = 12000.0
        flutes = 2
        tpf = rpm * flutes / 60.0  # 400 Hz
        # Only tooth-passing harmonics
        analyzer = _make_analyzer_with_signal(
            [tpf, 2 * tpf], [0.05, 0.02], capacity=2048, sample_rate=10000.0
        )
        result = analyzer.detect_chatter_frequency(rpm, flutes)
        assert result.isChatter is False

    def test_chatter_with_extra_peak(self):
        """Extra peak NOT at tooth passing harmonics -> chatter detected."""
        rpm = 12000.0
        flutes = 2
        tpf = rpm * flutes / 60.0  # 400 Hz
        chatter_freq = 1750.0  # not a harmonic of 400
        analyzer = _make_analyzer_with_signal(
            [tpf, chatter_freq], [0.03, 0.05], capacity=2048, sample_rate=10000.0
        )
        result = analyzer.detect_chatter_frequency(rpm, flutes)
        assert result.isChatter is True
        assert abs(result.chatterFrequencyHz - chatter_freq) < 15.0

    def test_chatter_confidence_high_when_dominant(self):
        rpm = 12000.0
        flutes = 2
        tpf = rpm * flutes / 60.0
        analyzer = _make_analyzer_with_signal(
            [tpf, 1750.0], [0.02, 0.06], capacity=2048, sample_rate=10000.0
        )
        result = analyzer.detect_chatter_frequency(rpm, flutes)
        assert result.isChatter is True
        assert result.confidence > 0.5

    def test_tooth_passing_frequency_value(self):
        rpm = 6000.0
        flutes = 4
        result = SpindleVibrationAnalyzer(capacity=256).detect_chatter_frequency(rpm, flutes)
        assert result.toothPassingFrequencyHz == pytest.approx(400.0)


class TestRunoutEstimate:
    """Runout estimation from 1x RPM component."""

    def test_runout_from_1x_rpm(self):
        rpm = 6000.0  # 100 Hz
        runout_amp = 0.02
        analyzer = _make_analyzer_with_signal(
            [100.0], [runout_amp], capacity=1024, sample_rate=10000.0
        )
        est = analyzer.get_runout_estimate(rpm)
        assert abs(est - runout_amp) < 0.003

    def test_runout_zero_rpm(self):
        analyzer = _make_analyzer_with_signal([500.0], [0.05])
        assert analyzer.get_runout_estimate(0.0) == 0.0

    def test_runout_with_other_components(self):
        rpm = 6000.0  # 100 Hz
        analyzer = _make_analyzer_with_signal(
            [100.0, 500.0, 1200.0], [0.015, 0.04, 0.01],
            capacity=1024, sample_rate=10000.0
        )
        est = analyzer.get_runout_estimate(rpm)
        assert abs(est - 0.015) < 0.003


class TestBearingHealth:
    """Bearing health from high-frequency energy ratio."""

    def test_healthy_low_frequency_signal(self):
        analyzer = _make_analyzer_with_signal([200.0], [0.05])
        health = analyzer.get_bearing_health_indicator()
        assert health > 0.9

    def test_degraded_high_frequency_signal(self):
        analyzer = _make_analyzer_with_signal([3000.0], [0.05])
        health = analyzer.get_bearing_health_indicator()
        assert health < 0.2

    def test_mixed_frequencies(self):
        analyzer = _make_analyzer_with_signal(
            [200.0, 3000.0], [0.05, 0.02]
        )
        health = analyzer.get_bearing_health_indicator()
        assert 0.2 < health < 0.95

    def test_empty_buffer_returns_one(self):
        analyzer = SpindleVibrationAnalyzer()
        assert analyzer.get_bearing_health_indicator() == 1.0


class TestHealthReport:
    """Health report generation."""

    def test_healthy_report(self):
        rpm = 12000.0
        flutes = 2
        tpf = rpm * flutes / 60.0
        analyzer = _make_analyzer_with_signal(
            [tpf], [0.005], capacity=2048, sample_rate=10000.0
        )
        report = analyzer.generate_health_report(rpm, flutes)
        assert report.chatterDetected is False
        assert report.overallHealth > 0.7
        assert len(report.recommendations) >= 1

    def test_unhealthy_chatter_report(self):
        rpm = 12000.0
        flutes = 2
        tpf = rpm * flutes / 60.0
        analyzer = _make_analyzer_with_signal(
            [tpf, 1750.0], [0.02, 0.08], capacity=2048, sample_rate=10000.0
        )
        report = analyzer.generate_health_report(rpm, flutes)
        assert report.chatterDetected is True
        assert any("Chatter" in r for r in report.recommendations)

    def test_report_overall_health_clamped(self):
        analyzer = _make_analyzer_with_signal([500.0], [0.001])
        report = analyzer.generate_health_report(12000.0, 2)
        assert 0.0 <= report.overallHealth <= 1.0

    def test_report_high_rms_recommendation(self):
        analyzer = _make_analyzer_with_signal([500.0], [0.1])
        report = analyzer.generate_health_report(12000.0, 2)
        assert any("RMS" in r for r in report.recommendations)


class TestEdgeCases:
    """Edge cases: empty buffer, single sample, DC offset."""

    def test_empty_buffer_spectrum(self):
        analyzer = SpindleVibrationAnalyzer()
        spec = analyzer.compute_spectrum()
        assert spec.totalRMS == 0.0
        assert spec.dominantFrequencyHz == 0.0
        assert len(spec.frequencies) == 0

    def test_single_sample(self):
        analyzer = SpindleVibrationAnalyzer(capacity=4096)
        analyzer.add_sample(0.05)
        spec = analyzer.compute_spectrum()
        assert spec.totalRMS == pytest.approx(0.05, abs=1e-6)

    def test_dc_offset_not_dominant(self):
        """DC offset should NOT be reported as the dominant frequency."""
        dc = 1.0
        analyzer = _make_analyzer_with_signal([500.0], [0.05], dc_offset=dc)
        spec = analyzer.compute_spectrum()
        assert spec.dominantFrequencyHz > 0.0  # Not 0 Hz (DC)

    def test_circular_buffer_wrap(self):
        """Adding more samples than capacity should still work correctly."""
        capacity = 64
        analyzer = SpindleVibrationAnalyzer(capacity=capacity, sample_rate=1000.0)
        freq = 100.0
        dt = 1.0 / 1000.0
        # Write 2x capacity worth of samples
        for n in range(capacity * 2):
            t = n * dt
            analyzer.add_sample(0.05 * math.sin(2.0 * math.pi * freq * t))
        assert analyzer.sample_count == capacity
        spec = analyzer.compute_spectrum()
        assert spec.dominantFrequencyHz > 0.0

    def test_detect_chatter_empty_buffer(self):
        analyzer = SpindleVibrationAnalyzer()
        result = analyzer.detect_chatter_frequency(12000.0, 2)
        assert result.isChatter is False

    def test_runout_empty_buffer(self):
        analyzer = SpindleVibrationAnalyzer()
        assert analyzer.get_runout_estimate(6000.0) == 0.0

    def test_health_report_empty_buffer(self):
        analyzer = SpindleVibrationAnalyzer()
        report = analyzer.generate_health_report(12000.0, 2)
        assert report.overallHealth >= 0.0
        assert len(report.recommendations) >= 1
