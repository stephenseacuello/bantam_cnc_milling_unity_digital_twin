"""Tests for VibrationFFTAnalyzer — pure-Python DFT chatter detection.

Covers:
- Tooth-passing frequency calculation
- DFT on empty signal
- DFT on a simple sinusoid (dominant frequency recovery)
- RMS value correctness
- Harmonic tagging of frequency bins
- Chatter detection when non-harmonic peak dominates
- No chatter when signal contains only harmonics
- Full report generation and recommendation text
- Detect_chatter threshold sensitivity
- Multiple-frequency signal analysis
"""

import math
import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_twin.cutting_sim_proxy import (
    FrequencyComponent,
    FFTResult,
    VibrationAnalysisReport,
    VibrationFFTAnalyzer,
)


@pytest.fixture
def analyzer():
    return VibrationFFTAnalyzer()


# -----------------------------------------------------------------------
# 1. Tooth-passing frequency
# -----------------------------------------------------------------------

def test_tooth_passing_frequency(analyzer):
    """TPF = RPM * flutes / 60."""
    tpf = analyzer.get_tooth_passing_frequency(6000.0, 4)
    assert tpf == pytest.approx(400.0)


def test_tooth_passing_frequency_single_flute(analyzer):
    """Single-flute at 3000 RPM -> 50 Hz."""
    tpf = analyzer.get_tooth_passing_frequency(3000.0, 1)
    assert tpf == pytest.approx(50.0)


# -----------------------------------------------------------------------
# 2. Empty signal
# -----------------------------------------------------------------------

def test_analyze_empty_signal(analyzer):
    """An empty signal should return zeroed FFTResult with no chatter."""
    result = analyzer.analyze([], 1000.0, 6000.0, 4)
    assert result.dominant_frequency_hz == 0.0
    assert result.dominant_amplitude == 0.0
    assert result.total_rms == 0.0
    assert result.chatter_detected is False
    assert result.chatter_frequency_hz is None
    assert len(result.components) == 0


# -----------------------------------------------------------------------
# 3. Single sinusoid — dominant frequency recovery
# -----------------------------------------------------------------------

def test_single_sinusoid_dominant_frequency(analyzer):
    """A pure sine at 200 Hz sampled at 1000 Hz should show up as dominant."""
    sample_rate = 1000.0
    freq = 200.0
    N = 256
    signal = [math.sin(2 * math.pi * freq * i / sample_rate) for i in range(N)]

    result = analyzer.analyze(signal, sample_rate, 3000.0, 4)

    # Dominant frequency should be near 200 Hz (within one bin width)
    bin_width = sample_rate / N
    assert abs(result.dominant_frequency_hz - freq) <= bin_width + 0.1
    assert result.dominant_amplitude > 0.0


# -----------------------------------------------------------------------
# 4. RMS is non-negative and reasonable
# -----------------------------------------------------------------------

def test_rms_nonnegative(analyzer):
    """RMS must be non-negative for any input."""
    signal = [math.sin(2 * math.pi * 100 * i / 500) for i in range(128)]
    result = analyzer.analyze(signal, 500.0, 6000.0, 4)
    assert result.total_rms >= 0.0


# -----------------------------------------------------------------------
# 5. Harmonic tagging
# -----------------------------------------------------------------------

def test_harmonic_tagging(analyzer):
    """Bins at multiples of TPF should be tagged is_harmonic=True."""
    # TPF = 6000 * 4 / 60 = 400 Hz
    # Build a signal at 400 Hz so the bin at ~400 Hz is marked harmonic
    sample_rate = 2000.0
    N = 200  # bin resolution = 10 Hz
    tpf = 400.0
    signal = [math.sin(2 * math.pi * tpf * i / sample_rate) for i in range(N)]

    result = analyzer.analyze(signal, sample_rate, 6000.0, 4)

    # Find the component closest to 400 Hz
    closest = min(result.components, key=lambda c: abs(c.frequency_hz - tpf))
    assert closest.is_harmonic is True


# -----------------------------------------------------------------------
# 6. Chatter detection — non-harmonic dominates
# -----------------------------------------------------------------------

def test_chatter_detected_non_harmonic(analyzer):
    """When a non-harmonic frequency dominates, chatter should be flagged."""
    # TPF = 6000 * 4 / 60 = 400 Hz
    # Place a strong signal at 275 Hz (not a harmonic of 400)
    sample_rate = 2000.0
    N = 200
    chatter_freq = 275.0
    signal = [3.0 * math.sin(2 * math.pi * chatter_freq * i / sample_rate)
              for i in range(N)]

    result = analyzer.analyze(signal, sample_rate, 6000.0, 4)
    assert result.chatter_detected is True
    assert result.chatter_frequency_hz is not None


# -----------------------------------------------------------------------
# 7. No chatter when only harmonics present
# -----------------------------------------------------------------------

def test_no_chatter_harmonics_only(analyzer):
    """Signal composed only of TPF harmonics should report no chatter."""
    sample_rate = 4000.0
    N = 400  # bin resolution = 10 Hz
    tpf = 400.0  # 6000 RPM, 4 flutes

    signal = [0.0] * N
    for i in range(N):
        for h in range(1, 4):
            signal[i] += math.sin(2 * math.pi * h * tpf * i / sample_rate)

    result = analyzer.analyze(signal, sample_rate, 6000.0, 4)
    assert result.chatter_detected is False


# -----------------------------------------------------------------------
# 8. Full report generation
# -----------------------------------------------------------------------

def test_generate_report(analyzer):
    """generate_report should produce a VibrationAnalysisReport."""
    sample_rate = 1000.0
    N = 128
    signal = [math.sin(2 * math.pi * 100 * i / sample_rate) for i in range(N)]

    report = analyzer.generate_report(signal, sample_rate, 6000.0, 4)

    assert isinstance(report, VibrationAnalysisReport)
    assert report.spindle_rpm == 6000.0
    assert report.num_flutes == 4
    assert isinstance(report.stability_margin, float)
    assert isinstance(report.recommendation, str)
    assert len(report.recommendation) > 0


# -----------------------------------------------------------------------
# 9. detect_chatter threshold sensitivity
# -----------------------------------------------------------------------

def test_detect_chatter_threshold(analyzer):
    """With a very high threshold, borderline chatter should not be flagged."""
    sample_rate = 2000.0
    N = 200
    tpf = 400.0  # 6000 RPM, 4 flutes

    # Mix harmonic (amplitude 1) and non-harmonic (amplitude 0.3)
    signal = [0.0] * N
    for i in range(N):
        signal[i] = math.sin(2 * math.pi * tpf * i / sample_rate)
        signal[i] += 0.3 * math.sin(2 * math.pi * 275.0 * i / sample_rate)

    result = analyzer.analyze(signal, sample_rate, 6000.0, 4)

    # With a very high threshold (e.g. 2.0), the non-harmonic peak should
    # be too small relative to the harmonic peak.
    detected, freq = analyzer.detect_chatter(result, 6000.0, 4, threshold_ratio=2.0)
    assert detected is False


# -----------------------------------------------------------------------
# 10. Multiple-frequency signal
# -----------------------------------------------------------------------

def test_multiple_frequencies(analyzer):
    """Signal with multiple frequencies should produce multiple components."""
    sample_rate = 2000.0
    N = 200
    signal = [0.0] * N
    for i in range(N):
        signal[i] = (math.sin(2 * math.pi * 100 * i / sample_rate)
                      + 0.5 * math.sin(2 * math.pi * 300 * i / sample_rate))

    result = analyzer.analyze(signal, sample_rate, 6000.0, 4)

    # Should have N//2 + 1 components (one-sided spectrum)
    assert len(result.components) == N // 2 + 1
    # Dominant should be near 100 Hz (the stronger component)
    bin_width = sample_rate / N
    assert abs(result.dominant_frequency_hz - 100.0) <= bin_width + 0.1
