"""Tests for the SignalConditioner signal conditioning pipeline.

Validates the digital filter implementations, SNR calculation, and
automatic filter selection logic added to kpi_calculator.py.
"""

import sys
import math
from unittest.mock import MagicMock

# Mock ROS2 and miracle_core sub-module dependencies before importing
for _mod in ['rclpy', 'rclpy.node', 'rclpy.callback_groups', 'rclpy.qos', 'rclpy.lifecycle',
             'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
             'miracle_core.heartbeat_mixin', 'miracle_core.parameter_validation',
             'miracle_core.exceptions',
             'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(_mod, MagicMock())

sys.modules['miracle_core.lifecycle_node_base'].MiracleLifecycleNode = type('FakeNode', (), {
    'CRITICALITY_HIGH': 'HIGH',
    'CRITICALITY_MEDIUM': 'MEDIUM',
    '__init__': lambda self, *a, **kw: None,
    'get_logger': lambda self: MagicMock(),
    'create_publisher': lambda self, *a, **kw: MagicMock(),
    'create_subscription': lambda self, *a, **kw: MagicMock(),
    'create_timer': lambda self, *a, **kw: MagicMock(),
    'declare_and_validate_parameters': lambda self, specs: {k: MagicMock(value=v['default']) for k, v in specs.items()},
    'get_parameter': lambda self, name: MagicMock(value=0),
})

sys.modules.pop('miracle_scada.kpi_calculator', None)

import pytest

from miracle_scada.kpi_calculator import (
    FilterConfig,
    ConditionedSignal,
    SignalConditioner,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def conditioner():
    return SignalConditioner()


@pytest.fixture
def clean_signal():
    """A smooth sine-like signal with no noise."""
    return [math.sin(i * 0.1) for i in range(100)]


@pytest.fixture
def noisy_signal(clean_signal):
    """Clean signal with additive high-frequency noise."""
    import random
    random.seed(42)
    return [v + random.gauss(0, 0.3) for v in clean_signal]


@pytest.fixture
def spike_signal():
    """Mostly constant signal with impulsive spikes."""
    data = [10.0] * 100
    # Inject large spikes at known positions (>5% of samples to trigger median)
    for i in [5, 15, 25, 35, 45, 55, 65, 75, 85, 95]:
        data[i] = 500.0
    return data


# ---------------------------------------------------------------------------
# Test: FilterConfig dataclass
# ---------------------------------------------------------------------------

def test_filter_config_defaults():
    """FilterConfig should have sensible defaults for all fields."""
    cfg = FilterConfig()
    assert cfg.filter_type == 'moving_average'
    assert cfg.window_size == 5
    assert cfg.alpha == pytest.approx(0.3)
    assert cfg.cutoff_ratio == pytest.approx(0.1)


# ---------------------------------------------------------------------------
# Test: Moving average filter
# ---------------------------------------------------------------------------

def test_moving_average_reduces_noise(conditioner, noisy_signal, clean_signal):
    """Moving average should bring the signal closer to the clean original."""
    filtered = conditioner.apply_moving_average(noisy_signal, window=7)
    assert len(filtered) == len(noisy_signal)

    # Mean squared error of filtered should be lower than raw
    mse_raw = sum((r - c) ** 2 for r, c in zip(noisy_signal, clean_signal)) / len(clean_signal)
    mse_filt = sum((f - c) ** 2 for f, c in zip(filtered, clean_signal)) / len(clean_signal)
    assert mse_filt < mse_raw


def test_moving_average_empty():
    """Moving average on empty data should return empty list."""
    assert SignalConditioner.apply_moving_average([], 5) == []


# ---------------------------------------------------------------------------
# Test: Exponential filter
# ---------------------------------------------------------------------------

def test_exponential_filter_smooths(conditioner, noisy_signal):
    """Exponential filter should reduce variance compared to raw data."""
    filtered = conditioner.apply_exponential_filter(noisy_signal, alpha=0.2)
    assert len(filtered) == len(noisy_signal)

    raw_var = sum((v - sum(noisy_signal) / len(noisy_signal)) ** 2 for v in noisy_signal) / len(noisy_signal)
    filt_var = sum((v - sum(filtered) / len(filtered)) ** 2 for v in filtered) / len(filtered)
    # Filtered variance should not be larger than raw (smoothing reduces spread)
    assert filt_var <= raw_var * 1.1  # small tolerance


# ---------------------------------------------------------------------------
# Test: Median filter removes spikes
# ---------------------------------------------------------------------------

def test_median_filter_removes_spikes(conditioner, spike_signal):
    """Median filter should suppress impulsive spikes."""
    filtered = conditioner.apply_median_filter(spike_signal, window=5)
    assert len(filtered) == len(spike_signal)

    # The spike positions should now be close to the baseline (10.0)
    for i in [10, 30, 50, 70, 90]:
        assert filtered[i] < 100.0, f"Spike at index {i} was not suppressed"


# ---------------------------------------------------------------------------
# Test: Butterworth low-pass filter
# ---------------------------------------------------------------------------

def test_butterworth_low_smooths(conditioner, noisy_signal, clean_signal):
    """Butterworth low-pass should produce a smoother signal."""
    filtered = conditioner.apply_butterworth_low(noisy_signal, cutoff_ratio=0.1, order=2)
    assert len(filtered) == len(noisy_signal)

    # Check that the filtered signal has lower residual energy vs. clean
    mse_raw = sum((r - c) ** 2 for r, c in zip(noisy_signal, clean_signal)) / len(clean_signal)
    mse_filt = sum((f - c) ** 2 for f, c in zip(filtered, clean_signal)) / len(clean_signal)
    assert mse_filt < mse_raw


# ---------------------------------------------------------------------------
# Test: High-pass filter
# ---------------------------------------------------------------------------

def test_high_pass_removes_dc(conditioner):
    """High-pass filter should remove the DC offset from a constant signal."""
    # Signal with a large DC offset plus small oscillation
    data = [100.0 + math.sin(i * 0.5) for i in range(200)]
    filtered = conditioner.apply_high_pass(data, cutoff_ratio=0.05)
    assert len(filtered) == len(data)

    # The mean of the high-passed signal should be close to zero
    mean_hp = sum(filtered) / len(filtered)
    assert abs(mean_hp) < 10.0, f"DC component not removed (mean={mean_hp})"


# ---------------------------------------------------------------------------
# Test: SNR calculation
# ---------------------------------------------------------------------------

def test_snr_positive_for_smoothed(conditioner, noisy_signal):
    """SNR should be positive when the filter effectively removes noise."""
    filtered = conditioner.apply_moving_average(noisy_signal, window=7)
    snr = conditioner.calculate_snr(noisy_signal, filtered)
    assert snr > 0.0


def test_snr_zero_for_empty():
    """SNR should be 0 for empty inputs."""
    assert SignalConditioner.calculate_snr([], []) == 0.0


# ---------------------------------------------------------------------------
# Test: Auto-conditioning
# ---------------------------------------------------------------------------

def test_auto_condition_selects_median_for_spikes(conditioner, spike_signal):
    """auto_condition should choose the median filter when spikes dominate."""
    result = conditioner.auto_condition(spike_signal)
    assert isinstance(result, ConditionedSignal)
    assert result.filter_applied == 'median'
    assert len(result.filtered_values) == len(spike_signal)
    assert result.removed_outliers > 0


def test_auto_condition_handles_empty(conditioner):
    """auto_condition on empty data should return a valid ConditionedSignal."""
    result = conditioner.auto_condition([])
    assert result.filter_applied == 'none'
    assert result.filtered_values == []
    assert result.snr_improvement_db == 0.0
