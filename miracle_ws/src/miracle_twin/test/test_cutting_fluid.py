"""Tests for CuttingFluidManager — cutting-fluid lifecycle tracking."""
import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants']:
    sys.modules.setdefault(mod, MagicMock())

import time
import pytest
from miracle_twin.cutting_sim_proxy import (
    CuttingFluidManager,
    FluidMaintenanceRecord,
    FluidSample,
    FluidStatus,
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

DAY = 86400.0  # seconds in a day


def _good_sample(ts: float = 1_000_000.0) -> FluidSample:
    """Return a sample with all parameters within healthy thresholds."""
    return FluidSample(
        timestamp=ts,
        concentration_pct=7.5,
        ph=9.0,
        bacteria_count=2_000,
        tramp_oil_pct=1.0,
        temperature_c=22.0,
    )


def _bad_sample(ts: float = 1_000_000.0) -> FluidSample:
    """Return a sample with multiple out-of-range parameters."""
    return FluidSample(
        timestamp=ts,
        concentration_pct=3.0,   # below 5 %
        ph=7.0,                  # below 8.5
        bacteria_count=60_000,   # way above 10 000
        tramp_oil_pct=6.0,      # above 3 %
        temperature_c=35.0,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestFluidSampleRecording:
    """Verify that samples can be recorded and retrieved."""

    def test_record_single_sample(self):
        mgr = CuttingFluidManager()
        sample = _good_sample()
        mgr.record_sample(sample)
        status = mgr.get_status()
        assert isinstance(status, FluidStatus)

    def test_get_status_raises_without_samples(self):
        mgr = CuttingFluidManager()
        with pytest.raises(ValueError, match="No fluid samples"):
            mgr.get_status()


class TestFluidStatus:
    """Verify health-score calculation and threshold checks."""

    def test_healthy_sample_scores_100(self):
        mgr = CuttingFluidManager()
        mgr.record_sample(_good_sample())
        status = mgr.get_status()
        assert status.health_score == 100.0
        assert status.concentration_ok is True
        assert status.ph_ok is True
        assert status.bacteria_ok is True
        assert status.tramp_oil_ok is True
        assert status.recommended_action == 'none'

    def test_unhealthy_sample_flags_issues(self):
        mgr = CuttingFluidManager()
        mgr.record_sample(_bad_sample())
        status = mgr.get_status()
        assert status.health_score < 50.0
        assert status.concentration_ok is False
        assert status.ph_ok is False
        assert status.bacteria_ok is False
        assert status.tramp_oil_ok is False

    def test_high_bacteria_recommends_full_change(self):
        mgr = CuttingFluidManager()
        sample = _good_sample()
        # Override bacteria to trigger full_change recommendation (>= 50 000)
        sample.bacteria_count = 60_000
        mgr.record_sample(sample)
        status = mgr.get_status()
        assert status.recommended_action == 'full_change'

    def test_days_since_change_tracked(self):
        mgr = CuttingFluidManager()
        t0 = 1_000_000.0
        mgr.record_maintenance(FluidMaintenanceRecord(
            timestamp=t0, action='full_change', amount_liters=50.0, notes='initial fill',
        ))
        mgr.record_sample(_good_sample(ts=t0 + 10 * DAY))
        status = mgr.get_status()
        assert status.days_since_change == 10


class TestMaintenanceRecords:
    """Verify maintenance logging and validation."""

    def test_record_and_retrieve_maintenance(self):
        mgr = CuttingFluidManager()
        rec = FluidMaintenanceRecord(
            timestamp=time.time(), action='top_up', amount_liters=5.0, notes='added emulsion',
        )
        mgr.record_maintenance(rec)
        history = mgr.get_maintenance_history()
        assert len(history) == 1
        assert history[0].action == 'top_up'

    def test_invalid_action_raises(self):
        mgr = CuttingFluidManager()
        with pytest.raises(ValueError, match="Unknown action"):
            mgr.record_maintenance(FluidMaintenanceRecord(
                timestamp=time.time(), action='drain', amount_liters=0.0, notes='',
            ))


class TestPredictNextChange:
    """Verify degradation-trend prediction."""

    def test_predict_with_degrading_trend(self):
        mgr = CuttingFluidManager()
        t0 = 1_000_000.0
        # Create a clear upward trend in bacteria and tramp oil over 5 days
        for day in range(6):
            mgr.record_sample(FluidSample(
                timestamp=t0 + day * DAY,
                concentration_pct=7.5,
                ph=9.0,
                bacteria_count=1_000 + day * 5_000,
                tramp_oil_pct=0.5 + day * 0.4,
                temperature_c=22.0,
            ))
        prediction = mgr.predict_next_change(max_bacteria=50_000, max_tramp_oil=5.0)
        assert prediction is not None
        assert prediction > 0

    def test_predict_returns_none_with_insufficient_data(self):
        mgr = CuttingFluidManager()
        mgr.record_sample(_good_sample())
        assert mgr.predict_next_change() is None


class TestCostAnalysis:
    """Verify cost computations."""

    def test_cost_analysis_basic(self):
        mgr = CuttingFluidManager()
        t0 = 1_000_000.0
        mgr.record_maintenance(FluidMaintenanceRecord(
            timestamp=t0, action='full_change', amount_liters=50.0, notes='initial',
        ))
        mgr.record_maintenance(FluidMaintenanceRecord(
            timestamp=t0 + 7 * DAY, action='top_up', amount_liters=5.0, notes='weekly top-up',
        ))
        mgr.record_maintenance(FluidMaintenanceRecord(
            timestamp=t0 + 30 * DAY, action='full_change', amount_liters=50.0, notes='monthly change',
        ))

        costs = mgr.get_cost_analysis(fluid_cost_per_liter=2.0, disposal_cost=100.0)
        assert costs['total_liters_used'] == 105.0
        assert costs['total_fluid_cost'] == 210.0
        assert costs['total_disposal_cost'] == 200.0
        assert costs['total_cost'] == 410.0
        assert costs['num_full_changes'] == 2

    def test_cost_analysis_no_records(self):
        mgr = CuttingFluidManager()
        costs = mgr.get_cost_analysis(fluid_cost_per_liter=3.0, disposal_cost=50.0)
        assert costs['total_cost'] == 0.0
        assert costs['num_full_changes'] == 0
        assert costs['total_liters_used'] == 0.0
