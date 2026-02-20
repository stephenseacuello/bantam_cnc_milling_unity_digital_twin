"""Tests for OEE (Overall Equipment Effectiveness) calculation logic.

OEE = Availability x Performance x Quality
Tests cover normal computation, edge cases with zero values,
and boundary conditions.
"""

import pytest


class _OEECalculator:
    """Extracted OEE computation logic from OEECalculatorNode.

    Computes Availability, Performance, and Quality independently and
    multiplies them to obtain OEE.
    """

    @staticmethod
    def availability(running_time: float, planned_time: float) -> float:
        """Availability = Running Time / Planned Time."""
        if planned_time <= 0:
            return 0.0
        return min(running_time / planned_time, 1.0)

    @staticmethod
    def performance(
        ideal_cycle_time: float, total_output: int, running_time: float
    ) -> float:
        """Performance = (Ideal Cycle Time * Output) / Running Time."""
        if running_time <= 0 or total_output <= 0:
            return 0.0
        return min((ideal_cycle_time * total_output) / running_time, 1.0)

    @staticmethod
    def quality(good_output: int, total_output: int) -> float:
        """Quality = Good Output / Total Output."""
        if total_output <= 0:
            return 1.0  # No output means no defects
        return min(good_output / total_output, 1.0)

    @classmethod
    def oee(
        cls,
        running_time: float,
        planned_time: float,
        ideal_cycle_time: float,
        total_output: int,
        good_output: int,
    ) -> float:
        """Compute OEE = Availability * Performance * Quality."""
        a = cls.availability(running_time, planned_time)
        p = cls.performance(ideal_cycle_time, total_output, running_time)
        q = cls.quality(good_output, total_output)
        return a * p * q


@pytest.fixture
def calc():
    return _OEECalculator()


class TestAvailability:
    """Tests for the Availability component."""

    def test_full_availability(self, calc):
        """Running all planned time gives 100% availability."""
        assert calc.availability(480.0, 480.0) == pytest.approx(1.0)

    def test_partial_availability(self, calc):
        """Running half the planned time gives 50% availability."""
        assert calc.availability(240.0, 480.0) == pytest.approx(0.5)

    def test_zero_planned_time(self, calc):
        """Zero planned time should return 0 (avoid division by zero)."""
        assert calc.availability(100.0, 0.0) == pytest.approx(0.0)

    def test_capped_at_one(self, calc):
        """Availability should not exceed 1.0 even if running > planned."""
        assert calc.availability(500.0, 480.0) == pytest.approx(1.0)

    def test_zero_running_time(self, calc):
        """Zero running time gives 0% availability."""
        assert calc.availability(0.0, 480.0) == pytest.approx(0.0)


class TestPerformance:
    """Tests for the Performance component."""

    def test_ideal_performance(self, calc):
        """Producing at exactly ideal rate gives 100% performance."""
        # 8 parts * 60s ideal cycle = 480s running
        assert calc.performance(60.0, 8, 480.0) == pytest.approx(1.0)

    def test_slow_performance(self, calc):
        """Producing slower than ideal rate gives < 100%."""
        # 4 parts * 60s = 240s ideal, but running 480s
        assert calc.performance(60.0, 4, 480.0) == pytest.approx(0.5)

    def test_zero_output(self, calc):
        """Zero output gives 0% performance."""
        assert calc.performance(60.0, 0, 480.0) == pytest.approx(0.0)

    def test_zero_running_time(self, calc):
        """Zero running time gives 0% performance (avoid division by zero)."""
        assert calc.performance(60.0, 10, 0.0) == pytest.approx(0.0)

    def test_capped_at_one(self, calc):
        """Performance should not exceed 1.0."""
        # Producing faster than ideal
        assert calc.performance(60.0, 100, 480.0) == pytest.approx(1.0)


class TestQuality:
    """Tests for the Quality component."""

    def test_perfect_quality(self, calc):
        """All good output gives 100% quality."""
        assert calc.quality(100, 100) == pytest.approx(1.0)

    def test_partial_quality(self, calc):
        """Half defective gives 50% quality."""
        assert calc.quality(50, 100) == pytest.approx(0.5)

    def test_zero_total_output(self, calc):
        """Zero total output should return 1.0 (no defects recorded)."""
        assert calc.quality(0, 0) == pytest.approx(1.0)

    def test_all_defective(self, calc):
        """Zero good output with total output gives 0% quality."""
        assert calc.quality(0, 100) == pytest.approx(0.0)


class TestOEE:
    """Tests for the combined OEE formula."""

    def test_perfect_oee(self, calc):
        """100% Availability, Performance, and Quality gives OEE = 1.0."""
        oee = calc.oee(
            running_time=480.0,
            planned_time=480.0,
            ideal_cycle_time=60.0,
            total_output=8,
            good_output=8,
        )
        assert oee == pytest.approx(1.0)

    def test_world_class_oee(self, calc):
        """A typical world-class OEE scenario (~85%)."""
        oee = calc.oee(
            running_time=432.0,    # 90% of 480
            planned_time=480.0,
            ideal_cycle_time=60.0,
            total_output=7,        # perf = 60*7/432 ~ 0.972
            good_output=7,         # 100% quality
        )
        assert 0.8 < oee <= 1.0, f"Expected world-class OEE, got {oee}"

    def test_all_zeros(self, calc):
        """All zeros should produce OEE = 0.0, not raise an error."""
        oee = calc.oee(
            running_time=0.0,
            planned_time=0.0,
            ideal_cycle_time=0.0,
            total_output=0,
            good_output=0,
        )
        assert oee == pytest.approx(0.0), "All-zero inputs should yield OEE = 0.0"

    def test_zero_quality_zeros_oee(self, calc):
        """Zero quality should zero out the entire OEE."""
        oee = calc.oee(
            running_time=480.0,
            planned_time=480.0,
            ideal_cycle_time=60.0,
            total_output=8,
            good_output=0,
        )
        assert oee == pytest.approx(0.0), "Zero quality should yield OEE = 0.0"

    def test_oee_decomposition(self, calc):
        """OEE should equal Availability * Performance * Quality."""
        a = calc.availability(400.0, 480.0)
        p = calc.performance(60.0, 6, 400.0)
        q = calc.quality(5, 6)
        expected = a * p * q
        actual = calc.oee(400.0, 480.0, 60.0, 6, 5)
        assert actual == pytest.approx(expected)
