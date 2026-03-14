"""Tests for WorkholdingSetup, WorkholdingAnalysis, and WorkholdingAnalyzer."""
import sys
from unittest.mock import MagicMock

# Mock ROS2 and related modules before importing our code
for mod in (
    'rclpy', 'rclpy.lifecycle', 'rclpy.node', 'rclpy.qos',
    'rclpy.parameter', 'rclpy.callback_groups', 'rclpy.executors',
    'std_msgs', 'std_msgs.msg',
):
    sys.modules.setdefault(mod, MagicMock())

# Mock miracle_core and miracle_msgs submodules (not top-level)
for mod in (
    'miracle_core.gcode_parser', 'miracle_core.tool_library',
    'miracle_msgs', 'miracle_msgs.msg', 'miracle_msgs.srv',
):
    if mod in sys.modules:
        existing = sys.modules[mod]
        if not hasattr(existing, '__path__'):
            setattr(existing, '__path__', [])
    else:
        sys.modules[mod] = MagicMock()

import math
import pytest

from miracle_twin.cutting_sim_proxy import (
    WorkholdingSetup,
    WorkholdingAnalysis,
    WorkholdingAnalyzer,
)

GRAVITY = 9.81


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def analyzer():
    return WorkholdingAnalyzer()


@pytest.fixture
def vise_setup():
    """Standard vise with steel jaws."""
    return WorkholdingSetup(
        setup_type='VISE',
        clamping_force_n=5000.0,
        friction_coefficient=0.15,
        num_clamp_points=2,
        workpiece_mass_kg=5.0,
        workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        safety_factor=2.0,
    )


@pytest.fixture
def vacuum_setup():
    return WorkholdingSetup(
        setup_type='VACUUM',
        clamping_force_n=800.0,
        friction_coefficient=0.4,
        num_clamp_points=1,
        workpiece_mass_kg=2.0,
        workpiece_dimensions_mm=(200.0, 200.0, 10.0),
        safety_factor=2.0,
    )


@pytest.fixture
def fixture_plate_setup():
    return WorkholdingSetup(
        setup_type='FIXTURE_PLATE',
        clamping_force_n=8000.0,
        friction_coefficient=0.25,
        num_clamp_points=4,
        workpiece_mass_kg=20.0,
        workpiece_dimensions_mm=(300.0, 200.0, 100.0),
        safety_factor=2.0,
    )


@pytest.fixture
def moderate_forces():
    return {'Fx': 500.0, 'Fy': 400.0, 'Fz': 200.0}


@pytest.fixture
def high_forces():
    return {'Fx': 3000.0, 'Fy': 2500.0, 'Fz': 1500.0}


# ---------------------------------------------------------------------------
# 1. Vise setup with adequate force
# ---------------------------------------------------------------------------

class TestViseAdequate:
    def test_vise_secure_with_moderate_forces(self, analyzer, vise_setup, moderate_forces):
        result = analyzer.analyze(vise_setup, moderate_forces)
        assert result.is_secure is True
        assert result.safety_margin >= vise_setup.safety_factor

    def test_actual_holding_force_calculation(self, analyzer, vise_setup):
        forces = {'Fx': 100.0, 'Fy': 50.0, 'Fz': 10.0}
        result = analyzer.analyze(vise_setup, forces)
        expected_lateral = 5000.0 * 0.15 * 2  # 1500 N
        assert result.actual_holding_force_n == pytest.approx(expected_lateral, rel=0.01)


# ---------------------------------------------------------------------------
# 2. Insufficient clamping → not secure
# ---------------------------------------------------------------------------

class TestInsufficientClamping:
    def test_weak_clamp_not_secure(self, analyzer):
        weak = WorkholdingSetup(
            setup_type='VISE',
            clamping_force_n=100.0,
            friction_coefficient=0.15,
            num_clamp_points=2,
            workpiece_mass_kg=1.0,
            workpiece_dimensions_mm=(100.0, 80.0, 50.0),
            safety_factor=2.0,
        )
        forces = {'Fx': 500.0, 'Fy': 400.0, 'Fz': 200.0}
        result = analyzer.analyze(weak, forces)
        assert result.is_secure is False
        assert result.safety_margin < 2.0

    def test_recommendations_present_when_insecure(self, analyzer):
        weak = WorkholdingSetup(
            setup_type='VISE',
            clamping_force_n=50.0,
            friction_coefficient=0.15,
            num_clamp_points=1,
            workpiece_mass_kg=1.0,
            workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        result = analyzer.analyze(weak, {'Fx': 1000.0, 'Fy': 0.0, 'Fz': 0.0})
        assert len(result.recommendations) > 0
        assert any('clamping force' in r.lower() for r in result.recommendations)


# ---------------------------------------------------------------------------
# 3. Lift-off risk detection
# ---------------------------------------------------------------------------

class TestLiftOffRisk:
    def test_high_axial_force_causes_liftoff(self, analyzer):
        setup = WorkholdingSetup(
            setup_type='VISE',
            clamping_force_n=500.0,
            friction_coefficient=0.15,
            num_clamp_points=2,
            workpiece_mass_kg=1.0,
            workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        # Vertical hold = 500*2 + 1*9.81 = 1009.81
        forces = {'Fx': 0.0, 'Fy': 0.0, 'Fz': 1200.0}
        result = analyzer.analyze(setup, forces)
        assert result.lift_off_risk is True
        assert any('lift-off' in r.lower() for r in result.recommendations)

    def test_no_liftoff_when_axial_low(self, analyzer, vise_setup):
        forces = {'Fx': 500.0, 'Fy': 400.0, 'Fz': 50.0}
        result = analyzer.analyze(vise_setup, forces)
        assert result.lift_off_risk is False


# ---------------------------------------------------------------------------
# 4. Rotation risk detection
# ---------------------------------------------------------------------------

class TestRotationRisk:
    def test_high_lateral_force_causes_rotation_risk(self, analyzer):
        setup = WorkholdingSetup(
            setup_type='VISE',
            clamping_force_n=200.0,
            friction_coefficient=0.1,
            num_clamp_points=1,
            workpiece_mass_kg=1.0,
            workpiece_dimensions_mm=(300.0, 50.0, 50.0),
        )
        # Cutting torque = 2000 * (300/2000) = 300 Nm
        # Friction torque = 200 * 0.1 * 1 * (50/2000) = 0.5 Nm
        forces = {'Fx': 2000.0, 'Fy': 0.0, 'Fz': 0.0}
        result = analyzer.analyze(setup, forces)
        assert result.rotation_risk is True
        assert any('rotation' in r.lower() for r in result.recommendations)

    def test_no_rotation_with_strong_clamp(self, analyzer, fixture_plate_setup):
        forces = {'Fx': 100.0, 'Fy': 50.0, 'Fz': 10.0}
        result = analyzer.analyze(fixture_plate_setup, forces)
        assert result.rotation_risk is False


# ---------------------------------------------------------------------------
# 5. Safety margin calculation
# ---------------------------------------------------------------------------

class TestSafetyMargin:
    def test_safety_margin_ratio(self, analyzer, vise_setup):
        forces = {'Fx': 300.0, 'Fy': 200.0, 'Fz': 50.0}
        result = analyzer.analyze(vise_setup, forces)
        # Lateral holding = 5000 * 0.15 * 2 = 1500
        # Max cutting = 300, margin = 1500/300 = 5.0
        assert result.safety_margin == pytest.approx(5.0, rel=0.01)

    def test_safety_margin_below_factor_means_insecure(self, analyzer):
        setup = WorkholdingSetup(
            setup_type='VISE',
            clamping_force_n=200.0,
            friction_coefficient=0.15,
            num_clamp_points=2,
            workpiece_mass_kg=1.0,
            workpiece_dimensions_mm=(100.0, 80.0, 50.0),
            safety_factor=3.0,
        )
        forces = {'Fx': 50.0, 'Fy': 30.0, 'Fz': 10.0}
        result = analyzer.analyze(setup, forces)
        # Lateral = 200*0.15*2 = 60, margin = 60/50 = 1.2 < 3.0
        assert result.is_secure is False
        assert result.safety_margin < 3.0


# ---------------------------------------------------------------------------
# 6. Different setup types (vacuum weaker than vise)
# ---------------------------------------------------------------------------

class TestSetupTypes:
    def test_vacuum_weaker_than_vise(self, analyzer, vise_setup, vacuum_setup):
        forces = {'Fx': 300.0, 'Fy': 200.0, 'Fz': 100.0}
        vise_result = analyzer.analyze(vise_setup, forces)
        vacuum_result = analyzer.analyze(vacuum_setup, forces)
        assert vise_result.safety_margin > vacuum_result.safety_margin

    def test_vacuum_warns_high_forces(self, analyzer, vacuum_setup):
        forces = {'Fx': 600.0, 'Fy': 400.0, 'Fz': 200.0}
        result = analyzer.analyze(vacuum_setup, forces)
        assert any('vacuum' in r.lower() for r in result.recommendations)

    def test_magnetic_setup(self, analyzer):
        setup = WorkholdingSetup(
            setup_type='MAGNETIC',
            clamping_force_n=3000.0,
            friction_coefficient=0.3,
            num_clamp_points=1,
            workpiece_mass_kg=10.0,
            workpiece_dimensions_mm=(200.0, 150.0, 30.0),
        )
        forces = {'Fx': 200.0, 'Fy': 100.0, 'Fz': 50.0}
        result = analyzer.analyze(setup, forces)
        # Lateral = 3000*0.3*1 = 900, margin = 900/200 = 4.5
        assert result.safety_margin >= 2.0

    def test_chuck_3jaw(self, analyzer):
        setup = WorkholdingSetup(
            setup_type='CHUCK_3JAW',
            clamping_force_n=4000.0,
            friction_coefficient=0.2,
            num_clamp_points=3,
            workpiece_mass_kg=3.0,
            workpiece_dimensions_mm=(80.0, 80.0, 120.0),
        )
        forces = {'Fx': 800.0, 'Fy': 600.0, 'Fz': 300.0}
        result = analyzer.analyze(setup, forces)
        # Lateral = 4000*0.2*3 = 2400, margin = 2400/800 = 3.0
        assert result.safety_margin == pytest.approx(3.0, rel=0.01)


# ---------------------------------------------------------------------------
# 7. Recommend setup for given forces
# ---------------------------------------------------------------------------

class TestRecommendSetup:
    def test_low_force_recommends_vacuum(self, analyzer):
        forces = {'Fx': 50.0, 'Fy': 30.0, 'Fz': 20.0}
        setup = analyzer.recommend_setup(forces, workpiece_mass_kg=1.0)
        assert setup.setup_type == 'VACUUM'

    def test_moderate_force_recommends_vise(self, analyzer):
        forces = {'Fx': 800.0, 'Fy': 600.0, 'Fz': 300.0}
        setup = analyzer.recommend_setup(forces, workpiece_mass_kg=5.0)
        assert setup.setup_type == 'VISE'

    def test_high_force_recommends_fixture_plate(self, analyzer):
        forces = {'Fx': 5000.0, 'Fy': 3000.0, 'Fz': 2000.0}
        setup = analyzer.recommend_setup(forces, workpiece_mass_kg=20.0)
        assert setup.setup_type == 'FIXTURE_PLATE'

    def test_recommended_setup_is_secure(self, analyzer):
        forces = {'Fx': 1000.0, 'Fy': 800.0, 'Fz': 500.0}
        setup = analyzer.recommend_setup(forces, workpiece_mass_kg=5.0)
        result = analyzer.analyze(setup, forces)
        assert result.is_secure is True


# ---------------------------------------------------------------------------
# 8. Max safe depth calculation
# ---------------------------------------------------------------------------

class TestMaxSafeDepth:
    def test_basic_max_depth(self, analyzer, vise_setup):
        # Holding = 5000*0.15*2 = 1500, safe = 1500/2.0 = 750
        # ap = 750 / (2000 * 0.1) = 3.75 mm
        depth = analyzer.get_max_safe_depth(vise_setup, feed_mm_per_tooth=0.1, speed_rpm=1000)
        assert depth == pytest.approx(3.75, rel=0.01)

    def test_higher_feed_reduces_max_depth(self, analyzer, vise_setup):
        d1 = analyzer.get_max_safe_depth(vise_setup, feed_mm_per_tooth=0.1, speed_rpm=1000)
        d2 = analyzer.get_max_safe_depth(vise_setup, feed_mm_per_tooth=0.2, speed_rpm=1000)
        assert d2 < d1

    def test_harder_material_reduces_max_depth(self, analyzer, vise_setup):
        d_steel = analyzer.get_max_safe_depth(
            vise_setup, feed_mm_per_tooth=0.1, speed_rpm=1000, material_kc=2000
        )
        d_titanium = analyzer.get_max_safe_depth(
            vise_setup, feed_mm_per_tooth=0.1, speed_rpm=1000, material_kc=3500
        )
        assert d_titanium < d_steel

    def test_zero_feed_returns_zero(self, analyzer, vise_setup):
        depth = analyzer.get_max_safe_depth(vise_setup, feed_mm_per_tooth=0.0, speed_rpm=1000)
        assert depth == 0.0


# ---------------------------------------------------------------------------
# 9. High friction coefficient → more holding force
# ---------------------------------------------------------------------------

class TestFrictionEffect:
    def test_soft_jaws_more_holding(self, analyzer):
        steel_jaws = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=5000.0,
            friction_coefficient=0.15, num_clamp_points=2,
            workpiece_mass_kg=5.0, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        soft_jaws = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=5000.0,
            friction_coefficient=0.3, num_clamp_points=2,
            workpiece_mass_kg=5.0, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        forces = {'Fx': 500.0, 'Fy': 300.0, 'Fz': 100.0}
        r_steel = analyzer.analyze(steel_jaws, forces)
        r_soft = analyzer.analyze(soft_jaws, forces)
        assert r_soft.safety_margin > r_steel.safety_margin

    def test_higher_friction_increases_max_depth(self, analyzer):
        low_fric = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=5000.0,
            friction_coefficient=0.1, num_clamp_points=2,
            workpiece_mass_kg=5.0, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        high_fric = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=5000.0,
            friction_coefficient=0.3, num_clamp_points=2,
            workpiece_mass_kg=5.0, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        d_low = analyzer.get_max_safe_depth(low_fric, 0.1, 1000)
        d_high = analyzer.get_max_safe_depth(high_fric, 0.1, 1000)
        assert d_high > d_low


# ---------------------------------------------------------------------------
# 10. Multiple clamp points
# ---------------------------------------------------------------------------

class TestMultipleClamps:
    def test_more_clamps_more_secure(self, analyzer):
        two_clamps = WorkholdingSetup(
            setup_type='FIXTURE_PLATE', clamping_force_n=3000.0,
            friction_coefficient=0.25, num_clamp_points=2,
            workpiece_mass_kg=10.0, workpiece_dimensions_mm=(200.0, 150.0, 50.0),
        )
        four_clamps = WorkholdingSetup(
            setup_type='FIXTURE_PLATE', clamping_force_n=3000.0,
            friction_coefficient=0.25, num_clamp_points=4,
            workpiece_mass_kg=10.0, workpiece_dimensions_mm=(200.0, 150.0, 50.0),
        )
        forces = {'Fx': 1000.0, 'Fy': 800.0, 'Fz': 400.0}
        r2 = analyzer.analyze(two_clamps, forces)
        r4 = analyzer.analyze(four_clamps, forces)
        assert r4.safety_margin > r2.safety_margin


# ---------------------------------------------------------------------------
# 11. Heavy workpiece → gravity contribution
# ---------------------------------------------------------------------------

class TestGravityContribution:
    def test_heavy_workpiece_resists_liftoff(self, analyzer):
        light = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=1000.0,
            friction_coefficient=0.15, num_clamp_points=2,
            workpiece_mass_kg=0.5, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        heavy = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=1000.0,
            friction_coefficient=0.15, num_clamp_points=2,
            workpiece_mass_kg=50.0, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        forces = {'Fx': 0.0, 'Fy': 0.0, 'Fz': 2200.0}
        r_light = analyzer.analyze(light, forces)
        r_heavy = analyzer.analyze(heavy, forces)
        # Heavy workpiece has more vertical resistance
        assert r_heavy.actual_holding_force_n > r_light.actual_holding_force_n

    def test_gravity_reduces_required_clamping(self, analyzer):
        """Heavy workpiece needs less clamping force for vertical hold."""
        setup = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=5000.0,
            friction_coefficient=0.15, num_clamp_points=2,
            workpiece_mass_kg=30.0, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        # Pure vertical force
        forces = {'Fx': 0.0, 'Fy': 0.0, 'Fz': 500.0}
        result = analyzer.analyze(setup, forces)
        # Gravity = 30 * 9.81 = 294.3 N helps hold the workpiece
        assert result.is_secure is True


# ---------------------------------------------------------------------------
# 12. Edge cases: zero force, zero clamps
# ---------------------------------------------------------------------------

class TestEdgeCases:
    def test_zero_cutting_force_is_secure(self, analyzer, vise_setup):
        forces = {'Fx': 0.0, 'Fy': 0.0, 'Fz': 0.0}
        result = analyzer.analyze(vise_setup, forces)
        assert result.is_secure is True
        assert result.safety_margin == float('inf')

    def test_zero_clamps_minimal_holding(self, analyzer):
        setup = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=5000.0,
            friction_coefficient=0.15, num_clamp_points=0,
            workpiece_mass_kg=1.0, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        forces = {'Fx': 100.0, 'Fy': 50.0, 'Fz': 10.0}
        result = analyzer.analyze(setup, forces)
        # With 0 clamp points, lateral holding = 0
        assert result.actual_holding_force_n == pytest.approx(0.0, abs=0.1)
        assert result.is_secure is False

    def test_missing_force_components(self, analyzer, vise_setup):
        """Only Fx provided — Fy and Fz default to 0."""
        forces = {'Fx': 200.0}
        result = analyzer.analyze(vise_setup, forces)
        assert result.is_secure is True

    def test_max_depth_zero_material_kc(self, analyzer, vise_setup):
        depth = analyzer.get_max_safe_depth(
            vise_setup, feed_mm_per_tooth=0.1, speed_rpm=1000, material_kc=0.0
        )
        assert depth == 0.0

    def test_critical_direction_vertical(self, analyzer):
        """When Fz dominates, critical direction should be Z."""
        setup = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=500.0,
            friction_coefficient=0.15, num_clamp_points=2,
            workpiece_mass_kg=0.1, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        forces = {'Fx': 10.0, 'Fy': 10.0, 'Fz': 5000.0}
        result = analyzer.analyze(setup, forces)
        assert result.critical_direction == 'Z'

    def test_critical_direction_lateral(self, analyzer, vise_setup):
        """When Fx dominates with moderate vertical hold, critical is X."""
        forces = {'Fx': 1000.0, 'Fy': 100.0, 'Fz': 10.0}
        result = analyzer.analyze(vise_setup, forces)
        assert result.critical_direction == 'X'


# ---------------------------------------------------------------------------
# Additional coverage
# ---------------------------------------------------------------------------

class TestRequiredClampingForce:
    def test_required_force_exceeds_actual_when_insecure(self, analyzer):
        setup = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=100.0,
            friction_coefficient=0.15, num_clamp_points=2,
            workpiece_mass_kg=1.0, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
        )
        forces = {'Fx': 500.0, 'Fy': 300.0, 'Fz': 100.0}
        result = analyzer.analyze(setup, forces)
        assert result.required_clamping_force_n > setup.clamping_force_n

    def test_low_safety_margin_warning(self, analyzer):
        """Safety margin between 1.0 and 1.5 should warn."""
        setup = WorkholdingSetup(
            setup_type='VISE', clamping_force_n=1000.0,
            friction_coefficient=0.15, num_clamp_points=2,
            workpiece_mass_kg=1.0, workpiece_dimensions_mm=(100.0, 80.0, 50.0),
            safety_factor=1.0,  # low safety factor to make it "secure" but marginal
        )
        # Lateral = 1000*0.15*2 = 300
        # margin = 300/250 = 1.2 (>1.0, <1.5)
        forces = {'Fx': 250.0, 'Fy': 100.0, 'Fz': 10.0}
        result = analyzer.analyze(setup, forces)
        assert result.is_secure is True
        assert any('low' in r.lower() for r in result.recommendations)


class TestWorkholdingDataclasses:
    def test_workholding_setup_valid_types(self):
        assert 'VISE' in WorkholdingSetup.VALID_TYPES
        assert 'CHUCK_3JAW' in WorkholdingSetup.VALID_TYPES
        assert 'VACUUM' in WorkholdingSetup.VALID_TYPES
        assert len(WorkholdingSetup.VALID_TYPES) == 6

    def test_analysis_default_recommendations(self):
        analysis = WorkholdingAnalysis(
            is_secure=True, safety_margin=5.0,
            required_clamping_force_n=100.0, actual_holding_force_n=500.0,
            critical_direction='X', max_cutting_force_n=100.0,
            lift_off_risk=False, rotation_risk=False,
        )
        assert analysis.recommendations == []
