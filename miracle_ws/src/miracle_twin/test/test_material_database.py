"""Tests for MaterialProperties and MaterialDatabase — workpiece material database."""
import math
import sys
import types
import pytest
from unittest.mock import MagicMock

# ---------------------------------------------------------------------------
# Mock ROS2 modules so we can import miracle_twin without a live ROS2 env
# ---------------------------------------------------------------------------
for mod in (
    'rclpy', 'rclpy.lifecycle', 'rclpy.node', 'rclpy.action',
    'rclpy.action.server', 'rclpy.callback_groups',
    'miracle_core', 'miracle_core.qos_profiles',
    'miracle_core.lifecycle_node_base',
    'miracle_msgs', 'miracle_msgs.msg',
    'miracle_msgs.action', 'miracle_msgs.srv',
):
    sys.modules.setdefault(mod, MagicMock())

from miracle_twin.tool_library import MaterialProperties, MaterialDatabase


# ======================================================================
# Fixtures
# ======================================================================

@pytest.fixture
def db():
    """A fresh MaterialDatabase with built-in materials."""
    return MaterialDatabase()


@pytest.fixture
def custom_material():
    return MaterialProperties(
        material_id='CF-Epoxy',
        name='Carbon Fiber Epoxy Composite',
        category='composite',
        hardness_hrc=0.0,
        tensile_strength_mpa=600.0,
        thermal_conductivity_w_mk=5.0,
        specific_heat_j_kgk=800.0,
        density_kg_m3=1600.0,
        machinability_rating=0.50,
        recommended_speed_sfm=(200.0, 500.0),
        recommended_feed_ipt=(0.002, 0.006),
        taylor_constant_c=400.0,
        taylor_exponent_n=0.20,
        specific_cutting_force_n_mm2=900.0,
        chip_formation='discontinuous',
    )


# ======================================================================
# Test: pre-loaded material retrieval
# ======================================================================

class TestBuiltinMaterials:
    """Verify all 10 pre-loaded materials are present and correct."""

    EXPECTED_IDS = [
        '6061-T6', '7075-T6', '1018', '4140', 'D2',
        '304-SS', '316-SS', 'Ti-6Al-4V', 'Inconel-718', 'PEEK',
    ]

    def test_all_builtins_loaded(self, db):
        for mid in self.EXPECTED_IDS:
            mat = db.get_material(mid)
            assert mat is not None
            assert mat.material_id == mid

    def test_6061_t6_properties(self, db):
        m = db.get_material('6061-T6')
        assert m.name == 'Aluminum 6061-T6'
        assert m.category == 'aluminum'
        assert m.tensile_strength_mpa == 310.0
        assert m.thermal_conductivity_w_mk == 167.0
        assert m.machinability_rating == 0.90

    def test_7075_t6_properties(self, db):
        m = db.get_material('7075-T6')
        assert m.category == 'aluminum'
        assert m.tensile_strength_mpa == 572.0

    def test_1018_properties(self, db):
        m = db.get_material('1018')
        assert m.category == 'steel'
        assert m.chip_formation == 'continuous'

    def test_4140_properties(self, db):
        m = db.get_material('4140')
        assert m.category == 'steel'
        assert m.hardness_hrc == 28.0

    def test_d2_properties(self, db):
        m = db.get_material('D2')
        assert m.category == 'steel'
        assert m.hardness_hrc == 60.0
        assert m.chip_formation == 'segmented'

    def test_304ss_properties(self, db):
        m = db.get_material('304-SS')
        assert m.category == 'stainless'
        assert m.density_kg_m3 == 8000.0

    def test_316ss_properties(self, db):
        m = db.get_material('316-SS')
        assert m.category == 'stainless'
        assert m.thermal_conductivity_w_mk == 14.0

    def test_ti6al4v_properties(self, db):
        m = db.get_material('Ti-6Al-4V')
        assert m.category == 'titanium'
        assert m.chip_formation == 'segmented'

    def test_inconel718_properties(self, db):
        m = db.get_material('Inconel-718')
        assert m.category == 'nickel_alloy'
        assert m.machinability_rating == 0.15

    def test_peek_properties(self, db):
        m = db.get_material('PEEK')
        assert m.category == 'plastic'
        assert m.machinability_rating == 0.95


# ======================================================================
# Test: category filtering
# ======================================================================

class TestCategoryFiltering:

    def test_aluminum_category(self, db):
        mats = db.get_materials_by_category('aluminum')
        assert len(mats) == 2
        ids = {m.material_id for m in mats}
        assert ids == {'6061-T6', '7075-T6'}

    def test_steel_category(self, db):
        mats = db.get_materials_by_category('steel')
        assert len(mats) == 3
        ids = {m.material_id for m in mats}
        assert ids == {'1018', '4140', 'D2'}

    def test_stainless_category(self, db):
        mats = db.get_materials_by_category('stainless')
        assert len(mats) == 2

    def test_titanium_category(self, db):
        mats = db.get_materials_by_category('titanium')
        assert len(mats) == 1
        assert mats[0].material_id == 'Ti-6Al-4V'

    def test_nickel_alloy_category(self, db):
        mats = db.get_materials_by_category('nickel_alloy')
        assert len(mats) == 1

    def test_plastic_category(self, db):
        mats = db.get_materials_by_category('plastic')
        assert len(mats) == 1

    def test_empty_category(self, db):
        mats = db.get_materials_by_category('composite')
        assert len(mats) == 0

    def test_case_insensitive_category(self, db):
        mats = db.get_materials_by_category('Aluminum')
        assert len(mats) == 2


# ======================================================================
# Test: recommended parameters
# ======================================================================

class TestRecommendedParams:

    def test_returns_all_keys(self, db):
        params = db.get_recommended_params('6061-T6', 10.0, 2)
        assert 'rpm' in params
        assert 'feed_mmpm' in params
        assert 'depth_mm' in params
        assert 'width_mm' in params

    def test_rpm_is_positive(self, db):
        params = db.get_recommended_params('6061-T6', 10.0, 2)
        assert params['rpm'] > 0

    def test_aluminum_higher_rpm_than_steel(self, db):
        al_params = db.get_recommended_params('6061-T6', 10.0, 2)
        st_params = db.get_recommended_params('4140', 10.0, 2)
        assert al_params['rpm'] > st_params['rpm']

    def test_aluminum_depth_equals_diameter(self, db):
        params = db.get_recommended_params('6061-T6', 10.0, 2)
        assert params['depth_mm'] == 10.0

    def test_steel_depth_half_diameter(self, db):
        params = db.get_recommended_params('4140', 10.0, 2)
        assert params['depth_mm'] == 5.0

    def test_width_half_diameter(self, db):
        params = db.get_recommended_params('6061-T6', 10.0, 2)
        assert params['width_mm'] == 5.0

    def test_more_flutes_higher_feed(self, db):
        p2 = db.get_recommended_params('6061-T6', 10.0, 2)
        p4 = db.get_recommended_params('6061-T6', 10.0, 4)
        assert p4['feed_mmpm'] > p2['feed_mmpm']

    def test_zero_diameter_raises(self, db):
        with pytest.raises(ValueError):
            db.get_recommended_params('6061-T6', 0.0, 2)

    def test_negative_diameter_raises(self, db):
        with pytest.raises(ValueError):
            db.get_recommended_params('6061-T6', -5.0, 2)

    def test_zero_flutes_raises(self, db):
        with pytest.raises(ValueError):
            db.get_recommended_params('6061-T6', 10.0, 0)


# ======================================================================
# Test: Taylor tool life
# ======================================================================

class TestTaylorToolLife:

    def test_positive_life(self, db):
        life = db.get_taylor_life('6061-T6', 200.0, 0.1)
        assert life > 0.0

    def test_higher_speed_shorter_life(self, db):
        life_slow = db.get_taylor_life('6061-T6', 100.0, 0.1)
        life_fast = db.get_taylor_life('6061-T6', 300.0, 0.1)
        assert life_slow > life_fast

    def test_higher_feed_shorter_life(self, db):
        life_low = db.get_taylor_life('6061-T6', 200.0, 0.05)
        life_high = db.get_taylor_life('6061-T6', 200.0, 0.2)
        assert life_low > life_high

    def test_zero_speed_returns_zero(self, db):
        assert db.get_taylor_life('6061-T6', 0.0, 0.1) == 0.0

    def test_zero_feed_returns_zero(self, db):
        assert db.get_taylor_life('6061-T6', 200.0, 0.0) == 0.0

    def test_inconel_shorter_life_than_aluminum(self, db):
        life_al = db.get_taylor_life('6061-T6', 100.0, 0.1)
        life_in = db.get_taylor_life('Inconel-718', 100.0, 0.1)
        assert life_al > life_in


# ======================================================================
# Test: material comparison
# ======================================================================

class TestMaterialComparison:

    def test_compare_returns_all_keys(self, db):
        result = db.compare_materials('6061-T6', '4140')
        assert 'machinability_ratio' in result
        assert 'speed_ratio' in result
        assert 'feed_ratio' in result
        assert 'cutting_force_ratio' in result

    def test_aluminum_more_machinable_than_titanium(self, db):
        result = db.compare_materials('6061-T6', 'Ti-6Al-4V')
        assert result['machinability_ratio'] > 1.0

    def test_aluminum_higher_speed_than_inconel(self, db):
        result = db.compare_materials('6061-T6', 'Inconel-718')
        assert result['speed_ratio'] > 1.0

    def test_aluminum_lower_cutting_force_than_steel(self, db):
        result = db.compare_materials('6061-T6', '4140')
        assert result['cutting_force_ratio'] < 1.0

    def test_same_material_ratios_are_one(self, db):
        result = db.compare_materials('6061-T6', '6061-T6')
        assert abs(result['machinability_ratio'] - 1.0) < 1e-9
        assert abs(result['speed_ratio'] - 1.0) < 1e-9


# ======================================================================
# Test: custom material
# ======================================================================

class TestCustomMaterial:

    def test_add_and_retrieve(self, db, custom_material):
        db.add_custom_material(custom_material)
        retrieved = db.get_material('CF-Epoxy')
        assert retrieved.name == 'Carbon Fiber Epoxy Composite'
        assert retrieved.category == 'composite'

    def test_custom_in_category_filter(self, db, custom_material):
        db.add_custom_material(custom_material)
        composites = db.get_materials_by_category('composite')
        assert len(composites) == 1

    def test_custom_overwrite(self, db, custom_material):
        db.add_custom_material(custom_material)
        modified = MaterialProperties(
            material_id='CF-Epoxy', name='Modified CF',
            category='composite', hardness_hrc=1.0,
            tensile_strength_mpa=700.0,
            thermal_conductivity_w_mk=6.0,
            specific_heat_j_kgk=850.0,
            density_kg_m3=1650.0,
            machinability_rating=0.45,
            recommended_speed_sfm=(180.0, 480.0),
            recommended_feed_ipt=(0.002, 0.005),
            taylor_constant_c=380.0, taylor_exponent_n=0.19,
            specific_cutting_force_n_mm2=950.0,
            chip_formation='discontinuous',
        )
        db.add_custom_material(modified)
        assert db.get_material('CF-Epoxy').name == 'Modified CF'


# ======================================================================
# Test: search
# ======================================================================

class TestMaterialSearch:

    def test_search_by_id_substring(self, db):
        results = db.search_materials('6061')
        assert len(results) == 1
        assert results[0].material_id == '6061-T6'

    def test_search_by_name_substring(self, db):
        results = db.search_materials('Stainless')
        assert len(results) == 2

    def test_search_case_insensitive(self, db):
        results = db.search_materials('peek')
        assert len(results) == 1

    def test_search_no_match(self, db):
        results = db.search_materials('Unobtanium')
        assert len(results) == 0

    def test_search_broad_match(self, db):
        results = db.search_materials('Steel')
        # 1018, 4140, D2 (name has "Steel"), 304-SS and 316-SS (name has "Steel")
        ids = {m.material_id for m in results}
        assert '1018' in ids
        assert '4140' in ids
        assert '304-SS' in ids


# ======================================================================
# Test: unknown material handling
# ======================================================================

class TestUnknownMaterial:

    def test_get_unknown_raises_keyerror(self, db):
        with pytest.raises(KeyError, match="Unknown material"):
            db.get_material('Unobtanium')

    def test_recommended_params_unknown_raises(self, db):
        with pytest.raises(KeyError):
            db.get_recommended_params('Unobtanium', 10.0, 2)

    def test_taylor_life_unknown_raises(self, db):
        with pytest.raises(KeyError):
            db.get_taylor_life('Unobtanium', 200.0, 0.1)

    def test_compare_unknown_raises(self, db):
        with pytest.raises(KeyError):
            db.compare_materials('6061-T6', 'Unobtanium')


# ======================================================================
# Test: machinability ordering and thermal conductivity ranges
# ======================================================================

class TestMachinabilityOrdering:

    def test_peek_most_machinable(self, db):
        """PEEK should have the highest machinability rating."""
        all_mats = [db.get_material(mid) for mid in [
            '6061-T6', '7075-T6', '1018', '4140', 'D2',
            '304-SS', '316-SS', 'Ti-6Al-4V', 'Inconel-718', 'PEEK',
        ]]
        best = max(all_mats, key=lambda m: m.machinability_rating)
        assert best.material_id == 'PEEK'

    def test_inconel_least_machinable(self, db):
        all_mats = [db.get_material(mid) for mid in [
            '6061-T6', '7075-T6', '1018', '4140', 'D2',
            '304-SS', '316-SS', 'Ti-6Al-4V', 'Inconel-718', 'PEEK',
        ]]
        worst = min(all_mats, key=lambda m: m.machinability_rating)
        assert worst.material_id == 'Inconel-718'

    def test_aluminum_higher_machinability_than_steel(self, db):
        al = db.get_material('6061-T6')
        st = db.get_material('4140')
        assert al.machinability_rating > st.machinability_rating


class TestThermalConductivity:

    def test_aluminum_highest_conductivity(self, db):
        al = db.get_material('6061-T6')
        ti = db.get_material('Ti-6Al-4V')
        assert al.thermal_conductivity_w_mk > ti.thermal_conductivity_w_mk

    def test_plastic_lowest_conductivity(self, db):
        peek = db.get_material('PEEK')
        assert peek.thermal_conductivity_w_mk < 1.0

    def test_titanium_low_conductivity(self, db):
        ti = db.get_material('Ti-6Al-4V')
        assert ti.thermal_conductivity_w_mk < 10.0
