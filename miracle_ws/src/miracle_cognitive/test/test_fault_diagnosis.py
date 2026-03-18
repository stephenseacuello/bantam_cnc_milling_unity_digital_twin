"""Tests for FaultDiagnosisTree decision tree classifier."""

import sys
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

import pytest
from miracle_cognitive.knowledge.reasoning_engine import (
    DiagnosisNode,
    DiagnosisResult,
    FaultDiagnosisTree,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_tree() -> FaultDiagnosisTree:
    """Return a FaultDiagnosisTree populated with the default CNC tree."""
    tree = FaultDiagnosisTree()
    tree.build_default_tree()
    return tree


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestFaultDiagnosisTreeConstruction:
    """Tests for tree construction and node management."""

    def test_add_node_and_set_root(self):
        tree = FaultDiagnosisTree()
        leaf = DiagnosisNode(
            node_id='leaf1', question='', feature='', threshold=0.0,
            diagnosis='TestFault', confidence=0.9, recommended_action='Fix it',
        )
        tree.add_node(leaf)
        tree.set_root('leaf1')
        result = tree.diagnose({})
        assert result.diagnosis == 'TestFault'
        assert result.confidence == 0.9

    def test_set_root_missing_node_raises(self):
        tree = FaultDiagnosisTree()
        with pytest.raises(KeyError, match="not found"):
            tree.set_root('nonexistent')

    def test_diagnose_without_root_raises(self):
        tree = FaultDiagnosisTree()
        with pytest.raises(RuntimeError, match="No root"):
            tree.diagnose({})


class TestDefaultTreeDiagnoses:
    """Tests that the default CNC decision tree returns correct diagnoses."""

    def test_chatter_high_vibration_high_force(self):
        tree = _default_tree()
        result = tree.diagnose({'vibration': 8.0, 'cutting_force': 1000.0})
        assert result.diagnosis == 'Chatter'
        assert result.confidence == 0.90
        assert 'vibration' in result.features_checked
        assert 'cutting_force' in result.features_checked

    def test_bearing_wear_high_vibration_low_force(self):
        tree = _default_tree()
        result = tree.diagnose({'vibration': 7.0, 'cutting_force': 500.0})
        assert result.diagnosis == 'Bearing Wear'
        assert result.confidence == 0.80

    def test_coolant_failure_high_temp_low_coolant_flow(self):
        """High temperature and coolant flow <= threshold -> Coolant System Failure."""
        tree = _default_tree()
        result = tree.diagnose({
            'vibration': 2.0,
            'temperature': 75.0,
            'coolant_flow': 4.0,  # <= 5.0 -> no_child -> Coolant System Failure
        })
        assert result.diagnosis == 'Coolant System Failure'

    def test_thermal_drift_high_temp_adequate_coolant_flow(self):
        """High temperature and coolant flow > threshold -> Thermal Drift."""
        tree = _default_tree()
        result = tree.diagnose({
            'vibration': 3.0,
            'temperature': 70.0,
            'coolant_flow': 8.0,  # > 5.0 -> yes_child -> Thermal Drift
        })
        assert result.diagnosis == 'Thermal Drift'
        assert result.confidence == 0.75

    def test_tool_wear_high_roughness_low_feed(self):
        tree = _default_tree()
        result = tree.diagnose({
            'vibration': 2.0,
            'temperature': 40.0,
            'surface_roughness': 4.5,
            'feed_rate': 300.0,
        })
        assert result.diagnosis == 'Tool Wear'
        assert result.confidence == 0.85
        assert result.recommended_action == 'Replace worn tool insert'

    def test_feed_rate_issue_high_roughness_high_feed(self):
        tree = _default_tree()
        result = tree.diagnose({
            'vibration': 2.0,
            'temperature': 40.0,
            'surface_roughness': 5.0,
            'feed_rate': 600.0,
        })
        assert result.diagnosis == 'Feed Rate Issue'

    def test_normal_operation(self):
        tree = _default_tree()
        result = tree.diagnose({
            'vibration': 1.0,
            'temperature': 30.0,
            'surface_roughness': 1.5,
        })
        assert result.diagnosis == 'Normal Operation'
        assert result.confidence == 0.95
        assert result.recommended_action == 'No action required'

    def test_path_recorded(self):
        tree = _default_tree()
        result = tree.diagnose({'vibration': 8.0, 'cutting_force': 1000.0})
        assert result.path[0] == 'root'
        assert 'diag_chatter' in result.path
        assert len(result.path) == 3


class TestGetAllDiagnoses:
    """Tests for get_all_diagnoses()."""

    def test_default_tree_diagnoses(self):
        tree = _default_tree()
        diagnoses = tree.get_all_diagnoses()
        assert 'Chatter' in diagnoses
        assert 'Bearing Wear' in diagnoses
        assert 'Normal Operation' in diagnoses
        assert 'Tool Wear' in diagnoses
        assert 'Thermal Drift' in diagnoses
        assert 'Feed Rate Issue' in diagnoses
        assert 'Coolant System Failure' in diagnoses
        # Sorted
        assert diagnoses == sorted(diagnoses)

    def test_empty_tree(self):
        tree = FaultDiagnosisTree()
        assert tree.get_all_diagnoses() == []


class TestValidateTree:
    """Tests for validate_tree()."""

    def test_valid_default_tree(self):
        tree = _default_tree()
        errors = tree.validate_tree()
        assert errors == [], f"Unexpected errors: {errors}"

    def test_missing_root(self):
        tree = FaultDiagnosisTree()
        errors = tree.validate_tree()
        assert any('No root' in e for e in errors)

    def test_missing_child_reference(self):
        tree = FaultDiagnosisTree()
        node = DiagnosisNode(
            node_id='n1', question='Q?', feature='x', threshold=1.0,
            yes_child='missing_node', no_child=None,
        )
        tree.add_node(node)
        tree.set_root('n1')
        errors = tree.validate_tree()
        assert any('missing_node' in e for e in errors)

    def test_cycle_detection(self):
        tree = FaultDiagnosisTree()
        n1 = DiagnosisNode(
            node_id='a', question='Q?', feature='x', threshold=1.0,
            yes_child='b', no_child='b',
        )
        n2 = DiagnosisNode(
            node_id='b', question='Q2?', feature='y', threshold=2.0,
            yes_child='a', no_child='a',
        )
        tree.add_node(n1)
        tree.add_node(n2)
        tree.set_root('a')
        errors = tree.validate_tree()
        assert any('Cycle' in e for e in errors)


class TestMissingFeature:
    """Test behaviour when a required feature is absent from the input dict."""

    def test_missing_feature_takes_no_branch(self):
        """When a feature is missing, the no-child branch is followed."""
        tree = _default_tree()
        # vibration missing -> treated as <= threshold -> no_child (check_temp)
        # temperature missing -> no_child (check_roughness)
        # surface_roughness missing -> no_child (diag_normal)
        result = tree.diagnose({})
        assert result.diagnosis == 'Normal Operation'
        # All checked features should be None
        for val in result.features_checked.values():
            assert val is None
