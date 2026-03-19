"""Tests for the LoadSheddingManager.

Covers service registration, load tracking, level classification,
shedding evaluation, shed/restore operations, and history.
"""

import sys
import time
from unittest.mock import MagicMock

for mod in ['miracle_core.datatypes', 'miracle_core.constants',
            'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
            'rclpy.callback_groups',
            'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_msgs', 'miracle_msgs.msg',
            'std_msgs', 'std_msgs.msg']:
    sys.modules.setdefault(mod, MagicMock())

import pytest

from miracle_resiliency.recovery_orchestrator import (
    LoadLevel,
    LoadSheddingManager,
    SheddableService,
    SheddingDecision,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _svc(service_id: str, priority: int = 5, load: float = 50.0,
         can_shed: bool = True, shed_amount: float = 20.0) -> SheddableService:
    return SheddableService(
        service_id=service_id,
        priority=priority,
        current_load_pct=load,
        can_shed=can_shed,
        shed_amount_pct=shed_amount,
    )


def _manager_with_services(*services: SheddableService) -> LoadSheddingManager:
    mgr = LoadSheddingManager()
    for s in services:
        mgr.register_service(s)
    return mgr


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestRegistrationAndLoad:
    """Tests for register_service, update_load, and get_system_load."""

    def test_register_and_system_load(self):
        """Registering services and computing weighted average load."""
        mgr = _manager_with_services(
            _svc('a', priority=2, load=40.0),
            _svc('b', priority=8, load=80.0),
        )
        # Weighted average: (40*2 + 80*8) / (2+8) = (80+640)/10 = 72.0
        assert abs(mgr.get_system_load() - 72.0) < 0.01

    def test_update_load(self):
        """update_load modifies the service's current load."""
        mgr = _manager_with_services(_svc('x', priority=5, load=30.0))
        mgr.update_load('x', 90.0)
        assert abs(mgr.get_system_load() - 90.0) < 0.01

    def test_update_load_unknown_service_raises(self):
        """Updating load for an unregistered service raises KeyError."""
        mgr = LoadSheddingManager()
        with pytest.raises(KeyError):
            mgr.update_load('nonexistent', 50.0)

    def test_register_invalid_priority_raises(self):
        """Registering a service with out-of-range priority raises ValueError."""
        mgr = LoadSheddingManager()
        with pytest.raises(ValueError):
            mgr.register_service(_svc('bad', priority=0))
        with pytest.raises(ValueError):
            mgr.register_service(_svc('bad2', priority=11))


class TestLoadLevelClassification:
    """Tests for get_load_level thresholds."""

    @pytest.mark.parametrize('load,expected_level', [
        (0.0, LoadLevel.NORMAL),
        (59.9, LoadLevel.NORMAL),
        (60.0, LoadLevel.ELEVATED),
        (74.9, LoadLevel.ELEVATED),
        (75.0, LoadLevel.HIGH),
        (84.9, LoadLevel.HIGH),
        (85.0, LoadLevel.CRITICAL),
        (94.9, LoadLevel.CRITICAL),
        (95.0, LoadLevel.EMERGENCY),
        (100.0, LoadLevel.EMERGENCY),
    ])
    def test_load_level_thresholds(self, load, expected_level):
        """Each threshold boundary maps to the correct LoadLevel."""
        mgr = _manager_with_services(_svc('s', priority=5, load=load))
        assert mgr.get_load_level() == expected_level


class TestEvaluateShedding:
    """Tests for evaluate_shedding decision logic."""

    def test_no_shedding_when_normal(self):
        """No shedding decision when load is below HIGH."""
        mgr = _manager_with_services(_svc('a', priority=5, load=50.0))
        assert mgr.evaluate_shedding() is None

    def test_shedding_returns_decision_when_high(self):
        """Shedding produces a decision when load level >= HIGH."""
        mgr = _manager_with_services(
            _svc('low', priority=1, load=90.0, can_shed=True, shed_amount=40.0),
            _svc('high', priority=9, load=90.0, can_shed=True, shed_amount=10.0),
        )
        decision = mgr.evaluate_shedding()
        assert decision is not None
        assert isinstance(decision, SheddingDecision)
        # Low-priority service should be shed first
        assert decision.services_to_shed[0] == 'low'

    def test_shedding_skips_essential_priority_10(self):
        """Services with priority 10 are never shed."""
        mgr = _manager_with_services(
            _svc('essential', priority=10, load=95.0, can_shed=True, shed_amount=50.0),
            _svc('optional', priority=2, load=95.0, can_shed=True, shed_amount=50.0),
        )
        decision = mgr.evaluate_shedding()
        assert decision is not None
        assert 'essential' not in decision.services_to_shed

    def test_shedding_skips_non_sheddable(self):
        """Services with can_shed=False are never selected for shedding."""
        mgr = _manager_with_services(
            _svc('fixed', priority=1, load=90.0, can_shed=False, shed_amount=50.0),
            _svc('flex', priority=3, load=90.0, can_shed=True, shed_amount=30.0),
        )
        decision = mgr.evaluate_shedding()
        assert decision is not None
        assert 'fixed' not in decision.services_to_shed
        assert 'flex' in decision.services_to_shed


class TestShedAndRestore:
    """Tests for shed() and restore() operations."""

    def test_shed_reduces_load(self):
        """shed() reduces the service's current load."""
        mgr = _manager_with_services(_svc('s', priority=5, load=90.0))
        mgr.shed('s', 30.0)
        assert abs(mgr.get_system_load() - 60.0) < 0.01

    def test_restore_returns_original_load(self):
        """restore() returns the service to its pre-shed load."""
        mgr = _manager_with_services(_svc('s', priority=5, load=90.0))
        mgr.shed('s', 30.0)
        mgr.restore('s')
        assert abs(mgr.get_system_load() - 90.0) < 0.01

    def test_shed_clamps_at_zero(self):
        """Shedding more than the current load clamps at 0%."""
        mgr = _manager_with_services(_svc('s', priority=5, load=10.0))
        mgr.shed('s', 50.0)
        assert mgr.get_system_load() == 0.0

    def test_shed_unknown_service_raises(self):
        """Shedding an unregistered service raises KeyError."""
        mgr = LoadSheddingManager()
        with pytest.raises(KeyError):
            mgr.shed('ghost', 10.0)


class TestHistory:
    """Tests for get_shedding_history."""

    def test_history_records_decisions(self):
        """Each evaluate_shedding call that produces a decision is recorded."""
        mgr = _manager_with_services(
            _svc('a', priority=1, load=90.0, can_shed=True, shed_amount=50.0),
            _svc('b', priority=5, load=90.0, can_shed=True, shed_amount=50.0),
        )
        mgr.evaluate_shedding()
        history = mgr.get_shedding_history()
        assert len(history) == 1
        assert isinstance(history[0].timestamp, float)
        assert history[0].total_load_reduction_pct > 0

    def test_history_empty_initially(self):
        """History is empty before any shedding evaluation."""
        mgr = LoadSheddingManager()
        assert mgr.get_shedding_history() == []


class TestEmptyManager:
    """Edge-case tests for an empty manager."""

    def test_system_load_no_services(self):
        """System load is 0 when no services are registered."""
        mgr = LoadSheddingManager()
        assert mgr.get_system_load() == 0.0

    def test_load_level_no_services(self):
        """Load level is NORMAL when no services are registered."""
        mgr = LoadSheddingManager()
        assert mgr.get_load_level() == LoadLevel.NORMAL
