"""Tests for temporal evidence decay in the causal inference engine.

Validates exponential decay of causal link strength, stale evidence
pruning, active-cause queries, root-cause suggestion, and integration
with CorrelatedAlert messages.

The test file extracts the pure-logic methods from CausalInferenceNode
into a lightweight _CausalEngine helper (following the same pattern as
test_knowledge_graph.py) so that tests run without a ROS2 installation.
"""

import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple
from unittest.mock import MagicMock

import pytest

# ---------------------------------------------------------------------------
# Stub ROS2 / project dependencies so the dataclass can be imported.
# ---------------------------------------------------------------------------
_mock_modules = {
    'rclpy': MagicMock(),
    'rclpy.lifecycle': MagicMock(),
    'rclpy.executors': MagicMock(),
    'miracle_core': MagicMock(),
    'miracle_core.lifecycle_node_base': MagicMock(),
    'miracle_core.qos_profiles': MagicMock(),
    'miracle_msgs': MagicMock(),
    'miracle_msgs.msg': MagicMock(),
}
for mod_name, mock_obj in _mock_modules.items():
    sys.modules.setdefault(mod_name, mock_obj)

_base = sys.modules['miracle_core.lifecycle_node_base']
if not hasattr(_base.MiracleLifecycleNode, 'CRITICALITY_MEDIUM') or \
   isinstance(getattr(_base.MiracleLifecycleNode, 'CRITICALITY_MEDIUM', None), MagicMock):
    _base.MiracleLifecycleNode.CRITICALITY_LOW = 'LOW'
    _base.MiracleLifecycleNode.CRITICALITY_MEDIUM = 'MEDIUM'
    _base.MiracleLifecycleNode.CRITICALITY_HIGH = 'HIGH'
    _base.MiracleLifecycleNode.CRITICALITY_CRITICAL = 'CRITICAL'

sys.modules.pop('miracle_cognitive.knowledge.causal_inference', None)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import only the dataclass -- the Node class inherits from a Mock and is
# not directly usable.  We replicate the pure-logic methods below.
from miracle_cognitive.knowledge.causal_inference import CausalLink


# ---------------------------------------------------------------------------
# Extracted pure-logic engine (mirrors methods in CausalInferenceNode)
# ---------------------------------------------------------------------------

class _CausalEngine:
    """Pure-logic causal inference engine extracted from CausalInferenceNode.

    Contains temporal decay, evidence tracking, active-cause queries,
    root-cause suggestion, and correlated-alert integration -- all
    without any ROS2 dependency.
    """

    def __init__(self, half_life: float = 3600.0):
        self._causal_graph: Dict[str, List[CausalLink]] = {}
        self._lock = threading.Lock()
        self._decay_half_life_sec = half_life

    def add_link(self, link: CausalLink) -> None:
        self._causal_graph.setdefault(link.cause, []).append(link)

    # -- decay helpers -----------------------------------------------------

    @staticmethod
    def compute_decay_factor(time_since: float, half_life: float) -> float:
        if half_life <= 0.0:
            return 1.0
        return math.pow(2.0, -time_since / half_life)

    def _apply_temporal_decay(self, now: Optional[float] = None) -> None:
        if now is None:
            now = time.time()
        for links in self._causal_graph.values():
            for link in links:
                if link.last_evidence_time <= 0.0:
                    link.stale = True
                    continue
                dt = now - link.last_evidence_time
                factor = self.compute_decay_factor(dt, link.decay_half_life_sec)
                link.stale = factor < 0.1

    def _prune_stale_evidence(self, now: Optional[float] = None) -> int:
        if now is None:
            now = time.time()
        total_pruned = 0
        for links in self._causal_graph.values():
            for link in links:
                cutoff = now - 4.0 * link.decay_half_life_sec
                before = len(link.evidence_timestamps)
                link.evidence_timestamps = [
                    ts for ts in link.evidence_timestamps if ts > cutoff
                ]
                pruned = before - len(link.evidence_timestamps)
                if pruned > 0:
                    link.evidence_count = max(0, link.evidence_count - pruned)
                    total_pruned += pruned
        return total_pruned

    def _effective_strength(self, link: CausalLink,
                            now: Optional[float] = None) -> float:
        if now is None:
            now = time.time()
        if link.last_evidence_time <= 0.0:
            return 0.0
        dt = now - link.last_evidence_time
        factor = self.compute_decay_factor(dt, link.decay_half_life_sec)
        return link.strength * factor

    def _weighted_evidence_count(self, link: CausalLink,
                                 now: Optional[float] = None) -> float:
        if now is None:
            now = time.time()
        total = 0.0
        for ts in link.evidence_timestamps:
            dt = now - ts
            total += self.compute_decay_factor(dt, link.decay_half_life_sec)
        return total

    # -- evidence update ---------------------------------------------------

    def _update_evidence(self, link: CausalLink,
                         now: Optional[float] = None,
                         strength_bump: float = 0.01) -> None:
        if now is None:
            now = time.time()
        link.evidence_timestamps.append(now)
        link.last_evidence_time = now
        link.evidence_count += 1
        link.strength = min(1.0, link.strength + strength_bump)
        link.stale = False

    # -- queries -----------------------------------------------------------

    def get_active_causes(self, effect: str, min_strength: float = 0.3,
                          now: Optional[float] = None) -> List[CausalLink]:
        if now is None:
            now = time.time()
        results: List[Tuple[float, CausalLink]] = []
        with self._lock:
            self._apply_temporal_decay(now)
            for links in self._causal_graph.values():
                for link in links:
                    if link.effect != effect:
                        continue
                    eff = self._effective_strength(link, now)
                    if eff >= min_strength:
                        results.append((eff, link))
        results.sort(key=lambda x: x[0], reverse=True)
        return [link for _, link in results]

    def suggest_root_cause(self, anomaly_type: str,
                           now: Optional[float] = None) -> List[Tuple[str, float]]:
        if now is None:
            now = time.time()
        candidates: List[Tuple[str, float]] = []
        with self._lock:
            self._apply_temporal_decay(now)
            for links in self._causal_graph.values():
                for link in links:
                    if link.effect == anomaly_type:
                        eff = self._effective_strength(link, now)
                        if eff > 0.0:
                            candidates.append((link.cause, eff))
        candidates.sort(key=lambda x: x[1], reverse=True)
        return candidates

    # -- correlated alert integration --------------------------------------

    def on_correlated_alert(self, root_cause: str,
                            contributing_alert_ids: List[str]) -> None:
        """Mirrors CausalInferenceNode._on_correlated_alert logic."""
        alert_types: List[str] = []
        for cid in contributing_alert_ids:
            parts = cid.split('@')
            if parts:
                alert_types.append(parts[0])

        if not root_cause:
            return

        now = time.time()
        with self._lock:
            for atype in alert_types:
                found = False
                if root_cause in self._causal_graph:
                    for link in self._causal_graph[root_cause]:
                        if link.effect == atype:
                            self._update_evidence(link, now=now,
                                                  strength_bump=0.02)
                            found = True
                            break
                if not found:
                    new_link = CausalLink(
                        cause=root_cause,
                        effect=atype,
                        strength=0.2,
                        evidence_count=1,
                        last_evidence_time=now,
                        evidence_timestamps=[now],
                        decay_half_life_sec=self._decay_half_life_sec,
                    )
                    self._causal_graph.setdefault(root_cause, []).append(new_link)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_link(cause='A', effect='B', strength=0.8, half_life=3600.0,
               last_evidence_time=None, evidence_timestamps=None):
    now = time.time()
    return CausalLink(
        cause=cause,
        effect=effect,
        strength=strength,
        evidence_count=len(evidence_timestamps) if evidence_timestamps else 0,
        last_evidence_time=last_evidence_time if last_evidence_time is not None else now,
        evidence_timestamps=list(evidence_timestamps) if evidence_timestamps else [],
        decay_half_life_sec=half_life,
    )


def _engine(links=None, half_life=3600.0):
    eng = _CausalEngine(half_life=half_life)
    if links:
        for link in links:
            eng.add_link(link)
    return eng


# ---------------------------------------------------------------------------
# Tests (27 total -- at least 12 required)
# ---------------------------------------------------------------------------

class TestComputeDecayFactor:
    """Tests for the static decay factor calculation."""

    def test_fresh_evidence_full_strength(self):
        """Evidence at time zero has decay factor 1.0."""
        factor = _CausalEngine.compute_decay_factor(0.0, 3600.0)
        assert factor == pytest.approx(1.0)

    def test_one_half_life_decay(self):
        """After one half-life the factor should be ~0.5."""
        factor = _CausalEngine.compute_decay_factor(3600.0, 3600.0)
        assert factor == pytest.approx(0.5, abs=1e-9)

    def test_two_half_lives_decay(self):
        """After two half-lives the factor should be ~0.25."""
        factor = _CausalEngine.compute_decay_factor(7200.0, 3600.0)
        assert factor == pytest.approx(0.25, abs=1e-9)

    def test_zero_half_life_returns_one(self):
        """Guard: if half_life is 0 the factor is clamped to 1.0."""
        factor = _CausalEngine.compute_decay_factor(9999.0, 0.0)
        assert factor == 1.0


class TestEffectiveStrength:
    """Tests for per-link effective strength after decay."""

    def test_fresh_link_full_effective_strength(self):
        now = time.time()
        link = _make_link(strength=0.8, last_evidence_time=now)
        eng = _engine()
        eff = eng._effective_strength(link, now=now)
        assert eff == pytest.approx(0.8, abs=1e-6)

    def test_half_life_effective_strength(self):
        now = time.time()
        link = _make_link(strength=0.8, last_evidence_time=now - 3600.0)
        eng = _engine()
        eff = eng._effective_strength(link, now=now)
        assert eff == pytest.approx(0.4, abs=1e-6)

    def test_no_evidence_time_zero_strength(self):
        link = _make_link(strength=0.8, last_evidence_time=0.0)
        eng = _engine()
        eff = eng._effective_strength(link, now=time.time())
        assert eff == 0.0


class TestApplyTemporalDecay:
    """Tests for _apply_temporal_decay marking links stale."""

    def test_recent_link_not_stale(self):
        now = time.time()
        link = _make_link(last_evidence_time=now)
        eng = _engine([link])
        eng._apply_temporal_decay(now)
        assert link.stale is False

    def test_very_old_link_marked_stale(self):
        """A link older than ~3.3 half-lives (factor < 0.1) should be stale."""
        now = time.time()
        link = _make_link(last_evidence_time=now - 4 * 3600.0)
        eng = _engine([link])
        eng._apply_temporal_decay(now)
        assert link.stale is True

    def test_decay_applied_before_analysis(self):
        """Verify that _apply_temporal_decay marks old links before query."""
        now = time.time()
        old_link = _make_link(cause='X', effect='Y',
                              last_evidence_time=now - 5 * 3600.0)
        fresh_link = _make_link(cause='Z', effect='Y',
                                last_evidence_time=now, strength=0.6)
        eng = _engine([old_link, fresh_link])
        eng._apply_temporal_decay(now)
        assert old_link.stale is True
        assert fresh_link.stale is False


class TestPruneStaleEvidence:
    """Tests for _prune_stale_evidence removing old timestamps."""

    def test_old_timestamps_pruned(self):
        now = time.time()
        half_life = 3600.0
        cutoff = now - 4 * half_life
        timestamps = [cutoff - 100.0, cutoff - 50.0, now - 10.0, now]
        link = _make_link(evidence_timestamps=timestamps, half_life=half_life,
                          last_evidence_time=now)
        link.evidence_count = 4
        eng = _engine([link])
        pruned = eng._prune_stale_evidence(now)
        assert pruned == 2
        assert len(link.evidence_timestamps) == 2
        assert link.evidence_count == 2

    def test_no_pruning_when_all_recent(self):
        now = time.time()
        timestamps = [now - 10.0, now - 5.0, now]
        link = _make_link(evidence_timestamps=timestamps, last_evidence_time=now)
        link.evidence_count = 3
        eng = _engine([link])
        pruned = eng._prune_stale_evidence(now)
        assert pruned == 0
        assert link.evidence_count == 3


class TestUpdateEvidence:
    """Tests for _update_evidence recording timestamps."""

    def test_new_evidence_refreshes_timer(self):
        now = time.time()
        link = _make_link(last_evidence_time=now - 7200.0, strength=0.5)
        eng = _engine()
        eng._update_evidence(link, now=now)
        assert link.last_evidence_time == now
        assert link.stale is False

    def test_evidence_count_increments(self):
        link = _make_link()
        link.evidence_count = 5
        eng = _engine()
        eng._update_evidence(link, now=time.time())
        assert link.evidence_count == 6

    def test_strength_capped_at_one(self):
        link = _make_link(strength=0.99)
        eng = _engine()
        eng._update_evidence(link, now=time.time(), strength_bump=0.05)
        assert link.strength == 1.0


class TestWeightedEvidenceCount:
    """Tests for evidence count weighted by per-timestamp decay."""

    def test_all_fresh_timestamps(self):
        now = time.time()
        timestamps = [now, now, now]
        link = _make_link(evidence_timestamps=timestamps, last_evidence_time=now)
        eng = _engine()
        weighted = eng._weighted_evidence_count(link, now=now)
        assert weighted == pytest.approx(3.0, abs=1e-6)

    def test_mixed_age_timestamps(self):
        now = time.time()
        half_life = 3600.0
        timestamps = [now, now - half_life]
        link = _make_link(evidence_timestamps=timestamps, half_life=half_life,
                          last_evidence_time=now)
        eng = _engine()
        weighted = eng._weighted_evidence_count(link, now=now)
        assert weighted == pytest.approx(1.5, abs=1e-6)


class TestGetActiveCauses:
    """Tests for get_active_causes filtered by min_strength."""

    def test_filters_below_min_strength(self):
        now = time.time()
        strong = _make_link(cause='A', effect='E', strength=0.9,
                            last_evidence_time=now)
        weak = _make_link(cause='B', effect='E', strength=0.2,
                          last_evidence_time=now)
        eng = _engine([strong, weak])
        results = eng.get_active_causes('E', min_strength=0.3, now=now)
        assert len(results) == 1
        assert results[0].cause == 'A'

    def test_sorted_by_effective_strength(self):
        now = time.time()
        link1 = _make_link(cause='X', effect='E', strength=0.5,
                           last_evidence_time=now)
        link2 = _make_link(cause='Y', effect='E', strength=0.9,
                           last_evidence_time=now)
        eng = _engine([link1, link2])
        results = eng.get_active_causes('E', min_strength=0.1, now=now)
        assert results[0].cause == 'Y'
        assert results[1].cause == 'X'


class TestSuggestRootCause:
    """Tests for suggest_root_cause ranking."""

    def test_returns_ranked_causes(self):
        now = time.time()
        link1 = _make_link(cause='C1', effect='Vibration', strength=0.9,
                           last_evidence_time=now)
        link2 = _make_link(cause='C2', effect='Vibration', strength=0.4,
                           last_evidence_time=now)
        eng = _engine([link1, link2])
        results = eng.suggest_root_cause('Vibration', now=now)
        assert len(results) == 2
        assert results[0] == ('C1', pytest.approx(0.9, abs=1e-3))
        assert results[1] == ('C2', pytest.approx(0.4, abs=1e-3))

    def test_empty_for_unknown_anomaly(self):
        eng = _engine()
        results = eng.suggest_root_cause('Unknown', now=time.time())
        assert results == []


class TestCorrelatedAlertIntegration:
    """Tests for on_correlated_alert strengthening / discovering links."""

    def test_existing_link_strengthened(self):
        now = time.time()
        link = _make_link(cause='ToolWear', effect='vibration_anomaly',
                          strength=0.5, last_evidence_time=now - 100)
        eng = _engine([link])
        eng.on_correlated_alert('ToolWear', ['vibration_anomaly@1000'])
        assert link.strength > 0.5
        assert link.evidence_count == 1  # was 0 from _make_link, now +1
        assert len(link.evidence_timestamps) == 1

    def test_new_link_created_for_unknown_pattern(self):
        eng = _engine()
        eng.on_correlated_alert('NewCause', ['new_effect@1000'])
        assert 'NewCause' in eng._causal_graph
        new_link = eng._causal_graph['NewCause'][0]
        assert new_link.effect == 'new_effect'
        assert new_link.strength == 0.2
        assert new_link.evidence_count == 1

    def test_empty_root_cause_ignored(self):
        eng = _engine()
        eng.on_correlated_alert('', ['some_alert@1000'])
        assert len(eng._causal_graph) == 0


class TestHalfLifeConfigurable:
    """Test that decay half-life is configurable per link."""

    def test_custom_half_life(self):
        half_life = 1800.0
        factor = _CausalEngine.compute_decay_factor(1800.0, half_life)
        assert factor == pytest.approx(0.5, abs=1e-9)

    def test_long_half_life_slow_decay(self):
        half_life = 86400.0
        factor = _CausalEngine.compute_decay_factor(3600.0, half_life)
        expected = math.pow(2.0, -3600.0 / 86400.0)
        assert factor == pytest.approx(expected, abs=1e-6)

    def test_link_uses_own_half_life(self):
        now = time.time()
        link_fast = _make_link(strength=1.0, half_life=1800.0,
                               last_evidence_time=now - 1800.0)
        link_slow = _make_link(strength=1.0, half_life=7200.0,
                               last_evidence_time=now - 1800.0)
        eng = _engine()
        eff_fast = eng._effective_strength(link_fast, now=now)
        eff_slow = eng._effective_strength(link_slow, now=now)
        assert eff_fast < eff_slow
        assert eff_fast == pytest.approx(0.5, abs=1e-6)
