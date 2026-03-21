"""Tests for ObservabilityTraceManager.

Covers span lifecycle (start/end), event and tag enrichment, full trace
reconstruction, slow-trace detection, error-trace detection, service
latency computation, parent-child span relationships, and edge cases.
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
    ObservabilityTraceManager,
    TraceSpan,
    Trace,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def manager():
    """Return a fresh ObservabilityTraceManager."""
    return ObservabilityTraceManager()


# ---------------------------------------------------------------------------
# Test: start_span creates a span with correct fields
# ---------------------------------------------------------------------------

class TestStartSpan:

    def test_creates_span_with_correct_fields(self, manager):
        span = manager.start_span('t1', 'svc_motion', 'plan_path')
        assert span.trace_id == 't1'
        assert span.service_name == 'svc_motion'
        assert span.operation_name == 'plan_path'
        assert span.parent_span_id is None
        assert span.status == 'ok'
        assert span.end_time is None
        assert span.duration_ms is None
        assert isinstance(span.span_id, str)
        assert span.start_time > 0

    def test_creates_child_span(self, manager):
        parent = manager.start_span('t1', 'svc_motion', 'plan_path')
        child = manager.start_span(
            't1', 'svc_kinematics', 'fk_solve', parent_span_id=parent.span_id
        )
        assert child.parent_span_id == parent.span_id
        assert child.trace_id == 't1'


# ---------------------------------------------------------------------------
# Test: end_span completes a span correctly
# ---------------------------------------------------------------------------

class TestEndSpan:

    def test_end_span_sets_duration_and_status(self, manager):
        span = manager.start_span('t1', 'svc_motion', 'plan_path')
        ended = manager.end_span(span.span_id, status='ok')
        assert ended.end_time is not None
        assert ended.duration_ms is not None
        assert ended.duration_ms >= 0.0
        assert ended.status == 'ok'

    def test_end_span_with_error_status(self, manager):
        span = manager.start_span('t1', 'svc_motion', 'plan_path')
        ended = manager.end_span(span.span_id, status='error')
        assert ended.status == 'error'

    def test_end_span_unknown_span_raises(self, manager):
        with pytest.raises(KeyError):
            manager.end_span('nonexistent', status='ok')

    def test_end_span_invalid_status_raises(self, manager):
        span = manager.start_span('t1', 'svc_motion', 'plan_path')
        with pytest.raises(ValueError):
            manager.end_span(span.span_id, status='cancelled')


# ---------------------------------------------------------------------------
# Test: add_event and add_tag
# ---------------------------------------------------------------------------

class TestEnrichment:

    def test_add_event_appends_to_span(self, manager):
        span = manager.start_span('t1', 'svc_coolant', 'adjust_flow')
        manager.add_event(span.span_id, 'flow_adjusted', {'rate': 4.5})
        manager.add_event(span.span_id, 'temp_checked', {'celsius': 22.1})
        assert len(span.events) == 2
        assert span.events[0]['name'] == 'flow_adjusted'
        assert span.events[0]['data'] == {'rate': 4.5}
        assert span.events[1]['name'] == 'temp_checked'

    def test_add_event_unknown_span_raises(self, manager):
        with pytest.raises(KeyError):
            manager.add_event('no_such_span', 'evt', {})

    def test_add_tag_stores_value(self, manager):
        span = manager.start_span('t1', 'svc_spindle', 'balance')
        manager.add_tag(span.span_id, 'priority', 'high')
        manager.add_tag(span.span_id, 'retry', 3)
        assert span.tags['priority'] == 'high'
        assert span.tags['retry'] == 3

    def test_add_tag_unknown_span_raises(self, manager):
        with pytest.raises(KeyError):
            manager.add_tag('no_such_span', 'k', 'v')


# ---------------------------------------------------------------------------
# Test: get_trace reconstructs a full trace
# ---------------------------------------------------------------------------

class TestGetTrace:

    def test_basic_trace_reconstruction(self, manager):
        s1 = manager.start_span('t1', 'svc_a', 'op1')
        s2 = manager.start_span('t1', 'svc_b', 'op2', parent_span_id=s1.span_id)
        manager.end_span(s1.span_id, 'ok')
        manager.end_span(s2.span_id, 'error')

        trace = manager.get_trace('t1')
        assert trace.trace_id == 't1'
        assert len(trace.spans) == 2
        assert trace.service_count == 2
        assert trace.error_count == 1
        assert trace.total_duration_ms >= 0.0

    def test_unknown_trace_raises(self, manager):
        with pytest.raises(KeyError):
            manager.get_trace('nonexistent')

    def test_trace_total_duration_spans_full_range(self, manager):
        """total_duration_ms should cover from the earliest start to the latest end."""
        s1 = manager.start_span('t1', 'svc_a', 'op1')
        time.sleep(0.005)
        s2 = manager.start_span('t1', 'svc_b', 'op2')
        manager.end_span(s1.span_id, 'ok')
        time.sleep(0.005)
        manager.end_span(s2.span_id, 'ok')

        trace = manager.get_trace('t1')
        # Total duration should be at least 10 ms (two 5 ms sleeps)
        assert trace.total_duration_ms >= 8.0


# ---------------------------------------------------------------------------
# Test: get_slow_traces
# ---------------------------------------------------------------------------

class TestGetSlowTraces:

    def test_returns_traces_exceeding_threshold(self, manager):
        # Fast trace
        s1 = manager.start_span('fast', 'svc_a', 'op1')
        manager.end_span(s1.span_id, 'ok')

        # Slow trace
        s2 = manager.start_span('slow', 'svc_b', 'op2')
        time.sleep(0.015)
        manager.end_span(s2.span_id, 'ok')

        slow = manager.get_slow_traces(threshold_ms=10.0)
        trace_ids = [t.trace_id for t in slow]
        assert 'slow' in trace_ids
        # fast trace should not be in the result
        assert 'fast' not in trace_ids

    def test_returns_empty_when_none_slow(self, manager):
        s = manager.start_span('t1', 'svc_a', 'op')
        manager.end_span(s.span_id, 'ok')
        assert manager.get_slow_traces(threshold_ms=999999.0) == []


# ---------------------------------------------------------------------------
# Test: get_error_traces
# ---------------------------------------------------------------------------

class TestGetErrorTraces:

    def test_returns_traces_with_error_spans(self, manager):
        s1 = manager.start_span('ok_trace', 'svc_a', 'op')
        manager.end_span(s1.span_id, 'ok')

        s2 = manager.start_span('err_trace', 'svc_b', 'op')
        manager.end_span(s2.span_id, 'error')

        errors = manager.get_error_traces()
        ids = [t.trace_id for t in errors]
        assert 'err_trace' in ids
        assert 'ok_trace' not in ids

    def test_returns_empty_when_no_errors(self, manager):
        s = manager.start_span('t1', 'svc', 'op')
        manager.end_span(s.span_id, 'ok')
        assert manager.get_error_traces() == []


# ---------------------------------------------------------------------------
# Test: get_service_latency
# ---------------------------------------------------------------------------

class TestGetServiceLatency:

    def test_average_latency_for_service(self, manager):
        for _ in range(3):
            s = manager.start_span('t1', 'svc_x', 'op')
            time.sleep(0.005)
            manager.end_span(s.span_id, 'ok')

        avg = manager.get_service_latency('svc_x')
        # Each span sleeps ~5 ms; average should be roughly >= 4 ms
        assert avg >= 3.0

    def test_returns_zero_for_unknown_service(self, manager):
        assert manager.get_service_latency('no_such_service') == 0.0

    def test_ignores_incomplete_spans(self, manager):
        """Spans that have not been ended should not affect latency."""
        manager.start_span('t1', 'svc_y', 'op')  # never ended
        assert manager.get_service_latency('svc_y') == 0.0


# ---------------------------------------------------------------------------
# Test: timeout status
# ---------------------------------------------------------------------------

class TestTimeoutStatus:

    def test_end_span_with_timeout_status(self, manager):
        span = manager.start_span('t1', 'svc_timeout', 'slow_op')
        ended = manager.end_span(span.span_id, status='timeout')
        assert ended.status == 'timeout'
        assert ended.duration_ms is not None
