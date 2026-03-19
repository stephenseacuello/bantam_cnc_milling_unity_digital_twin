"""Tests for RealTimeDashboardAggregator.

Validates widget registration, updates, layout creation, alarm queries,
and history tracking for the real-time SCADA dashboard.
"""

import sys
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
    DashboardWidget,
    DashboardLayout,
    RealTimeDashboardAggregator,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_widget(
    widget_id: str = 'w1',
    title: str = 'Test Widget',
    widget_type: str = 'gauge',
    data_source: str = 'test/source',
    current_value: float = 0.0,
    min_value: float = 0.0,
    max_value: float = 100.0,
    unit: str = '%',
    status: str = 'normal',
    update_rate_sec: float = 1.0,
) -> DashboardWidget:
    return DashboardWidget(
        widget_id=widget_id,
        title=title,
        widget_type=widget_type,
        data_source=data_source,
        current_value=current_value,
        min_value=min_value,
        max_value=max_value,
        unit=unit,
        status=status,
        update_rate_sec=update_rate_sec,
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestRegisterAndGetWidget:
    """Widget registration and retrieval."""

    def test_register_and_get(self):
        agg = RealTimeDashboardAggregator()
        w = _make_widget(widget_id='rpm_gauge')
        agg.register_widget(w)
        result = agg.get_widget('rpm_gauge')
        assert result is not None
        assert result.widget_id == 'rpm_gauge'
        assert result.title == 'Test Widget'

    def test_get_nonexistent_returns_none(self):
        agg = RealTimeDashboardAggregator()
        assert agg.get_widget('does_not_exist') is None


class TestUpdateWidget:
    """Widget value and status updates."""

    def test_update_value_and_status(self):
        agg = RealTimeDashboardAggregator()
        agg.register_widget(_make_widget(widget_id='load'))
        agg.update_widget('load', 85.5, 'warning')
        w = agg.get_widget('load')
        assert w.current_value == 85.5
        assert w.status == 'warning'

    def test_update_unknown_widget_raises(self):
        agg = RealTimeDashboardAggregator()
        with pytest.raises(KeyError, match='not_registered'):
            agg.update_widget('not_registered', 10.0)


class TestDefaultLayout:
    """Default CNC dashboard layout creation."""

    def test_creates_ten_widgets(self):
        agg = RealTimeDashboardAggregator()
        layout = agg.create_default_layout('cnc_1')
        assert len(layout.widgets) == 10

    def test_layout_contains_expected_widgets(self):
        agg = RealTimeDashboardAggregator()
        layout = agg.create_default_layout('cnc_1')
        ids = {w.widget_id for w in layout.widgets}
        expected = {
            'cnc_1_spindle_speed',
            'cnc_1_feed_rate',
            'cnc_1_spindle_load',
            'cnc_1_axis_x',
            'cnc_1_axis_y',
            'cnc_1_axis_z',
            'cnc_1_coolant_status',
            'cnc_1_part_count',
            'cnc_1_cycle_time',
            'cnc_1_oee',
        }
        assert ids == expected

    def test_layout_metadata(self):
        agg = RealTimeDashboardAggregator()
        layout = agg.create_default_layout('cnc_2')
        assert layout.layout_id == 'default_cnc_2'
        assert layout.machine_id == 'cnc_2'
        assert layout.name == 'cnc_2 CNC Dashboard'


class TestGetLayout:
    """Layout retrieval reflects live widget state."""

    def test_get_layout_reflects_updates(self):
        agg = RealTimeDashboardAggregator()
        agg.create_default_layout('cnc_1')
        agg.update_widget('cnc_1_spindle_speed', 12000.0, 'normal')
        layout = agg.get_layout('default_cnc_1')
        assert layout is not None
        spindle = next(w for w in layout.widgets if w.widget_id == 'cnc_1_spindle_speed')
        assert spindle.current_value == 12000.0

    def test_get_layout_nonexistent_returns_none(self):
        agg = RealTimeDashboardAggregator()
        assert agg.get_layout('no_such_layout') is None


class TestAlarmWidgets:
    """Alarm widget filtering."""

    def test_returns_only_alarm_widgets(self):
        agg = RealTimeDashboardAggregator()
        agg.register_widget(_make_widget(widget_id='a', status='normal'))
        agg.register_widget(_make_widget(widget_id='b', status='warning'))
        agg.register_widget(_make_widget(widget_id='c', status='alarm'))
        agg.register_widget(_make_widget(widget_id='d', status='alarm'))
        alarms = agg.get_alarm_widgets()
        alarm_ids = {w.widget_id for w in alarms}
        assert alarm_ids == {'c', 'd'}

    def test_no_alarms_returns_empty(self):
        agg = RealTimeDashboardAggregator()
        agg.register_widget(_make_widget(widget_id='ok'))
        assert agg.get_alarm_widgets() == []


class TestWidgetHistory:
    """History tracking and retrieval."""

    def test_history_records_updates(self):
        agg = RealTimeDashboardAggregator()
        agg.register_widget(_make_widget(widget_id='h1'))
        for v in [1.0, 2.0, 3.0, 4.0, 5.0]:
            agg.update_widget('h1', v, 'normal')
        history = agg.get_widget_history('h1', last_n=3)
        assert len(history) == 3
        values = [entry[1] for entry in history]
        assert values == [3.0, 4.0, 5.0]

    def test_history_empty_for_unregistered(self):
        agg = RealTimeDashboardAggregator()
        assert agg.get_widget_history('ghost') == []

    def test_history_respects_max_size(self):
        agg = RealTimeDashboardAggregator(history_size=5)
        agg.register_widget(_make_widget(widget_id='bounded'))
        for v in range(20):
            agg.update_widget('bounded', float(v), 'normal')
        full = agg.get_widget_history('bounded', last_n=100)
        assert len(full) == 5
        assert full[0][1] == 15.0  # oldest retained
        assert full[-1][1] == 19.0  # most recent
