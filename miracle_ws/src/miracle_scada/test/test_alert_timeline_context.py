"""Tests for AlertTimelinePanel G-code context, block annotations, and block-range filtering.

These tests exercise a pure-Python model that mirrors the C# AlertTimelinePanel logic
so we can validate data integrity, filtering, sorting, and formatting without Unity.
"""

import pytest
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Dict, List, Optional, Set


# ---------------------------------------------------------------------------
# Pure-Python mirror of the C# data structures
# ---------------------------------------------------------------------------


class AlertCategory(Enum):
    ANOMALY = "Anomaly"
    CORRELATED = "Correlated"
    SECURITY = "Security"


@dataclass
class AlertGCodeContext:
    """Mirrors MiracleTwin.UI.AlertGCodeContext."""
    block_index: int
    gcode_line: str = ""            # e.g. "G01 X50.0 Y25.0 F800"
    operation_type: str = ""        # "LINEAR_CUT", "ARC_CUT", "DRILL", etc.
    feed_rate: float = 0.0
    spindle_rpm: float = 0.0
    program_progress_pct: float = 0.0
    tool_id: str = ""

    def format_progress(self) -> str:
        """Return human-readable progress string."""
        return f"{self.program_progress_pct:.1f}%"


@dataclass
class AlertEntry:
    """Mirrors MiracleTwin.UI.AlertTimelinePanel.AlertEntry."""
    timestamp: datetime
    machine_id: str
    alert_type: str
    message: str
    severity: float
    category: AlertCategory


class AlertTimelineModel:
    """Pure-Python model that mirrors the essential C# AlertTimelinePanel logic
    for G-code context attachment, block filtering, and recurring block tracking."""

    def __init__(self, max_alerts: int = 100):
        self.max_alerts = max_alerts
        self._alerts: List[AlertEntry] = []
        self._gcode_contexts: Dict[int, AlertGCodeContext] = {}
        self._recurring_blocks: Set[int] = set()
        self._active_category: Optional[AlertCategory] = None
        self._filter_block_start: int = -1
        self._filter_block_end: int = -1

    @property
    def alert_count(self) -> int:
        return len(self._alerts)

    def add_alert(self, entry: AlertEntry) -> int:
        """Add an alert and return its index."""
        self._alerts.insert(0, entry)
        # Re-key contexts: shift existing indices by +1
        new_ctx = {}
        for idx, ctx in self._gcode_contexts.items():
            new_ctx[idx + 1] = ctx
        self._gcode_contexts = new_ctx
        # Trim
        while len(self._alerts) > self.max_alerts:
            removed_idx = len(self._alerts) - 1
            self._gcode_contexts.pop(removed_idx, None)
            self._alerts.pop()
        return 0  # inserted at front

    def set_alert_gcode_context(self, alert_index: int, context: AlertGCodeContext):
        if alert_index < 0 or alert_index >= len(self._alerts):
            return
        if context is None:
            return
        self._gcode_contexts[alert_index] = context

    def get_alert_gcode_context(self, alert_index: int) -> Optional[AlertGCodeContext]:
        return self._gcode_contexts.get(alert_index)

    def mark_block_recurring(self, block_index: int):
        self._recurring_blocks.add(block_index)

    def is_block_recurring(self, block_index: int) -> bool:
        return block_index in self._recurring_blocks

    def filter_by_block_range(self, start_block: int, end_block: int):
        if start_block < 0 or end_block < 0 or end_block < start_block:
            return
        self._filter_block_start = start_block
        self._filter_block_end = end_block

    def clear_block_filter(self):
        self._filter_block_start = -1
        self._filter_block_end = -1

    def set_category_filter(self, category: Optional[AlertCategory]):
        self._active_category = category

    def get_visible_alerts(self) -> List[tuple]:
        """Return list of (index, AlertEntry, Optional[AlertGCodeContext]) for visible alerts."""
        result = []
        block_filter_active = (
            self._filter_block_start >= 0 and self._filter_block_end >= 0
        )
        for i, alert in enumerate(self._alerts):
            if self._active_category is not None and alert.category != self._active_category:
                continue
            if block_filter_active:
                ctx = self._gcode_contexts.get(i)
                if ctx is None:
                    continue
                if ctx.block_index < self._filter_block_start or ctx.block_index > self._filter_block_end:
                    continue
            result.append((i, alert, self._gcode_contexts.get(i)))
        return result

    def get_alerts_sorted_by_block(self) -> List[tuple]:
        """Return visible alerts sorted by block index (ascending).
        Alerts without context are placed at the end."""
        visible = self.get_visible_alerts()
        return sorted(visible, key=lambda t: (
            0 if t[2] is not None else 1,
            t[2].block_index if t[2] is not None else 0,
        ))

    @staticmethod
    def classify_operation(gcode_line: str) -> str:
        """Classify a G-code line into an operation type."""
        if not gcode_line:
            return "UNKNOWN"
        line = gcode_line.strip().upper()
        if line.startswith("G00"):
            return "RAPID"
        if line.startswith("G01"):
            return "LINEAR_CUT"
        if line.startswith("G02") or line.startswith("G03"):
            return "ARC_CUT"
        if line.startswith("G81") or line.startswith("G83"):
            return "DRILL"
        if line.startswith("M06") or line.startswith("T"):
            return "TOOL_CHANGE"
        return "UNKNOWN"

    @staticmethod
    def block_badge_color(is_recurring: bool) -> str:
        """Return a CSS-style color identifier for block badge."""
        return "block-badge-recurring" if is_recurring else "block-badge"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_alert(
    alert_type: str = "vibration_anomaly",
    severity: float = 0.7,
    category: AlertCategory = AlertCategory.ANOMALY,
    machine_id: str = "cnc1",
) -> AlertEntry:
    return AlertEntry(
        timestamp=datetime.now(),
        machine_id=machine_id,
        alert_type=alert_type,
        message="test alert",
        severity=severity,
        category=category,
    )


def _make_context(
    block_index: int = 10,
    gcode_line: str = "G01 X50.0 Y25.0 F800",
    operation_type: str = "LINEAR_CUT",
    feed_rate: float = 800.0,
    spindle_rpm: float = 12000.0,
    program_progress_pct: float = 45.0,
    tool_id: str = "T01",
) -> AlertGCodeContext:
    return AlertGCodeContext(
        block_index=block_index,
        gcode_line=gcode_line,
        operation_type=operation_type,
        feed_rate=feed_rate,
        spindle_rpm=spindle_rpm,
        program_progress_pct=program_progress_pct,
        tool_id=tool_id,
    )


# ===================================================================
# Tests
# ===================================================================


class TestAlertGCodeContextCreation:
    """Test AlertGCodeContext creation with all fields."""

    def test_create_with_all_fields(self):
        ctx = _make_context()
        assert ctx.block_index == 10
        assert ctx.gcode_line == "G01 X50.0 Y25.0 F800"
        assert ctx.operation_type == "LINEAR_CUT"
        assert ctx.feed_rate == 800.0
        assert ctx.spindle_rpm == 12000.0
        assert ctx.program_progress_pct == 45.0
        assert ctx.tool_id == "T01"

    def test_create_with_defaults(self):
        ctx = AlertGCodeContext(block_index=0)
        assert ctx.block_index == 0
        assert ctx.gcode_line == ""
        assert ctx.feed_rate == 0.0

    def test_different_operation_types(self):
        for op in ["LINEAR_CUT", "ARC_CUT", "DRILL", "RAPID", "TOOL_CHANGE"]:
            ctx = _make_context(operation_type=op)
            assert ctx.operation_type == op


class TestBlockRangeFilterInclusion:
    """Test block range filter includes matching alerts."""

    def test_filter_includes_matching_block(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        model.set_alert_gcode_context(0, _make_context(block_index=15))
        model.filter_by_block_range(10, 20)
        visible = model.get_visible_alerts()
        assert len(visible) == 1

    def test_filter_includes_boundary_start(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        model.set_alert_gcode_context(0, _make_context(block_index=10))
        model.filter_by_block_range(10, 20)
        visible = model.get_visible_alerts()
        assert len(visible) == 1

    def test_filter_includes_boundary_end(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        model.set_alert_gcode_context(0, _make_context(block_index=20))
        model.filter_by_block_range(10, 20)
        visible = model.get_visible_alerts()
        assert len(visible) == 1


class TestBlockRangeFilterExclusion:
    """Test block range filter excludes non-matching alerts."""

    def test_filter_excludes_out_of_range(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        model.set_alert_gcode_context(0, _make_context(block_index=5))
        model.filter_by_block_range(10, 20)
        visible = model.get_visible_alerts()
        assert len(visible) == 0

    def test_filter_excludes_alerts_without_context(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        # No context set
        model.filter_by_block_range(10, 20)
        visible = model.get_visible_alerts()
        assert len(visible) == 0


class TestClearBlockFilter:
    """Test clear block filter shows all."""

    def test_clear_restores_all_alerts(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        model.set_alert_gcode_context(0, _make_context(block_index=5))
        model.filter_by_block_range(10, 20)
        assert len(model.get_visible_alerts()) == 0
        model.clear_block_filter()
        assert len(model.get_visible_alerts()) == 1

    def test_clear_shows_alerts_without_context(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())  # no context
        model.filter_by_block_range(10, 20)
        assert len(model.get_visible_alerts()) == 0
        model.clear_block_filter()
        assert len(model.get_visible_alerts()) == 1


class TestRecurringBlockDetection:
    """Test recurring block detection."""

    def test_not_recurring_by_default(self):
        model = AlertTimelineModel()
        assert not model.is_block_recurring(10)

    def test_mark_recurring(self):
        model = AlertTimelineModel()
        model.mark_block_recurring(42)
        assert model.is_block_recurring(42)
        assert not model.is_block_recurring(43)

    def test_mark_multiple_recurring(self):
        model = AlertTimelineModel()
        model.mark_block_recurring(10)
        model.mark_block_recurring(20)
        model.mark_block_recurring(30)
        assert model.is_block_recurring(10)
        assert model.is_block_recurring(20)
        assert model.is_block_recurring(30)
        assert not model.is_block_recurring(15)


class TestOperationTypeClassification:
    """Test operation type classification from G-code lines."""

    @pytest.mark.parametrize("line,expected", [
        ("G00 X10 Y20", "RAPID"),
        ("G01 X50.0 Y25.0 F800", "LINEAR_CUT"),
        ("G02 X10 Y10 I5 J0 F400", "ARC_CUT"),
        ("G03 X10 Y10 I5 J0 F400", "ARC_CUT"),
        ("G81 X10 Y10 Z-5 R2 F200", "DRILL"),
        ("G83 X10 Y10 Z-15 R2 Q3 F150", "DRILL"),
        ("M06 T02", "TOOL_CHANGE"),
        ("T03 M06", "TOOL_CHANGE"),
        ("M30", "UNKNOWN"),
        ("", "UNKNOWN"),
    ])
    def test_classify(self, line, expected):
        assert AlertTimelineModel.classify_operation(line) == expected


class TestProgramProgressFormatting:
    """Test program progress formatting."""

    def test_format_zero(self):
        ctx = _make_context(program_progress_pct=0.0)
        assert ctx.format_progress() == "0.0%"

    def test_format_midway(self):
        ctx = _make_context(program_progress_pct=45.67)
        assert ctx.format_progress() == "45.7%"

    def test_format_complete(self):
        ctx = _make_context(program_progress_pct=100.0)
        assert ctx.format_progress() == "100.0%"


class TestEmptyContextRendering:
    """Test empty context doesn't break rendering."""

    def test_alert_without_context_visible(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        visible = model.get_visible_alerts()
        assert len(visible) == 1
        assert visible[0][2] is None  # no context

    def test_set_none_context_ignored(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        model.set_alert_gcode_context(0, None)
        assert model.get_alert_gcode_context(0) is None

    def test_set_context_invalid_index(self):
        model = AlertTimelineModel()
        model.set_alert_gcode_context(99, _make_context())
        # Should not raise, just silently ignore


class TestMultipleAlertsSameBlock:
    """Test multiple alerts on same block group together."""

    def test_same_block_alerts_visible(self):
        model = AlertTimelineModel()
        for _ in range(3):
            model.add_alert(_make_alert())
        # Set all to block 42
        for i in range(3):
            model.set_alert_gcode_context(i, _make_context(block_index=42))
        model.filter_by_block_range(40, 45)
        visible = model.get_visible_alerts()
        assert len(visible) == 3

    def test_sorted_by_block_groups_same_block(self):
        model = AlertTimelineModel()
        # Add alerts with different blocks
        model.add_alert(_make_alert(alert_type="alert_a"))
        model.set_alert_gcode_context(0, _make_context(block_index=50))
        model.add_alert(_make_alert(alert_type="alert_b"))
        model.set_alert_gcode_context(0, _make_context(block_index=10))
        model.add_alert(_make_alert(alert_type="alert_c"))
        model.set_alert_gcode_context(0, _make_context(block_index=50))

        sorted_alerts = model.get_alerts_sorted_by_block()
        blocks = [t[2].block_index for t in sorted_alerts]
        assert blocks == [10, 50, 50]


class TestBlockBadgeColor:
    """Test block badge color for recurring vs non-recurring."""

    def test_non_recurring_badge(self):
        assert AlertTimelineModel.block_badge_color(False) == "block-badge"

    def test_recurring_badge(self):
        assert AlertTimelineModel.block_badge_color(True) == "block-badge-recurring"


class TestInvalidBlockRange:
    """Test filter with negative/invalid block range."""

    def test_negative_start_ignored(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        model.filter_by_block_range(-1, 10)
        # Invalid range should not activate filter
        visible = model.get_visible_alerts()
        assert len(visible) == 1

    def test_end_less_than_start_ignored(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        model.filter_by_block_range(20, 10)
        visible = model.get_visible_alerts()
        assert len(visible) == 1

    def test_both_negative_ignored(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert())
        model.filter_by_block_range(-5, -1)
        visible = model.get_visible_alerts()
        assert len(visible) == 1


class TestAlertSortingByBlock:
    """Test alert sorting with block context (by block index option)."""

    def test_sort_ascending_block_index(self):
        model = AlertTimelineModel()
        blocks = [30, 10, 20, 5, 50]
        for b in blocks:
            model.add_alert(_make_alert())
            model.set_alert_gcode_context(0, _make_context(block_index=b))

        sorted_alerts = model.get_alerts_sorted_by_block()
        result_blocks = [t[2].block_index for t in sorted_alerts]
        assert result_blocks == sorted(result_blocks)

    def test_sort_puts_no_context_last(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert(alert_type="no_ctx"))
        model.add_alert(_make_alert(alert_type="with_ctx"))
        model.set_alert_gcode_context(0, _make_context(block_index=5))

        sorted_alerts = model.get_alerts_sorted_by_block()
        assert sorted_alerts[0][2] is not None  # context first
        assert sorted_alerts[1][2] is None  # no context last

    def test_sort_stable_for_same_block(self):
        model = AlertTimelineModel()
        model.add_alert(_make_alert(alert_type="first"))
        model.set_alert_gcode_context(0, _make_context(block_index=10))
        model.add_alert(_make_alert(alert_type="second"))
        model.set_alert_gcode_context(0, _make_context(block_index=10))

        sorted_alerts = model.get_alerts_sorted_by_block()
        assert len(sorted_alerts) == 2
        # Both at block 10, order preserved from original list
        assert all(t[2].block_index == 10 for t in sorted_alerts)
