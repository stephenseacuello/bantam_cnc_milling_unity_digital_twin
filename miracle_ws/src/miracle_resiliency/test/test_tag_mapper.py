"""Tests for OPC-UA Tag Mapper.

Tests tag creation, value transformation, discovery, duplicate detection,
JSON round-trip, controller templates, health monitoring, and validation.
"""

import sys
import os
import json
import time
import tempfile
from unittest.mock import MagicMock

for mod in [
    'miracle_core.datatypes', 'miracle_core.constants',
    'rclpy', 'rclpy.node', 'rclpy.lifecycle', 'rclpy.qos',
    'rclpy.callback_groups',
    'miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
    'miracle_msgs', 'miracle_msgs.msg',
    'std_msgs', 'std_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())

import pytest
from miracle_resiliency.recovery_orchestrator import (
    OpcUaTagMapper,
    TagMapping,
    TagGroup,
    ControllerType,
)


@pytest.fixture
def mapper():
    return OpcUaTagMapper()


@pytest.fixture
def sample_tag():
    return TagMapping(
        opc_node_id='ns=2;s=Channel1.SpindleSpeed',
        internal_name='spindle_speed',
        data_type='float',
        scaling_factor=1.0,
        offset=0.0,
        unit='rpm',
        description='Spindle speed',
        poll_rate_ms=100,
    )


# ---- Tag creation ----

def test_add_and_get_tag(mapper, sample_tag):
    assert mapper.add_tag(sample_tag) is True
    retrieved = mapper.get_tag('spindle_speed')
    assert retrieved is not None
    assert retrieved.opc_node_id == 'ns=2;s=Channel1.SpindleSpeed'
    assert retrieved.unit == 'rpm'


def test_duplicate_tag_rejected(mapper, sample_tag):
    mapper.add_tag(sample_tag)
    assert mapper.add_tag(sample_tag) is False
    assert len(mapper.get_all_tags()) == 1


def test_add_tag_group(mapper):
    group = TagGroup(
        group_name='axes',
        tags=[
            TagMapping('ns=2;s=Axis.X', 'axis_x', 'float', 1.0, 0.0, 'mm'),
            TagMapping('ns=2;s=Axis.Y', 'axis_y', 'float', 1.0, 0.0, 'mm'),
        ],
        poll_rate_ms=50,
    )
    mapper.add_group(group)
    assert len(mapper.get_all_tags()) == 2
    assert mapper.get_tag('axis_x') is not None


# ---- Value transformation ----

def test_transform_value_with_scaling(mapper):
    tag = TagMapping('ns=2;s=Sensor.Temp', 'temperature', 'float',
                     scaling_factor=0.1, offset=-40.0, unit='°C')
    mapper.add_tag(tag)
    # result = 500 * 0.1 + (-40) = 10.0
    result = mapper.transform_value('temperature', 500.0)
    assert result == pytest.approx(10.0)


def test_transform_value_identity(mapper, sample_tag):
    mapper.add_tag(sample_tag)
    result = mapper.transform_value('spindle_speed', 8000.0)
    assert result == pytest.approx(8000.0)


def test_transform_unknown_tag(mapper):
    assert mapper.transform_value('nonexistent', 100.0) is None


# ---- Discovery simulation ----

def test_discover_tags(mapper):
    tags = mapper.discover_tags('opc.tcp://192.168.1.10/CNC')
    assert len(tags) == 8
    names = {t.internal_name for t in tags}
    assert 'spindle_speed' in names
    assert 'feed_rate' in names
    assert 'axis_x' in names
    assert 'tool_number' in names


# ---- Duplicate OPC node detection ----

def test_duplicate_opc_node_id_validation(mapper):
    t1 = TagMapping('ns=2;s=Same.Node', 'signal_a', 'float')
    t2 = TagMapping('ns=2;s=Same.Node', 'signal_b', 'float')
    mapper.add_tag(t1)
    mapper.add_tag(t2)
    result = mapper.validate()
    assert result.is_valid is False
    assert any('Duplicate OPC node ID' in e for e in result.errors)


# ---- JSON round-trip ----

def test_json_save_load_roundtrip(mapper, sample_tag):
    mapper.add_tag(sample_tag)
    mapper.add_tag(TagMapping('ns=2;s=FeedRate', 'feed_rate', 'float',
                              scaling_factor=1.0, unit='mm/min'))

    with tempfile.NamedTemporaryFile(suffix='.json', delete=False, mode='w') as f:
        path = f.name

    try:
        mapper.save_to_json(path)

        mapper2 = OpcUaTagMapper()
        count = mapper2.load_from_json(path)
        assert count == 2
        assert mapper2.get_tag('spindle_speed') is not None
        assert mapper2.get_tag('feed_rate') is not None
    finally:
        os.unlink(path)


# ---- Controller templates ----

def test_fanuc_template(mapper):
    count = mapper.load_controller_template(ControllerType.FANUC)
    assert count == 8
    tag = mapper.get_tag('spindle_speed')
    assert tag is not None
    assert 'GnCNC' in tag.opc_node_id


def test_siemens_template(mapper):
    count = mapper.load_controller_template(ControllerType.SIEMENS)
    assert count == 8
    tag = mapper.get_tag('axis_x')
    assert 'Sinumerik' in tag.opc_node_id


def test_haas_template(mapper):
    count = mapper.load_controller_template(ControllerType.HAAS)
    assert count == 8
    tag = mapper.get_tag('coolant_status')
    assert 'Haas' in tag.opc_node_id


# ---- Tag health ----

def test_tag_health_stale(mapper, sample_tag):
    mapper.add_tag(sample_tag)
    # No updates recorded, so tag should be stale
    health = mapper.get_tag_health()
    assert len(health) == 1
    assert health[0].is_stale is True
    assert health[0].tag_name == 'spindle_speed'


def test_tag_health_fresh(mapper, sample_tag):
    mapper.add_tag(sample_tag)
    # Transform records an update
    mapper.transform_value('spindle_speed', 8000.0)
    health = mapper.get_tag_health()
    assert len(health) == 1
    assert health[0].is_stale is False


# ---- Required tag validation ----

def test_validation_missing_required_tags(mapper):
    # Only add one tag — many required tags will be missing
    mapper.add_tag(TagMapping('ns=2;s=Speed', 'spindle_speed', 'float'))
    result = mapper.validate()
    assert result.is_valid is True  # warnings don't invalidate
    assert len(result.warnings) >= 5  # at least feed_rate, axis_x/y/z, tool_number, coolant missing


def test_validation_all_required_present(mapper):
    mapper.load_controller_template(ControllerType.GENERIC)
    result = mapper.validate()
    assert result.is_valid is True
    # No warnings about missing required tags
    missing_warnings = [w for w in result.warnings if 'Required tag' in w]
    assert len(missing_warnings) == 0


def test_invalid_data_type(mapper):
    mapper.add_tag(TagMapping('ns=2;s=Bad', 'bad_tag', 'complex'))
    result = mapper.validate()
    assert result.is_valid is False
    assert any('invalid data_type' in e for e in result.errors)
