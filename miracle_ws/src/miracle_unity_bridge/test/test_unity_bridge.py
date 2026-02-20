"""Tests for miracle_unity_bridge package."""

import pytest
from miracle_unity_bridge.unity_endpoint import UnityEndpointConfig


def test_package_import():
    """Verify the package can be imported."""
    import miracle_unity_bridge
    assert miracle_unity_bridge is not None


def test_endpoint_config_class_exists():
    """Verify the UnityEndpointConfig class exists."""
    assert UnityEndpointConfig is not None


def test_main_function_exists():
    """Verify main entry point exists."""
    from miracle_unity_bridge.unity_endpoint import main
    assert callable(main)
