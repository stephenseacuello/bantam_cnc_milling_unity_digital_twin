"""Tests for LifecycleClient."""
import pytest
from miracle_resiliency.lifecycle_client import (
    LifecycleClient, LifecycleTransition, LifecycleClientError,
)


class TestLifecycleTransition:
    """Test lifecycle transition enum values."""

    def test_configure_value(self):
        assert LifecycleTransition.CONFIGURE == 1

    def test_activate_value(self):
        assert LifecycleTransition.ACTIVATE == 3

    def test_deactivate_value(self):
        assert LifecycleTransition.DEACTIVATE == 4

    def test_cleanup_value(self):
        assert LifecycleTransition.CLEANUP == 2

    def test_shutdown_value(self):
        assert LifecycleTransition.SHUTDOWN == 5


class TestLifecycleClientError:
    """Test error class."""

    def test_error_message(self):
        err = LifecycleClientError("test error")
        assert str(err) == "test error"

    def test_is_exception(self):
        assert issubclass(LifecycleClientError, Exception)
