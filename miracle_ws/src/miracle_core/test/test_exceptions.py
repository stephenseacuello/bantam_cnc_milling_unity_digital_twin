"""Tests for the MIRACLE custom exception hierarchy.

Validates that all custom exceptions are properly organized in an inheritance
hierarchy rooted at MiracleError, enabling both fine-grained and broad
exception handling patterns.
"""

import pytest


# Exceptions module has no ROS2 dependencies, so import directly
from miracle_core.exceptions import (
    MiracleError,
    ConfigurationError,
    ParameterValidationError,
    CommunicationError,
    SafetyViolationError,
    SecurityError,
    RecoveryError,
    CalibrationError,
    GCodeError,
    ModelError,
    TimeoutError,
)


class TestMiracleErrorBase:
    """Test the base MiracleError exception."""

    def test_miracle_error_is_exception(self):
        """MiracleError should inherit from the built-in Exception."""
        assert issubclass(MiracleError, Exception), (
            "MiracleError must be a subclass of Exception"
        )

    def test_miracle_error_can_be_raised_and_caught(self):
        """MiracleError should be raisable and catchable."""
        with pytest.raises(MiracleError):
            raise MiracleError("test error")

    def test_miracle_error_carries_message(self):
        """MiracleError should carry a descriptive message string."""
        err = MiracleError("something went wrong")
        assert str(err) == "something went wrong"


class TestExceptionHierarchy:
    """Test that all exceptions are subclasses of MiracleError."""

    @pytest.mark.parametrize("exc_class", [
        ConfigurationError,
        ParameterValidationError,
        CommunicationError,
        SafetyViolationError,
        SecurityError,
        RecoveryError,
        CalibrationError,
        GCodeError,
        ModelError,
        TimeoutError,
    ])
    def test_subclass_of_miracle_error(self, exc_class):
        """Each custom exception should be a subclass of MiracleError."""
        assert issubclass(exc_class, MiracleError), (
            f"{exc_class.__name__} must be a subclass of MiracleError"
        )

    @pytest.mark.parametrize("exc_class", [
        ConfigurationError,
        ParameterValidationError,
        CommunicationError,
        SafetyViolationError,
        SecurityError,
        RecoveryError,
        CalibrationError,
        GCodeError,
        ModelError,
        TimeoutError,
    ])
    def test_catchable_as_miracle_error(self, exc_class):
        """All custom exceptions should be catchable via 'except MiracleError'."""
        with pytest.raises(MiracleError):
            raise exc_class("test")

    def test_specific_exception_not_caught_by_sibling(self):
        """A ConfigurationError should not be caught by 'except GCodeError'."""
        with pytest.raises(ConfigurationError):
            try:
                raise ConfigurationError("bad config")
            except GCodeError:
                pytest.fail("ConfigurationError should not be caught by GCodeError handler")


class TestExceptionMessages:
    """Test that exceptions correctly propagate messages."""

    def test_configuration_error_message(self):
        """ConfigurationError should carry its message through str()."""
        err = ConfigurationError("missing field 'machine_id'")
        assert "machine_id" in str(err)

    def test_safety_violation_error_message(self):
        """SafetyViolationError should carry its message through str()."""
        err = SafetyViolationError("spindle overload detected")
        assert "spindle overload" in str(err)

    def test_exception_chaining(self):
        """Exceptions should support 'raise ... from ...' chaining."""
        original = ValueError("original cause")
        try:
            try:
                raise original
            except ValueError as e:
                raise RecoveryError("recovery failed") from e
        except RecoveryError as exc:
            assert exc.__cause__ is original, (
                "Exception chaining should preserve the original cause"
            )
