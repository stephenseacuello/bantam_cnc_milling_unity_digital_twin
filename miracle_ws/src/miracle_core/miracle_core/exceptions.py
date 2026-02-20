"""Custom exceptions for the MIRACLE system."""


class MiracleError(Exception):
    """Base exception for all MIRACLE errors."""
    pass


class ConfigurationError(MiracleError):
    """Raised when a node configuration is invalid."""
    pass


class ParameterValidationError(MiracleError):
    """Raised when a parameter fails validation."""
    pass


class CommunicationError(MiracleError):
    """Raised when inter-node communication fails."""
    pass


class SafetyViolationError(MiracleError):
    """Raised when a safety constraint is violated."""
    pass


class SecurityError(MiracleError):
    """Raised when a security violation is detected."""
    pass


class RecoveryError(MiracleError):
    """Raised when recovery from a failure is not possible."""
    pass


class CalibrationError(MiracleError):
    """Raised when calibration fails or produces invalid results."""
    pass


class GCodeError(MiracleError):
    """Raised when G-code parsing or validation fails."""
    pass


class ModelError(MiracleError):
    """Raised when an ML model fails to load or predict."""
    pass


class TimeoutError(MiracleError):
    """Raised when an operation times out."""
    pass
