"""
Circuit Breaker pattern implementation for the MIRACLE system.

Protects downstream services from cascading failures by tracking
consecutive errors and temporarily blocking calls once a failure
threshold is reached.

State machine
-------------
CLOSED  -- normal operation; calls are forwarded.
          After ``failure_threshold`` consecutive failures the breaker
          transitions to OPEN.

OPEN    -- all calls are immediately rejected with ``CircuitOpenError``.
          After ``recovery_timeout`` seconds the breaker transitions to
          HALF_OPEN.

HALF_OPEN -- a limited number of *probe* calls are allowed through.
            After ``half_open_max`` consecutive successes the breaker
            resets to CLOSED.  A single failure during this phase sends
            it back to OPEN.
"""

from enum import Enum, auto
from typing import Any, Callable, TypeVar
import threading
import time

T = TypeVar('T')


class CircuitState(Enum):
    """Possible states of the circuit breaker."""

    CLOSED = auto()
    OPEN = auto()
    HALF_OPEN = auto()


class CircuitOpenError(Exception):
    """Raised when a call is attempted while the circuit breaker is OPEN."""

    def __init__(self, remaining: float = 0.0) -> None:
        self.remaining = remaining
        super().__init__(
            f"Circuit breaker is OPEN; retry in {remaining:.1f}s"
        )


class CircuitBreaker:
    """Thread-safe circuit breaker.

    Parameters
    ----------
    failure_threshold:
        Number of consecutive failures in the CLOSED state before
        transitioning to OPEN.  Default ``5``.
    recovery_timeout:
        Seconds to wait in the OPEN state before transitioning to
        HALF_OPEN.  Default ``30.0``.
    half_open_max:
        Number of consecutive successes required in the HALF_OPEN
        state to transition back to CLOSED.  Default ``3``.
    name:
        Optional human-readable label used in log / error messages.

    Usage
    -----
    >>> cb = CircuitBreaker(failure_threshold=3, recovery_timeout=10)
    >>> result = cb.call(some_function, arg1, kwarg=val)
    """

    def __init__(
        self,
        failure_threshold: int = 5,
        recovery_timeout: float = 30.0,
        half_open_max: int = 3,
        name: str = 'default',
    ) -> None:
        self._failure_threshold = failure_threshold
        self._recovery_timeout = recovery_timeout
        self._half_open_max = half_open_max
        self._name = name

        self._state: CircuitState = CircuitState.CLOSED
        self._failure_count: int = 0
        self._success_count: int = 0
        self._last_failure_time: float = 0.0
        self._lock = threading.Lock()

    # -- Public API ----------------------------------------------------

    @property
    def state(self) -> CircuitState:
        """Return the current circuit breaker state.

        Automatically transitions from OPEN to HALF_OPEN if the
        recovery timeout has elapsed.
        """
        with self._lock:
            self._maybe_transition_to_half_open()
            return self._state

    @property
    def failure_count(self) -> int:
        """Return the current consecutive failure count."""
        with self._lock:
            return self._failure_count

    def call(self, func: Callable[..., T], *args: Any, **kwargs: Any) -> T:
        """Execute *func* through the circuit breaker.

        Parameters
        ----------
        func:
            The callable to invoke.
        *args, **kwargs:
            Forwarded to *func*.

        Returns
        -------
        The return value of *func*.

        Raises
        ------
        CircuitOpenError:
            If the circuit is in the OPEN state and the recovery
            timeout has not elapsed.
        Exception:
            Any exception raised by *func* is re-raised after the
            breaker records the failure.
        """
        with self._lock:
            self._maybe_transition_to_half_open()

            if self._state == CircuitState.OPEN:
                remaining = self._recovery_timeout - (
                    time.monotonic() - self._last_failure_time
                )
                raise CircuitOpenError(remaining=max(remaining, 0.0))

        # Execute outside the lock to avoid holding it during I/O
        try:
            result = func(*args, **kwargs)
        except Exception as exc:
            self._record_failure()
            raise exc
        else:
            self._record_success()
            return result

    def reset(self) -> None:
        """Manually reset the breaker to the CLOSED state."""
        with self._lock:
            self._state = CircuitState.CLOSED
            self._failure_count = 0
            self._success_count = 0
            self._last_failure_time = 0.0

    # -- Internal helpers -----------------------------------------------

    def _maybe_transition_to_half_open(self) -> None:
        """Transition OPEN -> HALF_OPEN when the recovery timeout elapses.

        Must be called while holding ``_lock``.
        """
        if self._state == CircuitState.OPEN:
            elapsed = time.monotonic() - self._last_failure_time
            if elapsed >= self._recovery_timeout:
                self._state = CircuitState.HALF_OPEN
                self._success_count = 0

    def _record_failure(self) -> None:
        """Record a failed call and transition if necessary."""
        with self._lock:
            self._failure_count += 1
            self._last_failure_time = time.monotonic()

            if self._state == CircuitState.HALF_OPEN:
                # Any failure in HALF_OPEN sends us straight back to OPEN
                self._state = CircuitState.OPEN
                self._success_count = 0
            elif self._state == CircuitState.CLOSED:
                if self._failure_count >= self._failure_threshold:
                    self._state = CircuitState.OPEN

    def _record_success(self) -> None:
        """Record a successful call and transition if necessary."""
        with self._lock:
            if self._state == CircuitState.HALF_OPEN:
                self._success_count += 1
                if self._success_count >= self._half_open_max:
                    # Enough successful probes -- close the circuit
                    self._state = CircuitState.CLOSED
                    self._failure_count = 0
                    self._success_count = 0
            elif self._state == CircuitState.CLOSED:
                # A success resets the failure counter
                self._failure_count = 0

    def __repr__(self) -> str:
        return (
            f"CircuitBreaker(name={self._name!r}, state={self._state.name}, "
            f"failures={self._failure_count})"
        )
