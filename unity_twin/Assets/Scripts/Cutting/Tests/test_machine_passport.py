"""Tests for MachineDigitalPassport logic (Python mirror of C# implementation).

Validates machine identity, maintenance tracking, calibration status,
health summary, MTBF calculation, and calibration-due checks.
"""

import pytest
from dataclasses import dataclass, field
from typing import List, Dict, Optional
from datetime import datetime, timedelta


# ---- Python mirror of C# types ----

@dataclass
class MachineIdentity:
    serial_number: str = ""
    manufacturer: str = ""
    model: str = ""
    year_of_manufacture: int = 0
    controller_type: str = ""
    max_spindle_rpm: float = 0.0
    max_feed_rate: float = 0.0
    work_envelope: tuple = (0.0, 0.0, 0.0)  # (X, Y, Z) in mm
    axis_count: int = 3


@dataclass
class MaintenanceRecord:
    record_id: str = ""
    date: str = ""              # ISO-8601 date string
    type: str = ""              # preventive | corrective | predictive
    description: str = ""
    technician: str = ""
    parts_replaced: List[str] = field(default_factory=list)
    downtime_hours: float = 0.0
    cost: float = 0.0


@dataclass
class CalibrationRecord:
    record_id: str = ""
    date: str = ""              # ISO-8601 date string
    parameter: str = ""         # e.g. "X_axis_backlash", "spindle_runout"
    measured_value: float = 0.0
    nominal_value: float = 0.0
    tolerance: float = 0.0
    passed: bool = True
    calibrated_by: str = ""


@dataclass
class HealthSummary:
    uptime_percentage: float = 100.0
    calibration_status: Dict[str, bool] = field(default_factory=dict)
    maintenance_compliance_percentage: float = 100.0
    mtbf: float = 0.0
    total_parts_produced: int = 0
    total_operating_hours: float = 0.0


class MachineDigitalPassport:
    """Comprehensive digital passport for a CNC machine tool.

    Tracks identity, maintenance history, calibration records,
    operating hours, and parts produced over the machine's lifecycle.
    """

    def __init__(self, identity: MachineIdentity):
        if identity is None:
            raise ValueError("identity must not be None")
        self.identity = identity
        self.maintenance_history: List[MaintenanceRecord] = []
        self.calibration_history: List[CalibrationRecord] = []
        self.total_operating_hours: float = 0.0
        self.total_parts_produced: int = 0

    def add_maintenance_record(self, record: MaintenanceRecord) -> None:
        """Append a maintenance record to the history."""
        if record is None:
            raise ValueError("record must not be None")
        self.maintenance_history.append(record)

    def add_calibration_record(self, record: CalibrationRecord) -> None:
        """Append a calibration record to the history."""
        if record is None:
            raise ValueError("record must not be None")
        self.calibration_history.append(record)

    def get_maintenance_by_type(self, mtype: str) -> List[MaintenanceRecord]:
        """Filter maintenance records by type (preventive, corrective, predictive)."""
        return [r for r in self.maintenance_history if r.type == mtype]

    def get_calibration_status(self) -> Dict[str, bool]:
        """Returns a dict mapping each calibrated parameter to the pass/fail
        result of its most recent calibration."""
        status: Dict[str, bool] = {}
        for record in self.calibration_history:
            # Last one written wins (mirrors C# GroupBy + Last)
            status[record.parameter] = record.passed
        return status

    def get_health_summary(self) -> HealthSummary:
        """Overall machine health summary: uptime %, calibration status,
        maintenance compliance, MTBF, and production statistics."""
        total_downtime = sum(r.downtime_hours for r in self.maintenance_history)
        if self.total_operating_hours > 0:
            uptime = ((self.total_operating_hours - total_downtime)
                      / self.total_operating_hours) * 100.0
        else:
            uptime = 100.0
        uptime = max(0.0, min(100.0, uptime))

        cal_status = self.get_calibration_status()

        compliance = 100.0
        if cal_status:
            passed_count = sum(1 for v in cal_status.values() if v)
            compliance = (passed_count / len(cal_status)) * 100.0

        return HealthSummary(
            uptime_percentage=uptime,
            calibration_status=cal_status,
            maintenance_compliance_percentage=compliance,
            mtbf=self.get_mtbf(),
            total_parts_produced=self.total_parts_produced,
            total_operating_hours=self.total_operating_hours,
        )

    def is_calibration_due(self, parameter: str, interval_days: float) -> bool:
        """Check whether a specific calibration parameter is due for recalibration.

        Returns True if no calibration exists for the parameter or if the last
        calibration was performed more than interval_days ago.
        """
        records = [r for r in self.calibration_history if r.parameter == parameter]
        if not records:
            return True

        last_record = records[-1]
        try:
            last_date = datetime.fromisoformat(last_record.date)
        except (ValueError, TypeError):
            return True

        days_since = (datetime.now() - last_date).total_seconds() / 86400.0
        return days_since >= interval_days

    def get_mtbf(self) -> float:
        """Mean Time Between Failures from corrective maintenance records.

        Returns 0 if fewer than 2 corrective records or dates cannot be parsed.
        MTBF = average interval (hours) between consecutive corrective events.
        """
        corrective = [r for r in self.maintenance_history if r.type == "corrective"]
        if len(corrective) < 2:
            return 0.0

        dates = []
        for record in corrective:
            try:
                dates.append(datetime.fromisoformat(record.date))
            except (ValueError, TypeError):
                pass

        if len(dates) < 2:
            return 0.0

        dates.sort()

        total_hours = sum(
            (dates[i] - dates[i - 1]).total_seconds() / 3600.0
            for i in range(1, len(dates))
        )
        return total_hours / (len(dates) - 1)


# ---- Fixtures ----

@pytest.fixture
def identity():
    return MachineIdentity(
        serial_number="BNT-5000-001",
        manufacturer="Banatam",
        model="VMC-5000",
        year_of_manufacture=2023,
        controller_type="FANUC 31i-B5",
        max_spindle_rpm=15000.0,
        max_feed_rate=30000.0,
        work_envelope=(1020.0, 610.0, 510.0),
        axis_count=5,
    )


@pytest.fixture
def passport(identity):
    return MachineDigitalPassport(identity)


# ---- Tests ----

def test_identity_stored(passport, identity):
    """Machine identity is correctly stored in the passport."""
    assert passport.identity.serial_number == "BNT-5000-001"
    assert passport.identity.manufacturer == "Banatam"
    assert passport.identity.model == "VMC-5000"
    assert passport.identity.year_of_manufacture == 2023
    assert passport.identity.axis_count == 5
    assert passport.identity.max_spindle_rpm == 15000.0
    assert passport.identity.work_envelope == (1020.0, 610.0, 510.0)


def test_add_maintenance_record(passport):
    """Maintenance records can be added and retrieved."""
    rec = MaintenanceRecord(
        record_id="M001",
        date="2025-06-15",
        type="preventive",
        description="Spindle bearing lubrication",
        technician="J. Smith",
        parts_replaced=["bearing_grease"],
        downtime_hours=2.0,
        cost=150.0,
    )
    passport.add_maintenance_record(rec)
    assert len(passport.maintenance_history) == 1
    assert passport.maintenance_history[0].record_id == "M001"


def test_get_maintenance_by_type(passport):
    """Filtering maintenance records by type returns correct subset."""
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M001", date="2025-01-10", type="preventive",
        description="Scheduled PM", technician="A"))
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M002", date="2025-02-20", type="corrective",
        description="Spindle fault repair", technician="B"))
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M003", date="2025-03-15", type="preventive",
        description="Coolant flush", technician="A"))
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M004", date="2025-04-01", type="predictive",
        description="Vibration-based bearing check", technician="C"))

    preventive = passport.get_maintenance_by_type("preventive")
    corrective = passport.get_maintenance_by_type("corrective")
    predictive = passport.get_maintenance_by_type("predictive")

    assert len(preventive) == 2
    assert len(corrective) == 1
    assert len(predictive) == 1
    assert corrective[0].record_id == "M002"


def test_calibration_status_latest_wins(passport):
    """Calibration status returns the latest result per parameter."""
    passport.add_calibration_record(CalibrationRecord(
        record_id="C001", date="2025-01-01", parameter="X_axis_backlash",
        measured_value=0.012, nominal_value=0.0, tolerance=0.015,
        passed=True, calibrated_by="Tech A"))
    passport.add_calibration_record(CalibrationRecord(
        record_id="C002", date="2025-06-01", parameter="X_axis_backlash",
        measured_value=0.018, nominal_value=0.0, tolerance=0.015,
        passed=False, calibrated_by="Tech A"))
    passport.add_calibration_record(CalibrationRecord(
        record_id="C003", date="2025-03-01", parameter="spindle_runout",
        measured_value=0.002, nominal_value=0.0, tolerance=0.005,
        passed=True, calibrated_by="Tech B"))

    status = passport.get_calibration_status()
    assert status["X_axis_backlash"] is False   # latest failed
    assert status["spindle_runout"] is True


def test_health_summary_uptime(passport):
    """Health summary correctly calculates uptime percentage."""
    passport.total_operating_hours = 1000.0
    passport.total_parts_produced = 500

    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M001", date="2025-01-10", type="corrective",
        description="Repair", technician="A", downtime_hours=50.0, cost=2000.0))
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M002", date="2025-03-10", type="preventive",
        description="PM", technician="B", downtime_hours=10.0, cost=200.0))

    summary = passport.get_health_summary()
    # Total downtime = 60h, operating = 1000h => uptime = 94%
    assert summary.uptime_percentage == pytest.approx(94.0)
    assert summary.total_parts_produced == 500
    assert summary.total_operating_hours == 1000.0


def test_health_summary_maintenance_compliance(passport):
    """Health summary computes maintenance compliance from calibration pass rate."""
    passport.total_operating_hours = 500.0

    passport.add_calibration_record(CalibrationRecord(
        record_id="C001", date="2025-06-01", parameter="X_backlash",
        measured_value=0.01, nominal_value=0.0, tolerance=0.015,
        passed=True, calibrated_by="A"))
    passport.add_calibration_record(CalibrationRecord(
        record_id="C002", date="2025-06-01", parameter="Y_backlash",
        measured_value=0.02, nominal_value=0.0, tolerance=0.015,
        passed=False, calibrated_by="A"))
    passport.add_calibration_record(CalibrationRecord(
        record_id="C003", date="2025-06-01", parameter="spindle_runout",
        measured_value=0.003, nominal_value=0.0, tolerance=0.005,
        passed=True, calibrated_by="A"))

    summary = passport.get_health_summary()
    # 2 out of 3 parameters passed => 66.67%
    assert summary.maintenance_compliance_percentage == pytest.approx(66.6667, rel=1e-3)


def test_is_calibration_due_no_records(passport):
    """Calibration is due when no records exist for a parameter."""
    assert passport.is_calibration_due("X_axis_backlash", 90) is True


def test_is_calibration_due_recent(passport):
    """Calibration is NOT due when last calibration is within the interval."""
    yesterday = (datetime.now() - timedelta(days=1)).strftime("%Y-%m-%d")
    passport.add_calibration_record(CalibrationRecord(
        record_id="C001", date=yesterday, parameter="spindle_runout",
        measured_value=0.002, nominal_value=0.0, tolerance=0.005,
        passed=True, calibrated_by="Tech A"))

    assert passport.is_calibration_due("spindle_runout", 90) is False


def test_is_calibration_due_expired(passport):
    """Calibration IS due when last calibration exceeds the interval."""
    old_date = (datetime.now() - timedelta(days=100)).strftime("%Y-%m-%d")
    passport.add_calibration_record(CalibrationRecord(
        record_id="C001", date=old_date, parameter="spindle_runout",
        measured_value=0.002, nominal_value=0.0, tolerance=0.005,
        passed=True, calibrated_by="Tech A"))

    assert passport.is_calibration_due("spindle_runout", 90) is True


def test_mtbf_calculation(passport):
    """MTBF is correctly computed from corrective maintenance dates."""
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M001", date="2025-01-01T00:00:00", type="corrective",
        description="Failure 1", technician="A"))
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M002", date="2025-01-11T00:00:00", type="corrective",
        description="Failure 2", technician="A"))
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M003", date="2025-01-31T00:00:00", type="corrective",
        description="Failure 3", technician="B"))
    # Preventive records should be ignored for MTBF
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M004", date="2025-01-05T00:00:00", type="preventive",
        description="PM", technician="A"))

    mtbf = passport.get_mtbf()
    # Jan 1 -> Jan 11 = 240h, Jan 11 -> Jan 31 = 480h => avg = 360h
    assert mtbf == pytest.approx(360.0)


def test_mtbf_insufficient_records(passport):
    """MTBF returns 0 when fewer than 2 corrective records exist."""
    passport.add_maintenance_record(MaintenanceRecord(
        record_id="M001", date="2025-01-01", type="corrective",
        description="Single failure", technician="A"))

    assert passport.get_mtbf() == 0.0


def test_empty_passport_health_summary(passport):
    """Health summary on a fresh passport returns safe defaults."""
    summary = passport.get_health_summary()
    assert summary.uptime_percentage == 100.0
    assert summary.maintenance_compliance_percentage == 100.0
    assert summary.mtbf == 0.0
    assert summary.total_parts_produced == 0
    assert summary.total_operating_hours == 0.0
    assert summary.calibration_status == {}
