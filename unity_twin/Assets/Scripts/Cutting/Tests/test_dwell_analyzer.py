"""Tests for DwellAnalyzer logic (Python mirror of C# implementation).

Validates G4 parsing, implicit dwell detection, excessive dwell flagging,
percentage calculation, and optimization suggestions.
"""

import pytest
from dataclasses import dataclass, field
from typing import List
from enum import Enum


# ---- Python mirror of C# types ----

class DwellType(Enum):
    EXPLICIT = 'explicit'
    IMPLICIT = 'implicit'


@dataclass
class DwellEvent:
    block_index: int
    dwell_type: DwellType
    duration_seconds: float
    raw_line: str
    is_excessive: bool = False


@dataclass
class DwellOptimizationSuggestion:
    description: str
    block_index: int
    time_saving_seconds: float = 0.0


@dataclass
class DwellReport:
    total_dwell_time_seconds: float = 0.0
    dwell_count: int = 0
    excessive_dwell_count: int = 0
    implicit_dwell_count: int = 0
    dwell_percentage: float = 0.0
    total_program_time_seconds: float = 0.0
    events: List[DwellEvent] = field(default_factory=list)
    suggestions: List[DwellOptimizationSuggestion] = field(default_factory=list)


def _extract_number(line: str, start: int) -> str:
    end = start
    has_dot = False
    if end < len(line) and line[end] == '-':
        end += 1
    while end < len(line) and (line[end].isdigit() or (line[end] == '.' and not has_dot)):
        if line[end] == '.':
            has_dot = True
        end += 1
    return line[start:end]


def parse_g4_dwell(line: str) -> float:
    """Parse G4 dwell command. Returns seconds, or 0."""
    upper = line.upper()
    if 'G4' not in upper and 'G04' not in upper:
        return 0.0

    # P parameter = milliseconds
    p_idx = upper.find('P')
    if p_idx >= 0:
        num_str = _extract_number(upper, p_idx + 1)
        try:
            return float(num_str) / 1000.0
        except ValueError:
            pass

    # X parameter = seconds
    x_idx = upper.find('X')
    if x_idx >= 0:
        num_str = _extract_number(upper, x_idx + 1)
        try:
            return float(num_str)
        except ValueError:
            pass

    return 0.0


def _parse_position(line: str, current: tuple) -> tuple:
    x, y, z = current
    upper = line.upper()
    xi = upper.find('X')
    yi = upper.find('Y')
    zi = upper.find('Z')
    if xi >= 0:
        try:
            x = float(_extract_number(upper, xi + 1))
        except ValueError:
            pass
    if yi >= 0:
        try:
            y = float(_extract_number(upper, yi + 1))
        except ValueError:
            pass
    if zi >= 0:
        try:
            z = float(_extract_number(upper, zi + 1))
        except ValueError:
            pass
    return (x, y, z)


class DwellAnalyzer:
    def __init__(self, excessive_threshold_sec: float = 2.0):
        self._threshold = excessive_threshold_sec

    def analyze(self, lines: List[str], total_program_time_sec: float = 0.0) -> DwellReport:
        report = DwellReport(total_program_time_seconds=total_program_time_sec)

        last_pos = (0.0, 0.0, 0.0)
        has_last = False
        consecutive_same = 0

        for i, raw_line in enumerate(lines):
            line = raw_line.strip().upper()
            if not line or line.startswith('%') or line.startswith('('):
                continue

            dwell_sec = parse_g4_dwell(line)
            if dwell_sec > 0:
                evt = DwellEvent(
                    block_index=i,
                    dwell_type=DwellType.EXPLICIT,
                    duration_seconds=dwell_sec,
                    raw_line=raw_line,
                    is_excessive=dwell_sec > self._threshold,
                )
                report.events.append(evt)
                report.total_dwell_time_seconds += dwell_sec
                report.dwell_count += 1
                if evt.is_excessive:
                    report.excessive_dwell_count += 1
                continue

            pos = _parse_position(line, last_pos)
            if has_last and pos == last_pos:
                consecutive_same += 1
                if consecutive_same == 2:
                    evt = DwellEvent(
                        block_index=i - 2,
                        dwell_type=DwellType.IMPLICIT,
                        duration_seconds=0.0,
                        raw_line=f'(implicit dwell: {consecutive_same + 1} blocks at same position)',
                    )
                    report.events.append(evt)
                    report.implicit_dwell_count += 1
                    report.dwell_count += 1
            else:
                consecutive_same = 0

            last_pos = pos
            has_last = True

        if total_program_time_sec > 0:
            report.dwell_percentage = (report.total_dwell_time_seconds / total_program_time_sec) * 100.0

        self._generate_suggestions(report, lines)
        return report

    def _generate_suggestions(self, report: DwellReport, lines: List[str]):
        for i, evt in enumerate(report.events):
            if evt.dwell_type != DwellType.EXPLICIT:
                continue

            # Dwell before rapid
            next_idx = evt.block_index + 1
            while next_idx < len(lines):
                nxt = lines[next_idx].strip().upper()
                if not nxt or nxt.startswith('('):
                    next_idx += 1
                    continue
                if 'G0 ' in nxt or nxt.startswith('G0') or 'G00' in nxt:
                    report.suggestions.append(DwellOptimizationSuggestion(
                        description='Dwell before rapid move may be unnecessary',
                        block_index=evt.block_index,
                        time_saving_seconds=evt.duration_seconds,
                    ))
                break

            if evt.is_excessive:
                report.suggestions.append(DwellOptimizationSuggestion(
                    description=f'Excessive dwell ({evt.duration_seconds:.1f}s > {self._threshold:.1f}s threshold)',
                    block_index=evt.block_index,
                    time_saving_seconds=evt.duration_seconds - self._threshold,
                ))

        # Consecutive dwell consolidation
        for i in range(1, len(report.events)):
            if (report.events[i].dwell_type == DwellType.EXPLICIT and
                    report.events[i - 1].dwell_type == DwellType.EXPLICIT and
                    report.events[i].block_index == report.events[i - 1].block_index + 1):
                report.suggestions.append(DwellOptimizationSuggestion(
                    description='Consecutive dwells could be consolidated into one',
                    block_index=report.events[i].block_index,
                ))


# ---- Tests ----

@pytest.fixture
def analyzer():
    return DwellAnalyzer()


def test_g4_p_parameter_milliseconds():
    """G4 P1000 = 1.0 second dwell."""
    assert parse_g4_dwell('G4 P1000') == pytest.approx(1.0)
    assert parse_g4_dwell('G04 P500') == pytest.approx(0.5)


def test_g4_x_parameter_seconds():
    """G4 X2.5 = 2.5 second dwell."""
    assert parse_g4_dwell('G4 X2.5') == pytest.approx(2.5)


def test_non_dwell_line():
    assert parse_g4_dwell('G1 X10 Y20 F200') == 0.0
    assert parse_g4_dwell('M3 S8000') == 0.0


def test_excessive_dwell_detection(analyzer):
    lines = [
        'G1 X10 Y0 F200',
        'G4 P5000',   # 5s > 2s threshold
        'G1 X20 Y0',
    ]
    report = analyzer.analyze(lines)
    assert report.excessive_dwell_count == 1
    assert report.events[0].is_excessive is True


def test_normal_dwell_not_excessive(analyzer):
    lines = ['G4 P500']  # 0.5s < 2s
    report = analyzer.analyze(lines)
    assert report.excessive_dwell_count == 0
    assert report.events[0].is_excessive is False


def test_implicit_dwell_detection(analyzer):
    """3+ blocks at the same position = implicit dwell."""
    lines = [
        'G1 X10 Y20 Z-5 F200',
        'M8',   # no position change
        'M3 S8000',  # no position change
        'G1 X20 Y20',
    ]
    report = analyzer.analyze(lines)
    assert report.implicit_dwell_count == 1


def test_dwell_percentage(analyzer):
    lines = ['G4 P2000', 'G4 P3000']  # 2s + 3s = 5s total dwell
    report = analyzer.analyze(lines, total_program_time_sec=50.0)
    assert report.dwell_percentage == pytest.approx(10.0)


def test_dwell_optimization_before_rapid(analyzer):
    lines = [
        'G1 X10 Y0 F200',
        'G4 P1000',
        'G0 X0 Y0',
    ]
    report = analyzer.analyze(lines)
    descs = [s.description for s in report.suggestions]
    assert any('rapid' in d.lower() for d in descs)


def test_empty_program(analyzer):
    report = analyzer.analyze([])
    assert report.dwell_count == 0
    assert report.total_dwell_time_seconds == 0.0
    assert len(report.events) == 0


def test_mixed_program(analyzer):
    """Program with both explicit and implicit dwells."""
    lines = [
        'G0 X0 Y0 Z5',
        'G1 Z-2 F100',
        'G4 P1500',      # explicit 1.5s
        'G1 X10 Y0',
        'G1 X10 Y0',     # same pos
        'G1 X10 Y0',     # same pos (implicit dwell starts)
        'G1 X10 Y0',     # same pos
        'G4 P3000',      # explicit 3s (excessive)
        'G1 X20 Y10',
    ]
    report = analyzer.analyze(lines, total_program_time_sec=30.0)
    assert report.dwell_count >= 3  # 2 explicit + 1 implicit
    assert report.excessive_dwell_count == 1
    assert report.implicit_dwell_count == 1
    assert report.total_dwell_time_seconds == pytest.approx(4.5)


def test_consecutive_dwell_consolidation(analyzer):
    lines = [
        'G4 P500',
        'G4 P500',
    ]
    report = analyzer.analyze(lines)
    descs = [s.description for s in report.suggestions]
    assert any('consolidated' in d.lower() for d in descs)


def test_custom_threshold():
    analyzer = DwellAnalyzer(excessive_threshold_sec=1.0)
    lines = ['G4 P1500']  # 1.5s > 1.0s
    report = analyzer.analyze(lines)
    assert report.excessive_dwell_count == 1
