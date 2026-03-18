"""Tests for ToolMagazineManager logic (Python mirror of C# implementation).

Validates pocket management, tool loading/unloading, status reporting,
layout optimization, tool-change planning, and edge cases.
"""

import pytest
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import List, Optional, Tuple, Dict


# ---- Python mirror of C# types ----

@dataclass
class MagazinePocket:
    pocket_number: int = 0
    tool_id: Optional[str] = None
    tool_type: Optional[str] = None
    is_occupied: bool = False
    last_access_time: Optional[str] = None


@dataclass
class ToolChangeSequence:
    from_pocket: int = 0
    to_pocket: int = 0
    estimated_time_sec: float = 0.0
    is_optimized: bool = False


@dataclass
class MagazineStatus:
    total_pockets: int = 0
    occupied_pockets: int = 0
    empty_pockets: int = 0
    utilization_pct: float = 0.0
    tools: List[MagazinePocket] = field(default_factory=list)


class ToolMagazineManager:
    """Manages tool magazine pockets, tool assignments, and tool-change
    optimization for a CNC machining centre.

    Supports loading / unloading tools, querying status, optimizing pocket
    layout based on a planned usage order, and planning the load/unload
    sequence required to prepare the magazine for a new job.
    """

    BASE_CHANGE_TIME_SEC = 4.5
    TIME_PER_POCKET_SEC = 0.3

    def __init__(self, magazine_size: int = 24):
        if magazine_size <= 0:
            raise ValueError("magazine_size must be positive")
        self._magazine_size = magazine_size
        self._pockets: List[MagazinePocket] = [
            MagazinePocket(pocket_number=i + 1)
            for i in range(magazine_size)
        ]

    # -- Load / Unload -----------------------------------------------

    def load_tool(self, pocket_number: int, tool_id: str, tool_type: str) -> None:
        """Load a tool into a specific pocket."""
        if not tool_id:
            raise ValueError("tool_id must not be null or empty")
        if not tool_type:
            raise ValueError("tool_type must not be null or empty")
        pocket = self._get_pocket(pocket_number)
        if pocket.is_occupied:
            raise RuntimeError(
                f"Pocket {pocket_number} is already occupied by tool {pocket.tool_id}"
            )
        pocket.tool_id = tool_id
        pocket.tool_type = tool_type
        pocket.is_occupied = True
        pocket.last_access_time = datetime.now(timezone.utc).isoformat()

    def unload_tool(self, pocket_number: int) -> None:
        """Remove a tool from a pocket."""
        pocket = self._get_pocket(pocket_number)
        if not pocket.is_occupied:
            raise RuntimeError(f"Pocket {pocket_number} is already empty")
        pocket.tool_id = None
        pocket.tool_type = None
        pocket.is_occupied = False
        pocket.last_access_time = None

    # -- Query --------------------------------------------------------

    def find_tool(self, tool_id: str) -> int:
        """Find the pocket number holding the given tool_id. Returns -1 if not found."""
        if not tool_id:
            return -1
        for p in self._pockets:
            if p.is_occupied and p.tool_id == tool_id:
                return p.pocket_number
        return -1

    def get_next_empty_pocket(self) -> int:
        """Returns the first empty pocket number, or -1 if the magazine is full."""
        for p in self._pockets:
            if not p.is_occupied:
                return p.pocket_number
        return -1

    def get_status(self) -> MagazineStatus:
        """Build a snapshot of the magazine's current state."""
        occupied = sum(1 for p in self._pockets if p.is_occupied)
        return MagazineStatus(
            total_pockets=self._magazine_size,
            occupied_pockets=occupied,
            empty_pockets=self._magazine_size - occupied,
            utilization_pct=(occupied / self._magazine_size) * 100.0
            if self._magazine_size > 0 else 0.0,
            tools=list(self._pockets),
        )

    # -- Optimization -------------------------------------------------

    def optimize_layout(self, tool_usage_order: List[str]) -> List[ToolChangeSequence]:
        """Rearrange tools so consecutively-used tools sit in adjacent pockets."""
        if not tool_usage_order:
            return []

        moves: List[ToolChangeSequence] = []

        # Snapshot current positions
        current_positions: Dict[str, int] = {}
        for p in self._pockets:
            if p.is_occupied:
                current_positions[p.tool_id] = p.pocket_number

        target_pocket = 1
        for tool_id in tool_usage_order:
            if tool_id not in current_positions:
                target_pocket += 1
                continue

            current_pocket = current_positions[tool_id]
            if current_pocket != target_pocket:
                target_occupant = self._pockets[target_pocket - 1]
                source_pocket = self._pockets[current_pocket - 1]

                if target_occupant.is_occupied:
                    # Swap
                    swap_tool_id = target_occupant.tool_id
                    swap_tool_type = target_occupant.tool_type

                    target_occupant.tool_id = source_pocket.tool_id
                    target_occupant.tool_type = source_pocket.tool_type

                    source_pocket.tool_id = swap_tool_id
                    source_pocket.tool_type = swap_tool_type

                    if swap_tool_id is not None:
                        current_positions[swap_tool_id] = current_pocket
                    current_positions[tool_id] = target_pocket
                else:
                    # Simple move
                    target_occupant.tool_id = source_pocket.tool_id
                    target_occupant.tool_type = source_pocket.tool_type
                    target_occupant.is_occupied = True
                    target_occupant.last_access_time = datetime.now(timezone.utc).isoformat()

                    source_pocket.tool_id = None
                    source_pocket.tool_type = None
                    source_pocket.is_occupied = False
                    source_pocket.last_access_time = None

                    current_positions[tool_id] = target_pocket

                distance = abs(current_pocket - target_pocket)
                moves.append(ToolChangeSequence(
                    from_pocket=current_pocket,
                    to_pocket=target_pocket,
                    estimated_time_sec=self.BASE_CHANGE_TIME_SEC + distance * self.TIME_PER_POCKET_SEC,
                    is_optimized=True,
                ))

            target_pocket += 1

        return moves

    def plan_tool_changes(
        self, required_tools: List[Tuple[str, str]]
    ) -> List[ToolChangeSequence]:
        """Plan loads/unloads to prepare the magazine for a job's tool list."""
        if not required_tools:
            return []

        plan: List[ToolChangeSequence] = []

        present_ids = {p.tool_id for p in self._pockets if p.is_occupied}
        needed = [(tid, tt) for tid, tt in required_tools if tid not in present_ids]
        required_ids = {tid for tid, _ in required_tools}

        for tool_id, tool_type in needed:
            empty_pocket = self.get_next_empty_pocket()
            if empty_pocket != -1:
                self.load_tool(empty_pocket, tool_id, tool_type)
                plan.append(ToolChangeSequence(
                    from_pocket=-1,
                    to_pocket=empty_pocket,
                    estimated_time_sec=self.BASE_CHANGE_TIME_SEC,
                    is_optimized=False,
                ))
            else:
                # Find a pocket with a non-required tool to evict
                victim_pocket = -1
                for p in self._pockets:
                    if p.is_occupied and p.tool_id not in required_ids:
                        victim_pocket = p.pocket_number
                        break
                if victim_pocket == -1:
                    continue  # magazine full of required tools

                plan.append(ToolChangeSequence(
                    from_pocket=victim_pocket,
                    to_pocket=-1,
                    estimated_time_sec=self.BASE_CHANGE_TIME_SEC,
                    is_optimized=False,
                ))
                self.unload_tool(victim_pocket)

                self.load_tool(victim_pocket, tool_id, tool_type)
                plan.append(ToolChangeSequence(
                    from_pocket=-1,
                    to_pocket=victim_pocket,
                    estimated_time_sec=self.BASE_CHANGE_TIME_SEC,
                    is_optimized=False,
                ))

        return plan

    # -- Helpers ------------------------------------------------------

    def estimate_change_time(self, from_pocket: int, to_pocket: int) -> float:
        """Estimate carousel rotation time between two pockets."""
        if (from_pocket < 1 or from_pocket > self._magazine_size
                or to_pocket < 1 or to_pocket > self._magazine_size):
            raise ValueError("pocket numbers must be between 1 and magazine_size")
        distance = abs(from_pocket - to_pocket)
        distance = min(distance, self._magazine_size - distance)
        return self.BASE_CHANGE_TIME_SEC + distance * self.TIME_PER_POCKET_SEC

    def _get_pocket(self, pocket_number: int) -> MagazinePocket:
        if pocket_number < 1 or pocket_number > self._magazine_size:
            raise IndexError(
                f"Pocket number must be between 1 and {self._magazine_size}"
            )
        return self._pockets[pocket_number - 1]


# ---- Fixtures (pytest) ----

@pytest.fixture
def manager():
    return ToolMagazineManager(magazine_size=24)


@pytest.fixture
def small_manager():
    return ToolMagazineManager(magazine_size=4)


# ---- Tests ----

def test_initial_state(manager):
    """A freshly constructed magazine has all pockets empty."""
    status = manager.get_status()
    assert status.total_pockets == 24
    assert status.occupied_pockets == 0
    assert status.empty_pockets == 24
    assert status.utilization_pct == pytest.approx(0.0)
    assert len(status.tools) == 24
    for pocket in status.tools:
        assert pocket.is_occupied is False
        assert pocket.tool_id is None


def test_load_and_find_tool(manager):
    """Loading a tool makes it findable by ID and occupies the pocket."""
    manager.load_tool(1, "T01", "end_mill")
    manager.load_tool(5, "T02", "drill")

    assert manager.find_tool("T01") == 1
    assert manager.find_tool("T02") == 5
    assert manager.find_tool("T99") == -1

    status = manager.get_status()
    assert status.occupied_pockets == 2
    assert status.empty_pockets == 22
    assert status.utilization_pct == pytest.approx((2 / 24) * 100.0)


def test_unload_tool(manager):
    """Unloading a tool frees the pocket."""
    manager.load_tool(3, "T10", "face_mill")
    assert manager.find_tool("T10") == 3

    manager.unload_tool(3)
    assert manager.find_tool("T10") == -1

    status = manager.get_status()
    assert status.occupied_pockets == 0


def test_load_occupied_pocket_raises(manager):
    """Loading into an already-occupied pocket raises an error."""
    manager.load_tool(1, "T01", "end_mill")
    with pytest.raises(RuntimeError, match="already occupied"):
        manager.load_tool(1, "T02", "drill")


def test_unload_empty_pocket_raises(manager):
    """Unloading an empty pocket raises an error."""
    with pytest.raises(RuntimeError, match="already empty"):
        manager.unload_tool(1)


def test_invalid_pocket_number(manager):
    """Accessing a pocket outside [1, magazineSize] raises an error."""
    with pytest.raises(IndexError):
        manager.load_tool(0, "T01", "end_mill")
    with pytest.raises(IndexError):
        manager.load_tool(25, "T01", "end_mill")
    with pytest.raises(IndexError):
        manager.unload_tool(0)


def test_get_next_empty_pocket(small_manager):
    """GetNextEmptyPocket returns the first available pocket, or -1 if full."""
    mgr = small_manager
    assert mgr.get_next_empty_pocket() == 1

    mgr.load_tool(1, "T01", "end_mill")
    assert mgr.get_next_empty_pocket() == 2

    mgr.load_tool(2, "T02", "drill")
    mgr.load_tool(3, "T03", "tap")
    mgr.load_tool(4, "T04", "reamer")
    assert mgr.get_next_empty_pocket() == -1

    mgr.unload_tool(2)
    assert mgr.get_next_empty_pocket() == 2


def test_optimize_layout(manager):
    """OptimizeLayout rearranges tools to sit in usage-order pockets."""
    # Load tools scattered across the magazine
    manager.load_tool(10, "T01", "end_mill")
    manager.load_tool(20, "T02", "drill")
    manager.load_tool(5, "T03", "tap")

    usage_order = ["T01", "T02", "T03"]
    moves = manager.optimize_layout(usage_order)

    # After optimization, tools should be in pockets 1, 2, 3
    assert manager.find_tool("T01") == 1
    assert manager.find_tool("T02") == 2
    assert manager.find_tool("T03") == 3

    # Should have produced moves (since none started in the target pocket)
    assert len(moves) > 0
    for move in moves:
        assert move.is_optimized is True
        assert move.estimated_time_sec > 0


def test_optimize_layout_empty_list(manager):
    """Optimizing with an empty usage list produces no moves."""
    moves = manager.optimize_layout([])
    assert moves == []

    moves = manager.optimize_layout(None)
    assert moves == []


def test_plan_tool_changes_loads_missing(manager):
    """PlanToolChanges loads tools that are not yet in the magazine."""
    manager.load_tool(1, "T01", "end_mill")

    required = [("T01", "end_mill"), ("T02", "drill"), ("T03", "tap")]
    plan = manager.plan_tool_changes(required)

    # T01 already present, so we should see loads for T02 and T03
    assert len(plan) == 2  # two load operations

    # All required tools should now be in the magazine
    assert manager.find_tool("T01") != -1
    assert manager.find_tool("T02") != -1
    assert manager.find_tool("T03") != -1


def test_plan_tool_changes_evicts_non_required(small_manager):
    """When magazine is full, PlanToolChanges evicts non-required tools."""
    mgr = small_manager  # 4 pockets

    mgr.load_tool(1, "OLD-1", "end_mill")
    mgr.load_tool(2, "OLD-2", "drill")
    mgr.load_tool(3, "KEEP", "tap")
    mgr.load_tool(4, "OLD-3", "reamer")

    # Require KEEP plus two new tools
    required = [("KEEP", "tap"), ("NEW-1", "end_mill"), ("NEW-2", "drill")]
    plan = mgr.plan_tool_changes(required)

    # KEEP should remain; two OLD tools should be evicted for NEW tools
    assert mgr.find_tool("KEEP") != -1
    assert mgr.find_tool("NEW-1") != -1
    assert mgr.find_tool("NEW-2") != -1

    # Plan should have unload+load pairs for each evicted tool (2 pairs = 4 entries)
    assert len(plan) == 4


def test_estimate_change_time(manager):
    """EstimateChangeTime accounts for shortest rotation path."""
    # Adjacent pockets
    t1 = manager.estimate_change_time(1, 2)
    assert t1 == pytest.approx(4.5 + 0.3)

    # Opposite sides of a 24-pocket carousel
    t12 = manager.estimate_change_time(1, 13)
    assert t12 == pytest.approx(4.5 + 12 * 0.3)

    # Wrapping around is shorter: pocket 1 to 24 is distance 1 (not 23)
    t_wrap = manager.estimate_change_time(1, 24)
    assert t_wrap == pytest.approx(4.5 + 1 * 0.3)

    # Same pocket
    t_same = manager.estimate_change_time(5, 5)
    assert t_same == pytest.approx(4.5)


def test_invalid_magazine_size():
    """Constructing with a non-positive size raises an error."""
    with pytest.raises(ValueError, match="positive"):
        ToolMagazineManager(magazine_size=0)
    with pytest.raises(ValueError, match="positive"):
        ToolMagazineManager(magazine_size=-5)


def test_load_tool_validation(manager):
    """Loading with empty tool_id or tool_type raises ValueError."""
    with pytest.raises(ValueError, match="tool_id"):
        manager.load_tool(1, "", "end_mill")
    with pytest.raises(ValueError, match="tool_id"):
        manager.load_tool(1, None, "end_mill")
    with pytest.raises(ValueError, match="tool_type"):
        manager.load_tool(1, "T01", "")
    with pytest.raises(ValueError, match="tool_type"):
        manager.load_tool(1, "T01", None)
