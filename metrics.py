"""
Metrics Collection for Box-Pushing Robot.

Tracks performance metrics for analysis:
- Phase timing (exploration, navigation, delivery)
- Planning statistics (A* nodes expanded, path lengths)
- Execution statistics (replanning, recalibrations, push attempts)
- Mission outcome

Usage:
    metrics = Metrics()
    metrics.start_phase("exploration")
    # ... run exploration ...
    metrics.end_phase("exploration")
    metrics.record_astar_search(nodes_expanded=150, path_length=12)
    # ... at end ...
    metrics.print_summary()
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class AStarStats:
    """Statistics for a single A* search."""

    nodes_expanded: int
    path_length: int  # 0 if no path found
    found_path: bool
    search_time_ms: float
    start: tuple[int, int]
    goal: tuple[int, int]
    search_type: str  # "navigation", "exploration", "sokoban"


@dataclass
class PushAttempt:
    """Statistics for a single push attempt."""

    success: bool
    expected_box_pos: tuple[int, int]
    actual_box_pos: Optional[tuple[int, int]]  # None if unknown
    distance_pushed: float  # meters
    contact_maintained: bool


@dataclass
class PhaseMetrics:
    """Metrics for a mission phase."""

    name: str
    start_time: float = 0.0
    end_time: float = 0.0
    duration_sec: float = 0.0  # Wall-clock time

    # Simulated physics time (simulation steps * TIME_STEP)
    sim_steps: int = 0
    sim_time_sec: float = 0.0

    # Counts
    astar_searches: int = 0
    total_nodes_expanded: int = 0
    replanning_count: int = 0
    recalibration_count: int = 0
    waypoints_reached: int = 0
    waypoints_failed: int = 0

    # Push-specific (for delivery phase)
    push_attempts: int = 0
    push_successes: int = 0
    push_failures: int = 0


@dataclass
class Metrics:
    """
    Centralized metrics collection for mission analysis.

    Tracks:
    - Wall-clock time per phase
    - A* search statistics
    - Replanning and recalibration counts
    - Push attempt outcomes
    - Overall mission success/failure
    """

    # Phase tracking
    phases: dict[str, PhaseMetrics] = field(default_factory=dict)
    current_phase: Optional[str] = None

    # A* search history
    astar_history: list[AStarStats] = field(default_factory=list)

    # Push history
    push_history: list[PushAttempt] = field(default_factory=list)

    # Mission-level
    mission_start_time: float = 0.0
    mission_end_time: float = 0.0
    mission_success: bool = False

    # Test case info
    test_case_name: str = ""
    initial_robot_pos: tuple[int, int] = (0, 0)
    initial_box_pos: tuple[int, int] = (0, 0)
    goal_pos: tuple[int, int] = (0, 0)

    def __post_init__(self):
        """Initialize phase dictionaries."""
        if not self.phases:
            self.phases = {}
        if not self.astar_history:
            self.astar_history = []
        if not self.push_history:
            self.push_history = []

    # -------------------------------------------------------------------------
    # Test Case Setup
    # -------------------------------------------------------------------------

    def set_test_case(
        self,
        name: str,
        robot_pos: tuple[int, int],
        box_pos: tuple[int, int],
        goal_pos: tuple[int, int],
    ) -> None:
        """Record test case configuration."""
        self.test_case_name = name
        self.initial_robot_pos = robot_pos
        self.initial_box_pos = box_pos
        self.goal_pos = goal_pos

    # -------------------------------------------------------------------------
    # Mission Lifecycle
    # -------------------------------------------------------------------------

    def start_mission(self) -> None:
        """Mark mission start time."""
        self.mission_start_time = time.perf_counter()
        self.mission_success = False

    def end_mission(self, success: bool) -> None:
        """Mark mission end time and outcome."""
        self.mission_end_time = time.perf_counter()
        self.mission_success = success

    @property
    def mission_duration_sec(self) -> float:
        """Total mission wall-clock time in seconds."""
        if self.mission_end_time == 0.0:
            return time.perf_counter() - self.mission_start_time
        return self.mission_end_time - self.mission_start_time

    # -------------------------------------------------------------------------
    # Phase Tracking
    # -------------------------------------------------------------------------

    def start_phase(self, name: str) -> None:
        """Start timing a phase (e.g., 'exploration', 'delivery')."""
        if name not in self.phases:
            self.phases[name] = PhaseMetrics(name=name)
        self.phases[name].start_time = time.perf_counter()
        self.current_phase = name

    def end_phase(self, name: str) -> None:
        """End timing a phase."""
        if name in self.phases:
            phase = self.phases[name]
            phase.end_time = time.perf_counter()
            phase.duration_sec = phase.end_time - phase.start_time
        if self.current_phase == name:
            self.current_phase = None

    def _get_current_phase(self) -> Optional[PhaseMetrics]:
        """Get current phase metrics object."""
        if self.current_phase and self.current_phase in self.phases:
            return self.phases[self.current_phase]
        return None

    # -------------------------------------------------------------------------
    # A* Search Recording
    # -------------------------------------------------------------------------

    def record_astar_search(
        self,
        nodes_expanded: int,
        path_length: int,
        found_path: bool,
        search_time_ms: float,
        start: tuple[int, int],
        goal: tuple[int, int],
        search_type: str = "navigation",
    ) -> None:
        """Record statistics from an A* search."""
        stats = AStarStats(
            nodes_expanded=nodes_expanded,
            path_length=path_length,
            found_path=found_path,
            search_time_ms=search_time_ms,
            start=start,
            goal=goal,
            search_type=search_type,
        )
        self.astar_history.append(stats)

        # Update current phase
        phase = self._get_current_phase()
        if phase:
            phase.astar_searches += 1
            phase.total_nodes_expanded += nodes_expanded

    # -------------------------------------------------------------------------
    # Event Counters
    # -------------------------------------------------------------------------

    def record_replanning(self, reason: str = "") -> None:
        """Increment replanning counter for current phase."""
        phase = self._get_current_phase()
        if phase:
            phase.replanning_count += 1

    def record_recalibration(self) -> None:
        """Increment recalibration counter for current phase."""
        phase = self._get_current_phase()
        if phase:
            phase.recalibration_count += 1

    def record_waypoint_result(self, success: bool) -> None:
        """Record waypoint navigation result."""
        phase = self._get_current_phase()
        if phase:
            if success:
                phase.waypoints_reached += 1
            else:
                phase.waypoints_failed += 1

    def record_sim_steps(self, steps: int, time_step: float) -> None:
        """Record simulation steps (for simulated physics time)."""
        phase = self._get_current_phase()
        if phase:
            phase.sim_steps += steps
            phase.sim_time_sec += steps * time_step

    # -------------------------------------------------------------------------
    # Push Tracking
    # -------------------------------------------------------------------------

    def record_push_attempt(
        self,
        success: bool,
        expected_pos: tuple[int, int],
        actual_pos: Optional[tuple[int, int]] = None,
        distance_pushed: float = 0.0,
        contact_maintained: bool = True,
    ) -> None:
        """Record a push attempt and its outcome."""
        attempt = PushAttempt(
            success=success,
            expected_box_pos=expected_pos,
            actual_box_pos=actual_pos,
            distance_pushed=distance_pushed,
            contact_maintained=contact_maintained,
        )
        self.push_history.append(attempt)

        phase = self._get_current_phase()
        if phase:
            phase.push_attempts += 1
            if success:
                phase.push_successes += 1
            else:
                phase.push_failures += 1

    # -------------------------------------------------------------------------
    # Computed Statistics
    # -------------------------------------------------------------------------

    @property
    def total_nodes_expanded(self) -> int:
        """Total A* nodes expanded across all searches."""
        return sum(s.nodes_expanded for s in self.astar_history)

    @property
    def total_astar_searches(self) -> int:
        """Total number of A* searches."""
        return len(self.astar_history)

    @property
    def avg_nodes_per_search(self) -> float:
        """Average nodes expanded per A* search."""
        if not self.astar_history:
            return 0.0
        return self.total_nodes_expanded / len(self.astar_history)

    @property
    def total_replanning(self) -> int:
        """Total replanning events across all phases."""
        return sum(p.replanning_count for p in self.phases.values())

    @property
    def total_recalibrations(self) -> int:
        """Total recalibrations across all phases."""
        return sum(p.recalibration_count for p in self.phases.values())

    @property
    def total_path_length(self) -> int:
        """Sum of all path lengths from successful A* searches."""
        return sum(s.path_length for s in self.astar_history if s.found_path)

    @property
    def astar_success_rate(self) -> float:
        """Percentage of A* searches that found a path."""
        if not self.astar_history:
            return 0.0
        successes = sum(1 for s in self.astar_history if s.found_path)
        return 100.0 * successes / len(self.astar_history)

    @property
    def push_success_rate(self) -> float:
        """Percentage of push attempts that succeeded."""
        if not self.push_history:
            return 0.0
        successes = sum(1 for p in self.push_history if p.success)
        return 100.0 * successes / len(self.push_history)

    @property
    def total_sim_time_sec(self) -> float:
        """Total simulated physics time across all phases."""
        return sum(p.sim_time_sec for p in self.phases.values())

    @property
    def total_sim_steps(self) -> int:
        """Total simulation steps across all phases."""
        return sum(p.sim_steps for p in self.phases.values())

    # -------------------------------------------------------------------------
    # Output
    # -------------------------------------------------------------------------

    def print_summary(self) -> None:
        """Print a formatted summary of all metrics."""
        print("\n" + "=" * 70)
        print("METRICS SUMMARY")
        print("=" * 70)

        # Test case info
        if self.test_case_name:
            print(f"\nTest Case: {self.test_case_name}")
            print(f"  Robot start: {self.initial_robot_pos}")
            print(f"  Box start:   {self.initial_box_pos}")
            print(f"  Goal:        {self.goal_pos}")

        # Mission overview
        print(
            f"\nMission Outcome: {'SUCCESS ✓' if self.mission_success else 'FAILED ✗'}"
        )
        print(f"Wall-clock time: {self.mission_duration_sec:.2f} seconds")
        print(
            f"Simulated time:  {self.total_sim_time_sec:.2f} seconds ({self.total_sim_steps} physics steps)"
        )

        # Phase breakdown
        print("\n--- Phase Breakdown ---")
        for name, phase in self.phases.items():
            print(f"\n  [{name.upper()}]")
            print(f"    Wall-clock:     {phase.duration_sec:.2f}s")
            print(
                f"    Simulated:      {phase.sim_time_sec:.2f}s ({phase.sim_steps} steps)"
            )
            print(f"    A* searches:    {phase.astar_searches}")
            print(f"    Nodes expanded: {phase.total_nodes_expanded}")
            print(f"    Replanning:     {phase.replanning_count}")
            print(f"    Recalibrations: {phase.recalibration_count}")
            print(
                f"    Waypoints:      {phase.waypoints_reached} reached, {phase.waypoints_failed} failed"
            )
            if phase.push_attempts > 0:
                print(
                    f"    Push attempts:  {phase.push_attempts} ({phase.push_successes} success, {phase.push_failures} failed)"
                )

        # A* statistics
        print("\n--- A* Search Statistics ---")
        print(f"  Total searches:     {self.total_astar_searches}")
        print(f"  Total nodes:        {self.total_nodes_expanded}")
        print(f"  Avg nodes/search:   {self.avg_nodes_per_search:.1f}")
        print(f"  Success rate:       {self.astar_success_rate:.1f}%")
        print(f"  Total path length:  {self.total_path_length} cells")

        # Breakdown by search type
        by_type: dict[str, list[AStarStats]] = {}
        for s in self.astar_history:
            by_type.setdefault(s.search_type, []).append(s)

        for stype, searches in by_type.items():
            avg_nodes = sum(s.nodes_expanded for s in searches) / len(searches)
            avg_time = sum(s.search_time_ms for s in searches) / len(searches)
            print(f"\n  [{stype}]")
            print(f"    Count:     {len(searches)}")
            print(f"    Avg nodes: {avg_nodes:.1f}")
            print(f"    Avg time:  {avg_time:.2f}ms")

        # Push statistics
        if self.push_history:
            print("\n--- Push Statistics ---")
            print(f"  Total attempts:  {len(self.push_history)}")
            print(f"  Success rate:    {self.push_success_rate:.1f}%")
            avg_dist = sum(p.distance_pushed for p in self.push_history) / len(
                self.push_history
            )
            print(f"  Avg push dist:   {avg_dist:.3f}m")

        print("\n" + "=" * 70)

    def to_dict(self) -> dict:
        """Export metrics as a dictionary (for JSON serialization)."""
        return {
            "test_case": {
                "name": self.test_case_name,
                "robot_start": self.initial_robot_pos,
                "box_start": self.initial_box_pos,
                "goal": self.goal_pos,
            },
            "mission": {
                "success": self.mission_success,
                "wall_clock_sec": self.mission_duration_sec,
                "simulated_sec": self.total_sim_time_sec,
                "sim_steps": self.total_sim_steps,
            },
            "phases": {
                name: {
                    "wall_clock_sec": p.duration_sec,
                    "simulated_sec": p.sim_time_sec,
                    "sim_steps": p.sim_steps,
                    "astar_searches": p.astar_searches,
                    "nodes_expanded": p.total_nodes_expanded,
                    "replanning_count": p.replanning_count,
                    "recalibration_count": p.recalibration_count,
                    "waypoints_reached": p.waypoints_reached,
                    "waypoints_failed": p.waypoints_failed,
                    "push_attempts": p.push_attempts,
                    "push_successes": p.push_successes,
                }
                for name, p in self.phases.items()
            },
            "astar": {
                "total_searches": self.total_astar_searches,
                "total_nodes": self.total_nodes_expanded,
                "avg_nodes_per_search": self.avg_nodes_per_search,
                "success_rate": self.astar_success_rate,
                "total_path_length": self.total_path_length,
            },
            "push": {
                "total_attempts": len(self.push_history),
                "success_rate": self.push_success_rate,
            },
            "totals": {
                "replanning": self.total_replanning,
                "recalibrations": self.total_recalibrations,
            },
        }

    def to_latex_table(self) -> str:
        """Generate LaTeX table rows for the metrics."""
        lines = []
        lines.append(r"\begin{tabular}{@{}lrrr@{}}")
        lines.append(r"\toprule")
        lines.append(
            r"\textbf{Metric} & \textbf{Exploration} & \textbf{Delivery} & \textbf{Total} \\"
        )
        lines.append(r"\midrule")

        # Get phases (default to empty if not present)
        exp = self.phases.get("exploration", PhaseMetrics(name="exploration"))
        deliv = self.phases.get("delivery", PhaseMetrics(name="delivery"))

        lines.append(
            f"Wall-clock (s) & {exp.duration_sec:.2f} & {deliv.duration_sec:.2f} & {self.mission_duration_sec:.2f} \\\\"
        )
        lines.append(
            f"Simulated (s) & {exp.sim_time_sec:.2f} & {deliv.sim_time_sec:.2f} & {self.total_sim_time_sec:.2f} \\\\"
        )
        lines.append(
            f"A* searches & {exp.astar_searches} & {deliv.astar_searches} & {self.total_astar_searches} \\\\"
        )
        lines.append(
            f"Nodes expanded & {exp.total_nodes_expanded} & {deliv.total_nodes_expanded} & {self.total_nodes_expanded} \\\\"
        )
        lines.append(
            f"Replanning & {exp.replanning_count} & {deliv.replanning_count} & {self.total_replanning} \\\\"
        )
        lines.append(
            f"Recalibrations & {exp.recalibration_count} & {deliv.recalibration_count} & {self.total_recalibrations} \\\\"
        )

        if deliv.push_attempts > 0:
            lines.append(
                f"Push attempts & --- & {deliv.push_attempts} & {deliv.push_attempts} \\\\"
            )
            lines.append(
                f"Push success & --- & {deliv.push_successes} & {deliv.push_successes} \\\\"
            )

        lines.append(r"\bottomrule")
        lines.append(r"\end{tabular}")

        return "\n".join(lines)


# Global metrics instance (optional singleton pattern)
_global_metrics: Optional[Metrics] = None


def get_metrics() -> Metrics:
    """Get or create global metrics instance."""
    global _global_metrics
    if _global_metrics is None:
        _global_metrics = Metrics()
    return _global_metrics


def reset_metrics() -> Metrics:
    """Reset and return fresh global metrics instance."""
    global _global_metrics
    _global_metrics = Metrics()
    return _global_metrics
