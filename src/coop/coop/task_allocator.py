#!/usr/bin/env python3
from typing import Dict, Tuple, List, Optional
import math
import random


class GreedyTaskAllocator:
    """
    Very simple multi-robot task allocator for coverage.

    Strategy:
    1. Look at the coverage grid and pick a subset of unvisited cells as candidate goals.
    2. For each idle drone, assign the closest candidate that hasn't been taken yet.
    """

    def __init__(
        self,
        origin_x: float,
        origin_y: float,
        resolution: float,
        cells_x: int,
        cells_y: int,
    ):
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._resolution = resolution
        self._cells_x = cells_x
        self._cells_y = cells_y

    def _cell_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        wx = self._origin_x + (ix + 0.5) * self._resolution
        wy = self._origin_y + (iy + 0.5) * self._resolution
        return wx, wy

    def _collect_candidates(
        self,
        coverage_grid: List[List[bool]],
        max_candidates: int = 200,
        stride: int = 1,
    ) -> List[Tuple[float, float]]:
        """
        Collect up to max_candidates uncovered cells and convert them to world coords.
        We optionally stride through the grid so we don't pick every single cell.
        """
        candidates: List[Tuple[float, float]] = []

        # simple scan
        for iy in range(0, self._cells_y, stride):
            for ix in range(0, self._cells_x, stride):
                if not coverage_grid[iy][ix]:
                    candidates.append(self._cell_to_world(ix, iy))
                    if len(candidates) >= max_candidates:
                        return candidates

        return candidates

    @staticmethod
    def _dist2(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx * dx + dy * dy

    def assign(
        self,
        coverage_grid: List[List[bool]],
        idle_drones: Dict[str, Tuple[float, float]],
        max_candidates: int = 200,
    ) -> Dict[str, Tuple[float, float]]:
        """
        Given a coverage grid and a set of idle drones (name -> (x, y)),
        return a dict {drone_name: (goal_x, goal_y)}.
        """
        if not idle_drones:
            return {}

        # collect places we still need to visit
        # small stride to avoid being super dense
        candidates = self._collect_candidates(coverage_grid, max_candidates=max_candidates, stride=1)

        if not candidates:
            return {}

        assignments: Dict[str, Tuple[float, float]] = {}

        # greedy: for each drone, take closest free candidate
        used = set()
        for drone_name, drone_pos in idle_drones.items():
            best_idx: Optional[int] = None
            best_d2 = float('inf')
            for idx, pt in enumerate(candidates):
                if idx in used:
                    continue
                d2 = self._dist2(drone_pos, pt)
                if d2 < best_d2:
                    best_d2 = d2
                    best_idx = idx

            if best_idx is not None:
                assignments[drone_name] = candidates[best_idx]
                used.add(best_idx)

        return assignments
