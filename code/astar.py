import heapq
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

import collision_checker

__all__ = ["AStar"]


class AStar:
    """3-D (Weighted) A* on a *uniform* grid.

    This implementation is **self-contained** - it depends only on
    ``numpy`` and the provided ``collision_checker`` module and *does not*
    require external libraries such as ``pqdict``.
    """

    # 26‑connected neighbourhood (unit directions)
    _DELTA = np.asarray(
        [
            [dx, dy, dz]
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
            if not (dx == dy == dz == 0)
        ],
        dtype=int,
    )

    @staticmethod
    def _heuristic(p: np.ndarray, goal: np.ndarray) -> float:
        """Euclidean distance heuristic (admissible & consistent)."""
        return float(np.linalg.norm(p - goal))

    @staticmethod
    def _in_bounds(p: np.ndarray, boundary: np.ndarray) -> bool:
        """Return *True* iff *p* lies within the (closed) boundary box."""
        return np.all(p >= boundary[:3]) and np.all(p <= boundary[3:6])

    @classmethod
    def plan(
        cls,
        start: np.ndarray,
        goal: np.ndarray,
        boundary: np.ndarray,
        blocks: np.ndarray,
        *,
        resolution: float = 0.5,
        epsilon: float = 1.0,
    ) -> np.ndarray:
        """Weighted A* planner.

        Parameters
        ----------
        start, goal
            3-D coordinates of the query (metres).
        boundary
            Environment outer boundary ``shape (6,)`` -
            ``(xmin, ymin, zmin, xmax, ymax, zmax)``.
        blocks
            Obstacle list ``shape (N, ≥6)``; **only** the first six columns
            (AABB bounds) are used.
        resolution
            Grid size (metres).
        epsilon
            Inflation factor on the heuristic.  ``epsilon = 1`` gives classic
            A* (optimal); ``epsilon > 1`` yields *ε-optimal* anytime search.

        Returns
        -------
        np.ndarray, shape (M, 3)
            Piece-wise linear path including *start* and *goal* (continuous
            coordinates).  Empty array when no feasible path exists.
        """
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        boundary = np.asarray(boundary, dtype=float).ravel()
        boxes = blocks[:, :6] if blocks.ndim == 2 else np.asarray(blocks)

        # Helper lambdas ──────────────────────────────────────────────────────
        origin = boundary[:3]  # (xmin, ymin, zmin)

        def to_key(p: np.ndarray) -> Tuple[int, int, int]:
            "Convert *continuous* coord to integer grid key."
            return tuple(np.round((p - origin) / resolution).astype(int))

        def to_coord(k: Tuple[int, int, int]) -> np.ndarray:
            "Grid key → continuous coord (centre of the voxel)."
            return origin + np.asarray(k, dtype=float) * resolution

        # Priority queue elements: (f, entry_count, key)
        open_heap: List[Tuple[float, int, Tuple[int, int, int]]] = []
        g_cost: Dict[Tuple[int, int, int], float] = {}
        parent: Dict[Tuple[int, int, int], Optional[Tuple[int, int, int]]] = {}
        closed: set = set()
        counter = 0  # unique sequence number to ensure FIFO tie‑breaking

        start_k = to_key(start)
        goal_k = to_key(goal)

        g_cost[start_k] = 0.0
        h0 = cls._heuristic(start, goal)
        heapq.heappush(open_heap, (h0, counter, start_k))
        parent[start_k] = None

        reached_goal_key: Optional[Tuple[int, int, int]] = None

        while open_heap:
            f_curr, _, k_curr = heapq.heappop(open_heap)
            if k_curr in closed:
                continue  # Stale entry.
            closed.add(k_curr)

            if k_curr == goal_k:
                reached_goal_key = k_curr
                break  # Found optimal / ε‑optimal path.

            p_curr = to_coord(k_curr)
            g_curr = g_cost[k_curr]

            # ─── Expand neighbours ──────────────────────────────────────────
            for d in cls._DELTA:
                k_nbr = (k_curr[0] + d[0], k_curr[1] + d[1], k_curr[2] + d[2])
                if k_nbr in closed:
                    continue

                p_nbr = to_coord(k_nbr)

                # Bounds check.
                if not cls._in_bounds(p_nbr, boundary):
                    continue

                # Collision check (segment).  Cheap early reject: endpoint in
                # obstacle?  Already covered by segment check inside helper.
                if collision_checker.segment_intersects_any_aabb(
                    p_curr, p_nbr, boxes
                ):
                    continue

                step_cost = float(np.linalg.norm(p_nbr - p_curr))
                g_new = g_curr + step_cost

                if g_new < g_cost.get(k_nbr, math.inf):
                    g_cost[k_nbr] = g_new
                    parent[k_nbr] = k_curr
                    h_nbr = cls._heuristic(p_nbr, goal)
                    counter += 1
                    heapq.heappush(
                        open_heap,
                        (g_new + epsilon * h_nbr, counter, k_nbr),
                    )

            # Anytime Weighted A*: early exit once f_min ≥ g(goal)
            if (
                epsilon > 1.0
                and goal_k in g_cost
                and open_heap
                and g_cost[goal_k] <= open_heap[0][0]
            ):
                reached_goal_key = goal_k
                break

        # ─── Reconstruct path ───────────────────────────────────────────────
        if reached_goal_key is None:
            return np.empty((0, 3), dtype=float)  # Failure.

        rev_path: List[np.ndarray] = []
        k = reached_goal_key
        while k is not None:
            rev_path.append(to_coord(k))
            k = parent[k]
        rev_path.reverse()
        return np.vstack(rev_path)
