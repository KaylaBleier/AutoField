"""
path_planning.py

Receives waypoints from manual_waypoint_gen, stores them as a numpy
coordinate matrix, computes segment distances, and exposes a clean
interface for path_following to consume.

Flow:
    manual_waypoint_gen  →  PathPlanner  →  PathFollower

Data stored:
    coord_matrix  : np.ndarray, shape (N, 2)  — each row is [easting, northing]
    dist_matrix   : np.ndarray, shape (N-1,)  — Euclidean distance of each segment (metres)
    total_distance: float                      — sum of all segment lengths (metres)
"""

import numpy as np


class PathPlanner:
    def __init__(self):
        self.coord_matrix = None   # shape (N, 2): [[e1,n1], [e2,n2], ...]
        self.dist_matrix  = None   # shape (N-1,): [d_seg1, d_seg2, ...]
        self.total_distance = 0.0

    # ------------------------------------------------------------------
    # Ingest
    # ------------------------------------------------------------------

    def load_waypoints(self, waypoints: list):
        """
        Accept a list of (easting, northing) tuples from manual_waypoint_gen
        and build the coordinate + distance matrices.

        Parameters
        ----------
        waypoints : list of (float, float)
            Ordered list of (easting, northing) positions.

        Raises
        ------
        ValueError if fewer than 2 waypoints are provided.
        """
        if len(waypoints) < 2:
            raise ValueError(
                f"Need at least 2 waypoints, got {len(waypoints)}."
            )

        self.coord_matrix = np.array(waypoints, dtype=float)  # (N, 2)
        self._compute_distances()

        print(f"[PathPlanner] Loaded {len(waypoints)} waypoints.")
        print(f"[PathPlanner] Segment distances (m): {self.dist_matrix}")
        print(f"[PathPlanner] Total path distance  : {self.total_distance:.3f} m")

    def _compute_distances(self):
        """
        Compute Euclidean distance for each consecutive waypoint segment.
        dist_matrix[i] = distance from coord_matrix[i] to coord_matrix[i+1]
        """
        deltas = np.diff(self.coord_matrix, axis=0)          # (N-1, 2)
        self.dist_matrix = np.linalg.norm(deltas, axis=1)    # (N-1,)
        self.total_distance = float(np.sum(self.dist_matrix))

    # ------------------------------------------------------------------
    # Accessors for path_following
    # ------------------------------------------------------------------

    def get_waypoints(self) -> list:
        """
        Return waypoints as a plain list of (easting, northing) tuples,
        ready to pass directly into PathFollower.
        """
        if self.coord_matrix is None:
            raise RuntimeError("No waypoints loaded. Call load_waypoints() first.")
        return [tuple(row) for row in self.coord_matrix]

    def get_segment_distance(self, index: int) -> float:
        """
        Return the length (metres) of segment index → index+1.

        Parameters
        ----------
        index : int  Segment index (0-based).
        """
        if self.dist_matrix is None:
            raise RuntimeError("No waypoints loaded.")
        if index < 0 or index >= len(self.dist_matrix):
            raise IndexError(
                f"Segment index {index} out of range "
                f"(0–{len(self.dist_matrix)-1})."
            )
        return float(self.dist_matrix[index])

    def get_total_distance(self) -> float:
        """Return total path length in metres."""
        if self.total_distance == 0.0 and self.dist_matrix is None:
            raise RuntimeError("No waypoints loaded.")
        return self.total_distance

    def summary(self) -> dict:
        """
        Return a summary dict — useful for logging or a startup printout.
        """
        if self.coord_matrix is None:
            return {"status": "no waypoints loaded"}
        return {
            "num_waypoints"  : len(self.coord_matrix),
            "num_segments"   : len(self.dist_matrix),
            "coord_matrix"   : self.coord_matrix.tolist(),
            "dist_matrix_m"  : self.dist_matrix.tolist(),
            "total_distance_m": self.total_distance,
        }

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def validate(self) -> bool:
        """
        Basic sanity checks before handing off to path_following.
        Returns True if everything looks good, False otherwise.
        """
        if self.coord_matrix is None or self.dist_matrix is None:
            print("[PathPlanner] ERROR: No waypoints loaded.")
            return False

        if np.any(self.dist_matrix < 0.01):
            print("[PathPlanner] WARNING: One or more segments are < 1 cm — "
                  "possible duplicate waypoints.")
            return False

        print("[PathPlanner] Validation passed.")
        return True


# ---------------------------------------------------------------------------
# Example usage — wired into manual_waypoint_gen output
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Simulate output from manual_waypoint_gen
    # Replace with: from manual_waypoint_gen import generate_waypoints
    example_waypoints = [
        (500.000, 1000.000),   # wp_1 — initial GPS fix
        (520.000, 1000.000),   # wp_2 — wp_1 + 20 m east
    ]

    planner = PathPlanner()
    planner.load_waypoints(example_waypoints)

    if planner.validate():
        waypoints_for_follower = planner.get_waypoints()
        print(f"\nHanding off to PathFollower: {waypoints_for_follower}")
        print(f"Total distance: {planner.get_total_distance():.2f} m")

        # --- Wire into path_following ---
        # from path_following import PathFollower
        # follower = PathFollower(waypoints_for_follower)
