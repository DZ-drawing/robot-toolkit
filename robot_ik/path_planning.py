"""Path Planning for Robot Manipulators.

RRT* (Rapidly-exploring Random Tree Star) algorithm for collision-free
path planning in joint space.

Author: Danny Zeng
License: MIT
"""

import time
from dataclasses import dataclass, field

import numpy as np

from robot_ik.collision import CollisionChecker


@dataclass
class PathPlanningResult:
    """Result of path planning."""

    success: bool
    path: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))  # (N, dof)
    cost: float = 0.0
    planning_time: float = 0.0
    nodes_explored: int = 0
    message: str = ""


class RRTStar:
    """RRT* path planner for manipulators."""

    def __init__(
        self,
        robot,
        collision_checker: CollisionChecker,
        joint_limits: np.ndarray,
        step_size: float = 0.1,
        max_iterations: int = 1000,
        goal_threshold: float = 0.2,
        goal_sample_rate: float = 0.1,
    ):
        """Initialize RRT* planner.

        Args:
            robot: RobotModel with forward_kinematics method
            collision_checker: CollisionChecker instance
            joint_limits: (dof, 2) array with [[min, max], ...]
            step_size: Maximum step size for tree expansion
            max_iterations: Maximum planning iterations
            goal_threshold: Distance threshold for goal reaching
            goal_sample_rate: Probability of sampling goal directly
        """
        self.robot = robot
        self.collision_checker = collision_checker
        self.joint_limits = joint_limits
        self.dof = joint_limits.shape[0]
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.goal_threshold = goal_threshold
        self.goal_sample_rate = goal_sample_rate

    def plan(self, start: np.ndarray, goal: np.ndarray) -> PathPlanningResult:
        """Plan collision-free path from start to goal.

        Args:
            start: (dof,) start configuration
            goal: (dof,) goal configuration

        Returns:
            PathPlanningResult with path (if successful)
        """
        start_time = time.perf_counter()

        # Initialize tree
        nodes = [start.copy()]
        parents = [-1]  # Parent node index for each node
        costs = [0.0]  # Cost from start to each node

        best_goal_idx = -1
        best_cost = np.inf

        for _iteration in range(self.max_iterations):
            # Sample random configuration
            if np.random.random() < self.goal_sample_rate:
                q_sample = goal.copy()
            else:
                q_sample = self._sample_random()

            # Find nearest node
            nearest_idx = self._find_nearest(nodes, q_sample)
            q_nearest = nodes[nearest_idx]

            # Steer towards sample
            q_new = self._steer(q_nearest, q_sample)

            # Check collision
            if self._is_collision_free(q_nearest, q_new):
                # Find nearby nodes for rewiring
                nearby_indices = self._find_nearby(nodes, q_new)

                # Choose best parent
                new_cost = costs[nearest_idx] + self._distance(q_nearest, q_new)
                best_parent_idx = nearest_idx
                min_cost = new_cost

                for idx in nearby_indices:
                    if idx == nearest_idx:
                        continue
                    cost_to_new = costs[idx] + self._distance(nodes[idx], q_new)
                    if cost_to_new < min_cost and self._is_collision_free(nodes[idx], q_new):
                        min_cost = cost_to_new
                        best_parent_idx = idx

                # Add new node
                nodes.append(q_new)
                parents.append(best_parent_idx)
                costs.append(min_cost)

                new_idx = len(nodes) - 1

                # Rewire nearby nodes
                for idx in nearby_indices:
                    if idx == best_parent_idx:
                        continue
                    new_cost_to_idx = min_cost + self._distance(q_new, nodes[idx])
                    if new_cost_to_idx < costs[idx] and self._is_collision_free(q_new, nodes[idx]):
                        parents[idx] = new_idx
                        costs[idx] = new_cost_to_idx

                # Check if reached goal
                if self._distance(q_new, goal) < self.goal_threshold:
                    if min_cost < best_cost:
                        best_cost = min_cost
                        best_goal_idx = new_idx

        # Reconstruct path
        planning_time = time.perf_counter() - start_time

        if best_goal_idx >= 0:
            path = self._reconstruct_path(nodes, parents, best_goal_idx)
            # Smooth path (shortcut)
            path = self._shortcut_path(path)
            return PathPlanningResult(
                success=True,
                path=path,
                cost=best_cost,
                planning_time=planning_time,
                nodes_explored=len(nodes),
                message=f"Found path with {len(nodes)} nodes",
            )
        else:
            return PathPlanningResult(
                success=False,
                planning_time=planning_time,
                nodes_explored=len(nodes),
                message=f"Failed to reach goal after {self.max_iterations} iterations",
            )

    def _sample_random(self) -> np.ndarray:
        """Sample random configuration within limits."""
        return np.random.uniform(
            self.joint_limits[:, 0],
            self.joint_limits[:, 1],
            size=self.dof,
        )

    def _find_nearest(self, nodes: list[np.ndarray], q: np.ndarray) -> int:
        """Find index of nearest node."""
        distances = [self._distance(node, q) for node in nodes]
        return int(np.argmin(distances))

    def _find_nearby(self, nodes: list[np.ndarray], q: np.ndarray) -> list[int]:
        """Find indices of nearby nodes within rewiring radius."""
        n = len(nodes)
        radius = min(self.step_size * 3.0, 10.0 * np.sqrt(np.log(n) / n))
        nearby = []
        for i, node in enumerate(nodes):
            if self._distance(node, q) < radius:
                nearby.append(i)
        return nearby

    def _steer(self, q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
        """Steer from q_from towards q_to by step_size."""
        diff = q_to - q_from
        dist = np.linalg.norm(diff)
        if dist <= self.step_size:
            return q_to.copy()
        return q_from + (diff / dist) * self.step_size

    def _distance(self, q1: np.ndarray, q2: np.ndarray) -> float:
        """Weighted joint space distance."""
        return np.linalg.norm(q1 - q2)

    def _is_collision_free(self, q1: np.ndarray, q2: np.ndarray) -> bool:
        """Check if path segment is collision-free."""
        # Discretize segment into 10 steps
        n_steps = 10
        for i in range(n_steps + 1):
            alpha = i / n_steps
            q = q1 + alpha * (q2 - q1)

            # Check collision at this configuration
            transforms = self._get_link_transforms(q)
            collision = self.collision_checker.check_self_collision(transforms)
            if collision is not None:
                return False

        return True

    def _get_link_transforms(self, q: np.ndarray) -> dict:
        """Get link transforms for collision checking."""
        # Simplified: use forward kinematics for all links
        # In practice, you'd use the actual robot model
        transforms = {}
        T = np.eye(4)
        for i in range(self.dof):
            # Simplified DH transform (placeholder)
            transforms[f"link{i}"] = T.copy()
            # Update T for next link (simplified)
            T[:3, 3] += np.array([0, 0, 0.1])  # Simplified
        return transforms

    def _reconstruct_path(
        self, nodes: list[np.ndarray], parents: list[int], goal_idx: int
    ) -> np.ndarray:
        """Reconstruct path from goal to start."""
        path = [nodes[goal_idx]]
        idx = goal_idx
        while parents[idx] >= 0:
            idx = parents[idx]
            path.append(nodes[idx])
        path.reverse()
        return np.array(path)

    def _shortcut_path(self, path: np.ndarray) -> np.ndarray:
        """Shortcut path by removing unnecessary waypoints."""
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            # Find furthest visible node
            for j in range(len(path) - 1, i, -1):
                if self._is_collision_free(path[i], path[j]):
                    simplified.append(path[j])
                    i = j
                    break
            else:
                i += 1

        return np.array(simplified)


def plan_path_rrt_star(
    robot,
    collision_checker: CollisionChecker,
    start: np.ndarray,
    goal: np.ndarray,
    joint_limits: np.ndarray,
    **kwargs,
) -> PathPlanningResult:
    """Convenience function for RRT* path planning.

    Args:
        robot: RobotModel instance
        collision_checker: CollisionChecker instance
        start: (dof,) start configuration
        goal: (dof,) goal configuration
        joint_limits: (dof, 2) array with [[min, max], ...]
        **kwargs: Additional arguments for RRTStar

    Returns:
        PathPlanningResult
    """
    planner = RRTStar(robot, collision_checker, joint_limits, **kwargs)
    return planner.plan(start, goal)
