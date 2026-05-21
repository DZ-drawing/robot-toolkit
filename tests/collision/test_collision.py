"""Test suite for collision detection module."""

import numpy as np

from robot_ik.collision import (
    Box,
    Capsule,
    CollisionChecker,
    Sphere,
    distance_capsule_to_capsule,
    distance_sphere_to_box,
    distance_sphere_to_capsule,
    distance_sphere_to_sphere,
)


def test_sphere_sphere_distance():
    """Sphere-sphere distance computation."""
    s1 = Sphere(radius=0.1)
    s1.pose[:3, 3] = np.array([0.0, 0.0, 0.0])

    s2 = Sphere(radius=0.05)
    s2.pose[:3, 3] = np.array([0.2, 0.0, 0.0])

    dist = distance_sphere_to_sphere(s1, s2)
    # Distance = 0.2 - 0.1 - 0.05 = 0.05
    assert abs(dist - 0.05) < 1e-6, f"Expected 0.05, got {dist}"
    print("  [PASS] test_sphere_sphere_distance")


def test_sphere_sphere_collision():
    """Sphere-sphere collision detection."""
    s1 = Sphere(radius=0.1)
    s1.pose[:3, 3] = np.array([0.0, 0.0, 0.0])

    s2 = Sphere(radius=0.1)
    s2.pose[:3, 3] = np.array([0.15, 0.0, 0.0])  # Overlapping

    dist = distance_sphere_to_sphere(s1, s2)
    # Distance = 0.15 - 0.1 - 0.1 = -0.05 (penetrating)
    assert dist < 0, f"Expected negative distance (collision), got {dist}"
    print("  [PASS] test_sphere_sphere_collision")


def test_sphere_capsule_distance():
    """Sphere-capsule distance computation."""
    sphere = Sphere(radius=0.05)
    sphere.pose[:3, 3] = np.array([0.0, 0.0, 0.1])

    capsule = Capsule(p1=np.array([0.0, 0.0, 0.0]), p2=np.array([0.0, 0.0, 0.2]), radius=0.03)

    dist = distance_sphere_to_capsule(sphere, capsule)
    # Sphere center is at z=0.1, capsule spans z=[0, 0.2]
    # Distance to capsule line = 0, subtract radii = 0.05 + 0.03 = 0.08
    # But sphere is at (0, 0, 0.1), which is on the capsule line
    # So distance should be -0.08 (penetrating)
    assert dist < 0, f"Expected penetration, got {dist}"
    print("  [PASS] test_sphere_capsule_distance")


def test_capsule_capsule_distance():
    """Capsule-capsule distance computation."""
    c1 = Capsule(p1=np.array([0.0, 0.0, 0.0]), p2=np.array([0.0, 0.0, 0.2]), radius=0.03)

    c2 = Capsule(p1=np.array([0.2, 0.0, 0.0]), p2=np.array([0.2, 0.0, 0.2]), radius=0.03)

    dist = distance_capsule_to_capsule(c1, c2)
    # Parallel capsules, 0.2 apart in x
    # Distance = 0.2 - 0.03 - 0.03 = 0.14
    assert abs(dist - 0.14) < 1e-6, f"Expected 0.14, got {dist}"
    print("  [PASS] test_capsule_capsule_distance")


def test_sphere_box_distance():
    """Sphere-box distance computation."""
    sphere = Sphere(radius=0.05)
    sphere.pose[:3, 3] = np.array([0.2, 0.0, 0.0])

    box = Box(size=np.array([0.1, 0.1, 0.1]))
    box.pose[:3, 3] = np.array([0.0, 0.0, 0.0])

    dist = distance_sphere_to_box(sphere, box)
    # Sphere at x=0.2, box spans x=[-0.05, 0.05]
    # Distance = 0.2 - 0.05 - 0.05 = 0.1
    assert abs(dist - 0.1) < 1e-6, f"Expected 0.1, got {dist}"
    print("  [PASS] test_sphere_box_distance")


def test_sphere_box_penetration():
    """Sphere-box penetration detection."""
    sphere = Sphere(radius=0.05)
    sphere.pose[:3, 3] = np.array([0.03, 0.0, 0.0])

    box = Box(size=np.array([0.1, 0.1, 0.1]))
    box.pose[:3, 3] = np.array([0.0, 0.0, 0.0])

    dist = distance_sphere_to_box(sphere, box)
    # Sphere at x=0.03, box extends to x=0.05
    # Sphere radius = 0.05, so it penetrates by 0.03 + 0.05 - 0.05 = 0.03
    assert dist < 0, f"Expected penetration, got {dist}"
    print("  [PASS] test_sphere_box_penetration")


def test_self_collision_detection():
    """Self-collision detection for simple robot."""
    checker = CollisionChecker()

    # Add link geometries (3 links to test non-adjacent collision)
    link1_sphere = Sphere(radius=0.05)
    link1_sphere.pose[:3, 3] = np.array([0.0, 0.0, 0.0])
    checker.add_link_geometry("link1", link1_sphere)

    link2_sphere = Sphere(radius=0.05)
    link2_sphere.pose[:3, 3] = np.array([0.0, 0.0, 0.0])
    checker.add_link_geometry("link2", link2_sphere)

    link3_sphere = Sphere(radius=0.05)
    link3_sphere.pose[:3, 3] = np.array([0.0, 0.0, 0.0])
    checker.add_link_geometry("link3", link3_sphere)

    # Configuration 1: No collision (links far apart)
    transforms = {
        "link1": np.eye(4),
        "link2": np.array(
            [
                [1, 0, 0, 0.3],
                [0, 1, 0, 0.0],
                [0, 0, 1, 0.0],
                [0, 0, 0, 1.0],
            ]
        ),
        "link3": np.array(
            [
                [1, 0, 0, 0.6],
                [0, 1, 0, 0.0],
                [0, 0, 1, 0.0],
                [0, 0, 0, 1.0],
            ]
        ),
    }
    result = checker.check_self_collision(transforms)
    assert result is None, "Expected no collision"

    # Configuration 2: Collision (link1 and link3 overlapping, non-adjacent)
    transforms = {
        "link1": np.eye(4),
        "link2": np.array(
            [
                [1, 0, 0, 0.3],
                [0, 1, 0, 0.0],
                [0, 0, 1, 0.0],
                [0, 0, 0, 1.0],
            ]
        ),
        "link3": np.eye(4),  # Same as link1
    }
    result = checker.check_self_collision(transforms)
    assert result is not None, "Expected collision between link1 and link3"
    assert result.is_colliding, "Expected is_colliding=True"
    assert result.pair == ("link1", "link3"), f"Wrong pair: {result.pair}"

    print("  [PASS] test_self_collision_detection")


def test_environment_collision_detection():
    """Environment collision detection."""
    checker = CollisionChecker()

    # Add robot link
    link_sphere = Sphere(radius=0.05)
    checker.add_link_geometry("link1", link_sphere)

    # Add obstacle
    obstacle = Box(size=np.array([0.1, 0.1, 0.1]))
    obstacle.pose[:3, 3] = np.array([0.2, 0.0, 0.0])
    checker.add_obstacle(obstacle)

    # Configuration 1: No collision
    transforms = {"link1": np.eye(4)}
    result = checker.check_environment_collision(transforms)
    assert result is None, "Expected no collision"

    # Configuration 2: Collision
    transforms["link1"][:3, 3] = np.array([0.15, 0.0, 0.0])
    result = checker.check_environment_collision(transforms)
    assert result is not None, "Expected collision"
    assert result.is_colliding, "Expected is_colliding=True"
    assert result.pair[1] == "obstacle", f"Wrong pair: {result.pair}"

    print("  [PASS] test_environment_collision_detection")


def test_ignore_adjacent_links():
    """Ignore adjacent links in self-collision check."""
    checker = CollisionChecker()

    # Add 3 links, all at origin
    for i in range(3):
        sphere = Sphere(radius=0.05)
        # Don't set initial pose, use default (origin)
        checker.add_link_geometry(f"link{i}", sphere)

    # All links at same position (would collide if not ignoring adjacent)
    transforms = {f"link{i}": np.eye(4) for i in range(3)}

    # With ignore_adjacent=True (default), only link0 and link2 should collide
    result = checker.check_self_collision(transforms, ignore_adjacent=True)
    # link0 and link2 are not adjacent (index difference = 2), so they should collide
    assert result is not None, "Expected collision between non-adjacent links (link0, link2)"
    assert result.pair == ("link0", "link2"), f"Expected (link0, link2), got {result.pair}"

    # With ignore_adjacent=False, all pairs should collide
    result = checker.check_self_collision(transforms, ignore_adjacent=False)
    assert result is not None, "Expected collision when not ignoring adjacent"

    print("  [PASS] test_ignore_adjacent_links")


def test_contact_point_approximation():
    """Contact point approximation in collision result."""
    checker = CollisionChecker()

    s1 = Sphere(radius=0.1)
    s2 = Sphere(radius=0.1)
    s1.pose[:3, 3] = np.array([0.0, 0.0, 0.0])
    s2.pose[:3, 3] = np.array([0.15, 0.0, 0.0])

    checker.add_link_geometry("link1", s1)
    checker.add_link_geometry("link2", s2)

    transforms = {
        "link1": np.eye(4),
        "link2": np.eye(4),
    }

    # Don't ignore adjacent links for this test
    result = checker.check_self_collision(transforms, ignore_adjacent=False)
    assert result is not None, "Expected collision"
    assert result.contact_point is not None, "Expected contact point"

    # Contact point should be roughly midpoint between sphere centers
    expected = np.array([0.075, 0.0, 0.0])
    assert np.allclose(
        result.contact_point, expected, atol=0.01
    ), f"Contact point mismatch: {result.contact_point} vs {expected}"

    print("  [PASS] test_contact_point_approximation")


if __name__ == "__main__":
    print("=== Collision Detection Test Suite ===\n")

    test_sphere_sphere_distance()
    test_sphere_sphere_collision()
    test_sphere_capsule_distance()
    test_capsule_capsule_distance()
    test_sphere_box_distance()
    test_sphere_box_penetration()
    test_self_collision_detection()
    test_environment_collision_detection()
    test_ignore_adjacent_links()
    test_contact_point_approximation()

    print("\n=== All 10 tests passed ===")
