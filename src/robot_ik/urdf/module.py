"""URDF Parser — extract dynamics parameters from URDF robot models.

Converts URDF <link>/<joint> elements to RobotDynamicsModel
with DH parameters and LinkInertia entries.
"""

import xml.etree.ElementTree as ET

import numpy as np

from robot_ik.dynamics.solver import LinkInertia, RobotDynamicsModel


def _parse_origin(elem) -> tuple[np.ndarray, np.ndarray]:
    """Parse <origin xyz="..." rpy="..."/> element. Returns (pos, rpy)."""
    xyz = np.zeros(3)
    rpy = np.zeros(3)
    o = elem.find("origin")
    if o is not None:
        if "xyz" in o.attrib:
            xyz = np.array([float(x) for x in o.attrib["xyz"].split()])
        if "rpy" in o.attrib:
            rpy = np.array([float(x) for x in o.attrib["rpy"].split()])
    return xyz, rpy


def _rpy_to_rot(rpy):
    """Roll-Pitch-Yaw to 3x3 rotation matrix (fixed-axis: X-Y-Z)."""
    cr, sr = np.cos(rpy[0]), np.sin(rpy[0])
    cp, sp = np.cos(rpy[1]), np.sin(rpy[1])
    cy, sy = np.cos(rpy[2]), np.sin(rpy[2])
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _parse_inertia(elem) -> tuple[float, np.ndarray, np.ndarray]:
    """Parse <inertial><mass/><inertia/></inertial>. Returns (mass, com, I_3x3)."""
    inertial = elem.find("inertial")
    if inertial is None:
        return 0.0, np.zeros(3), np.eye(3) * 0.001

    mass_val = 0.0
    m = inertial.find("mass")
    if m is not None:
        mass_val = float(m.attrib.get("value", "0"))

    com_xyz, _ = _parse_origin(inertial)

    inertia_matrix = np.eye(3) * 0.001
    ixx = inertial.find("inertia")
    if ixx is not None:
        ixx_val = float(ixx.attrib.get("ixx", "0.001"))
        iyy_val = float(ixx.attrib.get("iyy", "0.001"))
        izz_val = float(ixx.attrib.get("izz", "0.001"))
        ixy_val = float(ixx.attrib.get("ixy", "0"))
        ixz_val = float(ixx.attrib.get("ixz", "0"))
        iyz_val = float(ixx.attrib.get("iyz", "0"))
        inertia_matrix = np.array(
            [
                [ixx_val, ixy_val, ixz_val],
                [ixy_val, iyy_val, iyz_val],
                [ixz_val, iyz_val, izz_val],
            ]
        )

    return mass_val, com_xyz, inertia_matrix


def urdf_to_dynamics_model(urdf_path: str) -> RobotDynamicsModel:
    """Parse a URDF file and return a RobotDynamicsModel.

    Walks the kinematic chain from base_link through all joints,
    extracting mass, COM, and inertia per link plus DH parameters.

    Args:
        urdf_path: Path to URDF .xml or .urdf file.

    Returns:
        RobotDynamicsModel ready for DynamicsSolver.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Index links and joints
    links: dict[str, dict] = {}
    joints: dict[str, dict] = {}

    for link in root.findall("link"):
        name = link.attrib["name"]
        mass, com, inertia = _parse_inertia(link)
        links[name] = {"mass": mass, "com": com, "inertia": inertia}

    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        jtype = joint.attrib["type"]
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        xyz, rpy = _parse_origin(joint)

        axis = np.array([0.0, 0.0, 1.0])
        a = joint.find("axis")
        if a is not None:
            axis = np.array([float(x) for x in a.attrib["xyz"].split()])

        # Joint limits
        lo, hi = -np.pi, np.pi
        lim = joint.find("limit")
        if lim is not None:
            if "lower" in lim.attrib:
                lo = float(lim.attrib["lower"])
            if "upper" in lim.attrib:
                hi = float(lim.attrib["upper"])

        joints[name] = {
            "type": jtype,
            "parent": parent,
            "child": child,
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
            "lower": lo,
            "upper": hi,
        }

    # Find root link
    child_links = {j["child"] for j in joints.values()}
    parent_links = {j["parent"] for j in joints.values()}
    root_candidates = parent_links - child_links
    if not root_candidates:
        root_candidates = {list(links.keys())[0]}
    root_link = sorted(root_candidates)[0]

    # Walk chain in DFS order
    dh_a, dh_alpha, dh_d, dh_theta_off = [], [], [], []
    joint_limits = []
    link_inertias = []
    gravity = np.array([0.0, 0.0, -9.81])  # default, override if URDF specifies

    # Build adjacency
    child_map = {}
    for j in joints.values():
        child_map.setdefault(j["parent"], []).append(j)

    def walk(link_name, parent_T):
        if link_name in child_map:
            for joint in child_map[link_name]:
                child_name = joint["child"]
                xyz, rpy = joint["xyz"], joint["rpy"]
                axis = joint["axis"]

                # URDF joint origin in parent frame: Trans(xyz) * Rot(rpy)
                R_j = _rpy_to_rot(rpy)
                p_j = xyz

                # Joint axis in parent frame
                z_parent = R_j @ axis

                # Compute DH parameters from URDF joint
                # DH a: distance along common normal (x-axis of link frame)
                # For simplicity: treat joint origin as DH offset
                # More accurate: decompose URDF joint into DH parameters
                # Simplified: a = norm of projection of p_j onto x-y plane perpendicular to z_parent
                # d = component along z_parent
                d = np.dot(p_j, z_parent)

                # a = magnitude of projection onto plane perpendicular to z_parent
                p_perp = p_j - d * z_parent
                a = np.linalg.norm(p_perp)

                # alpha: angle between z_parent and world z (for first joint)
                #         or angle between z_prev and z_parent (for subsequent joints)
                z_prev = parent_T[:3, 2]
                alpha = np.arccos(np.clip(np.dot(z_prev, z_parent), -1, 1))
                # Sign: cross(z_prev, z_parent) . x_direction
                if alpha > 1e-6:
                    cross_z = np.cross(z_prev, z_parent)
                    if np.linalg.norm(cross_z) > 1e-6:
                        x_dir = cross_z / np.linalg.norm(cross_z)
                        if np.dot(x_dir, z_parent) < 0:
                            alpha = -alpha

                # theta offset from joint origin rotation
                # The joint rotation is about z-axis after DH transform
                theta_off = 0.0

                dh_a.append(a)
                dh_alpha.append(float(alpha))
                dh_d.append(float(d))
                dh_theta_off.append(theta_off)

                joint_limits.append((joint["lower"], joint["upper"]))

                # Inertia from child link
                child = links.get(child_name, {})
                m = child.get("mass", 0.0)
                com_urdf = child.get("com", np.zeros(3))
                I_urdf = child.get("inertia", np.eye(3) * 0.001)

                # URDF COM is in link frame (proximal, at joint i).
                # DH COM is in distal frame (at joint i+1).
                # Convert: COM_DH = R_z(-theta)^T * (COM_urdf - [a, 0, d])
                # For zero config (theta=0): COM_DH = COM_urdf - [a, 0, d]
                # The DH transform from proximal to distal is:
                #   Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
                # At theta=0, the distal origin in proximal frame is [a, -d*sin(alpha), d*cos(alpha)]
                # Actually: Trans_z(d)*Trans_x(a)*Rot_x(alpha) origin = [a, 0, d] in the
                # intermediate frame (before Rot_z), and Rot_z doesn't change the origin.
                # So distal origin in proximal frame = [a, 0, d] at theta=0.
                # COM in distal frame = COM_urdf - [a, 0, d] (in proximal coords)
                # then rotated by R_z(-theta)^T * Rot_x(-alpha)
                # For simplicity at zero config: COM_DH = COM_urdf - [a, 0, d]
                com_dh = com_urdf - np.array([a, 0.0, d])

                link_inertias.append(LinkInertia(mass=m, com=com_dh, inertia=I_urdf))

                # Compute child transform for recursion
                # T_child = parent_T * T_joint * R_z(theta) where theta will be q
                # For now, use zero config (theta=0)
                T_j = np.eye(4)
                T_j[:3, :3] = _rpy_to_rot(rpy)
                T_j[:3, 3] = xyz
                child_T = parent_T @ T_j

                walk(child_name, child_T)

    T_base = np.eye(4)
    walk(root_link, T_base)

    return RobotDynamicsModel(
        dh_a=np.array(dh_a),
        dh_alpha=np.array(dh_alpha),
        dh_d=np.array(dh_d),
        links=link_inertias,
        gravity=gravity,
        joint_damping=np.zeros(len(link_inertias)),
    )


def quick_urdf(urdf_path):
    # type: (...) -> DynamicsSolver
    from robot_ik.dynamics.solver import DynamicsSolver

    return DynamicsSolver(urdf_to_dynamics_model(urdf_path))
