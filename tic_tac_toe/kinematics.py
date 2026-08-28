"""URDF serial-chain kinematics and damped-least-squares IK for the Panda."""

from __future__ import annotations

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import numpy as np


def _numbers(value, default):
    if value is None:
        return np.array(default, dtype=float)
    return np.array([float(item) for item in value.split()], dtype=float)


def _rotation_x(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]])


def _rotation_y(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]])


def _rotation_z(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def _axis_rotation(axis, angle):
    axis = axis / np.linalg.norm(axis)
    skew = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def _transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = _rotation_z(rpy[2]) @ _rotation_y(rpy[1]) @ _rotation_x(rpy[0])
    result[:3, 3] = xyz
    return result


def _rotation_error(current, target):
    relative = target @ current.T
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cosine)
    vector = np.array(
        [relative[2, 1] - relative[1, 2], relative[0, 2] - relative[2, 0], relative[1, 0] - relative[0, 1]]
    )
    if angle < 1e-7:
        return 0.5 * vector
    sine = math.sin(angle)
    if abs(sine) < 1e-6:
        return 0.5 * vector
    return angle * vector / (2.0 * sine)


@dataclass
class Joint:
    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


class SerialChain:
    """Parse a URDF chain and provide FK, a geometric Jacobian, and DLS IK."""

    def __init__(self, robot_description, base_link="panda_link0", tip_link="panda_tcp"):
        root = ET.fromstring(robot_description)
        child_joints = {}
        for element in root.findall("joint"):
            joint_type = element.attrib["type"]
            parent = element.find("parent").attrib["link"]
            child = element.find("child").attrib["link"]
            origin_element = element.find("origin")
            xyz = _numbers(None if origin_element is None else origin_element.get("xyz"), [0.0, 0.0, 0.0])
            rpy = _numbers(None if origin_element is None else origin_element.get("rpy"), [0.0, 0.0, 0.0])
            axis_element = element.find("axis")
            axis = _numbers(None if axis_element is None else axis_element.get("xyz"), [1.0, 0.0, 0.0])
            limit_element = element.find("limit")
            lower = -math.inf if limit_element is None else float(limit_element.get("lower", "-inf"))
            upper = math.inf if limit_element is None else float(limit_element.get("upper", "inf"))
            child_joints.setdefault(parent, []).append(
                Joint(element.attrib["name"], joint_type, parent, child, _transform(xyz, rpy), axis, lower, upper)
            )

        self.chain = self._find_chain(child_joints, base_link, tip_link, [])
        if not self.chain:
            raise ValueError(f"No URDF chain from {base_link} to {tip_link}")
        self.active_joints = [joint for joint in self.chain if joint.joint_type != "fixed"]
        self.joint_names = [joint.name for joint in self.active_joints]
        self.lower = np.array([joint.lower for joint in self.active_joints])
        self.upper = np.array([joint.upper for joint in self.active_joints])

    @classmethod
    def _find_chain(cls, graph, current, target, visited):
        if current == target:
            return visited
        for joint in graph.get(current, []):
            result = cls._find_chain(graph, joint.child, target, visited + [joint])
            if result:
                return result
        return []

    def forward(self, positions, with_jacobian=False):
        positions = np.asarray(positions, dtype=float)
        if positions.shape != (len(self.active_joints),):
            raise ValueError(f"Expected {len(self.active_joints)} joints, got {positions.shape}")
        transform = np.eye(4)
        origins, axes = [], []
        active_index = 0
        for joint in self.chain:
            transform = transform @ joint.origin
            if joint.joint_type == "fixed":
                continue
            world_axis = transform[:3, :3] @ joint.axis
            origins.append(transform[:3, 3].copy())
            axes.append(world_axis)
            value = positions[active_index]
            active_index += 1
            motion = np.eye(4)
            if joint.joint_type in ("revolute", "continuous"):
                motion[:3, :3] = _axis_rotation(joint.axis, value)
            elif joint.joint_type == "prismatic":
                motion[:3, 3] = joint.axis * value
            else:
                raise ValueError(f"Unsupported joint type: {joint.joint_type}")
            transform = transform @ motion
        if not with_jacobian:
            return transform

        end_position = transform[:3, 3]
        jacobian = np.zeros((6, len(self.active_joints)))
        for index, joint in enumerate(self.active_joints):
            if joint.joint_type in ("revolute", "continuous"):
                jacobian[:3, index] = np.cross(axes[index], end_position - origins[index])
                jacobian[3:, index] = axes[index]
            else:
                jacobian[:3, index] = axes[index]
        return transform, jacobian

    def inverse(
        self, target_position, target_rotation, seed, preferred=None,
        preference_weights=None, max_iterations=450,
    ):
        target_position = np.asarray(target_position, dtype=float)
        target_rotation = np.asarray(target_rotation, dtype=float)
        seed = np.asarray(seed, dtype=float)
        preferred = seed.copy() if preferred is None else np.asarray(preferred, dtype=float)
        preference_weights = (
            np.ones_like(seed)
            if preference_weights is None
            else np.asarray(preference_weights, dtype=float)
        )
        margin, orientation_weight, damping = 1e-4, 0.32, 0.035
        candidates = [seed.copy(), preferred.copy(), 0.65 * seed + 0.35 * preferred]
        best, best_score = None, math.inf

        for candidate in candidates:
            positions = np.clip(candidate, self.lower + margin, self.upper - margin)
            for _ in range(max_iterations):
                current, jacobian = self.forward(positions, with_jacobian=True)
                position_error = target_position - current[:3, 3]
                orientation_error = _rotation_error(current[:3, :3], target_rotation)
                error = np.concatenate([position_error, orientation_weight * orientation_error])
                weighted_jacobian = jacobian.copy()
                weighted_jacobian[3:, :] *= orientation_weight
                score = np.linalg.norm(position_error) + 0.12 * np.linalg.norm(orientation_error)
                if score < best_score:
                    best_score, best = score, positions.copy()
                if np.linalg.norm(position_error) < 0.004 and np.linalg.norm(orientation_error) < 0.12:
                    return positions
                system = weighted_jacobian @ weighted_jacobian.T + damping * damping * np.eye(6)
                step = weighted_jacobian.T @ np.linalg.solve(system, error)
                step += 0.012 * preference_weights * (preferred - positions)
                largest = np.max(np.abs(step))
                if largest > 0.13:
                    step *= 0.13 / largest
                positions = np.clip(positions + 0.72 * step, self.lower + margin, self.upper - margin)
        if best_score < 0.035:
            return best
        raise ValueError(f"Panda IK did not converge; best weighted error={best_score:.4f}")
