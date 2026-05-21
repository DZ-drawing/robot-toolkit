"""
Hardware Abstraction Layer (HAL) for robot-toolkit

Provides a unified interface for various hardware protocols (ROS2, Modbus, etc.)
with a simulated implementation for testing and development.
"""

from abc import ABC, abstractmethod

import numpy as np


class HardwareInterface(ABC):
    """Base class for hardware communication interfaces

    All hardware implementations (ROS2, Modbus, simulated, etc.) must
    inherit from this class and implement the required methods.

    Example:
        >>> hardware = SimulatedHardware(dof=6)
        >>> q = hardware.get_joint_positions()
        >>> hardware.set_joint_targets(target_q)
    """

    @abstractmethod
    def get_joint_positions(self) -> np.ndarray:
        """Read current joint positions

        Returns:
            Joint positions (rad) with shape (dof,)
        """
        pass

    @abstractmethod
    def set_joint_targets(self, q: np.ndarray):
        """Set target joint positions

        Args:
            q: Target joint positions (rad) with shape (dof,)
        """
        pass

    @abstractmethod
    def get_joint_velocities(self) -> np.ndarray:
        """Read current joint velocities

        Returns:
            Joint velocities (rad/s) with shape (dof,)
        """
        pass

    @abstractmethod
    def stop(self):
        """Emergency stop - halt all motion immediately"""
        pass

    def get_info(self) -> dict[str, any]:
        """Get hardware information (optional)

        Returns:
            Dictionary with hardware info (name, dof, etc.)
        """
        return {
            "name": "unknown",
            "dof": 6,
        }


class SimulatedHardware(HardwareInterface):
    """Simulated hardware for testing and development

    Simple in-memory simulation that stores joint positions and velocities.
    Useful for:
    - Unit testing
    - Algorithm development
    - Visualization testing
    - Performance benchmarking

    Example:
        >>> hardware = SimulatedHardware(dof=6)
        >>> q_target = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        >>> hardware.set_joint_targets(q_target)
        >>> q_current = hardware.get_joint_positions()
        >>> np.allclose(q_current, q_target)
        True
    """

    def __init__(self, dof: int = 6):
        """Initialize simulated hardware

        Args:
            dof: Number of degrees of freedom (default 6)
        """
        self.dof = dof
        self.q = np.zeros(dof)
        self.dq = np.zeros(dof)
        self.stopped = False

    def get_joint_positions(self) -> np.ndarray:
        """Read current joint positions

        Returns:
            Joint positions (rad) with shape (dof,)
        """
        return self.q.copy()

    def set_joint_targets(self, q: np.ndarray):
        """Set target joint positions (immediate in simulation)

        Args:
            q: Target joint positions (rad) with shape (dof,)
        """
        q = np.asarray(q)
        if q.shape != (self.dof,):
            raise ValueError(f"Expected q shape ({self.dof},), got {q.shape}")

        if not self.stopped:
            self.q = q.copy()

    def get_joint_velocities(self) -> np.ndarray:
        """Read current joint velocities

        Returns:
            Joint velocities (rad/s) with shape (dof,)
        """
        return self.dq.copy()

    def stop(self):
        """Emergency stop - halt all motion immediately"""
        self.stopped = True
        self.dq = np.zeros(self.dof)

    def reset(self):
        """Reset stopped state (for testing)"""
        self.stopped = False

    def get_info(self) -> dict[str, any]:
        """Get hardware information

        Returns:
            Dictionary with hardware info
        """
        return {
            "name": "simulated",
            "dof": self.dof,
            "stopped": self.stopped,
        }


class HardwareRegistry:
    """Registry for hardware interfaces

    Provides factory pattern for creating hardware instances.
    New hardware interfaces can be registered and instantiated by name.

    Example:
        >>> # Register custom hardware
        >>> HardwareRegistry.register("custom", MyCustomHardware)
        >>>
        >>> # Create instance
        >>> hardware = HardwareRegistry.create("custom", param1="value1")
    """

    _interfaces: dict[str, type[HardwareInterface]] = {}

    @classmethod
    def register(cls, name: str, interface_class: type[HardwareInterface]):
        """Register a hardware interface

        Args:
            name: Interface name (e.g., "simulated", "ros2", "modbus")
            interface_class: HardwareInterface subclass
        """
        cls._interfaces[name] = interface_class

    @classmethod
    def create(cls, protocol: str, **kwargs) -> HardwareInterface:
        """Create hardware interface instance

        Args:
            protocol: Protocol name (e.g., "simulated", "ros2")
            **kwargs: Arguments passed to interface class constructor

        Returns:
            HardwareInterface instance

        Raises:
            ValueError: If protocol not registered
        """
        if protocol not in cls._interfaces:
            available = list(cls._interfaces.keys())
            raise ValueError(f"Unknown hardware protocol: {protocol}. " f"Available: {available}")

        interface_class = cls._interfaces[protocol]
        return interface_class(**kwargs)

    @classmethod
    def list_protocols(cls) -> list:
        """List available protocols

        Returns:
            List of registered protocol names
        """
        return list(cls._interfaces.keys())


# Register built-in protocols
HardwareRegistry.register("simulated", SimulatedHardware)

# Optional protocols will be registered if their dependencies are installed
# Example:
# try:
#     from .ros2_interface import ROS2HardwareInterface
#     HardwareRegistry.register("ros2", ROS2HardwareInterface)
# except ImportError:
#     pass
#
# try:
#     from .modbus_interface import ModbusHardwareInterface
#     HardwareRegistry.register("modbus", ModbusHardwareInterface)
# except ImportError:
#     pass
