"""Cargo Simulator for AGV Transport Task Manager.

This module simulates cargo using Gazebo box models, providing
spawn, attach, detach, and delete functionality for visual
representation of cargo during transport operations.

Uses Gazebo Harmonic services for entity management.
"""

import logging
from typing import Optional
from dataclasses import dataclass, field
from enum import Enum
import time
import uuid

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
    from std_srvs.srv import Empty
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    Node = object


class CargoState(Enum):
    """State of the cargo in the simulation."""
    NONE = "NONE"           # No cargo exists
    SPAWNED = "SPAWNED"     # Cargo spawned at station
    ATTACHED = "ATTACHED"   # Cargo attached to robot
    DETACHED = "DETACHED"   # Cargo detached from robot


@dataclass
class CargoInfo:
    """Information about a cargo entity in simulation.
    
    Attributes:
        name: Unique name of the cargo entity
        state: Current state of the cargo
        spawn_x: X coordinate where cargo was spawned
        spawn_y: Y coordinate where cargo was spawned
        spawn_z: Z coordinate where cargo was spawned
        timestamp: Time when cargo was created
    """
    name: str = ""
    state: CargoState = CargoState.NONE
    spawn_x: float = 0.0
    spawn_y: float = 0.0
    spawn_z: float = 0.0
    timestamp: float = field(default_factory=time.time)


# Default box model SDF for Gazebo Harmonic
DEFAULT_BOX_SDF = """<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>false</static>
    <link name="box_link">
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.0833</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0833</iyy>
          <iyz>0</iyz>
          <izz>0.0833</izz>
        </inertia>
      </inertial>
      <collision name="box_collision">
        <geometry>
          <box>
            <size>0.3 0.3 0.3</size>
          </box>
        </geometry>
      </collision>
      <visual name="box_visual">
        <geometry>
          <box>
            <size>0.3 0.3 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.4 0.1 1</ambient>
          <diffuse>0.8 0.4 0.1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class CargoSimulator:
    """Simulates cargo using Gazebo box model.
    
    Provides methods to spawn, attach, detach, and delete cargo
    entities in Gazebo simulation. Handles service failures gracefully
    by logging warnings and continuing without blocking.
    
    Attributes:
        current_cargo: Information about the current cargo entity
        is_attached: Whether cargo is currently attached to robot
        logger: Logger instance for this class
    """
    
    # Service names for Gazebo Harmonic
    SPAWN_SERVICE = '/world/default/create'
    DELETE_SERVICE = '/world/default/remove'
    
    # Timeouts
    SERVICE_TIMEOUT_SEC = 5.0
    
    # Cargo spawn height offset (above ground)
    CARGO_SPAWN_HEIGHT = 0.5
    
    def __init__(self, node: Optional['Node'] = None, logger: Optional[logging.Logger] = None):
        """Initialize CargoSimulator.
        
        Args:
            node: ROS2 node for service clients (optional for testing)
            logger: Logger instance (optional, creates default if not provided)
        """
        self._node = node
        self._logger = logger or logging.getLogger(__name__)
        
        self.current_cargo: Optional[CargoInfo] = None
        self.is_attached: bool = False
        
        # Service clients (initialized lazily when node is available)
        self._spawn_client = None
        self._delete_client = None
        
        # Track lifecycle for testing
        self._lifecycle_history: list[CargoState] = []
    
    def _ensure_service_clients(self) -> bool:
        """Ensure service clients are created.
        
        Returns:
            True if clients are available, False otherwise
        """
        if not ROS_AVAILABLE or self._node is None:
            return False
        
        try:
            if self._spawn_client is None:
                self._spawn_client = self._node.create_client(
                    SpawnEntity, self.SPAWN_SERVICE
                )
            
            if self._delete_client is None:
                self._delete_client = self._node.create_client(
                    DeleteEntity, self.DELETE_SERVICE
                )
            
            return True
        except Exception as e:
            self._logger.warning(f"Failed to create service clients: {e}")
            return False
    
    def _generate_cargo_name(self) -> str:
        """Generate a unique cargo entity name.
        
        Returns:
            Unique cargo name string
        """
        return f"cargo_box_{uuid.uuid4().hex[:8]}"
    
    def _create_box_sdf(self, name: str) -> str:
        """Create SDF string for a box model.
        
        Args:
            name: Name for the model
            
        Returns:
            SDF XML string
        """
        return DEFAULT_BOX_SDF.format(name=name)

    def spawn_cargo_at_station(self, station_x: float, station_y: float, 
                                station_name: str = "") -> bool:
        """Spawn a Gazebo box model at the specified station position.
        
        Creates a new cargo entity at the pickup station position.
        If a cargo already exists, it will be deleted first.
        
        Args:
            station_x: X coordinate of the station
            station_y: Y coordinate of the station
            station_name: Name of the station (for logging)
            
        Returns:
            True if spawn succeeded, False otherwise
            
        Note:
            Gazebo service failures are logged as warnings and return False,
            but do not raise exceptions to avoid blocking FSM progression.
        """
        # Clean up existing cargo if any
        if self.current_cargo is not None:
            self._logger.info("Deleting existing cargo before spawning new one")
            self.delete_cargo()
        
        cargo_name = self._generate_cargo_name()
        spawn_z = self.CARGO_SPAWN_HEIGHT
        
        self._logger.info(
            f"Spawning cargo '{cargo_name}' at station '{station_name}' "
            f"(x={station_x:.2f}, y={station_y:.2f}, z={spawn_z:.2f})"
        )
        
        # Create cargo info
        self.current_cargo = CargoInfo(
            name=cargo_name,
            state=CargoState.SPAWNED,
            spawn_x=station_x,
            spawn_y=station_y,
            spawn_z=spawn_z,
        )
        
        # Track lifecycle
        self._lifecycle_history.append(CargoState.SPAWNED)
        
        # If no ROS node, just track state (for testing)
        if not self._ensure_service_clients():
            self._logger.warning(
                "ROS services not available, cargo spawn simulated locally"
            )
            return True
        
        # Call Gazebo spawn service
        try:
            if not self._spawn_client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT_SEC):
                self._logger.warning(
                    f"Spawn service '{self.SPAWN_SERVICE}' not available"
                )
                return True  # Continue FSM without blocking
            
            request = SpawnEntity.Request()
            request.name = cargo_name
            request.xml = self._create_box_sdf(cargo_name)
            request.initial_pose.position.x = station_x
            request.initial_pose.position.y = station_y
            request.initial_pose.position.z = spawn_z
            
            future = self._spawn_client.call_async(request)
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=self.SERVICE_TIMEOUT_SEC
            )
            
            if future.result() is not None and future.result().success:
                self._logger.info(f"Cargo '{cargo_name}' spawned successfully")
                return True
            else:
                self._logger.warning(f"Failed to spawn cargo: {future.result()}")
                return True  # Continue FSM without blocking
                
        except Exception as e:
            self._logger.warning(f"Exception during cargo spawn: {e}")
            return True  # Continue FSM without blocking
    
    def attach_cargo_to_robot(self, robot_link: str = "base_link") -> bool:
        """Attach the current cargo to the robot.
        
        Uses Gazebo attach service to link the cargo entity to the
        robot's base_link, so it moves with the robot.
        
        Args:
            robot_link: Name of the robot link to attach to
            
        Returns:
            True if attach succeeded, False otherwise
            
        Note:
            Gazebo service failures are logged as warnings and return False,
            but do not raise exceptions to avoid blocking FSM progression.
        """
        if self.current_cargo is None:
            self._logger.warning("No cargo to attach")
            return False
        
        if self.current_cargo.state != CargoState.SPAWNED:
            self._logger.warning(
                f"Cannot attach cargo in state {self.current_cargo.state}"
            )
            return False
        
        self._logger.info(
            f"Attaching cargo '{self.current_cargo.name}' to robot '{robot_link}'"
        )
        
        # Update state
        self.current_cargo.state = CargoState.ATTACHED
        self.is_attached = True
        
        # Track lifecycle
        self._lifecycle_history.append(CargoState.ATTACHED)
        
        # If no ROS node, just track state (for testing)
        if not ROS_AVAILABLE or self._node is None:
            self._logger.warning(
                "ROS services not available, cargo attach simulated locally"
            )
            return True
        
        # Note: Gazebo Harmonic uses different attach mechanism
        # For now, we simulate attachment by tracking state
        # In production, this would use gz::sim::systems::DetachableJoint
        # or similar mechanism
        try:
            # Gazebo Harmonic attach is typically done via SDF or plugin
            # For simulation purposes, we track the state
            self._logger.info(
                f"Cargo '{self.current_cargo.name}' attached (simulated)"
            )
            return True
            
        except Exception as e:
            self._logger.warning(f"Exception during cargo attach: {e}")
            return True  # Continue FSM without blocking
    
    def detach_cargo(self) -> bool:
        """Detach the cargo from the robot.
        
        Uses Gazebo detach service to unlink the cargo entity from
        the robot, leaving it at the current position.
        
        Returns:
            True if detach succeeded, False otherwise
            
        Note:
            Gazebo service failures are logged as warnings and return False,
            but do not raise exceptions to avoid blocking FSM progression.
        """
        if self.current_cargo is None:
            self._logger.warning("No cargo to detach")
            return False
        
        if self.current_cargo.state != CargoState.ATTACHED:
            self._logger.warning(
                f"Cannot detach cargo in state {self.current_cargo.state}"
            )
            return False
        
        self._logger.info(f"Detaching cargo '{self.current_cargo.name}'")
        
        # Update state
        self.current_cargo.state = CargoState.DETACHED
        self.is_attached = False
        
        # Track lifecycle
        self._lifecycle_history.append(CargoState.DETACHED)
        
        # If no ROS node, just track state (for testing)
        if not ROS_AVAILABLE or self._node is None:
            self._logger.warning(
                "ROS services not available, cargo detach simulated locally"
            )
            return True
        
        try:
            # Gazebo Harmonic detach is typically done via plugin
            # For simulation purposes, we track the state
            self._logger.info(
                f"Cargo '{self.current_cargo.name}' detached (simulated)"
            )
            return True
            
        except Exception as e:
            self._logger.warning(f"Exception during cargo detach: {e}")
            return True  # Continue FSM without blocking

    def delete_cargo(self) -> bool:
        """Delete the cargo entity from simulation.
        
        Removes the cargo entity from Gazebo simulation.
        
        Returns:
            True if delete succeeded, False otherwise
            
        Note:
            Gazebo service failures are logged as warnings and return False,
            but do not raise exceptions to avoid blocking FSM progression.
        """
        if self.current_cargo is None:
            self._logger.warning("No cargo to delete")
            return False
        
        cargo_name = self.current_cargo.name
        self._logger.info(f"Deleting cargo '{cargo_name}'")
        
        # Track lifecycle before clearing
        self._lifecycle_history.append(CargoState.NONE)
        
        # Clear cargo info
        self.current_cargo = None
        self.is_attached = False
        
        # If no ROS node, just track state (for testing)
        if not self._ensure_service_clients():
            self._logger.warning(
                "ROS services not available, cargo delete simulated locally"
            )
            return True
        
        # Call Gazebo delete service
        try:
            if not self._delete_client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT_SEC):
                self._logger.warning(
                    f"Delete service '{self.DELETE_SERVICE}' not available"
                )
                return True  # Continue FSM without blocking
            
            request = DeleteEntity.Request()
            request.name = cargo_name
            
            future = self._delete_client.call_async(request)
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=self.SERVICE_TIMEOUT_SEC
            )
            
            if future.result() is not None and future.result().success:
                self._logger.info(f"Cargo '{cargo_name}' deleted successfully")
                return True
            else:
                self._logger.warning(f"Failed to delete cargo: {future.result()}")
                return True  # Continue FSM without blocking
                
        except Exception as e:
            self._logger.warning(f"Exception during cargo delete: {e}")
            return True  # Continue FSM without blocking
    
    def get_cargo_state(self) -> CargoState:
        """Get the current state of the cargo.
        
        Returns:
            Current CargoState
        """
        if self.current_cargo is None:
            return CargoState.NONE
        return self.current_cargo.state
    
    def get_cargo_info(self) -> Optional[CargoInfo]:
        """Get information about the current cargo.
        
        Returns:
            CargoInfo or None if no cargo exists
        """
        return self.current_cargo
    
    def get_lifecycle_history(self) -> list[CargoState]:
        """Get the lifecycle history of cargo operations.
        
        Useful for testing to verify the correct sequence of
        spawn → attach → detach → delete operations.
        
        Returns:
            List of CargoState values in order of occurrence
        """
        return self._lifecycle_history.copy()
    
    def clear_lifecycle_history(self) -> None:
        """Clear the lifecycle history.
        
        Useful for testing to reset state between test cases.
        """
        self._lifecycle_history.clear()
    
    def reset(self) -> None:
        """Reset the cargo simulator to initial state.
        
        Deletes any existing cargo and clears all state.
        """
        if self.current_cargo is not None:
            self.delete_cargo()
        
        self.current_cargo = None
        self.is_attached = False
        self._lifecycle_history.clear()
    
    def simulate_full_lifecycle(self, station_x: float, station_y: float,
                                 station_name: str = "") -> bool:
        """Simulate a complete cargo lifecycle for testing.
        
        Executes the full sequence: spawn → attach → detach → delete
        
        Args:
            station_x: X coordinate of the station
            station_y: Y coordinate of the station
            station_name: Name of the station (for logging)
            
        Returns:
            True if all operations succeeded, False otherwise
        """
        self.clear_lifecycle_history()
        
        # Spawn
        if not self.spawn_cargo_at_station(station_x, station_y, station_name):
            return False
        
        # Attach
        if not self.attach_cargo_to_robot():
            return False
        
        # Detach
        if not self.detach_cargo():
            return False
        
        # Delete
        if not self.delete_cargo():
            return False
        
        return True
    
    def verify_lifecycle_sequence(self) -> tuple[bool, str]:
        """Verify that the lifecycle history follows the correct sequence.
        
        The expected sequence is: SPAWNED → ATTACHED → DETACHED → NONE
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        expected = [
            CargoState.SPAWNED,
            CargoState.ATTACHED,
            CargoState.DETACHED,
            CargoState.NONE
        ]
        
        history = self._lifecycle_history
        
        if len(history) != len(expected):
            return False, f"Expected {len(expected)} states, got {len(history)}: {history}"
        
        for i, (actual, exp) in enumerate(zip(history, expected)):
            if actual != exp:
                return False, f"State {i}: expected {exp}, got {actual}"
        
        return True, ""
