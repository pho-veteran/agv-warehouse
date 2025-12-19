#!/usr/bin/env python3
"""
Example script to spawn a box model in Gazebo using ROS2 service.

Usage:
    python3 spawn_box_example.py
    python3 spawn_box_example.py --x 1.0 --y 2.0 --z 0.5 --size 0.3
"""

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
import argparse


def create_box_sdf(name: str, x: float, y: float, z: float, 
                   size_x: float, size_y: float, size_z: float,
                   r: float = 0.8, g: float = 0.4, b: float = 0.1) -> str:
    """Create SDF XML string for a box model.
    
    Args:
        name: Model name
        x, y, z: Position coordinates
        size_x, size_y, size_z: Box dimensions
        r, g, b: RGB color values (0-1)
    
    Returns:
        SDF XML string
    """
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <static>false</static>
    <pose>{x} {y} {z} 0 0 0</pose>
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
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
      </collision>
      <visual name="box_visual">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
    return sdf


class BoxSpawner(Node):
    """Node to spawn box models in Gazebo."""
    
    def __init__(self, world_name: str = "tugbot_warehouse"):
        super().__init__('box_spawner')
        self.world_name = world_name
        self.service_name = f'/world/{world_name}/create'
        self.client = self.create_client(SpawnEntity, self.service_name)
        
        # Wait for service to be available
        self.get_logger().info(f'Waiting for service {self.service_name}...')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {self.service_name} not available!')
            raise RuntimeError(f'Service {self.service_name} not available')
        
        self.get_logger().info(f'Service {self.service_name} is ready!')
    
    def spawn_box(self, name: str, x: float, y: float, z: float,
                  size_x: float = 0.5, size_y: float = 0.5, size_z: float = 0.5,
                  r: float = 0.8, g: float = 0.4, b: float = 0.1) -> bool:
        """Spawn a box model in Gazebo.
        
        Args:
            name: Unique name for the box model
            x, y, z: Position coordinates
            size_x, size_y, size_z: Box dimensions
            r, g, b: RGB color values (0-1)
        
        Returns:
            True if spawn succeeded, False otherwise
        """
        # Create SDF XML
        sdf_xml = create_box_sdf(name, x, y, z, size_x, size_y, size_z, r, g, b)
        
        # Create request
        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf_xml
        
        # Call service
        self.get_logger().info(f'Spawning box "{name}" at ({x}, {y}, {z})...')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Box "{name}" spawned successfully!')
                    return True
                else:
                    self.get_logger().error(f'Failed to spawn box: {response.message}')
                    return False
            except Exception as e:
                self.get_logger().error(f'Exception while spawning box: {e}')
                return False
        else:
            self.get_logger().error('Service call timed out!')
            return False


def main():
    parser = argparse.ArgumentParser(description='Spawn a box model in Gazebo')
    parser.add_argument('--world', type=str, default='tugbot_warehouse',
                       help='Gazebo world name')
    parser.add_argument('--name', type=str, default='test_box',
                       help='Model name')
    parser.add_argument('--x', type=float, default=0.0, help='X position')
    parser.add_argument('--y', type=float, default=0.0, help='Y position')
    parser.add_argument('--z', type=float, default=1.0, help='Z position')
    parser.add_argument('--size', type=float, default=0.5,
                       help='Box size (uniform for all dimensions)')
    parser.add_argument('--size-x', type=float, help='Box size X')
    parser.add_argument('--size-y', type=float, help='Box size Y')
    parser.add_argument('--size-z', type=float, help='Box size Z')
    parser.add_argument('--r', type=float, default=0.8, help='Red color (0-1)')
    parser.add_argument('--g', type=float, default=0.4, help='Green color (0-1)')
    parser.add_argument('--b', type=float, default=0.1, help='Blue color (0-1)')
    
    args = parser.parse_args()
    
    # Determine box dimensions
    size_x = args.size_x if args.size_x is not None else args.size
    size_y = args.size_y if args.size_y is not None else args.size
    size_z = args.size_z if args.size_z is not None else args.size
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create spawner node
        spawner = BoxSpawner(world_name=args.world)
        
        # Spawn box
        success = spawner.spawn_box(
            name=args.name,
            x=args.x,
            y=args.y,
            z=args.z,
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            r=args.r,
            g=args.g,
            b=args.b
        )
        
        if success:
            print(f"✓ Box '{args.name}' spawned successfully at ({args.x}, {args.y}, {args.z})")
        else:
            print(f"✗ Failed to spawn box '{args.name}'")
            exit(1)
            
    except Exception as e:
        print(f"Error: {e}")
        exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
