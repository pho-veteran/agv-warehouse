#!/usr/bin/env python3
"""CLI script to send transport orders to the AGV Transport Task Manager.

This script provides a command-line interface to publish TransportOrder messages
to the /agv/transport_orders topic for testing the transport system.

Usage:
    ros2 run agv_transport send_transport_order <pickup_station> <dropoff_station>
    ros2 run agv_transport send_transport_order --list-stations
    ros2 run agv_transport send_transport_order --help

Examples:
    ros2 run agv_transport send_transport_order dock_in_1 dock_out
    ros2 run agv_transport send_transport_order dock_in_2 charging_station

Requirements: 6.1, 6.2, 6.3
"""

import argparse
import sys
import uuid
import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


# Available stations from warehouse_stations.yaml
AVAILABLE_STATIONS = [
    'charging_station',
    'dock_in_1',
    'dock_in_2',
    'dock_out',
]


class TransportOrderPublisher(Node):
    """ROS2 node to publish transport orders."""
    
    def __init__(self):
        super().__init__('send_transport_order')
        
        # QoS profile must match transport_task_manager_node subscriber
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.publisher = self.create_publisher(
            String, 
            '/agv/transport_orders', 
            qos_profile
        )
        # Wait for publisher to be ready
        time.sleep(0.5)
    
    def send_order(self, pickup_station: str, dropoff_station: str, priority: int = 0) -> str:
        """Create and publish a transport order.
        
        Args:
            pickup_station: Station name for pickup
            dropoff_station: Station name for dropoff
            priority: Order priority (default 0)
            
        Returns:
            The order_id of the created order
        """
        order_id = str(uuid.uuid4())
        
        order = {
            'order_id': order_id,
            'pickup_station': pickup_station,
            'dropoff_station': dropoff_station,
            'priority': priority,
            'timestamp': time.time(),
        }
        
        msg = String()
        msg.data = json.dumps(order)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published transport order: {order_id}')
        self.get_logger().info(f'  Pickup: {pickup_station} -> Dropoff: {dropoff_station}')
        
        return order_id


def print_usage_help():
    """Display usage help with available station names."""
    print("\n" + "=" * 60)
    print("AGV Transport Order CLI")
    print("=" * 60)
    print("\nUsage:")
    print("  ros2 run agv_transport send_transport_order <pickup> <dropoff>")
    print("  ros2 run agv_transport send_transport_order --list-stations")
    print("  ros2 run agv_transport send_transport_order --help")
    print("\nAvailable Stations:")
    print("-" * 40)
    for station in AVAILABLE_STATIONS:
        station_type = "charger" if "charging" in station else "dock"
        print(f"  - {station:<20} ({station_type})")
    print("-" * 40)
    print("\nExamples:")
    print("  # Transport from dock_in_1 to dock_out")
    print("  ros2 run agv_transport send_transport_order dock_in_1 dock_out")
    print("\n  # Transport from dock_in_2 to charging_station")
    print("  ros2 run agv_transport send_transport_order dock_in_2 charging_station")
    print("\n  # With priority (higher = more urgent)")
    print("  ros2 run agv_transport send_transport_order dock_in_1 dock_out --priority 5")
    print("=" * 60 + "\n")


def validate_station(station_name: str) -> bool:
    """Check if station name is valid.
    
    Args:
        station_name: Name of the station to validate
        
    Returns:
        True if station exists, False otherwise
    """
    return station_name in AVAILABLE_STATIONS


def main(args=None):
    """Main entry point for the CLI script."""
    parser = argparse.ArgumentParser(
        description='Send transport orders to AGV Transport Task Manager',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Available Stations:
  - charging_station    (charger)
  - dock_in_1          (dock)
  - dock_in_2          (dock)
  - dock_out           (dock)

Examples:
  %(prog)s dock_in_1 dock_out
  %(prog)s dock_in_2 charging_station --priority 5
        """
    )
    
    parser.add_argument(
        'pickup_station',
        nargs='?',
        help='Pickup station name'
    )
    parser.add_argument(
        'dropoff_station',
        nargs='?',
        help='Dropoff station name'
    )
    parser.add_argument(
        '--priority', '-p',
        type=int,
        default=0,
        help='Order priority (higher = more urgent, default: 0)'
    )
    parser.add_argument(
        '--list-stations', '-l',
        action='store_true',
        help='List available stations and exit'
    )
    
    parsed_args = parser.parse_args()
    
    # Handle --list-stations flag
    if parsed_args.list_stations:
        print_usage_help()
        return 0
    
    # If no arguments provided, show help
    if parsed_args.pickup_station is None or parsed_args.dropoff_station is None:
        print_usage_help()
        if parsed_args.pickup_station is None and parsed_args.dropoff_station is None:
            print("Error: Both pickup_station and dropoff_station are required.\n")
        elif parsed_args.pickup_station is None:
            print("Error: pickup_station is required.\n")
        else:
            print("Error: dropoff_station is required.\n")
        return 1
    
    pickup = parsed_args.pickup_station
    dropoff = parsed_args.dropoff_station
    priority = parsed_args.priority
    
    # Validate stations
    errors = []
    if not validate_station(pickup):
        errors.append(f"Invalid pickup station: '{pickup}'")
    if not validate_station(dropoff):
        errors.append(f"Invalid dropoff station: '{dropoff}'")
    if pickup == dropoff:
        errors.append("Pickup and dropoff stations must be different")
    
    if errors:
        print("\nValidation Error(s):")
        for error in errors:
            print(f"  - {error}")
        print("\nUse --list-stations to see available stations.\n")
        return 1
    
    # Initialize ROS2 and send order
    rclpy.init(args=args)
    
    try:
        node = TransportOrderPublisher()
        order_id = node.send_order(pickup, dropoff, priority)
        
        print(f"\n✓ Transport order sent successfully!")
        print(f"  Order ID: {order_id}")
        print(f"  Route: {pickup} -> {dropoff}")
        print(f"  Priority: {priority}")
        print("\nMonitor status with:")
        print("  ros2 topic echo /agv/transport_status\n")
        
        # Give time for message to be sent
        rclpy.spin_once(node, timeout_sec=0.5)
        
        node.destroy_node()
    except Exception as e:
        print(f"\nError: Failed to send transport order: {e}\n")
        return 1
    finally:
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
