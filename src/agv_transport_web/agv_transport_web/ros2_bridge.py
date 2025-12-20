#!/usr/bin/env python3
"""
ROS2 Bridge for AGV Transport Web Dashboard

This module provides the ROS2Bridge class that handles communication between
the FastAPI web server and the ROS2 transport system. It manages subscriptions
to transport status topics and publishing of transport orders.
"""

import asyncio
import json
import threading
import time
from typing import Optional, Dict, Any, Callable, List
from dataclasses import asdict
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

# Import transport models from the transport package
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../../agv_transport'))
from agv_transport.models import TransportOrder, TransportStatus


class ROS2Bridge(Node):
    """
    ROS2 Bridge for web dashboard integration.
    
    This class manages ROS2 communication for the web dashboard, including:
    - Subscribing to transport status updates
    - Publishing transport orders
    - Caching latest status data
    - Handling connection errors gracefully
    """
    
    def __init__(self, node_name: str = 'agv_transport_web_bridge'):
        """Initialize the ROS2 bridge.
        
        Args:
            node_name: Name for the ROS2 node
        """
        super().__init__(node_name)
        
        # Thread-safe message caching with history
        self._status_lock = threading.RLock()
        self._cached_status: Optional[TransportStatus] = None
        self._status_history: deque = deque(maxlen=10)  # Keep last 10 status updates
        self._last_status_time: float = 0.0
        self._status_timeout: float = 10.0  # Status considered stale after 10 seconds
        
        # Order tracking - store created orders
        self._orders_lock = threading.RLock()
        self._orders: Dict[str, Dict[str, Any]] = {}  # order_id -> order data
        
        # Connection health monitoring with metrics
        self._connection_healthy = False
        self._last_heartbeat = time.time()
        self._connection_attempts = 0
        self._successful_messages = 0
        self._failed_messages = 0
        
        # Thread pool for async operations
        self._thread_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix='ros2_bridge')
        
        # Status update callbacks with error handling
        self._status_callbacks: List[Callable[[TransportStatus], None]] = []
        self._callback_errors: Dict[str, int] = {}  # Track callback errors
        
        # Initialize ROS2 publishers and subscribers
        self._setup_ros2_communication()
        
        self.get_logger().info(f'ROS2 Bridge initialized as node: {node_name}')
    
    def _setup_ros2_communication(self):
        """Set up ROS2 publishers and subscribers."""
        try:
            # QoS profile matching the transport task manager
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
            
            # Subscribe to transport status topic
            self._status_subscriber = self.create_subscription(
                String,
                '/agv/transport_status',
                self._status_callback,
                qos_profile
            )
            
            # Publisher for transport orders - must match task manager's subscriber QoS
            self._order_publisher = self.create_publisher(
                String,
                '/agv/transport_orders',
                qos_profile
            )
            
            # Health check timer
            self._health_timer = self.create_timer(
                5.0,  # Check every 5 seconds
                self._health_check_callback
            )
            
            self._connection_healthy = True
            self.get_logger().info('ROS2 communication setup completed with RELIABLE QoS')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup ROS2 communication: {e}')
            self._connection_healthy = False
            raise
    
    def _status_callback(self, msg: String):
        """Handle incoming transport status messages with thread safety.
        
        Args:
            msg: ROS2 String message containing JSON status data
        """
        try:
            # Parse JSON status data
            status_data = json.loads(msg.data)
            status = TransportStatus.from_dict(status_data)
            
            # Update cached status with thread safety
            with self._status_lock:
                self._cached_status = status
                self._last_status_time = time.time()
                self._last_heartbeat = time.time()
                
                # Add to history with timestamp
                self._status_history.append({
                    'status': status.to_dict(),
                    'timestamp': self._last_status_time
                })
                
                self._successful_messages += 1
            
            # Notify callbacks asynchronously to avoid blocking
            self._notify_callbacks_async(status)
            
            self.get_logger().debug(f'Status updated: {status.current_state}')
            
        except json.JSONDecodeError as e:
            self._failed_messages += 1
            self.get_logger().error(f'Failed to parse status JSON: {e}')
        except Exception as e:
            self._failed_messages += 1
            self.get_logger().error(f'Status callback error: {e}')
    
    def _notify_callbacks_async(self, status: TransportStatus):
        """Notify status callbacks asynchronously to prevent blocking.
        
        Args:
            status: TransportStatus to send to callbacks
        """
        def notify_callbacks():
            for i, callback in enumerate(self._status_callbacks):
                try:
                    callback(status)
                except Exception as e:
                    callback_id = f'callback_{i}'
                    self._callback_errors[callback_id] = self._callback_errors.get(callback_id, 0) + 1
                    self.get_logger().warning(
                        f'Status callback {i} failed (error #{self._callback_errors[callback_id]}): {e}'
                    )
        
        # Submit to thread pool to avoid blocking ROS2 callback
        self._thread_pool.submit(notify_callbacks)
    
    def _health_check_callback(self):
        """Periodic health check for connection monitoring."""
        current_time = time.time()
        
        # Check if we've received recent status updates
        if current_time - self._last_heartbeat > self._status_timeout:
            if self._connection_healthy:
                self.get_logger().warning('Transport system connection appears unhealthy')
                self._connection_healthy = False
        else:
            if not self._connection_healthy:
                self.get_logger().info('Transport system connection restored')
                self._connection_healthy = True
    
    def get_cached_status(self) -> Optional[Dict[str, Any]]:
        """Get the latest cached transport status with thread safety.
        
        Returns:
            Dictionary containing status data and metadata, or None if no data
        """
        with self._status_lock:
            if self._cached_status is None:
                return None
            
            current_time = time.time()
            age = current_time - self._last_status_time
            is_stale = age > self._status_timeout
            
            return {
                'status': self._cached_status.to_dict(),
                'timestamp': self._last_status_time,
                'age_seconds': age,
                'is_stale': is_stale,
                'connection_healthy': self._connection_healthy,
                'message_stats': {
                    'successful': self._successful_messages,
                    'failed': self._failed_messages,
                    'success_rate': (
                        self._successful_messages / max(1, self._successful_messages + self._failed_messages)
                    )
                }
            }
    
    def get_status_history(self) -> List[Dict[str, Any]]:
        """Get recent status history with thread safety.
        
        Returns:
            List of recent status updates with timestamps
        """
        with self._status_lock:
            return list(self._status_history)
    
    def publish_transport_order(self, order: TransportOrder) -> bool:
        """Publish a transport order to ROS2.
        
        Args:
            order: TransportOrder to publish
            
        Returns:
            True if published successfully, False otherwise
        """
        try:
            # Validate order before publishing
            is_valid, error_msg = order.validate()
            if not is_valid:
                self.get_logger().error(f'Invalid transport order: {error_msg}')
                return False
            
            # Convert to JSON and publish
            order_json = json.dumps(order.to_dict())
            msg = String()
            msg.data = order_json
            
            self._order_publisher.publish(msg)
            
            # Track the order
            with self._orders_lock:
                self._orders[order.order_id] = {
                    'order_id': order.order_id,
                    'pickup_station': order.pickup_station,
                    'dropoff_station': order.dropoff_station,
                    'priority': order.priority,
                    'status': 'QUEUED',
                    'created_at': time.time(),
                    'timestamp': order.timestamp
                }
            
            self.get_logger().info(
                f'Published transport order: {order.order_id} '
                f'({order.pickup_station} -> {order.dropoff_station})'
            )
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish transport order: {e}')
            return False
    
    def get_orders(self) -> List[Dict[str, Any]]:
        """Get all tracked orders.
        
        Returns:
            List of order dictionaries
        """
        with self._orders_lock:
            # Update order statuses based on current status
            if self._cached_status:
                active_order_id = self._cached_status.active_order_id
                current_state = self._cached_status.current_state
                
                for order_id, order in self._orders.items():
                    if order_id == active_order_id:
                        order['status'] = 'ACTIVE'
                    elif order['status'] == 'ACTIVE' and order_id != active_order_id:
                        # Order was active but no longer - mark as completed
                        order['status'] = 'COMPLETED'
            
            # Return orders sorted by created_at (newest first)
            return sorted(
                list(self._orders.values()),
                key=lambda x: x.get('created_at', 0),
                reverse=True
            )
    
    def add_status_callback(self, callback: Callable[[TransportStatus], None]):
        """Add a callback for status updates.
        
        Args:
            callback: Function to call when status is updated
        """
        self._status_callbacks.append(callback)
    
    def remove_status_callback(self, callback: Callable[[TransportStatus], None]):
        """Remove a status update callback.
        
        Args:
            callback: Function to remove from callbacks
        """
        if callback in self._status_callbacks:
            self._status_callbacks.remove(callback)
    
    def is_connection_healthy(self) -> bool:
        """Check if the ROS2 connection is healthy.
        
        Returns:
            True if connection is healthy, False otherwise
        """
        return self._connection_healthy
    
    def get_connection_info(self) -> Dict[str, Any]:
        """Get detailed connection information with thread safety.
        
        Returns:
            Dictionary with connection status and metrics
        """
        current_time = time.time()
        
        with self._status_lock:
            return {
                'healthy': self._connection_healthy,
                'last_heartbeat': self._last_heartbeat,
                'heartbeat_age': current_time - self._last_heartbeat,
                'has_cached_status': self._cached_status is not None,
                'last_status_time': self._last_status_time,
                'status_age': current_time - self._last_status_time if self._cached_status else None,
                'node_name': self.get_name(),
                'namespace': self.get_namespace(),
                'message_stats': {
                    'successful': self._successful_messages,
                    'failed': self._failed_messages,
                    'total': self._successful_messages + self._failed_messages,
                    'success_rate': (
                        self._successful_messages / max(1, self._successful_messages + self._failed_messages)
                    )
                },
                'callback_stats': {
                    'active_callbacks': len(self._status_callbacks),
                    'callback_errors': dict(self._callback_errors)
                },
                'history_size': len(self._status_history)
            }
    
    def cleanup(self):
        """Clean up resources including thread pool."""
        try:
            self._thread_pool.shutdown(wait=True, timeout=2.0)
        except Exception as e:
            self.get_logger().warning(f'Thread pool shutdown error: {e}')


class AsyncROS2Bridge:
    """
    Async wrapper for ROS2Bridge to enable concurrent operation with FastAPI.
    
    This class manages the ROS2Bridge in a separate thread while providing
    async interfaces for FastAPI integration with proper error handling and
    connection monitoring.
    """
    
    def __init__(self, node_name: str = 'agv_transport_web_bridge', timeout: float = 10.0, retry_attempts: int = 3):
        """Initialize the async ROS2 bridge wrapper.
        
        Args:
            node_name: Name for the ROS2 node
            timeout: Timeout for ROS2 operations
            retry_attempts: Number of retry attempts for failed operations
        """
        self._node_name = node_name
        self._timeout = timeout
        self._retry_attempts = retry_attempts
        self._bridge: Optional[ROS2Bridge] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._ros2_thread: Optional[threading.Thread] = None
        self._shutdown_event = threading.Event()
        self._initialized = False
        self._initialization_lock = asyncio.Lock()
        self._thread_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix='async_bridge')
    
    async def initialize(self):
        """Initialize ROS2 bridge in separate thread with proper error handling."""
        async with self._initialization_lock:
            if self._initialized:
                return
            
            # Initialize ROS2 if not already done
            if not rclpy.ok():
                rclpy.init()
            
            # Create bridge and executor in separate thread
            def ros2_thread():
                try:
                    self._bridge = ROS2Bridge(self._node_name)
                    self._executor = SingleThreadedExecutor()
                    self._executor.add_node(self._bridge)
                    
                    # Spin until shutdown with proper error handling
                    while not self._shutdown_event.is_set() and rclpy.ok():
                        try:
                            self._executor.spin_once(timeout_sec=0.1)
                        except Exception as e:
                            if self._bridge:
                                self._bridge.get_logger().error(f'Executor spin error: {e}')
                            # Continue spinning unless shutdown requested
                            if not self._shutdown_event.is_set():
                                time.sleep(0.1)
                        
                except Exception as e:
                    if self._bridge:
                        self._bridge.get_logger().error(f'ROS2 thread error: {e}')
                    raise
                finally:
                    # Clean up resources
                    if self._executor and self._bridge:
                        try:
                            self._executor.remove_node(self._bridge)
                        except Exception as e:
                            print(f'Error removing node: {e}')
                    if self._bridge:
                        try:
                            self._bridge.cleanup()
                            self._bridge.destroy_node()
                        except Exception as e:
                            print(f'Error destroying node: {e}')
            
            # Start ROS2 thread
            self._ros2_thread = threading.Thread(target=ros2_thread, daemon=True)
            self._ros2_thread.start()
            
            # Wait for bridge to be ready with timeout
            max_wait = 5.0  # 5 second timeout
            wait_time = 0.0
            while self._bridge is None and wait_time < max_wait:
                await asyncio.sleep(0.1)
                wait_time += 0.1
            
            if self._bridge is None:
                raise RuntimeError('Failed to initialize ROS2 bridge within timeout')
            
            self._initialized = True
    
    async def shutdown(self):
        """Shutdown the ROS2 bridge with proper cleanup."""
        if not self._initialized:
            return
        
        # Signal shutdown
        self._shutdown_event.set()
        
        # Wait for ROS2 thread to finish
        if self._ros2_thread and self._ros2_thread.is_alive():
            # Run thread join in executor to avoid blocking event loop
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self._ros2_thread.join, 2.0)
        
        # Shutdown thread pool
        self._thread_pool.shutdown(wait=False)
        
        self._initialized = False
    
    async def get_status(self) -> Optional[Dict[str, Any]]:
        """Get cached transport status asynchronously with error handling.
        
        Returns:
            Status dictionary or None if not available
        """
        if not self._initialized or not self._bridge:
            return None
        
        try:
            # Run in thread pool to avoid blocking
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(self._thread_pool, self._bridge.get_cached_status)
        except Exception as e:
            if self._bridge:
                self._bridge.get_logger().error(f'Error getting status: {e}')
            return None
    
    async def get_status_history(self) -> List[Dict[str, Any]]:
        """Get status history asynchronously.
        
        Returns:
            List of recent status updates
        """
        if not self._initialized or not self._bridge:
            return []
        
        try:
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(self._thread_pool, self._bridge.get_status_history)
        except Exception as e:
            if self._bridge:
                self._bridge.get_logger().error(f'Error getting status history: {e}')
            return []
    
    async def get_orders(self) -> List[Dict[str, Any]]:
        """Get all tracked orders asynchronously.
        
        Returns:
            List of order dictionaries
        """
        if not self._initialized or not self._bridge:
            return []
        
        try:
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(self._thread_pool, self._bridge.get_orders)
        except Exception as e:
            if self._bridge:
                self._bridge.get_logger().error(f'Error getting orders: {e}')
            return []
    
    async def publish_order(self, order: TransportOrder) -> bool:
        """Publish transport order asynchronously with error handling.
        
        Args:
            order: TransportOrder to publish
            
        Returns:
            True if published successfully, False otherwise
        """
        if not self._initialized or not self._bridge:
            return False
        
        try:
            # Run in thread pool to avoid blocking
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(
                self._thread_pool, 
                self._bridge.publish_transport_order, 
                order
            )
        except Exception as e:
            if self._bridge:
                self._bridge.get_logger().error(f'Error publishing order: {e}')
            return False
    
    async def get_connection_info(self) -> Dict[str, Any]:
        """Get connection information asynchronously with error handling.
        
        Returns:
            Connection info dictionary
        """
        if not self._initialized or not self._bridge:
            return {
                'healthy': False,
                'error': 'Bridge not initialized',
                'initialized': self._initialized
            }
        
        try:
            # Run in thread pool to avoid blocking
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(self._thread_pool, self._bridge.get_connection_info)
        except Exception as e:
            return {
                'healthy': False,
                'error': f'Error getting connection info: {e}',
                'initialized': self._initialized
            }
    
    def add_status_callback(self, callback: Callable[[TransportStatus], None]):
        """Add status update callback with error handling.
        
        Args:
            callback: Function to call on status updates
        """
        if self._bridge:
            self._bridge.add_status_callback(callback)
    
    def remove_status_callback(self, callback: Callable[[TransportStatus], None]):
        """Remove status update callback.
        
        Args:
            callback: Function to remove
        """
        if self._bridge:
            self._bridge.remove_status_callback(callback)
    
    @property
    def is_initialized(self) -> bool:
        """Check if bridge is initialized."""
        return self._initialized
    
    @property
    def is_healthy(self) -> bool:
        """Check if bridge connection is healthy."""
        return self._initialized and self._bridge and self._bridge.is_connection_healthy()