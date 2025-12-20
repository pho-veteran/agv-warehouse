#!/usr/bin/env python3
"""
Tests for ROS2 Bridge functionality.

This module contains unit tests for the ROS2Bridge and AsyncROS2Bridge classes,
verifying core functionality, error handling, and concurrent operations.
"""

import asyncio
import json
import pytest
import threading
import time
from unittest.mock import Mock, patch, MagicMock

# Import the modules under test
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '../agv_transport_web'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../../agv_transport'))

from agv_transport_web.ros2_bridge import ROS2Bridge, AsyncROS2Bridge
from agv_transport.models import TransportOrder, TransportStatus


class TestROS2Bridge:
    """Unit tests for ROS2Bridge class."""
    
    @patch('agv_transport_web.ros2_bridge.rclpy')
    def test_bridge_initialization(self, mock_rclpy):
        """Test ROS2Bridge initialization."""
        # Mock ROS2 components
        mock_node = Mock()
        mock_rclpy.create_node.return_value = mock_node
        
        with patch('agv_transport_web.ros2_bridge.Node.__init__'):
            bridge = ROS2Bridge('test_bridge')
            
            # Verify initialization
            assert bridge._cached_status is None
            assert bridge._connection_healthy is False
            assert len(bridge._status_callbacks) == 0
            assert bridge._successful_messages == 0
            assert bridge._failed_messages == 0
    
    def test_transport_order_validation(self):
        """Test transport order validation before publishing."""
        # Create valid order
        valid_order = TransportOrder(
            pickup_station="dock_in_1",
            dropoff_station="dock_out_1"
        )
        
        # Create invalid order (same pickup and dropoff)
        invalid_order = TransportOrder(
            pickup_station="dock_in_1",
            dropoff_station="dock_in_1"
        )
        
        # Test validation
        is_valid, error = valid_order.validate()
        assert is_valid is True
        assert error == ""
        
        is_invalid, error = invalid_order.validate()
        assert is_invalid is False
        assert "must be different" in error
    
    def test_status_caching_thread_safety(self):
        """Test thread-safe status caching."""
        with patch('agv_transport_web.ros2_bridge.Node.__init__'):
            with patch.object(ROS2Bridge, '_setup_ros2_communication'):
                bridge = ROS2Bridge('test_bridge')
                
                # Create test status
                test_status = TransportStatus(
                    current_state="GO_PICKUP",
                    current_order_id="test-order-123",
                    queue_length=2,
                    current_station="dock_in_1"
                )
                
                # Simulate concurrent access
                def update_status():
                    with bridge._status_lock:
                        bridge._cached_status = test_status
                        bridge._last_status_time = time.time()
                
                def read_status():
                    return bridge.get_cached_status()
                
                # Run concurrent operations
                threads = []
                for _ in range(5):
                    t1 = threading.Thread(target=update_status)
                    t2 = threading.Thread(target=read_status)
                    threads.extend([t1, t2])
                
                for t in threads:
                    t.start()
                for t in threads:
                    t.join()
                
                # Verify final state
                cached = bridge.get_cached_status()
                assert cached is not None
                assert cached['status']['current_state'] == "GO_PICKUP"
    
    def test_connection_health_monitoring(self):
        """Test connection health monitoring functionality."""
        with patch('agv_transport_web.ros2_bridge.Node.__init__'):
            with patch.object(ROS2Bridge, '_setup_ros2_communication'):
                bridge = ROS2Bridge('test_bridge')
                
                # Initially unhealthy
                assert bridge.is_connection_healthy() is False
                
                # Simulate receiving a message (should mark as healthy)
                bridge._last_heartbeat = time.time()
                bridge._connection_healthy = True
                assert bridge.is_connection_healthy() is True
                
                # Simulate stale connection
                bridge._last_heartbeat = time.time() - 15.0  # 15 seconds ago
                bridge._health_check_callback()
                assert bridge.is_connection_healthy() is False


class TestAsyncROS2Bridge:
    """Unit tests for AsyncROS2Bridge class."""
    
    @pytest.mark.asyncio
    async def test_async_bridge_initialization(self):
        """Test AsyncROS2Bridge initialization."""
        with patch('agv_transport_web.ros2_bridge.rclpy'):
            with patch('agv_transport_web.ros2_bridge.ROS2Bridge') as mock_bridge_class:
                mock_bridge = Mock()
                mock_bridge_class.return_value = mock_bridge
                
                async_bridge = AsyncROS2Bridge('test_async_bridge')
                
                # Mock the thread creation to avoid actual ROS2 initialization
                with patch('threading.Thread'):
                    async_bridge._bridge = mock_bridge
                    async_bridge._initialized = True
                    
                    assert async_bridge.is_initialized is True
                    assert async_bridge._bridge is mock_bridge
    
    @pytest.mark.asyncio
    async def test_async_status_retrieval(self):
        """Test asynchronous status retrieval."""
        with patch('agv_transport_web.ros2_bridge.rclpy'):
            async_bridge = AsyncROS2Bridge('test_async_bridge')
            
            # Mock bridge and status
            mock_bridge = Mock()
            test_status = {
                'status': {'current_state': 'IDLE'},
                'timestamp': time.time(),
                'connection_healthy': True
            }
            mock_bridge.get_cached_status.return_value = test_status
            
            async_bridge._bridge = mock_bridge
            async_bridge._initialized = True
            
            # Test async status retrieval
            status = await async_bridge.get_status()
            assert status is not None
            assert status['status']['current_state'] == 'IDLE'
    
    @pytest.mark.asyncio
    async def test_async_order_publishing(self):
        """Test asynchronous order publishing."""
        with patch('agv_transport_web.ros2_bridge.rclpy'):
            async_bridge = AsyncROS2Bridge('test_async_bridge')
            
            # Mock bridge
            mock_bridge = Mock()
            mock_bridge.publish_transport_order.return_value = True
            
            async_bridge._bridge = mock_bridge
            async_bridge._initialized = True
            
            # Test async order publishing
            test_order = TransportOrder(
                pickup_station="dock_in_1",
                dropoff_station="dock_out_1"
            )
            
            result = await async_bridge.publish_order(test_order)
            assert result is True
            mock_bridge.publish_transport_order.assert_called_once_with(test_order)
    
    @pytest.mark.asyncio
    async def test_error_handling_when_not_initialized(self):
        """Test error handling when bridge is not initialized."""
        async_bridge = AsyncROS2Bridge('test_async_bridge')
        
        # Test operations on uninitialized bridge
        status = await async_bridge.get_status()
        assert status is None
        
        test_order = TransportOrder(
            pickup_station="dock_in_1",
            dropoff_station="dock_out_1"
        )
        result = await async_bridge.publish_order(test_order)
        assert result is False
        
        conn_info = await async_bridge.get_connection_info()
        assert conn_info['healthy'] is False
        assert 'not initialized' in conn_info['error']


class TestConcurrentOperations:
    """Tests for concurrent operation handling."""
    
    @pytest.mark.asyncio
    async def test_concurrent_status_access(self):
        """Test concurrent status access doesn't cause race conditions."""
        with patch('agv_transport_web.ros2_bridge.rclpy'):
            async_bridge = AsyncROS2Bridge('test_async_bridge')
            
            # Mock bridge with thread-safe status
            mock_bridge = Mock()
            test_status = {
                'status': {'current_state': 'IDLE'},
                'timestamp': time.time(),
                'connection_healthy': True
            }
            mock_bridge.get_cached_status.return_value = test_status
            
            async_bridge._bridge = mock_bridge
            async_bridge._initialized = True
            
            # Run multiple concurrent status requests
            tasks = []
            for _ in range(10):
                task = asyncio.create_task(async_bridge.get_status())
                tasks.append(task)
            
            # Wait for all tasks to complete
            results = await asyncio.gather(*tasks)
            
            # Verify all results are consistent
            for result in results:
                assert result is not None
                assert result['status']['current_state'] == 'IDLE'
    
    @pytest.mark.asyncio
    async def test_concurrent_order_publishing(self):
        """Test concurrent order publishing."""
        with patch('agv_transport_web.ros2_bridge.rclpy'):
            async_bridge = AsyncROS2Bridge('test_async_bridge')
            
            # Mock bridge
            mock_bridge = Mock()
            mock_bridge.publish_transport_order.return_value = True
            
            async_bridge._bridge = mock_bridge
            async_bridge._initialized = True
            
            # Create multiple orders
            orders = []
            for i in range(5):
                order = TransportOrder(
                    pickup_station=f"dock_in_{i}",
                    dropoff_station=f"dock_out_{i}"
                )
                orders.append(order)
            
            # Publish orders concurrently
            tasks = []
            for order in orders:
                task = asyncio.create_task(async_bridge.publish_order(order))
                tasks.append(task)
            
            # Wait for all publications to complete
            results = await asyncio.gather(*tasks)
            
            # Verify all orders were published successfully
            assert all(results)
            assert mock_bridge.publish_transport_order.call_count == 5


if __name__ == '__main__':
    pytest.main([__file__])