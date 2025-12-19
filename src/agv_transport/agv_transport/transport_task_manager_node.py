"""Transport Task Manager Node for AGV Transport.

This module implements the main ROS2 node for managing transport tasks,
integrating FSM, station management, and Nav2 navigation.

**Feature: agv-transport-task-manager**
**Validates: Requirements 1.1, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 4.1, 5.1**
"""

import os
from collections import deque
from typing import Optional, Callable
from enum import Enum
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatus

from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from agv_transport.models import TransportOrder, TransportStatus
from agv_transport.transport_fsm import TransportFSM, TransportState
from agv_transport.station_manager import StationManager
# from agv_transport.cargo_simulator import CargoSimulator  # DISABLED for debugging


class NavigationResult(Enum):
    """Result of a navigation attempt."""
    SUCCESS = "SUCCESS"
    REJECTED = "REJECTED"
    ABORTED = "ABORTED"
    CANCELED = "CANCELED"
    TIMEOUT = "TIMEOUT"
    SERVER_UNAVAILABLE = "SERVER_UNAVAILABLE"


class NavigationHandler:
    """Handles navigation with retry logic.
    
    Manages navigation attempts with configurable retry count,
    tracking failures and determining when to give up.
    
    **Validates: Requirements 2.4, 2.5, 2.6**
    """
    
    def __init__(self, max_retries: int = 3):
        """Initialize NavigationHandler.
        
        Args:
            max_retries: Maximum number of retry attempts (default 3)
        """
        self.max_retries = max_retries
        self._retry_count = 0
        self._consecutive_failures = 0
        self._last_result: Optional[NavigationResult] = None
        self._order_failed = False
    
    @property
    def retry_count(self) -> int:
        """Get current retry count."""
        return self._retry_count
    
    @property
    def consecutive_failures(self) -> int:
        """Get consecutive failure count."""
        return self._consecutive_failures
    
    @property
    def order_failed(self) -> bool:
        """Check if current order has failed (exceeded retries)."""
        return self._order_failed
    
    def reset(self) -> None:
        """Reset handler for a new navigation target."""
        self._retry_count = 0
        self._order_failed = False
        self._last_result = None
    
    def reset_consecutive_failures(self) -> None:
        """Reset consecutive failure counter (called on successful order)."""
        self._consecutive_failures = 0
    
    def handle_result(self, result: NavigationResult) -> bool:
        """Handle a navigation result.
        
        Determines if navigation should be retried or if the order has failed.
        
        Args:
            result: The navigation result
            
        Returns:
            True if navigation succeeded, False if failed or needs retry
            
        **Validates: Requirements 2.4, 2.5, 2.6**
        """
        self._last_result = result
        
        if result == NavigationResult.SUCCESS:
            # Success - reset retry count, don't increment consecutive failures
            self._retry_count = 0
            return True
        
        # Navigation failed - increment retry count
        self._retry_count += 1
        
        if self._retry_count >= self.max_retries:
            # Exceeded retry limit - mark order as failed
            self._order_failed = True
            self._consecutive_failures += 1
            return False
        
        # Can still retry
        return False
    
    def should_retry(self) -> bool:
        """Check if navigation should be retried.
        
        Returns:
            True if retry is possible and should be attempted
        """
        if self._order_failed:
            return False
        if self._last_result == NavigationResult.SUCCESS:
            return False
        return self._retry_count < self.max_retries
    
    def get_failure_reason(self) -> str:
        """Get human-readable failure reason.
        
        Returns:
            Description of why navigation failed
        """
        if not self._order_failed:
            return ""
        
        result_str = self._last_result.value if self._last_result else "UNKNOWN"
        return f"Navigation failed after {self.max_retries} retries. Last result: {result_str}"


class TransportTaskManagerNode(Node):
    """Main ROS2 node for transport task management.
    
    Manages transport orders using FSM, integrates with Nav2 for navigation,
    and publishes status updates.
    
    Attributes:
        fsm: TransportFSM instance for state management
        station_manager: StationManager for station lookups
        order_queue: FIFO queue of pending transport orders
        nav_client: ActionClient for NavigateToPose
        consecutive_errors: Counter for consecutive FSM execution errors
        max_consecutive_errors: Threshold for entering ERROR state
    """
    
    def __init__(self):
        """Initialize TransportTaskManagerNode."""
        super().__init__('transport_task_manager')
        
        # Declare parameters
        self.declare_parameter('config_path', '')
        self.declare_parameter('status_publish_rate', 1.0)  # Hz
        self.declare_parameter('loading_timeout', 5.0)  # seconds
        self.declare_parameter('unloading_timeout', 5.0)  # seconds
        self.declare_parameter('nav_timeout', 300.0)  # seconds
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('max_consecutive_errors', 3)  # For ERROR state threshold
        
        # Get parameters
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if not config_path:
            # Default config path
            config_path = os.path.join(
                os.path.dirname(__file__),
                '..', 'config', 'warehouse_stations.yaml'
            )
        
        self.status_publish_rate = self.get_parameter('status_publish_rate').get_parameter_value().double_value
        self.loading_timeout = self.get_parameter('loading_timeout').get_parameter_value().double_value
        self.unloading_timeout = self.get_parameter('unloading_timeout').get_parameter_value().double_value
        self.nav_timeout = self.get_parameter('nav_timeout').get_parameter_value().double_value
        max_retries = self.get_parameter('max_retries').get_parameter_value().integer_value
        self.max_consecutive_errors = self.get_parameter('max_consecutive_errors').get_parameter_value().integer_value
        
        # Initialize components
        self.fsm = TransportFSM(max_retries=max_retries)
        
        # Error tracking for ERROR state (Requirement 8.3, 8.4)
        self._consecutive_errors = 0
        self.station_manager = StationManager(config_path)
        self.order_queue: deque[TransportOrder] = deque()
        
        # Initialize cargo simulator for RViz visualization (DISABLED for debugging)
        # self.cargo_simulator = CargoSimulator(node=self, logger=self.get_logger())
        self.cargo_simulator = None
        
        # Navigation state tracking
        self._nav_goal_handle: Optional[ClientGoalHandle] = None
        self._nav_start_time: Optional[float] = None
        self._navigation_progress: float = 0.0
        self._current_target_station: str = ""
        self._nav_result: Optional[NavigationResult] = None
        self._nav_result_callback: Optional[Callable[[NavigationResult], None]] = None
        self._action_server_timeout: float = 10.0  # seconds
        
        # Navigation handler for retry logic (Requirement 2.5, 2.6)
        self._nav_handler = NavigationHandler(max_retries=max_retries)
        
        # Callback group for async operations
        self._callback_group = ReentrantCallbackGroup()
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Publishers
        # /agv/transport_status - publishes current transport status
        self.status_publisher = self.create_publisher(
            String,
            '/agv/transport_status',
            qos_profile
        )
        
        # Subscribers
        # /agv/transport_orders - receives incoming transport orders
        self.order_subscriber = self.create_subscription(
            String,
            '/agv/transport_orders',
            self._order_callback,
            qos_profile
        )
        
        # Action Clients
        # NavigateToPose action client for Nav2 integration
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._callback_group
        )
        
        # Services
        # /agv/reset_transport - reset FSM to IDLE
        self.reset_service = self.create_service(
            Trigger,
            '/agv/reset_transport',
            self._reset_callback
        )
        
        # /agv/reload_stations - reload stations from YAML
        self.reload_stations_service = self.create_service(
            Trigger,
            '/agv/reload_stations',
            self._reload_stations_callback
        )
        
        # Timers
        # Status publishing timer (1 Hz by default)
        status_period = 1.0 / self.status_publish_rate
        self.status_timer = self.create_timer(
            status_period,
            self._publish_status
        )
        
        # FSM execution timer (10 Hz for responsive state handling)
        self._fsm_timer_period = 0.1  # 100ms
        self.fsm_timer = self.create_timer(
            self._fsm_timer_period,
            self._fsm_execution_callback
        )
        
        # State tracking for wait states
        self._wait_start_time: Optional[float] = None
        self._last_published_state: Optional[TransportState] = None
        
        self.get_logger().info(
            f'TransportTaskManagerNode initialized. '
            f'Loaded {len(self.station_manager.stations)} stations.'
        )

    def _order_callback(self, msg: String) -> None:
        """Handle incoming transport order messages.
        
        Validates the order and adds it to the queue if valid.
        
        Args:
            msg: String message containing JSON-encoded TransportOrder
        """
        try:
            import json
            data = json.loads(msg.data)
            order = TransportOrder.from_dict(data)
            
            # Validate order
            is_valid, error = self._validate_order(order)
            if not is_valid:
                self.get_logger().error(f'Invalid order rejected: {error}')
                self._publish_error_status(error)
                return
            
            # Add to queue
            self.order_queue.append(order)
            self.get_logger().info(
                f'Order {order.order_id} queued. '
                f'Pickup: {order.pickup_station}, Dropoff: {order.dropoff_station}. '
                f'Queue length: {len(self.order_queue)}'
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse order JSON: {e}')
            self._publish_error_status(f'Invalid JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing order: {e}')
            self._publish_error_status(f'Error: {e}')
    
    def _validate_order(self, order: TransportOrder) -> tuple[bool, str]:
        """Validate a transport order.
        
        Checks that:
        1. Order passes basic validation (non-empty stations, different stations)
        2. Both pickup and dropoff stations exist in StationManager
        
        Args:
            order: TransportOrder to validate
            
        Returns:
            Tuple of (is_valid, error_message)
        """
        # Basic validation
        is_valid, error = order.validate()
        if not is_valid:
            return False, error
        
        # Validate pickup station exists
        if not self.station_manager.station_exists(order.pickup_station):
            return False, f"pickup_station '{order.pickup_station}' does not exist"
        
        # Validate dropoff station exists
        if not self.station_manager.station_exists(order.dropoff_station):
            return False, f"dropoff_station '{order.dropoff_station}' does not exist"
        
        return True, ""
    
    def _publish_status(self) -> None:
        """Publish current transport status.
        
        Called by timer at configured rate (default 1 Hz).
        """
        status = self._build_status()
        
        import json
        msg = String()
        msg.data = json.dumps(status.to_dict())
        self.status_publisher.publish(msg)
    
    def _build_status(self) -> TransportStatus:
        """Build current TransportStatus.
        
        Returns:
            TransportStatus with current state information
            
        **Validates: Requirements 5.2, 5.3, 5.4**
        """
        current_order = self.fsm.current_order
        current_state = self.fsm.get_current_state()
        
        # Calculate navigation progress for navigation states (Requirement 5.4)
        nav_progress = 0.0
        if current_state in (TransportState.GO_PICKUP, TransportState.GO_DROPOFF):
            nav_progress = self._navigation_progress
        elif current_state == TransportState.WAIT_LOADING:
            # Show loading progress based on timeout
            if self._wait_start_time is not None:
                elapsed = time.time() - self._wait_start_time
                nav_progress = min(1.0, elapsed / self.loading_timeout)
        elif current_state == TransportState.WAIT_UNLOADING:
            # Show unloading progress based on timeout
            if self._wait_start_time is not None:
                elapsed = time.time() - self._wait_start_time
                nav_progress = min(1.0, elapsed / self.unloading_timeout)
        elif current_state == TransportState.DONE:
            nav_progress = 1.0
        
        return TransportStatus(
            current_state=current_state.value,
            current_order_id=current_order.order_id if current_order else "",
            queue_length=len(self.order_queue),
            current_station=self._current_target_station,
            navigation_progress=nav_progress,
            error_message="",
            timestamp=time.time()
        )
    
    def _publish_error_status(self, error_message: str) -> None:
        """Publish error status message.
        
        Args:
            error_message: Error description
        """
        status = self._build_status()
        status.error_message = error_message
        
        import json
        msg = String()
        msg.data = json.dumps(status.to_dict())
        self.status_publisher.publish(msg)
    
    def _reset_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Handle reset service request.
        
        Resets FSM to IDLE and clears the order queue.
        This is the manual reset required to exit ERROR state (Requirement 8.4).
        
        Args:
            request: Trigger request
            response: Trigger response
            
        Returns:
            Trigger response with success status
            
        **Validates: Requirements 8.4**
        """
        try:
            # Cancel any ongoing navigation
            if self._nav_goal_handle is not None:
                self._nav_goal_handle.cancel_goal_async()
                self._nav_goal_handle = None
            
            # Reset FSM
            self.fsm.reset()
            
            # Clear queue
            self.order_queue.clear()
            
            # Reset navigation state
            self._nav_start_time = None
            self._navigation_progress = 0.0
            self._current_target_station = ""
            self._nav_result = None
            
            # Reset wait state tracking
            self._wait_start_time = None
            self._last_published_state = None
            
            # Reset navigation handler
            self._nav_handler.reset()
            self._nav_handler.reset_consecutive_failures()
            
            # Clean up any existing cargo
            try:
                # self.cargo_simulator.reset()  # DISABLED for debugging
                pass
            except Exception as e:
                self.get_logger().warning(f"Failed to reset cargo simulator: {e}")
            
            # Reset consecutive error counter (Requirement 8.4)
            self._consecutive_errors = 0
            
            response.success = True
            response.message = "Transport manager reset to IDLE"
            self.get_logger().info("Transport manager reset to IDLE")
            
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {e}"
            self.get_logger().error(f"Reset failed: {e}")
        
        return response
    
    def _reload_stations_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Handle reload stations service request.
        
        Reloads station configurations from YAML file.
        
        Args:
            request: Trigger request
            response: Trigger response
            
        Returns:
            Trigger response with success status
        """
        try:
            success = self.station_manager.load_stations()
            if success:
                response.success = True
                response.message = f"Reloaded {len(self.station_manager.stations)} stations"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = "Failed to reload stations from YAML"
                self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Reload failed: {e}"
            self.get_logger().error(f"Reload failed: {e}")
        
        return response
    
    # =========================================================================
    # FSM Execution Loop
    # **Validates: Requirements 1.2, 1.3, 1.4, 1.5, 1.6, 1.7**
    # =========================================================================
    
    def _fsm_execution_callback(self) -> None:
        """Main FSM execution loop called by timer.
        
        Processes state transitions based on current state:
        - IDLE: Check queue, start new order
        - GO_PICKUP/GO_DROPOFF: Monitor navigation
        - WAIT_LOADING/WAIT_UNLOADING: Wait for timeout
        - DONE: Complete order, return to IDLE
        - ERROR: Wait for manual reset
        
        Error handling (Requirements 8.3, 8.4):
        - Catches exceptions in FSM execution
        - Transitions to IDLE on recoverable errors
        - Transitions to ERROR on 3+ consecutive failures
        
        **Validates: Requirements 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 8.3, 8.4**
        """
        try:
            current_state = self.fsm.get_current_state()
            
            if current_state == TransportState.IDLE:
                self._handle_idle_state()
            elif current_state == TransportState.GO_PICKUP:
                self._handle_go_pickup_state()
            elif current_state == TransportState.WAIT_LOADING:
                self._handle_wait_loading_state()
            elif current_state == TransportState.GO_DROPOFF:
                self._handle_go_dropoff_state()
            elif current_state == TransportState.WAIT_UNLOADING:
                self._handle_wait_unloading_state()
            elif current_state == TransportState.DONE:
                self._handle_done_state()
            elif current_state == TransportState.ERROR:
                self._handle_error_state()
            
            # Reset consecutive errors on successful execution
            # (only if we're not already in ERROR state)
            if current_state != TransportState.ERROR:
                self._consecutive_errors = 0
                
        except Exception as e:
            self._handle_fsm_exception(e)
    
    def _handle_fsm_exception(self, exception: Exception) -> None:
        """Handle exceptions during FSM execution.
        
        Implements error recovery logic:
        - Logs the error and publishes error status
        - Increments consecutive error counter
        - Transitions to IDLE on recoverable errors
        - Transitions to ERROR on 3+ consecutive failures
        
        Args:
            exception: The exception that occurred
            
        **Validates: Requirements 8.3, 8.4**
        """
        self._consecutive_errors += 1
        error_msg = f"FSM execution error: {exception}"
        
        self.get_logger().error(
            f"{error_msg} (consecutive errors: {self._consecutive_errors})"
        )
        self._publish_error_status(error_msg)
        
        # Cancel any ongoing navigation
        if self._nav_goal_handle is not None:
            try:
                self._nav_goal_handle.cancel_goal_async()
            except Exception:
                pass  # Ignore cancel errors
            self._nav_goal_handle = None
        
        # Clean up cargo on error
        try:
            # self.cargo_simulator.delete_cargo()  # DISABLED for debugging
            pass
        except Exception:
            pass  # Ignore cleanup errors
        
        # Clear current order
        self.fsm.current_order = None
        
        # Reset navigation state
        self._nav_start_time = None
        self._navigation_progress = 0.0
        self._wait_start_time = None
        
        # Check for consecutive failures threshold (Requirement 8.4)
        if self._consecutive_errors >= self.max_consecutive_errors:
            self.get_logger().error(
                f"{self.max_consecutive_errors}+ consecutive errors - "
                "entering ERROR state. Manual reset required."
            )
            self.fsm.transition_to(TransportState.ERROR)
        else:
            # Transition to IDLE on recoverable errors (Requirement 8.3)
            self.get_logger().warning(
                "Recoverable error - transitioning to IDLE state"
            )
            self.fsm.transition_to(TransportState.IDLE)
        
        self._publish_state_change()
    
    def get_consecutive_errors(self) -> int:
        """Get the current consecutive error count.
        
        Returns:
            Number of consecutive FSM execution errors
        """
        return self._consecutive_errors
    
    def reset_transport(self) -> bool:
        """Reset the transport manager programmatically.
        
        This is equivalent to calling the /agv/reset_transport service.
        Useful for testing and programmatic control.
        
        Returns:
            True if reset was successful, False otherwise
            
        **Validates: Requirements 8.4**
        """
        request = Trigger.Request()
        response = Trigger.Response()
        result = self._reset_callback(request, response)
        return result.success
    
    def _handle_idle_state(self) -> None:
        """Handle IDLE state: check queue and start new order.
        
        **Validates: Requirements 1.1, 1.2**
        """
        # Check if there are orders in the queue
        if not self.order_queue:
            return
        
        # Pop next order from queue (FIFO)
        order = self.order_queue.popleft()
        
        self.get_logger().info(
            f"Starting order {order.order_id}: "
            f"{order.pickup_station} -> {order.dropoff_station}"
        )
        
        # Start the order (transitions to GO_PICKUP)
        if self.fsm.start_order(order):
            self._current_target_station = order.pickup_station
            self._nav_handler.reset()
            
            # Spawn cargo at pickup station
            self.get_logger().info(f"Spawning cargo at pickup station '{order.pickup_station}'")
            try:
                # spawn_success = self.cargo_simulator.spawn_cargo_at_named_station(  # DISABLED for debugging
                #     order.pickup_station, self.station_manager
                # )
                spawn_success = True  # Simulate success
                if not spawn_success:
                    self.get_logger().warning(
                        f"Failed to spawn cargo at '{order.pickup_station}' - continuing anyway"
                    )
            except Exception as e:
                self.get_logger().warning(f"Exception during cargo spawn: {e} - continuing anyway")
            
            # Send navigation goal to pickup station
            if not self.send_nav_goal(order.pickup_station):
                self.get_logger().error(
                    f"Failed to send navigation goal to {order.pickup_station}"
                )
                # Return order to front of queue and go back to IDLE
                self.order_queue.appendleft(order)
                self.fsm.reset()
                # Clean up cargo
                try:
                    # self.cargo_simulator.delete_cargo()  # DISABLED for debugging
                    pass
                except Exception as e:
                    self.get_logger().warning(f"Failed to clean up cargo: {e}")
            else:
                self._publish_state_change()
        else:
            self.get_logger().error("Failed to start order - FSM not in IDLE state")
    
    def _handle_go_pickup_state(self) -> None:
        """Handle GO_PICKUP state: monitor navigation to pickup station.
        
        **Validates: Requirements 1.2, 1.3, 2.4, 2.5, 2.6**
        """
        # Check for navigation timeout (Requirement 8.2)
        if self.check_nav_timeout():
            self.get_logger().warning("Navigation timeout - canceling goal")
            self.cancel_nav_goal()
            self._nav_result = NavigationResult.TIMEOUT
            self._handle_navigation_failure(NavigationResult.TIMEOUT)
            return
        
        # Check if navigation is still in progress
        if self.is_navigating():
            return
        
        # Navigation completed - check result
        result = self.get_nav_result()
        if result is None:
            return
        
        if result == NavigationResult.SUCCESS:
            # Navigation succeeded - transition to WAIT_LOADING
            self.get_logger().info("Arrived at pickup station")
            self.fsm.transition_to(TransportState.WAIT_LOADING)
            self._wait_start_time = time.time()
            self._navigation_progress = 0.0
            
            # Attach cargo to robot
            self.get_logger().info("Attaching cargo to robot")
            try:
                # attach_success = self.cargo_simulator.attach_cargo_to_robot("base_link")  # DISABLED for debugging
                attach_success = True  # Simulate success
                if not attach_success:
                    self.get_logger().warning("Failed to attach cargo - continuing anyway")
            except Exception as e:
                self.get_logger().warning(f"Exception during cargo attach: {e} - continuing anyway")
            
            self._publish_state_change()
        else:
            # Navigation failed - handle with retry logic
            self._handle_navigation_failure(result)
    
    def _handle_navigation_failure(self, result: NavigationResult) -> None:
        """Handle navigation failure with retry logic.
        
        Args:
            result: The navigation failure result
            
        **Validates: Requirements 2.5, 2.6**
        """
        success = self._nav_handler.handle_result(result)
        
        if success:
            # This shouldn't happen for failures, but handle it
            return
        
        if self._nav_handler.order_failed:
            # Exceeded retry limit - mark order as FAILED
            failure_reason = self._nav_handler.get_failure_reason()
            self.get_logger().error(f"Order failed: {failure_reason}")
            self._publish_error_status(failure_reason)
            
            # Check for consecutive failures (Requirement 8.4)
            if self._nav_handler.consecutive_failures >= 3:
                self.get_logger().error(
                    "3+ consecutive order failures - entering ERROR state"
                )
                self.fsm.transition_to(TransportState.ERROR)
            else:
                # Transition to IDLE (Requirement 2.6)
                self.fsm.transition_to(TransportState.IDLE)
            
            self.fsm.current_order = None
            self._nav_handler.reset()
            self._publish_state_change()
        elif self._nav_handler.should_retry():
            # Retry navigation
            retry_num = self._nav_handler.retry_count
            self.get_logger().warning(
                f"Navigation failed, retrying ({retry_num}/{self._nav_handler.max_retries})"
            )
            self.send_nav_goal(self._current_target_station)
    
    def _handle_wait_loading_state(self) -> None:
        """Handle WAIT_LOADING state: wait for loading timeout.
        
        **Validates: Requirements 1.3, 1.4**
        """
        if self._wait_start_time is None:
            self._wait_start_time = time.time()
            return
        
        elapsed = time.time() - self._wait_start_time
        
        if elapsed >= self.loading_timeout:
            # Loading timeout completed - transition to GO_DROPOFF
            self.get_logger().info("Loading complete, heading to dropoff station")
            self.fsm.transition_to(TransportState.GO_DROPOFF)
            self._wait_start_time = None
            
            # Get dropoff station from current order
            order = self.fsm.current_order
            if order:
                self._current_target_station = order.dropoff_station
                self._nav_handler.reset()
                
                if not self.send_nav_goal(order.dropoff_station):
                    self.get_logger().error(
                        f"Failed to send navigation goal to {order.dropoff_station}"
                    )
                    self._publish_error_status(
                        f"Failed to navigate to {order.dropoff_station}"
                    )
                    self.fsm.transition_to(TransportState.IDLE)
                    self.fsm.current_order = None
            
            self._publish_state_change()
    
    def _handle_go_dropoff_state(self) -> None:
        """Handle GO_DROPOFF state: monitor navigation to dropoff station.
        
        **Validates: Requirements 1.4, 1.5, 2.4, 2.5, 2.6**
        """
        # Check for navigation timeout (Requirement 8.2)
        if self.check_nav_timeout():
            self.get_logger().warning("Navigation timeout - canceling goal")
            self.cancel_nav_goal()
            self._nav_result = NavigationResult.TIMEOUT
            self._handle_navigation_failure(NavigationResult.TIMEOUT)
            return
        
        # Check if navigation is still in progress
        if self.is_navigating():
            return
        
        # Navigation completed - check result
        result = self.get_nav_result()
        if result is None:
            return
        
        if result == NavigationResult.SUCCESS:
            # Navigation succeeded - transition to WAIT_UNLOADING
            self.get_logger().info("Arrived at dropoff station")
            self.fsm.transition_to(TransportState.WAIT_UNLOADING)
            self._wait_start_time = time.time()
            self._navigation_progress = 0.0
            
            # Detach cargo at dropoff station
            self.get_logger().info("Detaching cargo at dropoff station")
            try:
                # Get dropoff station coordinates for cargo positioning
                order = self.fsm.current_order
                if order:
                    station_config = self.station_manager.stations.get(order.dropoff_station)
                    if station_config:
                        # detach_success = self.cargo_simulator.detach_cargo(  # DISABLED for debugging
                        #     station_config.x, station_config.y
                        # )
                        detach_success = True  # Simulate success
                        if not detach_success:
                            self.get_logger().warning("Failed to detach cargo - continuing anyway")
                    else:
                        self.get_logger().warning(f"Could not get dropoff station config - detaching at current position")
                        # self.cargo_simulator.detach_cargo()  # DISABLED for debugging
                else:
                    self.get_logger().warning("No current order - detaching at current position")
                    # self.cargo_simulator.detach_cargo()  # DISABLED for debugging
            except Exception as e:
                self.get_logger().warning(f"Exception during cargo detach: {e} - continuing anyway")
            
            self._publish_state_change()
        else:
            # Navigation failed - handle with retry logic
            self._handle_navigation_failure(result)
    
    def _handle_wait_unloading_state(self) -> None:
        """Handle WAIT_UNLOADING state: wait for unloading timeout.
        
        **Validates: Requirements 1.5, 1.6**
        """
        if self._wait_start_time is None:
            self._wait_start_time = time.time()
            return
        
        elapsed = time.time() - self._wait_start_time
        
        if elapsed >= self.unloading_timeout:
            # Unloading timeout completed - transition to DONE
            self.get_logger().info("Unloading complete")
            self.fsm.transition_to(TransportState.DONE)
            self._wait_start_time = None
            self._publish_state_change()
    
    def _handle_done_state(self) -> None:
        """Handle DONE state: complete order and return to IDLE.
        
        **Validates: Requirements 1.6, 1.7**
        """
        order = self.fsm.current_order
        if order:
            self.get_logger().info(
                f"Order {order.order_id} completed successfully"
            )
        
        # Delete cargo marker (cargo has been delivered)
        self.get_logger().info("Deleting cargo marker (delivery complete)")
        try:
            # delete_success = self.cargo_simulator.delete_cargo()  # DISABLED for debugging
            delete_success = True  # Simulate success
            if not delete_success:
                self.get_logger().warning("Failed to delete cargo marker - continuing anyway")
        except Exception as e:
            self.get_logger().warning(f"Exception during cargo delete: {e} - continuing anyway")
        
        # Reset consecutive failures on successful order
        self._nav_handler.reset_consecutive_failures()
        
        # Reset consecutive errors on successful order completion
        self._consecutive_errors = 0
        
        # Complete the order (transitions to IDLE)
        self.fsm.complete_order()
        self._current_target_station = ""
        self._navigation_progress = 0.0
        self._publish_state_change()
    
    def _handle_error_state(self) -> None:
        """Handle ERROR state: wait for manual reset.
        
        **Validates: Requirements 8.4**
        """
        # ERROR state requires manual reset via service call
        # Just log periodically to indicate we're in error state
        pass
    
    def _publish_state_change(self) -> None:
        """Publish status immediately on state change.
        
        **Validates: Requirements 5.2**
        """
        current_state = self.fsm.get_current_state()
        
        # Only publish if state actually changed
        if current_state != self._last_published_state:
            self._last_published_state = current_state
            self._publish_status()
            self.get_logger().debug(f"State changed to: {current_state.value}")
    
    # =========================================================================
    # Nav2 Integration Methods
    # **Validates: Requirements 2.2, 2.3, 2.4, 2.5, 2.6**
    # =========================================================================
    
    def send_nav_goal(
        self,
        station_name: str,
        result_callback: Optional[Callable[[NavigationResult], None]] = None
    ) -> bool:
        """Send a navigation goal to Nav2 for the specified station.
        
        Waits for the action server with a 10 second timeout, then sends
        the NavigateToPose goal asynchronously.
        
        Args:
            station_name: Name of the station to navigate to
            result_callback: Optional callback to be called when navigation completes
            
        Returns:
            True if goal was sent successfully, False otherwise
            
        **Validates: Requirements 2.2, 2.3**
        """
        # Get station pose
        pose = self.station_manager.get_station_pose(station_name)
        if pose is None:
            self.get_logger().error(f"Station '{station_name}' not found")
            return False
        
        # Wait for action server with timeout (Requirement 2.2)
        if not self.nav_client.wait_for_server(timeout_sec=self._action_server_timeout):
            self.get_logger().error(
                f"Nav2 action server not available after {self._action_server_timeout}s timeout"
            )
            if result_callback:
                result_callback(NavigationResult.SERVER_UNAVAILABLE)
            return False
        
        # Create NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Store callback and reset state
        self._nav_result_callback = result_callback
        self._nav_result = None
        self._navigation_progress = 0.0
        self._current_target_station = station_name
        self._nav_start_time = time.time()
        
        # Send goal async with feedback callback (Requirement 2.3)
        self.get_logger().info(f"Sending navigation goal to station '{station_name}'")
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )
        send_goal_future.add_done_callback(self._nav_goal_response_callback)
        
        return True
    
    def _nav_feedback_callback(self, feedback_msg) -> None:
        """Handle navigation feedback from Nav2.
        
        Updates navigation progress based on feedback.
        
        Args:
            feedback_msg: NavigateToPose feedback message
            
        **Validates: Requirements 2.3**
        """
        feedback = feedback_msg.feedback
        
        # Calculate progress based on distance remaining
        # Nav2 feedback includes current_pose and navigation_time
        # We estimate progress based on time elapsed vs expected time
        if self._nav_start_time is not None:
            elapsed = time.time() - self._nav_start_time
            # Estimate progress (cap at 0.99 until actually complete)
            # Use a simple time-based estimate, capped at 99%
            estimated_total_time = 60.0  # Assume 60s average navigation
            self._navigation_progress = min(0.99, elapsed / estimated_total_time)
        
        self.get_logger().debug(
            f"Navigation feedback: progress={self._navigation_progress:.2f}"
        )
    
    def _nav_goal_response_callback(self, future) -> None:
        """Handle goal response from Nav2 action server.
        
        Called when the goal is accepted or rejected by the action server.
        
        Args:
            future: Future containing the goal handle
            
        **Validates: Requirements 2.4**
        """
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warning("Navigation goal was rejected by Nav2")
            self._nav_goal_handle = None
            self._nav_result = NavigationResult.REJECTED
            if self._nav_result_callback:
                self._nav_result_callback(NavigationResult.REJECTED)
            return
        
        self.get_logger().info("Navigation goal accepted by Nav2")
        self._nav_goal_handle = goal_handle
        
        # Get result async
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_response_callback)
    
    def _nav_result_response_callback(self, future) -> None:
        """Handle navigation result from Nav2.
        
        Called when navigation completes (success, abort, or cancel).
        
        Args:
            future: Future containing the result
            
        **Validates: Requirements 2.4, 2.5**
        """
        result = future.result()
        status = result.status
        
        self._nav_goal_handle = None
        self._navigation_progress = 1.0 if status == GoalStatus.STATUS_SUCCEEDED else 0.0
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Navigation to '{self._current_target_station}' succeeded"
            )
            self._nav_result = NavigationResult.SUCCESS
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warning(
                f"Navigation to '{self._current_target_station}' was aborted"
            )
            self._nav_result = NavigationResult.ABORTED
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(
                f"Navigation to '{self._current_target_station}' was canceled"
            )
            self._nav_result = NavigationResult.CANCELED
        else:
            self.get_logger().warning(
                f"Navigation ended with unexpected status: {status}"
            )
            self._nav_result = NavigationResult.ABORTED
        
        # Call result callback if provided
        if self._nav_result_callback:
            self._nav_result_callback(self._nav_result)
    
    def cancel_nav_goal(self) -> bool:
        """Cancel the current navigation goal.
        
        Returns:
            True if cancel request was sent, False if no active goal
        """
        if self._nav_goal_handle is None:
            return False
        
        self.get_logger().info("Canceling navigation goal")
        cancel_future = self._nav_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._nav_cancel_callback)
        return True
    
    def _nav_cancel_callback(self, future) -> None:
        """Handle cancel response.
        
        Args:
            future: Future containing cancel response
        """
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Navigation goal cancel request accepted")
        else:
            self.get_logger().warning("Navigation goal cancel request rejected")
    
    def is_navigating(self) -> bool:
        """Check if navigation is currently in progress.
        
        Returns:
            True if a navigation goal is active
        """
        return self._nav_goal_handle is not None
    
    def get_navigation_progress(self) -> float:
        """Get current navigation progress.
        
        Returns:
            Progress value between 0.0 and 1.0
        """
        return self._navigation_progress
    
    def get_nav_result(self) -> Optional[NavigationResult]:
        """Get the result of the last navigation attempt.
        
        Returns:
            NavigationResult or None if no navigation has completed
        """
        return self._nav_result
    
    def check_nav_timeout(self) -> bool:
        """Check if current navigation has exceeded timeout.
        
        Returns:
            True if navigation has timed out, False otherwise
            
        **Validates: Requirements 8.2**
        """
        if self._nav_start_time is None:
            return False
        
        elapsed = time.time() - self._nav_start_time
        return elapsed > self.nav_timeout
    
    # =========================================================================
    # Navigation Result Handling with Retry Logic
    # **Validates: Requirements 2.4, 2.5, 2.6**
    # =========================================================================
    
    def handle_navigation_result(self, result: NavigationResult) -> bool:
        """Handle navigation result with retry logic.
        
        Processes the navigation result, determines if retry is needed,
        and updates FSM state accordingly.
        
        Args:
            result: The navigation result
            
        Returns:
            True if navigation succeeded, False if failed or retrying
            
        **Validates: Requirements 2.4, 2.5, 2.6**
        """
        success = self._nav_handler.handle_result(result)
        
        if success:
            self.get_logger().info(
                f"Navigation to '{self._current_target_station}' completed successfully"
            )
            self._nav_handler.reset_consecutive_failures()
            return True
        
        if self._nav_handler.order_failed:
            # Exceeded retry limit - mark order as FAILED
            failure_reason = self._nav_handler.get_failure_reason()
            self.get_logger().error(f"Order failed: {failure_reason}")
            self._publish_error_status(failure_reason)
            
            # Transition FSM to IDLE (Requirement 2.6)
            self.fsm.transition_to(TransportState.IDLE)
            self.fsm.current_order = None
            self._nav_handler.reset()
            return False
        
        if self._nav_handler.should_retry():
            # Retry navigation
            retry_num = self._nav_handler.retry_count
            self.get_logger().warning(
                f"Navigation failed, retrying ({retry_num}/{self._nav_handler.max_retries})"
            )
            return False
        
        return False
    
    def start_navigation_to_station(self, station_name: str) -> bool:
        """Start navigation to a station with retry handling.
        
        Resets the navigation handler and sends the navigation goal.
        
        Args:
            station_name: Name of the station to navigate to
            
        Returns:
            True if navigation goal was sent, False otherwise
        """
        self._nav_handler.reset()
        return self.send_nav_goal(station_name)
    
    def retry_navigation(self) -> bool:
        """Retry navigation to the current target station.
        
        Returns:
            True if retry was initiated, False if no retry possible
        """
        if not self._nav_handler.should_retry():
            return False
        
        if not self._current_target_station:
            return False
        
        return self.send_nav_goal(self._current_target_station)
    
    def get_nav_handler(self) -> NavigationHandler:
        """Get the navigation handler for testing.
        
        Returns:
            The NavigationHandler instance
        """
        return self._nav_handler
    
    def simulate_nav_failure(self, result: NavigationResult = NavigationResult.ABORTED) -> bool:
        """Simulate a navigation failure for testing.
        
        Args:
            result: The failure result to simulate
            
        Returns:
            Result of handle_navigation_result
        """
        self._nav_result = result
        return self.handle_navigation_result(result)
    
    def get_order_queue(self) -> deque[TransportOrder]:
        """Get the current order queue.
        
        Returns:
            The order queue (deque of TransportOrder)
        """
        return self.order_queue
    
    def get_queue_length(self) -> int:
        """Get the number of orders in the queue.
        
        Returns:
            Number of pending orders
        """
        return len(self.order_queue)
    
    def add_order(self, order: TransportOrder) -> tuple[bool, str]:
        """Add an order to the queue programmatically.
        
        Validates the order before adding.
        
        Args:
            order: TransportOrder to add
            
        Returns:
            Tuple of (success, error_message)
        """
        is_valid, error = self._validate_order(order)
        if not is_valid:
            return False, error
        
        self.order_queue.append(order)
        return True, ""
    
    def pop_next_order(self) -> Optional[TransportOrder]:
        """Pop the next order from the queue (FIFO).
        
        Returns:
            Next TransportOrder or None if queue is empty
        """
        if self.order_queue:
            return self.order_queue.popleft()
        return None


def main(args=None):
    """Main entry point for transport_task_manager_node."""
    rclpy.init(args=args)
    
    node = TransportTaskManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
