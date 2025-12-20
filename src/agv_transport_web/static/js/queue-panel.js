/**
 * Queue Panel JavaScript for AGV Transport Web Dashboard
 * 
 * Handles task queue management including:
 * - Fetching order queue data from API
 * - Rendering queue table with order information
 * - Auto-refresh functionality
 * - Error handling and user feedback
 */

(function() {
    'use strict';

    // Queue panel state
    let queueUpdateTimer = null;
    let lastQueueData = null;
    let isUpdating = false;

    // DOM elements
    let queueTableBody = null;
    let queueEmptyState = null;
    let totalOrdersSpan = null;
    let activeOrdersSpan = null;
    let pendingOrdersSpan = null;
    let queueLastUpdateSpan = null;
    let queueRefreshButton = null;

    /**
     * Initialize queue panel functionality
     * @param {object} config - Configuration options
     */
    function init(config = {}) {
        // Get DOM elements
        queueTableBody = document.getElementById('queue-table-body');
        queueEmptyState = document.getElementById('queue-empty-state');
        totalOrdersSpan = document.getElementById('total-orders');
        activeOrdersSpan = document.getElementById('active-orders');
        pendingOrdersSpan = document.getElementById('pending-orders');
        queueLastUpdateSpan = document.getElementById('queue-last-update');
        queueRefreshButton = document.getElementById('queue-refresh');

        if (!queueTableBody || !queueEmptyState) {
            console.error('Queue panel: Required DOM elements not found');
            return;
        }

        // Setup refresh button
        if (queueRefreshButton) {
            queueRefreshButton.addEventListener('click', function() {
                fetchOrders(true); // Force refresh
            });
        }

        // Initial fetch
        fetchOrders();

        // Setup auto-refresh timer
        const updateInterval = config.queueUpdateInterval || 10000; // 10 seconds default
        startAutoRefresh(updateInterval);

        // NOTE: Page visibility is handled by Dashboard.js to avoid duplicate listeners

        console.log('Queue panel initialized');
    }

    /**
     * Fetch orders from the API with error handling
     * @param {boolean} forceRefresh - Force refresh even if currently updating
     */
    async function fetchOrders(forceRefresh = false) {
        if (isUpdating && !forceRefresh) {
            return;
        }

        isUpdating = true;

        try {
            // Show loading state
            if (queueRefreshButton) {
                window.Utils.setLoadingState(queueRefreshButton, true);
            }

            // Fetch data from API
            const data = await window.Utils.fetchWithTimeout('/api/orders', {
                method: 'GET',
                headers: {
                    'Accept': 'application/json'
                },
                timeout: 8000 // 8 second timeout
            });

            // Update queue display
            updateQueueDisplay(data);
            
            // Update connection status
            if (window.ConnectionStatus) {
                window.ConnectionStatus.setConnected(true);
            }

            // Update last update time
            updateLastUpdateTime();

            lastQueueData = data;

        } catch (error) {
            console.error('Error fetching orders:', error);
            
            // Update connection status using enhanced system
            if (window.ConnectionStatus) {
                window.ConnectionStatus.setConnected(false, error.message);
            }

            // Show error notification using enhanced system
            if (window.Notifications) {
                if (error.message.includes('timeout')) {
                    window.Notifications.warning('Queue update timeout - retrying...', 3000);
                } else if (error.message.includes('Failed to fetch')) {
                    window.Notifications.error('Cannot connect to AGV system', 0);
                } else {
                    window.Notifications.showApiError('fetch order queue', error);
                }
            }

            // Show empty state if no cached data
            if (!lastQueueData) {
                showEmptyState('Connection error - unable to load orders');
            }

        } finally {
            isUpdating = false;
            
            // Hide loading state
            if (queueRefreshButton) {
                window.Utils.setLoadingState(queueRefreshButton, false);
            }
        }
    }

    /**
     * Update the queue display with new data
     * @param {object} data - Queue data from API
     */
    function updateQueueDisplay(data) {
        if (!data || !data.orders) {
            showEmptyState('No order data available');
            return;
        }

        const orders = data.orders;

        if (orders.length === 0) {
            showEmptyState();
            updateQueueSummary(data);
            return;
        }

        // Hide empty state and show table
        hideEmptyState();

        // Clear existing table rows
        window.Utils.clearElement(queueTableBody);

        // Generate table rows
        orders.forEach(order => {
            const row = createOrderRow(order);
            queueTableBody.appendChild(row);
        });

        // Update summary
        updateQueueSummary(data);

        // Highlight update
        if (window.Utils.highlightUpdate) {
            window.Utils.highlightUpdate(queueTableBody.parentElement);
        }
    }

    /**
     * Create a table row for an order
     * @param {object} order - Order data
     * @returns {HTMLElement} Table row element
     */
    function createOrderRow(order) {
        const row = document.createElement('tr');
        
        // Order ID cell
        const orderIdCell = document.createElement('td');
        orderIdCell.textContent = order.order_id || 'Unknown';
        orderIdCell.title = order.order_id || 'Unknown';
        row.appendChild(orderIdCell);

        // Pickup station cell
        const pickupCell = document.createElement('td');
        pickupCell.textContent = order.pickup_station || 'Unknown';
        pickupCell.title = order.pickup_station || 'Unknown';
        row.appendChild(pickupCell);

        // Dropoff station cell
        const dropoffCell = document.createElement('td');
        dropoffCell.textContent = order.dropoff_station || 'Unknown';
        dropoffCell.title = order.dropoff_station || 'Unknown';
        row.appendChild(dropoffCell);

        // Status cell with badge
        const statusCell = document.createElement('td');
        const statusBadge = document.createElement('span');
        statusBadge.className = `order-status ${(order.status || 'UNKNOWN').toUpperCase()}`;
        statusBadge.textContent = order.status || 'UNKNOWN';
        statusCell.appendChild(statusBadge);
        row.appendChild(statusCell);

        // Created time cell
        const createdCell = document.createElement('td');
        if (order.created_at) {
            createdCell.textContent = window.Utils.formatTime(order.created_at);
            createdCell.title = new Date(order.created_at).toLocaleString();
        } else {
            createdCell.textContent = 'Unknown';
        }
        row.appendChild(createdCell);

        // Add data attributes for potential filtering/sorting
        row.dataset.orderId = order.order_id || '';
        row.dataset.status = (order.status || '').toLowerCase();
        row.dataset.priority = order.priority || '0';

        return row;
    }

    /**
     * Show empty state message
     * @param {string} message - Optional custom message
     */
    function showEmptyState(message = null) {
        if (queueEmptyState) {
            queueEmptyState.style.display = 'block';
            
            // Update message if provided
            if (message) {
                const messageElement = queueEmptyState.querySelector('p');
                if (messageElement) {
                    messageElement.textContent = message;
                }
            }
        }

        // Hide table
        if (queueTableBody && queueTableBody.parentElement) {
            queueTableBody.parentElement.style.display = 'none';
        }
    }

    /**
     * Hide empty state and show table
     */
    function hideEmptyState() {
        if (queueEmptyState) {
            queueEmptyState.style.display = 'none';
        }

        // Show table
        if (queueTableBody && queueTableBody.parentElement) {
            queueTableBody.parentElement.style.display = 'block';
        }
    }

    /**
     * Update queue summary statistics
     * @param {object} data - Queue data from API
     */
    function updateQueueSummary(data) {
        if (totalOrdersSpan) {
            totalOrdersSpan.textContent = data.total_count || 0;
        }
        
        if (activeOrdersSpan) {
            activeOrdersSpan.textContent = data.active_orders || 0;
        }
        
        if (pendingOrdersSpan) {
            pendingOrdersSpan.textContent = data.pending_orders || 0;
        }
    }

    /**
     * Update last update timestamp
     */
    function updateLastUpdateTime() {
        if (queueLastUpdateSpan) {
            queueLastUpdateSpan.textContent = window.Utils.formatTime(new Date());
        }
    }

    /**
     * Start auto-refresh timer
     * @param {number} interval - Update interval in milliseconds
     */
    function startAutoRefresh(interval) {
        stopAutoRefresh(); // Clear any existing timer
        
        queueUpdateTimer = setInterval(() => {
            // Only update if page is visible (to save resources)
            if (window.Utils && window.Utils.isPageVisible && window.Utils.isPageVisible()) {
                fetchOrders();
            }
        }, interval);
    }

    /**
     * Stop auto-refresh timer
     */
    function stopAutoRefresh() {
        if (queueUpdateTimer) {
            clearInterval(queueUpdateTimer);
            queueUpdateTimer = null;
        }
    }

    /**
     * Refresh queue data (public method)
     */
    function refresh() {
        fetchOrders(true);
    }

    /**
     * Get current queue data (public method)
     * @returns {object} Current queue data
     */
    function getCurrentData() {
        return lastQueueData;
    }

    /**
     * Cleanup function
     */
    function cleanup() {
        stopAutoRefresh();
        
        // Remove event listeners
        if (queueRefreshButton) {
            queueRefreshButton.removeEventListener('click', fetchOrders);
        }
    }

    // Public API
    window.QueuePanel = {
        init: init,
        refresh: refresh,
        getCurrentData: getCurrentData,
        cleanup: cleanup
    };

    // Auto-initialize if DOM is ready
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', function() {
            // Will be initialized by dashboard.js with proper config
        });
    }

})();