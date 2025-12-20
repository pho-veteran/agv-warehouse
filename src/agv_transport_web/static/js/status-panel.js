/**
 * Status Panel JavaScript for AGV Transport Web Dashboard
 * 
 * Handles real-time AGV status updates including:
 * - Current state display with color coding
 * - Battery level with progress bar
 * - Location and navigation information
 * - Auto-refresh functionality
 */

window.StatusPanel = {
    // DOM elements
    elements: {
        currentState: null,
        batteryFill: null,
        batteryPercentage: null,
        currentLocation: null,
        targetDestination: null,
        activeOrder: null,
        queueLength: null,
        navigationProgress: null,
        navigationPercentage: null,
        statusLastUpdate: null,
        statusRefresh: null
    },
    
    // Configuration
    config: {
        updateInterval: 5000,        // 5 seconds - reasonable interval
        staleDataThreshold: 15000,   // 15 seconds
        maxRetries: 3,
        retryDelay: 1000
    },
    
    // State
    state: {
        updateTimer: null,
        isUpdating: false,
        retryCount: 0,
        lastUpdateTime: null,
        lastData: null,
        isVisible: true
    },

    /**
     * Initialize the status panel
     * @param {object} options - Configuration options
     */
    init: function(options = {}) {
        // Merge configuration
        Object.assign(this.config, options);
        
        // Get DOM elements
        this.initializeElements();
        
        if (!this.elements.currentState) {
            console.error('Status panel elements not found');
            return;
        }
        
        // Set up event listeners
        this.setupEventListeners();
        
        // Start updates
        this.startUpdates();
        
        console.log('Status panel initialized');
    },

    /**
     * Initialize DOM element references
     */
    initializeElements: function() {
        this.elements = {
            currentState: document.getElementById('current-state'),
            batteryFill: document.getElementById('battery-fill'),
            batteryPercentage: document.getElementById('battery-percentage'),
            currentLocation: document.getElementById('current-location'),
            targetDestination: document.getElementById('target-destination'),
            activeOrder: document.getElementById('active-order'),
            queueLength: document.getElementById('queue-length'),
            navigationProgress: document.getElementById('navigation-progress'),
            navigationPercentage: document.getElementById('navigation-percentage'),
            statusLastUpdate: document.getElementById('status-last-update'),
            statusRefresh: document.getElementById('status-refresh')
        };
    },

    /**
     * Set up event listeners
     */
    setupEventListeners: function() {
        // Manual refresh button only - visibility is handled by Dashboard
        if (this.elements.statusRefresh) {
            this.elements.statusRefresh.addEventListener('click', () => {
                this.forceUpdate();
            });
        }
        
        // NOTE: Page visibility is handled by Dashboard.js to avoid duplicate listeners
    },

    /**
     * Start periodic status updates
     * @param {number} interval - Update interval in milliseconds
     */
    startUpdates: function(interval = null) {
        this.stopUpdates();
        
        const updateInterval = interval || this.config.updateInterval;
        
        // Initial update
        this.fetchStatus();
        
        // Set up periodic updates
        this.state.updateTimer = setInterval(() => {
            if (!this.state.isUpdating) {
                this.fetchStatus();
            }
        }, updateInterval);
    },

    /**
     * Stop periodic updates
     */
    stopUpdates: function() {
        if (this.state.updateTimer) {
            clearInterval(this.state.updateTimer);
            this.state.updateTimer = null;
        }
    },

    /**
     * Force immediate status update
     */
    forceUpdate: function() {
        this.state.retryCount = 0;
        this.fetchStatus();
        
        // Add visual feedback to refresh button
        if (this.elements.statusRefresh) {
            const refreshIcon = this.elements.statusRefresh.querySelector('.refresh-icon');
            if (refreshIcon) {
                refreshIcon.style.animation = 'spin 1s linear';
                setTimeout(() => {
                    refreshIcon.style.animation = '';
                }, 1000);
            }
        }
    },

    /**
     * Fetch AGV status from API
     */
    fetchStatus: async function() {
        if (this.state.isUpdating) {
            return;
        }
        
        this.state.isUpdating = true;
        
        try {
            // Show loading state
            this.setLoadingState(true);
            
            const data = await this.makeStatusRequest();
            
            if (data) {
                this.updateStatusDisplay(data);
                this.state.retryCount = 0;
                this.state.lastUpdateTime = new Date();
                this.state.lastData = data;
            }
            
        } catch (error) {
            this.handleFetchError(error);
        } finally {
            this.state.isUpdating = false;
            this.setLoadingState(false);
        }
    },

    /**
     * Make HTTP request to status API
     * @returns {Promise<object>} Status data
     */
    makeStatusRequest: async function() {
        const fetchFunction = (typeof Utils !== 'undefined' && Utils.fetchWithTimeout) 
            ? Utils.fetchWithTimeout 
            : fetch;
        
        const response = await fetchFunction('/api/status', {
            method: 'GET',
            headers: {
                'Accept': 'application/json',
                'Cache-Control': 'no-cache'
            },
            timeout: 8000
        });
        
        if (typeof response.json === 'function') {
            return await response.json();
        }
        
        return response;
    },

    /**
     * Update status display with new data
     * @param {object} data - Status data from API
     */
    updateStatusDisplay: function(data) {
        try {
            // Update current state
            this.updateCurrentState(data.current_state);
            
            // Update battery level
            this.updateBatteryLevel(data.battery_level);
            
            // Update location information
            this.updateLocationInfo(data.current_station, data.target_station);
            
            // Update active order
            this.updateActiveOrder(data.active_order_id);
            
            // Update queue length
            this.updateQueueLength(data.queue_length);
            
            // Update navigation progress
            this.updateNavigationProgress(data.navigation_progress);
            
            // Update last update time
            this.updateLastUpdateTime(data.last_update);
            
            // Check for stale data
            this.checkDataFreshness(data);
            
            // Highlight updated elements
            this.highlightUpdates();
            
        } catch (error) {
            console.error('Error updating status display:', error);
        }
    },

    /**
     * Update current state display
     * @param {string} state - Current AGV state
     */
    updateCurrentState: function(state) {
        if (!this.elements.currentState) return;
        
        const normalizedState = (state || 'UNKNOWN').toUpperCase();
        
        // Remove all state classes
        this.elements.currentState.className = 'state-badge';
        
        // Add current state class
        this.elements.currentState.classList.add(normalizedState);
        
        // Update text content
        this.elements.currentState.textContent = normalizedState;
    },

    /**
     * Update battery level display
     * @param {number} batteryLevel - Battery percentage (0-100)
     */
    updateBatteryLevel: function(batteryLevel) {
        if (!this.elements.batteryFill || !this.elements.batteryPercentage) return;
        
        const battery = (typeof Utils !== 'undefined' && Utils.formatBattery) 
            ? Utils.formatBattery(batteryLevel)
            : { percentage: Math.max(0, Math.min(100, batteryLevel || 0)), text: `${Math.round(batteryLevel || 0)}%`, className: 'high' };
        
        // Update progress bar
        this.elements.batteryFill.style.width = `${battery.percentage}%`;
        this.elements.batteryFill.className = `battery-fill ${battery.className}`;
        
        // Update percentage text
        this.elements.batteryPercentage.textContent = battery.text;
    },

    /**
     * Update location information
     * @param {string} currentStation - Current station name
     * @param {string} targetStation - Target station name
     */
    updateLocationInfo: function(currentStation, targetStation) {
        if (this.elements.currentLocation) {
            this.elements.currentLocation.textContent = currentStation || 'Unknown';
        }
        
        if (this.elements.targetDestination) {
            this.elements.targetDestination.textContent = targetStation || 'None';
        }
    },

    /**
     * Update active order display
     * @param {string} orderId - Active order ID
     */
    updateActiveOrder: function(orderId) {
        if (!this.elements.activeOrder) return;
        
        this.elements.activeOrder.textContent = orderId || 'None';
    },

    /**
     * Update queue length display
     * @param {number} queueLength - Number of orders in queue
     */
    updateQueueLength: function(queueLength) {
        if (!this.elements.queueLength) return;
        
        const count = Math.max(0, queueLength || 0);
        this.elements.queueLength.textContent = count.toString();
    },

    /**
     * Update navigation progress
     * @param {number} progress - Navigation progress percentage (0-100)
     */
    updateNavigationProgress: function(progress) {
        if (!this.elements.navigationProgress || !this.elements.navigationPercentage) return;
        
        const percentage = Math.max(0, Math.min(100, progress || 0));
        
        // Update progress bar
        this.elements.navigationProgress.style.width = `${percentage}%`;
        
        // Update percentage text
        this.elements.navigationPercentage.textContent = `${Math.round(percentage)}%`;
    },

    /**
     * Update last update time display
     * @param {string} lastUpdate - Last update timestamp
     */
    updateLastUpdateTime: function(lastUpdate) {
        if (!this.elements.statusLastUpdate) return;
        
        const timeText = (typeof Utils !== 'undefined' && Utils.formatTime) 
            ? Utils.formatTime(lastUpdate || new Date())
            : new Date(lastUpdate || Date.now()).toLocaleTimeString();
        
        this.elements.statusLastUpdate.textContent = timeText;
    },

    /**
     * Check if data is stale and show warning
     * @param {object} data - Status data
     */
    checkDataFreshness: function(data) {
        const isStale = data.is_stale || (data.age_seconds > this.config.staleDataThreshold / 1000);
        
        if (isStale && typeof Notifications !== 'undefined') {
            Notifications.warning('Status data may be outdated', 3000);
        }
    },

    /**
     * Highlight updated elements with animation
     */
    highlightUpdates: function() {
        if (typeof Utils !== 'undefined' && Utils.highlightUpdate) {
            // Highlight main status elements
            const elementsToHighlight = [
                this.elements.currentState,
                this.elements.batteryFill,
                this.elements.currentLocation,
                this.elements.targetDestination
            ];
            
            elementsToHighlight.forEach(element => {
                if (element) {
                    Utils.highlightUpdate(element);
                }
            });
        }
    },

    /**
     * Handle fetch errors
     * @param {Error} error - Fetch error
     */
    handleFetchError: function(error) {
        console.error('Status fetch error:', error);
        
        this.state.retryCount++;
        
        // Show error notification using enhanced system
        if (typeof Notifications !== 'undefined') {
            if (error.message.includes('timeout')) {
                Notifications.warning('Status update timeout - retrying...', 2000);
            } else if (error.message.includes('Failed to fetch')) {
                Notifications.error('Cannot connect to AGV system', 0);
            } else if (error.message.includes('503')) {
                Notifications.error('AGV system is unavailable', 0);
            } else {
                Notifications.showApiError('fetch AGV status', error);
            }
        }
        
        // Update connection status
        if (typeof ConnectionStatus !== 'undefined') {
            ConnectionStatus.setConnected(false, error.message);
        }
        
        // Implement exponential backoff for retries
        if (this.state.retryCount <= this.config.maxRetries) {
            const retryDelay = Math.min(
                this.config.retryDelay * Math.pow(2, this.state.retryCount - 1),
                30000
            );
            
            // Show retry notification
            if (typeof Notifications !== 'undefined') {
                Notifications.showRetryNotification(
                    this.state.retryCount, 
                    this.config.maxRetries, 
                    retryDelay
                );
            }
            
            setTimeout(() => {
                if (!this.state.isUpdating) {
                    this.fetchStatus();
                }
            }, retryDelay);
        } else {
            // Max retries reached
            if (typeof Notifications !== 'undefined') {
                Notifications.error(
                    'Status updates failed after multiple attempts. Please check your connection.',
                    0
                );
            }
        }
    },

    /**
     * Set loading state on status panel
     * @param {boolean} loading - Loading state
     */
    setLoadingState: function(loading) {
        if (typeof Utils !== 'undefined' && Utils.setLoadingState) {
            const statusContent = document.getElementById('status-content');
            if (statusContent) {
                Utils.setLoadingState(statusContent, loading);
            }
        }
        
        // Update refresh button state
        if (this.elements.statusRefresh) {
            if (loading) {
                this.elements.statusRefresh.style.opacity = '0.6';
                this.elements.statusRefresh.style.pointerEvents = 'none';
            } else {
                this.elements.statusRefresh.style.opacity = '';
                this.elements.statusRefresh.style.pointerEvents = '';
            }
        }
    },

    /**
     * Get current status panel state
     * @returns {object} Current state information
     */
    getState: function() {
        return {
            isUpdating: this.state.isUpdating,
            lastUpdateTime: this.state.lastUpdateTime,
            retryCount: this.state.retryCount,
            isVisible: this.state.isVisible,
            hasData: !!this.state.lastData
        };
    },

    /**
     * Destroy status panel and clean up resources
     */
    destroy: function() {
        this.stopUpdates();
        
        // Remove event listeners
        if (this.elements.statusRefresh) {
            this.elements.statusRefresh.removeEventListener('click', this.forceUpdate);
        }
        
        // Clear state
        this.state = {
            updateTimer: null,
            isUpdating: false,
            retryCount: 0,
            lastUpdateTime: null,
            lastData: null,
            isVisible: true
        };
        
        console.log('Status panel destroyed');
    }
};

// CSS animation for refresh button
const style = document.createElement('style');
style.textContent = `
    @keyframes spin {
        from { transform: rotate(0deg); }
        to { transform: rotate(360deg); }
    }
`;
document.head.appendChild(style);

// Do NOT auto-initialize - Dashboard.js will handle initialization
// This prevents duplicate timers
console.log('StatusPanel module loaded');