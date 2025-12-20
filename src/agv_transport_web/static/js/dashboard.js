/**
 * Dashboard Controller for AGV Transport Web Dashboard
 * 
 * Main controller that coordinates all dashboard components:
 * - Status panel updates
 * - Order form functionality
 * - Error handling and retry logic
 * - Page visibility API for reduced polling
 * - Global error handling
 */

window.Dashboard = {
    // Configuration
    config: {
        statusUpdateInterval: 5000,     // 5 seconds - reasonable for status
        staleDataThreshold: 15000,      // 15 seconds
        maxRetries: 3,
        retryDelay: 1000,
        reducedUpdateInterval: 60000    // 60 seconds when page hidden
    },
    
    // State
    state: {
        initialized: false,
        isVisible: true,
        statusTimer: null,
        retryCount: 0,
        lastError: null,
        components: {
            statusPanel: null,
            orderForm: null,
            connectionStatus: null,
            notifications: null
        }
    },

    /**
     * Initialize the dashboard with configuration
     * @param {object} options - Configuration options
     */
    init: function(options = {}) {
        if (this.state.initialized) {
            console.warn('Dashboard already initialized');
            return;
        }

        // Merge configuration
        Object.assign(this.config, options);
        
        console.log('Initializing AGV Transport Dashboard...');
        
        // Initialize components
        this.initializeComponents();
        
        // Set up global error handling
        this.setupGlobalErrorHandling();
        
        // Set up page visibility handling
        this.setupVisibilityHandling();
        
        // Set up periodic updates
        this.startPeriodicUpdates();
        
        // Set up event listeners
        this.setupEventListeners();
        
        // Mark as initialized
        this.state.initialized = true;
        
        console.log('Dashboard initialized successfully');
    },

    /**
     * Initialize all dashboard components
     */
    initializeComponents: function() {
        try {
            // Initialize connection status monitoring
            if (typeof ConnectionStatus !== 'undefined') {
                this.state.components.connectionStatus = ConnectionStatus;
                if (!ConnectionStatus.initialized) {
                    ConnectionStatus.init();
                }
            }
            
            // Initialize notifications system
            if (typeof Notifications !== 'undefined') {
                this.state.components.notifications = Notifications;
                if (!Notifications.container) {
                    Notifications.init();
                }
            }
            
            // Initialize status panel
            if (typeof StatusPanel !== 'undefined') {
                this.state.components.statusPanel = StatusPanel;
                StatusPanel.init({
                    updateInterval: this.config.statusUpdateInterval,
                    staleDataThreshold: this.config.staleDataThreshold,
                    maxRetries: this.config.maxRetries,
                    retryDelay: this.config.retryDelay
                });
            }
            
            // Initialize order form
            if (typeof OrderForm !== 'undefined') {
                this.state.components.orderForm = OrderForm;
                if (!OrderForm.form) {
                    OrderForm.init();
                }
            }
            
        } catch (error) {
            console.error('Error initializing dashboard components:', error);
            this.handleError('Component initialization failed', error);
        }
    },

    /**
     * Set up global error handling
     */
    setupGlobalErrorHandling: function() {
        // Handle unhandled promise rejections
        window.addEventListener('unhandledrejection', (event) => {
            console.error('Unhandled promise rejection:', event.reason);
            this.handleError('Unhandled promise rejection', event.reason);
            event.preventDefault();
        });
        
        // Handle JavaScript errors
        window.addEventListener('error', (event) => {
            console.error('JavaScript error:', event.error);
            this.handleError('JavaScript error', event.error);
        });
        
        // Handle fetch errors globally
        const originalFetch = window.fetch;
        window.fetch = async function(...args) {
            try {
                const response = await originalFetch.apply(this, args);
                
                // Reset retry count on successful request
                if (response.ok) {
                    Dashboard.state.retryCount = 0;
                }
                
                return response;
            } catch (error) {
                Dashboard.handleFetchError(error, args[0]);
                throw error;
            }
        };
    },

    /**
     * Set up page visibility handling for reduced polling
     */
    setupVisibilityHandling: function() {
        // Only ONE visibility handler for the entire dashboard
        const handleVisibility = () => {
            const isVisible = document.visibilityState === 'visible';
            
            // Avoid duplicate calls
            if (this.state.isVisible === isVisible) {
                return;
            }
            
            this.handleVisibilityChange(isVisible);
        };
        
        document.addEventListener('visibilitychange', handleVisibility);
        
        // No fallback focus/blur - they cause too many false triggers
    },

    /**
     * Handle page visibility changes
     * @param {boolean} isVisible - Whether page is visible
     */
    handleVisibilityChange: function(isVisible) {
        this.state.isVisible = isVisible;
        
        if (isVisible) {
            console.log('Page became visible - resuming normal update frequency');
            
            // Resume normal update frequency
            this.startPeriodicUpdates();
            
            // Single refresh after becoming visible (not forceUpdateAll which triggers too many requests)
            if (this.state.components.statusPanel && this.state.components.statusPanel.fetchStatus) {
                this.state.components.statusPanel.fetchStatus();
            }
            
        } else {
            console.log('Page became hidden - reducing update frequency');
            
            // Reduce update frequency to save resources
            this.startPeriodicUpdates(true);
        }
    },

    /**
     * Start periodic updates for status
     * @param {boolean} reduced - Whether to use reduced frequency
     */
    startPeriodicUpdates: function(reduced = false) {
        this.stopPeriodicUpdates();
        
        const statusInterval = reduced ? this.config.reducedUpdateInterval : this.config.statusUpdateInterval;
        
        // Start status updates - StatusPanel manages its own timer
        if (this.state.components.statusPanel) {
            this.state.components.statusPanel.startUpdates(statusInterval);
        }
        
        console.log(`Periodic updates started - Status: ${statusInterval}ms`);
    },

    /**
     * Stop all periodic updates
     */
    stopPeriodicUpdates: function() {
        // Stop status panel updates
        if (this.state.components.statusPanel && this.state.components.statusPanel.stopUpdates) {
            this.state.components.statusPanel.stopUpdates();
        }
    },

    /**
     * Set up additional event listeners
     */
    setupEventListeners: function() {
        // Listen for successful order creation
        document.addEventListener('orderCreated', (event) => {
            console.log('Order created event received:', event.detail);
        });
        
        // Listen for connection status changes
        document.addEventListener('connectionStatusChanged', (event) => {
            const { connected, error } = event.detail;
            this.handleConnectionStatusChange(connected, error);
        });
        
        // Listen for keyboard shortcuts
        document.addEventListener('keydown', (event) => {
            this.handleKeyboardShortcuts(event);
        });
        
        // Listen for beforeunload to cleanup
        window.addEventListener('beforeunload', () => {
            this.cleanup();
        });
    },

    /**
     * Handle connection status changes
     * @param {boolean} connected - Connection status
     * @param {string} error - Error message if disconnected
     */
    handleConnectionStatusChange: function(connected, error) {
        if (connected) {
            // Connection restored - reset retry count
            this.state.retryCount = 0;
            this.state.lastError = null;
            
            // Don't call forceUpdateAll - let normal polling handle it
            // This prevents request spam when connection is restored
            
        } else {
            // Connection lost - implement retry logic
            this.handleConnectionLoss(error);
        }
    },

    /**
     * Handle connection loss with retry logic
     * @param {string} error - Error message
     */
    handleConnectionLoss: function(error) {
        this.state.retryCount++;
        this.state.lastError = error;
        
        console.warn(`Connection lost (attempt ${this.state.retryCount}):`, error);
        
        if (this.state.retryCount <= this.config.maxRetries) {
            // Implement exponential backoff
            const retryDelay = Math.min(
                this.config.retryDelay * Math.pow(2, this.state.retryCount - 1),
                30000 // Max 30 seconds
            );
            
            console.log(`Retrying connection in ${retryDelay}ms...`);
            
            setTimeout(() => {
                this.attemptReconnection();
            }, retryDelay);
            
        } else {
            // Max retries reached
            console.error('Max connection retries reached');
            
            if (this.state.components.notifications) {
                this.state.components.notifications.error(
                    'Connection failed after multiple attempts. Please check your network and refresh the page.',
                    0 // Persistent
                );
            }
        }
    },

    /**
     * Attempt to reconnect to the server
     */
    attemptReconnection: async function() {
        try {
            console.log('Attempting to reconnect...');
            
            // Try to fetch health endpoint
            const response = await Utils.fetchWithTimeout('/health', {
                timeout: 5000,
                method: 'GET',
                cache: 'no-cache'
            });
            
            if (response && response.status === 'healthy') {
                console.log('Reconnection successful');
                
                // Reset state
                this.state.retryCount = 0;
                this.state.lastError = null;
                
                // Resume normal operations
                this.startPeriodicUpdates();
                this.forceUpdateAll();
                
                // Show success notification
                if (this.state.components.notifications) {
                    this.state.components.notifications.success('Connection restored', 3000);
                }
                
                // Dispatch connection restored event
                document.dispatchEvent(new CustomEvent('connectionStatusChanged', {
                    detail: { connected: true }
                }));
                
            } else {
                throw new Error('Health check failed');
            }
            
        } catch (error) {
            console.error('Reconnection attempt failed:', error);
            this.handleConnectionLoss(error.message);
        }
    },

    /**
     * Handle keyboard shortcuts
     * @param {KeyboardEvent} event - Keyboard event
     */
    handleKeyboardShortcuts: function(event) {
        // Ctrl/Cmd + R: Force refresh all data
        if ((event.ctrlKey || event.metaKey) && event.key === 'r' && event.shiftKey) {
            event.preventDefault();
            this.forceUpdateAll();
            
            if (this.state.components.notifications) {
                this.state.components.notifications.info('Data refreshed', 1000);
            }
        }
        
        // Escape: Clear all notifications
        if (event.key === 'Escape') {
            if (this.state.components.notifications) {
                this.state.components.notifications.clear();
            }
        }
    },

    /**
     * Force update all dashboard components
     */
    forceUpdateAll: function() {
        console.log('Forcing update of all dashboard components');
        
        try {
            // Force status panel update
            if (this.state.components.statusPanel && this.state.components.statusPanel.forceUpdate) {
                this.state.components.statusPanel.forceUpdate();
            }
            
            // Refresh order form stations
            if (this.state.components.orderForm && this.state.components.orderForm.refresh) {
                this.state.components.orderForm.refresh();
            }
            
            // Force connection status check
            if (this.state.components.connectionStatus && this.state.components.connectionStatus.forceCheck) {
                this.state.components.connectionStatus.forceCheck();
            }
            
        } catch (error) {
            console.error('Error during force update:', error);
            this.handleError('Force update failed', error);
        }
    },

    /**
     * Handle fetch errors globally
     * @param {Error} error - Fetch error
     * @param {string} url - Request URL
     */
    handleFetchError: function(error, url) {
        console.error(`Fetch error for ${url}:`, error);
        
        // Increment retry count
        this.state.retryCount++;
        
        // Show appropriate notification based on error type
        if (this.state.components.notifications) {
            if (error.message.includes('timeout')) {
                this.state.components.notifications.warning(
                    `Request timeout for ${url}`, 
                    3000
                );
            } else if (error.message.includes('Failed to fetch')) {
                this.state.components.notifications.error(
                    'Network connection error',
                    0 // Persistent
                );
            } else {
                this.state.components.notifications.error(
                    `API error: ${error.message}`,
                    5000
                );
            }
        }
    },

    /**
     * Handle general errors
     * @param {string} context - Error context
     * @param {Error} error - Error object
     */
    handleError: function(context, error) {
        console.error(`Dashboard error (${context}):`, error);
        
        this.state.lastError = {
            context: context,
            error: error,
            timestamp: new Date()
        };
        
        // Show error notification
        if (this.state.components.notifications) {
            this.state.components.notifications.error(
                `${context}: ${error.message || error}`,
                0 // Persistent for serious errors
            );
        }
    },

    /**
     * Get current dashboard state
     * @returns {object} Dashboard state information
     */
    getState: function() {
        return {
            initialized: this.state.initialized,
            isVisible: this.state.isVisible,
            retryCount: this.state.retryCount,
            lastError: this.state.lastError,
            components: {
                statusPanel: this.state.components.statusPanel ? 
                    (this.state.components.statusPanel.getState ? this.state.components.statusPanel.getState() : 'available') : 'unavailable',
                orderForm: this.state.components.orderForm ? 'available' : 'unavailable',
                connectionStatus: this.state.components.connectionStatus ? 
                    (this.state.components.connectionStatus.getStatus ? this.state.components.connectionStatus.getStatus() : 'available') : 'unavailable',
                notifications: this.state.components.notifications ? 'available' : 'unavailable'
            }
        };
    },

    /**
     * Get dashboard health information
     * @returns {object} Health information
     */
    getHealth: function() {
        const state = this.getState();
        const health = {
            status: 'healthy',
            components: {},
            errors: [],
            uptime: Date.now() - (this.initTime || Date.now())
        };
        
        // Check component health
        for (const [name, status] of Object.entries(state.components)) {
            if (status === 'unavailable') {
                health.status = 'degraded';
                health.errors.push(`${name} component unavailable`);
            }
            health.components[name] = status;
        }
        
        // Check for recent errors
        if (state.lastError) {
            const errorAge = Date.now() - state.lastError.timestamp.getTime();
            if (errorAge < 60000) { // Last error within 1 minute
                health.status = 'degraded';
                health.errors.push(`Recent error: ${state.lastError.context}`);
            }
        }
        
        // Check retry count
        if (state.retryCount > 0) {
            health.status = 'degraded';
            health.errors.push(`${state.retryCount} failed requests`);
        }
        
        return health;
    },

    /**
     * Cleanup dashboard resources
     */
    cleanup: function() {
        console.log('Cleaning up dashboard resources...');
        
        // Stop periodic updates
        this.stopPeriodicUpdates();
        
        // Cleanup components
        if (this.state.components.statusPanel && this.state.components.statusPanel.destroy) {
            this.state.components.statusPanel.destroy();
        }
        
        if (this.state.components.connectionStatus && this.state.components.connectionStatus.stopMonitoring) {
            this.state.components.connectionStatus.stopMonitoring();
        }
        
        // Clear notifications
        if (this.state.components.notifications) {
            this.state.components.notifications.clear();
        }
        
        // Reset state
        this.state.initialized = false;
        this.state.components = {};
        
        console.log('Dashboard cleanup complete');
    },

    /**
     * Restart dashboard (cleanup and reinitialize)
     */
    restart: function() {
        console.log('Restarting dashboard...');
        
        this.cleanup();
        
        // Wait a moment before reinitializing
        setTimeout(() => {
            this.init(this.config);
        }, 1000);
    }
};

// Store initialization time
Dashboard.initTime = Date.now();

// Mark Dashboard as available globally
window.Dashboard.initialized = false;

// Export for module systems if available
if (typeof module !== 'undefined' && module.exports) {
    module.exports = Dashboard;
}

// Auto-initialize when DOM is ready (if not already done)
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function() {
        // Dashboard will be initialized by the template script
        console.log('DOM ready - Dashboard available for initialization');
    });
} else {
    console.log('DOM already ready - Dashboard available for initialization');
}