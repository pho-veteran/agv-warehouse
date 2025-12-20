/**
 * Connection status monitoring for AGV Transport Web Dashboard
 */

window.ConnectionStatus = {
    statusDot: null,
    statusText: null,
    lastUpdateTime: null,
    isConnected: false,
    hasEverConnected: false,  // Track if we've ever connected (to avoid notification on first load)
    initialized: false,       // Track if already initialized
    checkInterval: null,
    retryCount: 0,
    maxRetries: 3,
    
    /**
     * Initialize connection status monitoring
     */
    init: function() {
        if (this.initialized) {
            console.warn('ConnectionStatus already initialized');
            return;
        }
        
        this.statusDot = document.getElementById('status-dot');
        this.statusText = document.getElementById('status-text');
        this.lastUpdateTime = document.getElementById('last-update-time');
        
        if (!this.statusDot || !this.statusText) {
            console.warn('Connection status elements not found');
            return;
        }
        
        // Start monitoring - visibility is handled by Dashboard.js
        this.startMonitoring();
        
        // Listen for online/offline events only
        window.addEventListener('online', () => this.handleNetworkChange(true));
        window.addEventListener('offline', () => this.handleNetworkChange(false));
        
        this.initialized = true;
        console.log('ConnectionStatus initialized');
    },

    /**
     * Start connection monitoring
     * @param {number} interval - Check interval in milliseconds
     */
    startMonitoring: function(interval = 15000) {
        this.stopMonitoring();
        
        // Initial check
        this.checkConnection();
        
        // Set up periodic checks - less frequent since status/queue panels also check
        this.checkInterval = setInterval(() => {
            this.checkConnection();
        }, interval);
    },

    /**
     * Stop connection monitoring
     */
    stopMonitoring: function() {
        if (this.checkInterval) {
            clearInterval(this.checkInterval);
            this.checkInterval = null;
        }
    },

    /**
     * Check connection to the server
     */
    checkConnection: async function() {
        try {
            // Don't show "Checking..." status to avoid UI flicker
            const wasConnected = this.isConnected;
            
            const response = await Utils.fetchWithTimeout('/health', {
                timeout: 5000,
                method: 'GET',
                cache: 'no-cache'
            });
            
            if (response && response.status === 'healthy') {
                const wasDisconnected = !wasConnected;
                this.isConnected = true;
                this.handleConnectionSuccess(wasDisconnected);
            } else {
                const wasConnectedBefore = wasConnected;
                this.isConnected = false;
                this.handleConnectionError(new Error('Unhealthy response'), wasConnectedBefore);
            }
        } catch (error) {
            const wasConnected = this.isConnected;
            this.isConnected = false;
            this.handleConnectionError(error, wasConnected);
        }
    },

    /**
     * Set connection status with enhanced error handling
     * @param {boolean} connected - Connection status
     * @param {string} error - Error message if disconnected
     */
    setConnected: function(connected, error = '') {
        const wasConnected = this.isConnected;
        const wasDisconnected = !wasConnected;
        this.isConnected = connected;
        
        if (connected) {
            this.handleConnectionSuccess(wasDisconnected);
        } else {
            this.handleConnectionError(error, wasConnected);
        }
        
        // Dispatch connection status change event
        document.dispatchEvent(new CustomEvent('connectionStatusChanged', {
            detail: { connected, error }
        }));
    },

    /**
     * Handle successful connection with notifications
     * @param {boolean} wasDisconnected - Whether we were previously disconnected
     */
    handleConnectionSuccess: function(wasDisconnected = false) {
        this.retryCount = 0;
        this.updateStatus('connected', 'Connected');
        this.updateLastUpdateTime();
        
        // Remove error styling from connection status
        const connectionStatus = document.querySelector('.connection-status');
        if (connectionStatus) {
            connectionStatus.classList.remove('error');
        }
        
        // Only show reconnection notification if:
        // 1. We were actually disconnected before (not on initial connection)
        // 2. We have connected at least once before (not first page load)
        if (wasDisconnected && this.hasEverConnected && typeof Notifications !== 'undefined') {
            Notifications.showConnectionStatus(true);
        }
        
        // Mark that we've connected at least once
        this.hasEverConnected = true;
    },

    /**
     * Handle connection error with enhanced feedback
     * @param {string} error - Error message
     * @param {boolean} wasConnected - Whether we were previously connected
     */
    handleConnectionError: function(error, wasConnected = false) {
        this.retryCount++;
        
        let statusText = 'Disconnected';
        
        // Determine specific error type
        if (error) {
            if (error.includes('timeout')) {
                statusText = 'Timeout';
            } else if (error.includes('Failed to fetch') || error.includes('NetworkError')) {
                statusText = 'Network Error';
            } else if (error.includes('503')) {
                statusText = 'Service Unavailable';
            } else if (error.includes('500')) {
                statusText = 'Server Error';
            } else if (!navigator.onLine) {
                statusText = 'Offline';
            }
        }
        
        this.updateStatus('disconnected', statusText);
        
        // Add error styling to connection status
        const connectionStatus = document.querySelector('.connection-status');
        if (connectionStatus) {
            connectionStatus.classList.add('error');
        }
        
        // Show connection lost notification if we were previously connected
        if (wasConnected && typeof Notifications !== 'undefined') {
            Notifications.showConnectionStatus(false, error);
        }
        
        // Show retry notification if we have retries left
        if (this.retryCount <= this.maxRetries && typeof Notifications !== 'undefined') {
            const retryDelay = Math.min(1000 * Math.pow(2, this.retryCount), 30000);
            Notifications.showRetryNotification(this.retryCount, this.maxRetries, retryDelay);
        }
    },

    /**
     * Handle network status change
     * @param {boolean} online - Network online status
     */
    handleNetworkChange: function(online) {
        if (online) {
            this.retryCount = 0;
            this.checkConnection();
        } else {
            this.isConnected = false;
            this.updateStatus('disconnected', 'Offline');
            
            if (typeof Notifications !== 'undefined') {
                Notifications.showConnectionStatus(false);
            }
        }
    },

    /**
     * Update connection status display
     * @param {string} status - Status type (connected, disconnected, connecting)
     * @param {string} text - Status text
     */
    updateStatus: function(status, text) {
        if (!this.statusDot || !this.statusText) return;
        
        // Remove all status classes
        this.statusDot.classList.remove('connected', 'disconnected', 'connecting');
        
        // Add current status class
        this.statusDot.classList.add(status);
        
        // Update text
        this.statusText.textContent = text;
        
        // Update document title to reflect connection status
        this.updateDocumentTitle(status);
    },

    /**
     * Update last update time display
     */
    updateLastUpdateTime: function() {
        if (!this.lastUpdateTime) return;
        
        const now = new Date();
        this.lastUpdateTime.textContent = Utils.formatTime(now);
    },

    /**
     * Update document title with connection status
     * @param {string} status - Connection status
     */
    updateDocumentTitle: function(status) {
        const baseTitle = 'AGV Transport Dashboard';
        
        switch (status) {
            case 'connected':
                document.title = baseTitle;
                break;
            case 'disconnected':
                document.title = `${baseTitle} - Disconnected`;
                break;
            case 'connecting':
                document.title = `${baseTitle} - Connecting...`;
                break;
        }
    },

    /**
     * Get current connection status
     * @returns {object} Connection status information
     */
    getStatus: function() {
        return {
            connected: this.isConnected,
            retryCount: this.retryCount,
            lastCheck: new Date()
        };
    },

    /**
     * Force connection check
     */
    forceCheck: function() {
        this.retryCount = 0;
        this.checkConnection();
    },

    /**
     * Get detailed connection information
     */
    getDetailedStatus: async function() {
        try {
            const response = await Utils.fetchWithTimeout('/health/detailed', {
                timeout: 10000,
                method: 'GET',
                cache: 'no-cache'
            });
            
            return response;
        } catch (error) {
            console.error('Failed to get detailed status:', error);
            return {
                status: 'error',
                error: error.message,
                components: {}
            };
        }
    }
};

// Do NOT auto-initialize - Dashboard.js will handle initialization
// This prevents duplicate timers and event listeners
console.log('ConnectionStatus module loaded');