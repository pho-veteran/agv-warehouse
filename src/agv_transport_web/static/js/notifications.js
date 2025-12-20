/**
 * Notification system for AGV Transport Web Dashboard
 */

window.Notifications = {
    container: null,
    notifications: new Map(),
    
    /**
     * Initialize the notification system
     */
    init: function() {
        this.container = document.getElementById('notification-area');
        if (!this.container) {
            console.warn('Notification container not found');
            return;
        }
        
        // Set up auto-cleanup interval
        setInterval(() => this.cleanup(), 30000); // Clean up every 30 seconds
    },

    /**
     * Show a notification
     * @param {string} message - Notification message
     * @param {string} type - Notification type (success, error, warning, info)
     * @param {number} duration - Auto-hide duration in ms (0 = no auto-hide)
     * @param {boolean} persistent - Whether notification persists across page changes
     * @returns {string} Notification ID
     */
    show: function(message, type = 'info', duration = 5000, persistent = false) {
        if (!this.container) {
            console.warn('Notification system not initialized');
            return null;
        }

        const id = Utils.generateId();
        const notification = this.createNotificationElement(id, message, type);
        
        // Store notification data
        this.notifications.set(id, {
            element: notification,
            type: type,
            message: message,
            timestamp: Date.now(),
            persistent: persistent
        });

        // Add to container
        this.container.appendChild(notification);

        // Auto-hide if duration is specified
        if (duration > 0) {
            setTimeout(() => this.hide(id), duration);
        }

        // Limit total notifications
        this.limitNotifications();

        return id;
    },

    /**
     * Show success notification
     * @param {string} message - Success message
     * @param {number} duration - Auto-hide duration
     * @returns {string} Notification ID
     */
    success: function(message, duration = 3000) {
        return this.show(message, 'success', duration);
    },

    /**
     * Show error notification
     * @param {string} message - Error message
     * @param {number} duration - Auto-hide duration (0 = persistent)
     * @returns {string} Notification ID
     */
    error: function(message, duration = 0) {
        return this.show(message, 'error', duration, true);
    },

    /**
     * Show warning notification
     * @param {string} message - Warning message
     * @param {number} duration - Auto-hide duration
     * @returns {string} Notification ID
     */
    warning: function(message, duration = 5000) {
        return this.show(message, 'warning', duration);
    },

    /**
     * Show info notification
     * @param {string} message - Info message
     * @param {number} duration - Auto-hide duration
     * @returns {string} Notification ID
     */
    info: function(message, duration = 4000) {
        return this.show(message, 'info', duration);
    },

    /**
     * Hide a specific notification
     * @param {string} id - Notification ID
     */
    hide: function(id) {
        const notificationData = this.notifications.get(id);
        if (!notificationData) return;

        const element = notificationData.element;
        
        // Animate out
        element.style.opacity = '0';
        element.style.transform = 'translateY(-10px)';
        
        setTimeout(() => {
            if (element.parentNode) {
                element.parentNode.removeChild(element);
            }
            this.notifications.delete(id);
        }, 300);
    },

    /**
     * Hide all notifications of a specific type
     * @param {string} type - Notification type to hide
     */
    hideByType: function(type) {
        for (const [id, data] of this.notifications.entries()) {
            if (data.type === type) {
                this.hide(id);
            }
        }
    },

    /**
     * Clear all notifications
     */
    clear: function() {
        for (const id of this.notifications.keys()) {
            this.hide(id);
        }
    },

    /**
     * Create notification DOM element
     * @param {string} id - Notification ID
     * @param {string} message - Notification message
     * @param {string} type - Notification type
     * @returns {HTMLElement} Notification element
     */
    createNotificationElement: function(id, message, type) {
        const notification = Utils.createElement('div', {
            className: `notification ${type}`,
            'data-id': id
        });

        const messageSpan = Utils.createElement('span', {
            className: 'notification-message'
        }, message);

        const closeButton = Utils.createElement('button', {
            className: 'notification-close',
            type: 'button',
            'aria-label': 'Close notification'
        }, '×');

        closeButton.addEventListener('click', () => this.hide(id));

        notification.appendChild(messageSpan);
        notification.appendChild(closeButton);

        return notification;
    },

    /**
     * Limit the number of visible notifications
     * @param {number} maxNotifications - Maximum number of notifications
     */
    limitNotifications: function(maxNotifications = 5) {
        const notifications = Array.from(this.notifications.entries())
            .sort((a, b) => a[1].timestamp - b[1].timestamp);

        while (notifications.length > maxNotifications) {
            const [id] = notifications.shift();
            this.hide(id);
        }
    },

    /**
     * Clean up old notifications
     * @param {number} maxAge - Maximum age in milliseconds
     */
    cleanup: function(maxAge = 300000) { // 5 minutes default
        const now = Date.now();
        
        for (const [id, data] of this.notifications.entries()) {
            if (!data.persistent && (now - data.timestamp) > maxAge) {
                this.hide(id);
            }
        }
    },

    /**
     * Show connection status notification
     * @param {boolean} connected - Connection status
     */
    showConnectionStatus: function(connected) {
        // Hide previous connection notifications
        this.hideByType('connection');
        
        if (connected) {
            this.show('Connected to AGV system', 'success', 2000);
        } else {
            this.show('Connection lost - attempting to reconnect...', 'error', 0, true);
        }
    },

    /**
     * Show API error notification
     * @param {string} operation - Operation that failed
     * @param {Error} error - Error object
     */
    showApiError: function(operation, error) {
        let message = `Failed to ${operation}`;
        
        if (error.message) {
            message += `: ${error.message}`;
        }
        
        this.error(message);
    },

    /**
     * Show form validation errors
     * @param {object} errors - Validation errors object
     */
    showValidationErrors: function(errors) {
        const errorMessages = Object.values(errors);
        if (errorMessages.length > 0) {
            const message = errorMessages.join(', ');
            this.error(`Validation failed: ${message}`);
        }
    },

    /**
     * Show order creation success
     * @param {string} orderId - Created order ID
     */
    showOrderCreated: function(orderId) {
        this.success(`Order ${orderId} created successfully`);
    },

    /**
     * Show system status notification
     * @param {string} status - System status
     * @param {string} details - Additional details
     */
    showSystemStatus: function(status, details = '') {
        let type = 'info';
        let message = `System status: ${status}`;
        
        if (details) {
            message += ` - ${details}`;
        }
        
        switch (status.toLowerCase()) {
            case 'healthy':
            case 'online':
                type = 'success';
                break;
            case 'error':
            case 'offline':
            case 'failed':
                type = 'error';
                break;
            case 'warning':
            case 'degraded':
                type = 'warning';
                break;
        }
        
        this.show(message, type, type === 'error' ? 0 : 4000);
    },

    /**
     * Show retry notification with countdown
     * @param {number} retryCount - Current retry attempt
     * @param {number} maxRetries - Maximum retry attempts
     * @param {number} delay - Retry delay in milliseconds
     * @returns {string} Notification ID
     */
    showRetryNotification: function(retryCount, maxRetries, delay) {
        const message = `Connection failed. Retrying (${retryCount}/${maxRetries}) in ${Math.ceil(delay/1000)}s...`;
        return this.show(message, 'warning', delay + 1000);
    },

    /**
     * Show connection status with visual indicator
     * @param {boolean} connected - Connection status
     * @param {string} error - Error message if disconnected
     */
    showConnectionStatus: function(connected, error = '') {
        // Hide previous connection notifications
        this.hideByType('connection');
        
        if (connected) {
            const id = this.show('🟢 Connected to AGV system', 'success', 2000);
            // Mark as connection notification
            const notification = this.notifications.get(id);
            if (notification) {
                notification.type = 'connection';
            }
        } else {
            let message = '🔴 Connection lost';
            if (error) {
                message += ` - ${error}`;
            }
            message += ' - attempting to reconnect...';
            
            const id = this.show(message, 'error', 0, true);
            // Mark as connection notification
            const notification = this.notifications.get(id);
            if (notification) {
                notification.type = 'connection';
            }
        }
    },

    /**
     * Show loading notification
     * @param {string} operation - Operation being performed
     * @returns {string} Notification ID
     */
    showLoading: function(operation) {
        return this.show(`⏳ ${operation}...`, 'info', 0);
    },

    /**
     * Hide loading notification and show result
     * @param {string} loadingId - Loading notification ID
     * @param {boolean} success - Whether operation succeeded
     * @param {string} message - Result message
     */
    hideLoadingAndShowResult: function(loadingId, success, message) {
        if (loadingId) {
            this.hide(loadingId);
        }
        
        if (success) {
            this.success(message);
        } else {
            this.error(message);
        }
    },

    /**
     * Show form validation error with field highlighting
     * @param {string} fieldId - Field element ID
     * @param {string} message - Error message
     */
    showFieldError: function(fieldId, message) {
        const field = document.getElementById(fieldId);
        if (field) {
            // Add error class to field
            field.classList.add('error');
            
            // Find or create error display element
            let errorElement = document.getElementById(`${fieldId}-error`);
            if (!errorElement) {
                errorElement = document.createElement('div');
                errorElement.id = `${fieldId}-error`;
                errorElement.className = 'field-error';
                field.parentNode.appendChild(errorElement);
            }
            
            // Show error message
            errorElement.textContent = message;
            errorElement.style.display = 'block';
            
            // Auto-clear error when field is modified
            const clearError = () => {
                this.clearFieldError(fieldId);
                field.removeEventListener('input', clearError);
                field.removeEventListener('change', clearError);
            };
            
            field.addEventListener('input', clearError);
            field.addEventListener('change', clearError);
        }
        
        // Also show as notification
        this.error(`Validation error: ${message}`, 3000);
    },

    /**
     * Clear field error highlighting
     * @param {string} fieldId - Field element ID
     */
    clearFieldError: function(fieldId) {
        const field = document.getElementById(fieldId);
        if (field) {
            field.classList.remove('error');
        }
        
        const errorElement = document.getElementById(`${fieldId}-error`);
        if (errorElement) {
            errorElement.style.display = 'none';
            errorElement.textContent = '';
        }
    },

    /**
     * Clear all field errors
     */
    clearAllFieldErrors: function() {
        // Remove error classes from all form fields
        const errorFields = document.querySelectorAll('.error');
        errorFields.forEach(field => {
            field.classList.remove('error');
        });
        
        // Hide all error messages
        const errorElements = document.querySelectorAll('.field-error');
        errorElements.forEach(element => {
            element.style.display = 'none';
            element.textContent = '';
        });
    },

    /**
     * Show batch validation errors
     * @param {object} errors - Object with field names as keys and error messages as values
     */
    showValidationErrors: function(errors) {
        // Clear previous errors
        this.clearAllFieldErrors();
        
        const errorMessages = [];
        
        for (const [fieldId, message] of Object.entries(errors)) {
            this.showFieldError(fieldId, message);
            errorMessages.push(message);
        }
        
        // Show summary notification
        if (errorMessages.length > 0) {
            const summary = errorMessages.length === 1 
                ? errorMessages[0]
                : `${errorMessages.length} validation errors found`;
            this.error(`Form validation failed: ${summary}`, 5000);
        }
    },

    /**
     * Show progress notification with updates
     * @param {string} operation - Operation name
     * @param {number} progress - Progress percentage (0-100)
     * @returns {string} Notification ID
     */
    showProgress: function(operation, progress = 0) {
        const message = `${operation}: ${Math.round(progress)}%`;
        const id = Utils.generateId();
        
        const notification = this.createProgressNotificationElement(id, operation, progress);
        
        // Store notification data
        this.notifications.set(id, {
            element: notification,
            type: 'progress',
            message: message,
            timestamp: Date.now(),
            persistent: true,
            progress: progress
        });

        // Add to container
        if (this.container) {
            this.container.appendChild(notification);
        }

        return id;
    },

    /**
     * Update progress notification
     * @param {string} id - Notification ID
     * @param {number} progress - Progress percentage (0-100)
     * @param {string} message - Optional message update
     */
    updateProgress: function(id, progress, message = null) {
        const notificationData = this.notifications.get(id);
        if (!notificationData) return;

        notificationData.progress = progress;
        
        const element = notificationData.element;
        const progressBar = element.querySelector('.progress-bar-fill');
        const progressText = element.querySelector('.progress-text');
        
        if (progressBar) {
            progressBar.style.width = `${Math.round(progress)}%`;
        }
        
        if (progressText) {
            const text = message || `${Math.round(progress)}%`;
            progressText.textContent = text;
        }
        
        // Auto-hide when complete
        if (progress >= 100) {
            setTimeout(() => this.hide(id), 2000);
        }
    },

    /**
     * Create progress notification element
     * @param {string} id - Notification ID
     * @param {string} operation - Operation name
     * @param {number} progress - Initial progress
     * @returns {HTMLElement} Progress notification element
     */
    createProgressNotificationElement: function(id, operation, progress) {
        const notification = Utils.createElement('div', {
            className: 'notification progress',
            'data-id': id
        });

        const header = Utils.createElement('div', {
            className: 'progress-header'
        });

        const title = Utils.createElement('span', {
            className: 'progress-title'
        }, operation);

        const closeButton = Utils.createElement('button', {
            className: 'notification-close',
            type: 'button',
            'aria-label': 'Close notification'
        }, '×');

        closeButton.addEventListener('click', () => this.hide(id));

        header.appendChild(title);
        header.appendChild(closeButton);

        const progressContainer = Utils.createElement('div', {
            className: 'progress-container'
        });

        const progressBar = Utils.createElement('div', {
            className: 'progress-bar'
        });

        const progressFill = Utils.createElement('div', {
            className: 'progress-bar-fill'
        });
        progressFill.style.width = `${Math.round(progress)}%`;

        const progressText = Utils.createElement('div', {
            className: 'progress-text'
        }, `${Math.round(progress)}%`);

        progressBar.appendChild(progressFill);
        progressContainer.appendChild(progressBar);
        progressContainer.appendChild(progressText);

        notification.appendChild(header);
        notification.appendChild(progressContainer);

        return notification;
    }
};

// Auto-initialize when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => Notifications.init());
} else {
    Notifications.init();
}