/**
 * Utility functions for AGV Transport Web Dashboard
 */

// Global utilities namespace
window.Utils = {
    /**
     * Format timestamp to readable string
     * @param {string|Date} timestamp - ISO timestamp or Date object
     * @returns {string} Formatted time string
     */
    formatTime: function(timestamp) {
        if (!timestamp) return 'Never';
        
        try {
            const date = new Date(timestamp);
            if (isNaN(date.getTime())) return 'Invalid';
            
            return date.toLocaleTimeString('en-US', {
                hour: '2-digit',
                minute: '2-digit',
                second: '2-digit',
                hour12: false
            });
        } catch (error) {
            console.error('Error formatting time:', error);
            return 'Error';
        }
    },

    /**
     * Format relative time (e.g., "2 minutes ago")
     * @param {string|Date} timestamp - ISO timestamp or Date object
     * @returns {string} Relative time string
     */
    formatRelativeTime: function(timestamp) {
        if (!timestamp) return 'Never';
        
        try {
            const date = new Date(timestamp);
            if (isNaN(date.getTime())) return 'Invalid';
            
            const now = new Date();
            const diffMs = now - date;
            const diffSec = Math.floor(diffMs / 1000);
            const diffMin = Math.floor(diffSec / 60);
            const diffHour = Math.floor(diffMin / 60);
            const diffDay = Math.floor(diffHour / 24);
            
            if (diffSec < 60) return `${diffSec}s ago`;
            if (diffMin < 60) return `${diffMin}m ago`;
            if (diffHour < 24) return `${diffHour}h ago`;
            return `${diffDay}d ago`;
        } catch (error) {
            console.error('Error formatting relative time:', error);
            return 'Error';
        }
    },

    /**
     * Format battery percentage with appropriate class
     * @param {number} percentage - Battery percentage (0-100)
     * @returns {object} Object with percentage and CSS class
     */
    formatBattery: function(percentage) {
        const level = Math.max(0, Math.min(100, percentage || 0));
        let className = 'high';
        
        if (level < 20) className = 'low';
        else if (level < 50) className = 'medium';
        
        return {
            percentage: level,
            text: `${level}%`,
            className: className
        };
    },

    /**
     * Debounce function to limit rapid function calls
     * @param {Function} func - Function to debounce
     * @param {number} wait - Wait time in milliseconds
     * @returns {Function} Debounced function
     */
    debounce: function(func, wait) {
        let timeout;
        return function executedFunction(...args) {
            const later = () => {
                clearTimeout(timeout);
                func(...args);
            };
            clearTimeout(timeout);
            timeout = setTimeout(later, wait);
        };
    },

    /**
     * Throttle function to limit function calls to once per interval
     * @param {Function} func - Function to throttle
     * @param {number} limit - Time limit in milliseconds
     * @returns {Function} Throttled function
     */
    throttle: function(func, limit) {
        let inThrottle;
        return function(...args) {
            if (!inThrottle) {
                func.apply(this, args);
                inThrottle = true;
                setTimeout(() => inThrottle = false, limit);
            }
        };
    },

    /**
     * Make HTTP request with error handling
     * @param {string} url - Request URL
     * @param {object} options - Fetch options
     * @returns {Promise} Promise resolving to response data
     */
    fetchWithTimeout: async function(url, options = {}) {
        const timeout = options.timeout || 10000; // 10 second default timeout
        
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), timeout);
        
        try {
            const response = await fetch(url, {
                ...options,
                signal: controller.signal
            });
            
            clearTimeout(timeoutId);
            
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }
            
            const contentType = response.headers.get('content-type');
            if (contentType && contentType.includes('application/json')) {
                return await response.json();
            } else {
                return await response.text();
            }
        } catch (error) {
            clearTimeout(timeoutId);
            
            if (error.name === 'AbortError') {
                throw new Error('Request timeout');
            }
            throw error;
        }
    },

    /**
     * Safely get nested object property
     * @param {object} obj - Object to traverse
     * @param {string} path - Dot-separated path (e.g., 'a.b.c')
     * @param {*} defaultValue - Default value if path not found
     * @returns {*} Property value or default
     */
    getNestedProperty: function(obj, path, defaultValue = null) {
        try {
            return path.split('.').reduce((current, key) => {
                return current && current[key] !== undefined ? current[key] : defaultValue;
            }, obj);
        } catch (error) {
            return defaultValue;
        }
    },

    /**
     * Validate form data
     * @param {object} data - Form data to validate
     * @param {object} rules - Validation rules
     * @returns {object} Validation result with errors
     */
    validateForm: function(data, rules) {
        const errors = {};
        
        for (const [field, rule] of Object.entries(rules)) {
            const value = data[field];
            
            if (rule.required && (!value || value.trim() === '')) {
                errors[field] = `${field} is required`;
                continue;
            }
            
            if (value && rule.minLength && value.length < rule.minLength) {
                errors[field] = `${field} must be at least ${rule.minLength} characters`;
                continue;
            }
            
            if (value && rule.maxLength && value.length > rule.maxLength) {
                errors[field] = `${field} must be no more than ${rule.maxLength} characters`;
                continue;
            }
            
            if (value && rule.pattern && !rule.pattern.test(value)) {
                errors[field] = rule.message || `${field} format is invalid`;
                continue;
            }
            
            if (rule.custom && typeof rule.custom === 'function') {
                const customError = rule.custom(value, data);
                if (customError) {
                    errors[field] = customError;
                }
            }
        }
        
        return {
            isValid: Object.keys(errors).length === 0,
            errors: errors
        };
    },

    /**
     * Generate unique ID
     * @returns {string} Unique identifier
     */
    generateId: function() {
        return 'id_' + Math.random().toString(36).substr(2, 9) + '_' + Date.now();
    },

    /**
     * Check if page is visible (for reducing update frequency when hidden)
     * @returns {boolean} True if page is visible
     */
    isPageVisible: function() {
        return !document.hidden;
    },

    /**
     * Add event listener for page visibility changes
     * @param {Function} callback - Function to call on visibility change
     */
    onVisibilityChange: function(callback) {
        document.addEventListener('visibilitychange', callback);
    },

    /**
     * Remove all child elements from a DOM element
     * @param {HTMLElement} element - Element to clear
     */
    clearElement: function(element) {
        if (element) {
            while (element.firstChild) {
                element.removeChild(element.firstChild);
            }
        }
    },

    /**
     * Create DOM element with attributes and content
     * @param {string} tag - HTML tag name
     * @param {object} attributes - Element attributes
     * @param {string|HTMLElement} content - Element content
     * @returns {HTMLElement} Created element
     */
    createElement: function(tag, attributes = {}, content = '') {
        const element = document.createElement(tag);
        
        for (const [key, value] of Object.entries(attributes)) {
            if (key === 'className') {
                element.className = value;
            } else if (key === 'dataset') {
                for (const [dataKey, dataValue] of Object.entries(value)) {
                    element.dataset[dataKey] = dataValue;
                }
            } else {
                element.setAttribute(key, value);
            }
        }
        
        if (typeof content === 'string') {
            element.textContent = content;
        } else if (content instanceof HTMLElement) {
            element.appendChild(content);
        }
        
        return element;
    },

    /**
     * Show/hide loading state on an element
     * @param {HTMLElement} element - Element to modify
     * @param {boolean} loading - True to show loading, false to hide
     */
    setLoadingState: function(element, loading) {
        if (!element) return;
        
        if (loading) {
            element.classList.add('loading');
            element.style.opacity = '0.6';
            element.style.pointerEvents = 'none';
        } else {
            element.classList.remove('loading');
            element.style.opacity = '';
            element.style.pointerEvents = '';
        }
    },

    /**
     * Animate element with highlight effect
     * @param {HTMLElement} element - Element to animate
     */
    highlightUpdate: function(element) {
        if (!element) return;
        
        element.classList.remove('data-updated');
        // Force reflow
        element.offsetHeight;
        element.classList.add('data-updated');
    }
};

// Export for module systems if available
if (typeof module !== 'undefined' && module.exports) {
    module.exports = Utils;
}