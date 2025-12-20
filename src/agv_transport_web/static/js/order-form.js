/**
 * Order Form Management for AGV Transport Web Dashboard
 * 
 * Handles:
 * - Station dropdown population
 * - Form validation
 * - Order submission
 * - Success/error feedback
 */

window.OrderForm = {
    // Form elements
    form: null,
    pickupSelect: null,
    dropoffSelect: null,
    submitButton: null,
    resetButton: null,
    messageArea: null,
    
    // Error display elements
    pickupError: null,
    dropoffError: null,
    
    // State
    stations: [],
    isSubmitting: false,
    
    /**
     * Initialize the order form
     */
    init: function() {
        this.bindElements();
        this.bindEvents();
        this.loadStations();
    },
    
    /**
     * Bind DOM elements
     */
    bindElements: function() {
        this.form = document.getElementById('order-form');
        this.pickupSelect = document.getElementById('pickup-station');
        this.dropoffSelect = document.getElementById('dropoff-station');
        this.submitButton = document.getElementById('submit-order');
        this.resetButton = document.getElementById('reset-form');
        this.messageArea = document.getElementById('form-message');
        this.pickupError = document.getElementById('pickup-error');
        this.dropoffError = document.getElementById('dropoff-error');
        
        if (!this.form) {
            console.error('Order form not found');
            return false;
        }
        
        return true;
    },
    
    /**
     * Bind event listeners
     */
    bindEvents: function() {
        if (!this.form) return;
        
        // Form submission
        this.form.addEventListener('submit', (e) => {
            e.preventDefault();
            this.handleSubmit();
        });
        
        // Form reset
        if (this.resetButton) {
            this.resetButton.addEventListener('click', () => {
                this.resetForm();
            });
        }
        
        // Station selection validation
        if (this.pickupSelect) {
            this.pickupSelect.addEventListener('change', () => {
                this.validateStationSelection();
                this.clearError('pickup');
            });
        }
        
        if (this.dropoffSelect) {
            this.dropoffSelect.addEventListener('change', () => {
                this.validateStationSelection();
                this.clearError('dropoff');
            });
        }
    },
    
    /**
     * Load stations from API and populate dropdowns
     */
    loadStations: async function() {
        try {
            // Load pickup stations (excludes charging stations)
            const pickupStations = await Utils.fetchWithTimeout('/api/stations/pickup');
            
            // Load all stations for dropoff
            const dropoffStations = await Utils.fetchWithTimeout('/api/stations/dropoff');
            
            this.populatePickupDropdown(pickupStations);
            this.populateDropoffDropdown(dropoffStations);
            
            this.stations = {
                pickup: pickupStations,
                dropoff: dropoffStations
            };
            
        } catch (error) {
            console.error('Error loading stations:', error);
            this.showError('Failed to load station data. Please refresh the page.');
            
            if (window.Notifications) {
                Notifications.showApiError('load stations', error);
            }
        }
    },
    
    /**
     * Populate pickup station dropdown
     * @param {Array} stations - Array of station objects
     */
    populatePickupDropdown: function(stations) {
        if (!this.pickupSelect) return;
        
        // Clear existing options except the first placeholder
        while (this.pickupSelect.children.length > 1) {
            this.pickupSelect.removeChild(this.pickupSelect.lastChild);
        }
        
        // Add station options
        stations.forEach(station => {
            const option = Utils.createElement('option', {
                value: station.station_name
            }, `${station.station_name} (${station.station_type})`);
            
            this.pickupSelect.appendChild(option);
        });
    },
    
    /**
     * Populate dropoff station dropdown
     * @param {Array} stations - Array of station objects
     */
    populateDropoffDropdown: function(stations) {
        if (!this.dropoffSelect) return;
        
        // Clear existing options except the first placeholder
        while (this.dropoffSelect.children.length > 1) {
            this.dropoffSelect.removeChild(this.dropoffSelect.lastChild);
        }
        
        // Add station options
        stations.forEach(station => {
            const option = Utils.createElement('option', {
                value: station.station_name
            }, `${station.station_name} (${station.station_type})`);
            
            this.dropoffSelect.appendChild(option);
        });
    },
    
    /**
     * Validate station selection
     * @returns {boolean} True if validation passes
     */
    validateStationSelection: function() {
        const pickup = this.pickupSelect?.value;
        const dropoff = this.dropoffSelect?.value;
        
        let isValid = true;
        
        // Clear previous errors
        this.clearAllErrors();
        
        // Check if stations are selected
        if (!pickup) {
            this.showFieldError('pickup', 'Please select a pickup station');
            isValid = false;
        }
        
        if (!dropoff) {
            this.showFieldError('dropoff', 'Please select a dropoff station');
            isValid = false;
        }
        
        // Check if stations are different
        if (pickup && dropoff && pickup === dropoff) {
            this.showFieldError('dropoff', 'Pickup and dropoff stations must be different');
            isValid = false;
        }
        
        return isValid;
    },
    
    /**
     * Handle form submission
     */
    handleSubmit: async function() {
        if (this.isSubmitting) return;
        
        // Validate form
        if (!this.validateStationSelection()) {
            return;
        }
        
        const formData = {
            pickup_station: this.pickupSelect.value,
            dropoff_station: this.dropoffSelect.value,
            priority: 0 // Default priority
        };
        
        this.setSubmittingState(true);
        
        try {
            const response = await Utils.fetchWithTimeout('/api/orders', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(formData),
                timeout: 15000 // 15 second timeout for order creation
            });
            
            // Success
            this.handleSubmitSuccess(response);
            
        } catch (error) {
            console.error('Error creating order:', error);
            this.handleSubmitError(error);
        } finally {
            this.setSubmittingState(false);
        }
    },
    
    /**
     * Handle successful order submission
     * @param {object} response - API response
     */
    handleSubmitSuccess: function(response) {
        // Show success notification
        if (window.Notifications) {
            Notifications.showOrderCreated(response.order_id);
        } else {
            this.showSuccess(`Order ${response.order_id} created successfully!`);
        }
        
        // Reset form
        this.resetForm();
        
        // Dispatch order created event for dashboard coordination
        document.dispatchEvent(new CustomEvent('orderCreated', {
            detail: { orderId: response.order_id, response: response }
        }));
        
        // Trigger queue refresh if available
        if (window.QueuePanel && typeof window.QueuePanel.refresh === 'function') {
            setTimeout(() => window.QueuePanel.refresh(), 1000);
        }
    },
    
    /**
     * Handle order submission error
     * @param {Error} error - Error object
     */
    handleSubmitError: function(error) {
        let errorMessage = 'Failed to create transport order';
        
        // Parse error message
        if (error.message) {
            if (error.message.includes('400')) {
                errorMessage = 'Invalid order data. Please check your selections.';
            } else if (error.message.includes('503')) {
                errorMessage = 'AGV system is currently unavailable. Please try again later.';
            } else if (error.message.includes('timeout')) {
                errorMessage = 'Request timed out. Please try again.';
            } else {
                errorMessage = `Error: ${error.message}`;
            }
        }
        
        // Show error using enhanced notification system
        if (window.Notifications) {
            Notifications.showApiError('create order', error);
        } else {
            this.showError(errorMessage);
        }
    },
    
    /**
     * Set form submitting state
     * @param {boolean} submitting - True if submitting
     */
    setSubmittingState: function(submitting) {
        this.isSubmitting = submitting;
        
        if (!this.submitButton) return;
        
        const buttonText = this.submitButton.querySelector('.btn-text');
        const buttonSpinner = this.submitButton.querySelector('.btn-spinner');
        
        if (submitting) {
            this.submitButton.disabled = true;
            if (buttonText) buttonText.style.display = 'none';
            if (buttonSpinner) buttonSpinner.style.display = 'inline';
            
            // Disable form elements
            if (this.pickupSelect) this.pickupSelect.disabled = true;
            if (this.dropoffSelect) this.dropoffSelect.disabled = true;
            if (this.resetButton) this.resetButton.disabled = true;
            
        } else {
            this.submitButton.disabled = false;
            if (buttonText) buttonText.style.display = 'inline';
            if (buttonSpinner) buttonSpinner.style.display = 'none';
            
            // Re-enable form elements
            if (this.pickupSelect) this.pickupSelect.disabled = false;
            if (this.dropoffSelect) this.dropoffSelect.disabled = false;
            if (this.resetButton) this.resetButton.disabled = false;
        }
    },
    
    /**
     * Reset form to initial state
     */
    resetForm: function() {
        if (!this.form) return;
        
        // Reset form fields
        this.form.reset();
        
        // Clear all errors and messages
        this.clearAllErrors();
        this.clearMessage();
        
        // Reset submitting state
        this.setSubmittingState(false);
    },
    
    /**
     * Show field-specific error
     * @param {string} field - Field name ('pickup' or 'dropoff')
     * @param {string} message - Error message
     */
    showFieldError: function(field, message) {
        if (typeof Notifications !== 'undefined') {
            const fieldId = field === 'pickup' ? 'pickup-station' : 'dropoff-station';
            Notifications.showFieldError(fieldId, message);
        } else {
            // Fallback to original method
            const errorElement = field === 'pickup' ? this.pickupError : this.dropoffError;
            const selectElement = field === 'pickup' ? this.pickupSelect : this.dropoffSelect;
            
            if (errorElement) {
                errorElement.textContent = message;
                errorElement.style.display = 'block';
            }
            
            if (selectElement) {
                selectElement.classList.add('error');
            }
        }
    },
    
    /**
     * Clear field-specific error
     * @param {string} field - Field name ('pickup' or 'dropoff')
     */
    clearError: function(field) {
        if (typeof Notifications !== 'undefined') {
            const fieldId = field === 'pickup' ? 'pickup-station' : 'dropoff-station';
            Notifications.clearFieldError(fieldId);
        } else {
            // Fallback to original method
            const errorElement = field === 'pickup' ? this.pickupError : this.dropoffError;
            const selectElement = field === 'pickup' ? this.pickupSelect : this.dropoffSelect;
            
            if (errorElement) {
                errorElement.textContent = '';
                errorElement.style.display = 'none';
            }
            
            if (selectElement) {
                selectElement.classList.remove('error');
            }
        }
    },
    
    /**
     * Clear all field errors
     */
    clearAllErrors: function() {
        if (typeof Notifications !== 'undefined') {
            Notifications.clearAllFieldErrors();
        } else {
            // Fallback to original method
            this.clearError('pickup');
            this.clearError('dropoff');
        }
    },
    
    /**
     * Show general form message
     * @param {string} message - Message to show
     * @param {string} type - Message type ('success', 'error', 'info')
     */
    showMessage: function(message, type = 'info') {
        if (!this.messageArea) return;
        
        this.messageArea.textContent = message;
        this.messageArea.className = `form-message ${type}`;
        this.messageArea.style.display = 'block';
        
        // Auto-hide success messages
        if (type === 'success') {
            setTimeout(() => this.clearMessage(), 5000);
        }
    },
    
    /**
     * Show success message
     * @param {string} message - Success message
     */
    showSuccess: function(message) {
        this.showMessage(message, 'success');
    },
    
    /**
     * Show error message
     * @param {string} message - Error message
     */
    showError: function(message) {
        this.showMessage(message, 'error');
    },
    
    /**
     * Clear form message
     */
    clearMessage: function() {
        if (!this.messageArea) return;
        
        this.messageArea.textContent = '';
        this.messageArea.className = 'form-message';
        this.messageArea.style.display = 'none';
    },
    
    /**
     * Refresh station data
     */
    refresh: function() {
        this.loadStations();
    },
    
    /**
     * Get current form data
     * @returns {object} Current form data
     */
    getFormData: function() {
        return {
            pickup_station: this.pickupSelect?.value || '',
            dropoff_station: this.dropoffSelect?.value || '',
            isValid: this.validateStationSelection()
        };
    }
};

// Auto-initialize when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => OrderForm.init());
} else {
    OrderForm.init();
}

// Export for module systems if available
if (typeof module !== 'undefined' && module.exports) {
    module.exports = OrderForm;
}