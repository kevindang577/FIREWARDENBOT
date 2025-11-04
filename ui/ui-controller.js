/**
 * Fire Warden Bot UI Controller
 * Implements Shneiderman's 8 Golden Rules
 */

class FireWardenUI {
    constructor() {
        this.missionState = 'standby';
        this.isRecording = false;
        this.waypoints = [];
        this.missionStartTime = null;
        this.kpiUpdateInterval = null;
        
        this.initializeUI();
        this.bindEvents();
    }

    // === Golden Rule 1: Strive for consistency ===
    initializeUI() {
        this.updateMissionStatus('standby');
        this.updateConnectionStatus(true);
        this.updateBatteryLevel(85);
    }

    // === Golden Rule 2: Enable frequent users to use shortcuts ===
    bindEvents() {
        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            if (e.ctrlKey) {
                switch(e.key) {
                    case 'Enter':
                        e.preventDefault();
                        this.handleLaunchMission();
                        break;
                    case ' ':
                        e.preventDefault();
                        this.handlePauseMission();
                        break;
                    case 'h':
                        e.preventDefault();
                        this.handleReturnHome();
                        break;
                    case 'x':
                        e.preventDefault();
                        this.handleEmergencyStop();
                        break;
                }
            }
        });

        // Mission Control Buttons
        document.getElementById('launch-mission').addEventListener('click', () => this.handleLaunchMission());
        document.getElementById('pause-mission').addEventListener('click', () => this.handlePauseMission());
        document.getElementById('return-home').addEventListener('click', () => this.handleReturnHome());
        document.getElementById('abort-mission').addEventListener('click', () => this.handleAbortMission());
        document.getElementById('emergency-stop').addEventListener('click', () => this.handleEmergencyStop());

        // Zone Selection
        document.getElementById('map-container').addEventListener('click', (e) => this.handleMapClick(e));
        document.getElementById('confirm-zone').addEventListener('click', () => this.handleConfirmZone());
        document.getElementById('cancel-zone').addEventListener('click', () => this.handleCancelZone());

        // Recording Controls
        document.getElementById('start-recording').addEventListener('click', () => this.handleStartRecording());
        document.getElementById('stop-recording').addEventListener('click', () => this.handleStopRecording());
        document.getElementById('view-logs').addEventListener('click', () => this.handleViewLogs());

        // Waypoint Management
        document.getElementById('add-waypoint').addEventListener('click', () => this.handleAddWaypoint());

        // Modal Controls
        document.getElementById('modal-close').addEventListener('click', () => this.hideModal());
        document.getElementById('modal-cancel').addEventListener('click', () => this.hideModal());
        document.getElementById('modal-confirm').addEventListener('click', () => this.handleModalConfirm());

        // View Controls
        document.getElementById('toggle-view').addEventListener('click', () => this.handleToggleView());
        document.getElementById('reset-view').addEventListener('click', () => this.handleResetView());

        // Parameter Changes - Reduced Memory Load
        document.getElementById('world-select').addEventListener('change', () => this.updateMissionSummary());
        document.getElementById('mission-type').addEventListener('change', () => this.updateMissionSummary());
        document.getElementById('altitude').addEventListener('change', () => this.updateMissionSummary());
        document.getElementById('speed').addEventListener('change', () => this.updateMissionSummary());
    }

    // === Golden Rule 3: Offer informative feedback ===
    showToast(type, title, message, duration = 5000) {
        const toast = document.createElement('div');
        toast.className = `toast ${type}`;
        
        let icon = '';
        switch(type) {
            case 'success': icon = 'fas fa-check-circle'; break;
            case 'warning': icon = 'fas fa-exclamation-triangle'; break;
            case 'error': icon = 'fas fa-times-circle'; break;
            case 'info': icon = 'fas fa-info-circle'; break;
        }

        toast.innerHTML = `
            <i class="toast-icon ${icon}"></i>
            <div class="toast-content">
                <div class="toast-title">${title}</div>
                <div class="toast-message">${message}</div>
            </div>
            <button class="toast-close">
                <i class="fas fa-times"></i>
            </button>
        `;

        // Add close functionality
        toast.querySelector('.toast-close').addEventListener('click', () => {
            this.removeToast(toast);
        });

        // Add to container
        document.getElementById('toast-container').appendChild(toast);

        // Animate in
        setTimeout(() => toast.classList.add('show'), 100);

        // Auto remove
        setTimeout(() => this.removeToast(toast), duration);
    }

    removeToast(toast) {
        toast.classList.remove('show');
        setTimeout(() => {
            if (toast.parentNode) {
                toast.parentNode.removeChild(toast);
            }
        }, 300);
    }

    updateMissionStatus(status) {
        this.missionState = status;
        const statusText = document.getElementById('mission-status-text');
        const statusIndicator = document.getElementById('status-indicator');
        
        statusText.textContent = status.charAt(0).toUpperCase() + status.slice(1);
        statusIndicator.className = `status-indicator ${status}`;

        // Update UI state based on mission status
        this.updateControlButtonStates();
    }

    updateControlButtonStates() {
        const launchBtn = document.getElementById('launch-mission');
        const pauseBtn = document.getElementById('pause-mission');
        const returnBtn = document.getElementById('return-home');
        const abortBtn = document.getElementById('abort-mission');

        switch(this.missionState) {
            case 'standby':
                launchBtn.disabled = false;
                pauseBtn.disabled = true;
                returnBtn.disabled = true;
                abortBtn.disabled = true;
                break;
            case 'executing':
                launchBtn.disabled = true;
                pauseBtn.disabled = false;
                returnBtn.disabled = false;
                abortBtn.disabled = false;
                break;
            case 'paused':
                launchBtn.disabled = false;
                pauseBtn.disabled = true;
                returnBtn.disabled = false;
                abortBtn.disabled = false;
                break;
            case 'returning':
                launchBtn.disabled = true;
                pauseBtn.disabled = true;
                returnBtn.disabled = true;
                abortBtn.disabled = false;
                break;
        }
    }

    // === Golden Rule 4: Design dialog to yield closure ===
    handleLaunchMission() {
        const missionParams = this.getMissionParameters();
        
        this.showConfirmationModal(
            'Launch Mission',
            'Are you sure you want to launch the mission with the following parameters?',
            this.formatMissionParameters(missionParams),
            () => this.executeLaunchMission(missionParams)
        );
    }

    executeLaunchMission(params) {
        this.showToast('info', 'Mission Starting', 'Initializing mission parameters...');
        
        setTimeout(() => {
            this.updateMissionStatus('executing');
            this.missionStartTime = new Date();
            this.showToast('success', 'Mission Launched', 'Drone is now executing the mission plan');
            
            // Start mission timer
            this.startMissionTimer();
        }, 2000);
    }

    handlePauseMission() {
        this.updateMissionStatus('paused');
        this.showToast('warning', 'Mission Paused', 'Mission execution has been paused');
    }

    handleReturnHome() {
        this.showConfirmationModal(
            'Return to Base',
            'This will interrupt the current mission. Continue?',
            'The drone will return to its home position.',
            () => this.executeReturnHome()
        );
    }

    executeReturnHome() {
        this.updateMissionStatus('returning');
        this.showToast('info', 'Returning Home', 'Drone is returning to base station');
        
        setTimeout(() => {
            this.updateMissionStatus('standby');
            this.showToast('success', 'Mission Complete', 'Drone has returned to base');
            this.stopMissionTimer();
        }, 5000);
    }

    handleAbortMission() {
        this.showConfirmationModal(
            'Abort Mission',
            'This will immediately stop all mission activities. Continue?',
            'This action cannot be undone.',
            () => this.executeAbortMission()
        );
    }

    executeAbortMission() {
        this.updateMissionStatus('standby');
        this.showToast('error', 'Mission Aborted', 'All mission activities have been stopped');
        this.stopMissionTimer();
    }

    handleEmergencyStop() {
        this.updateMissionStatus('standby');
        this.showToast('error', 'Emergency Stop', 'All operations halted immediately');
        this.stopMissionTimer();
    }

    // === Golden Rule 5: Offer simple error handling ===
    handleError(error, context) {
        console.error(`Error in ${context}:`, error);
        this.showToast('error', 'System Error', `An error occurred: ${error.message || 'Unknown error'}`);
        
        // Reset to safe state if needed
        if (this.missionState === 'executing') {
            this.handleAbortMission();
        }
    }

    // === Golden Rule 6: Permit easy reversal of actions ===
    handleMapClick(event) {
        const rect = event.target.getBoundingClientRect();
        const x = event.clientX - rect.left;
        const y = event.clientY - rect.top;
        
        // Show zone selection overlay
        document.getElementById('selected-area').textContent = `(${Math.round(x)}, ${Math.round(y)})`;
        document.getElementById('zone-selection-overlay').classList.remove('hidden');
    }

    handleConfirmZone() {
        const area = document.getElementById('selected-area').textContent;
        this.showToast('success', 'Zone Selected', `Patrol zone confirmed: ${area}`);
        this.hideZoneSelection();
    }

    handleCancelZone() {
        this.showToast('info', 'Selection Cancelled', 'Zone selection has been cancelled');
        this.hideZoneSelection();
    }

    hideZoneSelection() {
        document.getElementById('zone-selection-overlay').classList.add('hidden');
    }

    // === Golden Rule 7: Support internal locus of control ===
    showConfirmationModal(title, message, details, confirmCallback) {
        document.getElementById('modal-title').textContent = title;
        document.getElementById('modal-message').textContent = message;
        document.getElementById('modal-details').innerHTML = details;
        
        this.modalConfirmCallback = confirmCallback;
        document.getElementById('confirmation-modal').classList.remove('hidden');
    }

    hideModal() {
        document.getElementById('confirmation-modal').classList.add('hidden');
        this.modalConfirmCallback = null;
    }

    handleModalConfirm() {
        if (this.modalConfirmCallback) {
            this.modalConfirmCallback();
        }
        this.hideModal();
    }

    // === Golden Rule 8: Reduce short-term memory load ===
    getMissionParameters() {
        return {
            world: document.getElementById('world-select').value,
            missionType: document.getElementById('mission-type').value,
            altitude: document.getElementById('altitude').value,
            speed: document.getElementById('speed').value,
            waypoints: this.waypoints.length,
            battery: this.getBatteryLevel()
        };
    }

    formatMissionParameters(params) {
        return `
            <strong>Environment:</strong> ${params.world}<br>
            <strong>Mission Type:</strong> ${params.missionType}<br>
            <strong>Altitude:</strong> ${params.altitude}m<br>
            <strong>Speed:</strong> ${params.speed} m/s<br>
            <strong>Waypoints:</strong> ${params.waypoints}<br>
            <strong>Battery:</strong> ${params.battery}%
        `;
    }

    updateMissionSummary() {
        // Visual feedback that parameters have been updated
        const sections = document.querySelectorAll('.mission-params');
        sections.forEach(section => {
            section.style.backgroundColor = '#e8f4fd';
            setTimeout(() => {
                section.style.backgroundColor = '';
            }, 1000);
        });
    }

    // === Recording Management ===
    handleStartRecording() {
        this.isRecording = true;
        document.getElementById('start-recording').disabled = true;
        document.getElementById('stop-recording').disabled = false;
        document.getElementById('recording-status').textContent = 'Recording';
        document.getElementById('recording-indicator').classList.add('recording');
        
        this.showToast('success', 'Recording Started', 'Session logging has begun');
    }

    handleStopRecording() {
        this.isRecording = false;
        document.getElementById('start-recording').disabled = false;
        document.getElementById('stop-recording').disabled = true;
        document.getElementById('recording-status').textContent = 'Not Recording';
        document.getElementById('recording-indicator').classList.remove('recording');
        
        this.showToast('info', 'Recording Stopped', 'Session log saved successfully');
    }

    handleViewLogs() {
        this.showToast('info', 'Opening Logs', 'Loading session replay interface...');
        // Here you would open the log viewer
    }

    // === Waypoint Management ===
    handleAddWaypoint() {
        const waypointNumber = this.waypoints.length + 1;
        const waypoint = {
            id: waypointNumber,
            name: `Waypoint ${waypointNumber}`,
            coords: [Math.random() * 100, Math.random() * 100],
            status: 'pending'
        };
        
        this.waypoints.push(waypoint);
        this.renderWaypoints();
        
        this.showToast('success', 'Waypoint Added', `Waypoint ${waypointNumber} added to mission plan`);
    }

    renderWaypoints() {
        const container = document.getElementById('waypoint-list');
        container.innerHTML = '';
        
        // Add home base
        const homeBase = document.createElement('div');
        homeBase.className = 'waypoint-item';
        homeBase.innerHTML = `
            <span class="waypoint-name">Home Base</span>
            <span class="waypoint-coords">(0, 0)</span>
            <div class="waypoint-status">
                <i class="fas fa-check-circle status-reached"></i>
            </div>
        `;
        container.appendChild(homeBase);
        
        // Add waypoints
        this.waypoints.forEach(waypoint => {
            const item = document.createElement('div');
            item.className = 'waypoint-item';
            
            let statusIcon = '';
            switch(waypoint.status) {
                case 'reached':
                    statusIcon = '<i class="fas fa-check-circle status-reached"></i>';
                    break;
                case 'current':
                    statusIcon = '<i class="fas fa-circle-notch fa-spin status-current"></i>';
                    break;
                default:
                    statusIcon = '<i class="fas fa-circle status-pending"></i>';
            }
            
            item.innerHTML = `
                <span class="waypoint-name">${waypoint.name}</span>
                <span class="waypoint-coords">(${Math.round(waypoint.coords[0])}, ${Math.round(waypoint.coords[1])})</span>
                <div class="waypoint-status">${statusIcon}</div>
            `;
            container.appendChild(item);
        });
    }

    getCompletedWaypoints() {
        return this.waypoints.filter(w => w.status === 'reached').length;
    }

    // === Utility Functions ===
    updateConnectionStatus(connected) {
        const icon = document.getElementById('connection-icon');
        const text = document.getElementById('connection-text');
        
        if (connected) {
            icon.className = 'fas fa-wifi connected';
            text.textContent = 'Connected';
        } else {
            icon.className = 'fas fa-wifi-slash disconnected';
            text.textContent = 'Disconnected';
        }
    }

    updateBatteryLevel(level) {
        const bar = document.querySelector('.battery-bar');
        const percent = document.getElementById('battery-percent');
        
        bar.style.setProperty('--battery-level', `${level}%`);
        percent.textContent = `${Math.round(level)}%`;
        
        // Update battery bar color
        bar.className = 'battery-bar';
        if (level < 20) {
            bar.classList.add('low');
        } else if (level < 50) {
            bar.classList.add('medium');
        }
    }

    getBatteryLevel() {
        const text = document.getElementById('battery-percent').textContent;
        return parseInt(text.replace('%', ''));
    }

    handleToggleView() {
        const button = document.getElementById('toggle-view');
        const icon = button.querySelector('i');
        const text = button.querySelector('span') || button.childNodes[1];
        
        if (icon.classList.contains('fa-camera')) {
            icon.className = 'fas fa-map';
            button.innerHTML = '<i class="fas fa-map"></i> Switch to Map';
            this.showToast('info', 'View Changed', 'Switched to camera view');
        } else {
            icon.className = 'fas fa-camera';
            button.innerHTML = '<i class="fas fa-camera"></i> Switch to Camera';
            this.showToast('info', 'View Changed', 'Switched to map view');
        }
    }

    handleResetView() {
        this.showToast('info', 'View Reset', 'View has been reset to default position');
    }
}

// Initialize the UI when the page loads
document.addEventListener('DOMContentLoaded', () => {
    window.fireWardenUI = new FireWardenUI();
});
