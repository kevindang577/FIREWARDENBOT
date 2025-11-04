# Fire Warden Bot UI - Shneiderman's Golden Rules Implementation

This UI implements **Shneiderman's 8 Golden Rules of Interface Design** for the Fire Warden Bot mission control system.

## Features Overview

### 🎯 **Golden Rule 1: Strive for Consistency**
- Consistent button styles and layouts throughout
- Standardized color coding (green=success, red=danger, yellow=warning, blue=info)
- Uniform spacing, typography, and interaction patterns
- Consistent icon usage from Font Awesome

### ⚡ **Golden Rule 2: Enable Frequent Users to Use Shortcuts**
- **Keyboard Shortcuts:**
  - `Ctrl+Enter` - Launch Mission
  - `Ctrl+Space` - Pause/Resume Mission
  - `Ctrl+H` - Return Home
  - `Ctrl+X` - Emergency Stop
- Quick action buttons in header
- Single-click zone selection on map

### 💬 **Golden Rule 3: Offer Informative Feedback**
- **Status Toasts** for all major actions:
  - ✅ Success: "Goal Received", "Mission Launched", "Goal Reached"
  - ⚠️ Warning: "Mission Paused", "Low Battery"
  - ❌ Error: "Mission Failed", "Emergency Stop"
  - ℹ️ Info: "Connecting", "View Changed"
- Real-time mission status indicator with color coding
- Connection status with visual feedback
- Recording indicator with blinking animation

### 🔄 **Golden Rule 4: Design Dialog to Yield Closure**
- Clear mission start/stop sequences
- Toast notifications confirm action completion
- Mission status progresses through defined states: Standby → Executing → Complete
- Each waypoint shows completion status

### 🛡️ **Golden Rule 5: Offer Simple Error Handling**
- Graceful error recovery with informative messages
- Automatic fallback to safe states on errors
- Clear error descriptions in toast notifications
- Emergency stop always available

### ↩️ **Golden Rule 6: Permit Easy Reversal of Actions**
- Zone selection can be cancelled before confirmation
- Mission can be paused and resumed
- Emergency stop for immediate reversal
- Modal dialogs allow cancellation of critical actions

### 🎮 **Golden Rule 7: Support Internal Locus of Control**
- **Confirmation Modals** for critical actions:
  - Launch Mission (with parameter summary)
  - Return to Base
  - Abort Mission
- Users must explicitly confirm zone selections
- All destructive actions require confirmation
- Users control when to start/stop recording

### 🧠 **Golden Rule 8: Reduce Short-Term Memory Load**
- **Mission Summary Panel** shows all current parameters:
  - Environment, mission type, altitude, speed
  - Battery level, waypoint count
  - Visual parameter summary before launch
- KPI dashboard shows all mission metrics at a glance
- Waypoint list shows current progress
- Recording status always visible

## File Structure

```
ui/
├── index.html          # Main UI structure
├── styles.css          # CSS implementing design principles
├── ui-controller.js    # JavaScript UI logic and Shneiderman's rules
├── ros-integration.js  # ROS 2 system integration
└── README.md          # This documentation
```

## Usage

### Quick Start
1. Open `index.html` in a web browser
2. Configure mission parameters in the side panel
3. Click on the map to select patrol zones
4. Confirm zone selection
5. Review mission summary
6. Click "Launch Mission" and confirm

### Mission Control Flow
1. **Setup Phase:**
   - Select world environment
   - Set mission parameters (altitude, speed, type)
   - Add waypoints
   - Optionally start session recording

2. **Execution Phase:**
   - Click "Launch Mission"
   - Review and confirm mission parameters
   - Monitor real-time KPIs
   - Receive status updates via toasts

3. **Monitoring Phase:**
   - Watch mission progress on KPI dashboard
   - Monitor waypoint completion
   - Receive fire detection alerts
   - Track battery levels

4. **Completion Phase:**
   - Mission completion notification
   - Automatic return to base
   - Session log saved (if recording)

## ROS 2 Integration

### Topics Published:
- `/mission/goal` - Mission waypoints
- `/emergency_stop` - Emergency stop commands
- `/mission/command` - Mission control commands

### Topics Subscribed:
- `/robot/pose` - Robot position updates
- `/mission/status` - Mission status updates
- `/battery_status` - Battery level monitoring
- `/fire_detection` - Fire alert notifications
- `/waypoint/status` - Waypoint completion status

### Services:
- `/launch_world` - Launch simulation environments
- `/save_session` - Save session logs (rosbag2)

## Session Logging (rosbag2)

The UI provides controls for session recording:
- **Start Recording** - Begins rosbag2 capture
- **Stop Recording** - Ends capture and saves file
- **View Logs** - Opens log replay interface

Sessions are automatically named with timestamps:
`firewardenbot_session_YYYYMMDD_HHMMSS`

## Customization

### Adding New Worlds
Update the world selector in `index.html`:
```html
<option value="new_world">New World</option>
```

### Modifying Toast Types
Add new toast types in `ui-controller.js`:
```javascript
this.showToast('custom', 'Title', 'Message');
```

## Accessibility Features

- High contrast mode support
- Reduced motion preferences respected
- Keyboard navigation support
- Focus indicators for all interactive elements
- ARIA labels for screen readers

## Browser Compatibility

- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## Dependencies

- Font Awesome 6.0.0 (CDN)
- Modern browser with ES6 support
- WebSocket support for ROS Bridge (optional)

## Development Notes

This UI is designed as a demonstration of Shneiderman's principles. For production deployment:

1. **Security**: Add authentication and authorization
2. **ROS Bridge**: Configure actual ROSBridge WebSocket connection
3. **Maps**: Integrate real mapping visualization (e.g., Leaflet, Google Maps)
4. **Camera**: Add live video feed integration
5. **Logging**: Implement actual rosbag2 integration
6. **Testing**: Add unit tests for UI components

## Integration with Fire Warden Bot

To use with your ROS 2 system:

1. Install ROSBridge: `sudo apt install ros-humble-rosbridge-suite`
2. Launch ROSBridge: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
3. Update `ros-integration.js` with your actual topic names
4. Configure your world launch files to match the UI selectors
5. Set up your navigation stack to accept goals from `/mission/goal`

The UI is intentionally minimal and focused on situational awareness, following the design principle of emphasizing essential information while providing comprehensive mission control capabilities.
