#!/bin/bash
# Fire Warden Bot SLAM Map Operations
# Quick commands for map management

echo "🗺️ Fire Warden Bot SLAM Map Operations"
echo "======================================"

case "$1" in
    "view")
        echo "👁️ Viewing current SLAM map..."
        if timeout 5 ros2 topic echo /map --once > /dev/null 2>&1; then
            echo "✅ Map is being published"
            echo "📊 Map details:"
            ros2 topic echo /map --once | grep -E "(width|height|resolution|origin)"
        else
            echo "❌ No map being published. Start SLAM first:"
            echo "   ./launch_slam_realtime.sh"
        fi
        
        echo ""
        echo "📁 Existing saved maps:"
        echo "In /tmp/:"
        ls -la /tmp/firewardenbot* 2>/dev/null || echo "   No maps in /tmp"
        echo "In current directory:"
        ls -la *.yaml *.pgm 2>/dev/null || echo "   No maps in current directory"
        ;;
    "save")
        MAP_NAME=${2:-"firewardenbot_map"}
        echo "💾 Saving current map as: $MAP_NAME"
        ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"
        if [ -f "$MAP_NAME.yaml" ]; then
            echo "✅ Map saved successfully!"
            echo "📁 Files created:"
            echo "   • $MAP_NAME.yaml (metadata)"
            echo "   • $MAP_NAME.pgm (image)"
            echo "👁️ View with: eog $MAP_NAME.pgm"
        else
            echo "❌ Failed to save map. Is SLAM running?"
        fi
        ;;
    "load")
        MAP_FILE=${2:-"firewardenbot_map.yaml"}
        if [ -f "$MAP_FILE" ]; then
            echo "📂 Loading map: $MAP_FILE"
            ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP_FILE" &
            sleep 2
            echo "✅ Map server started with $MAP_FILE"
        else
            echo "❌ Map file not found: $MAP_FILE"
            echo "Available maps:"
            ls -la *.yaml 2>/dev/null || echo "   No saved maps found"
        fi
        ;;
    "list")
        echo "📋 Available saved maps:"
        echo ""
        echo "🗂️ In /tmp/ directory:"
        ls -la /tmp/firewardenbot* 2>/dev/null || echo "   No maps found"
        echo ""
        echo "📁 In current directory:"
        ls -la *.yaml *.pgm 2>/dev/null || echo "   No map files found"
        echo ""
        echo "🔍 Search all directories:"
        find /home/student -name "*.yaml" -path "*/firewardenbot*" 2>/dev/null | head -10 || echo "   No firewardenbot maps found"
        echo ""
        echo "🔄 Current SLAM status:"
        if timeout 3 ros2 topic echo /map --once > /dev/null 2>&1; then
            echo "   ✅ SLAM is active"
        else
            echo "   ❌ SLAM is not running"
        fi
        ;;
    "info")
        echo "ℹ️ SLAM Map Information:"
        if timeout 5 ros2 topic echo /map --once > /dev/null 2>&1; then
            echo "📊 Live map statistics:"
            ros2 topic echo /map --once | head -20
        else
            echo "❌ No live map available"
        fi
        
        echo ""
        echo "🗂️ Saved maps:"
        for map_file in *_map.yaml; do
            if [ -f "$map_file" ]; then
                echo "   📄 $map_file"
                grep -E "(resolution|origin|occupied_thresh)" "$map_file" 2>/dev/null | sed 's/^/      /'
            fi
        done
        ;;
    "monitor")
        echo "📡 Monitoring SLAM map updates..."
        echo "Press Ctrl+C to stop"
        echo ""
        ros2 topic echo /map --field info
        ;;
    "find")
        echo "🔍 Finding all saved maps..."
        echo ""
        echo "📂 Firewardenbot maps:"
        find /home/student -name "*firewardenbot*.yaml" 2>/dev/null | while read file; do
            echo "   📄 $file"
            echo "      $(ls -lh "$file" | awk '{print $5, $6, $7, $8}')"
        done
        
        echo ""
        echo "📂 All .yaml map files:"
        find /home/student -name "*.yaml" -exec grep -l "resolution\|occupied_thresh" {} \; 2>/dev/null | head -5 | while read file; do
            echo "   📄 $file"
        done
        
        echo ""
        echo "🗂️ Recent map files:"
        find /tmp -name "*.yaml" -mtime -7 2>/dev/null | head -5 | while read file; do
            echo "   📄 $file ($(stat -c %y "$file" | cut -d' ' -f1))"
        done
        ;;
    *)
        echo ""
        echo "🎮 Usage: $0 <command> [options]"
        echo ""
        echo "Commands:"
        echo "   view                    - View current map status & saved maps"
        echo "   save [name]            - Save current map (default: firewardenbot_map)"
        echo "   load [file.yaml]       - Load existing map"
        echo "   list                   - List all saved maps in detail"
        echo "   find                   - Find all map files on system"
        echo "   info                   - Show detailed map information"
        echo "   monitor                - Monitor live map updates"
        echo ""
        echo "Examples:"
        echo "   $0 save my_office_map"
        echo "   $0 load my_office_map.yaml"
        echo "   $0 view"
        echo ""
        echo "🚀 Quick Start:"
        echo "   1. ./launch_slam_realtime.sh    # Start SLAM system"
        echo "   2. Drive robot around            # Build map"
        echo "   3. $0 save final_map            # Save when done"
        ;;
esac
