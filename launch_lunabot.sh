#!/bin/bash
# LunaBot Navigation Launch Helper

cd /home/aids/lunabot_ws
source install/setup.bash
export AMENT_PREFIX_PATH="/home/aids/lunabot_ws/install/lunabot_navigation:$AMENT_PREFIX_PATH"

echo "======================================"
echo "  LunaBot Navigation Launch Helper"
echo "======================================"
echo ""
echo "Choose launch option:"
echo "  1) Navigation only (no RViz)"
echo "  2) Full system with RViz (RECOMMENDED)"
echo "  3) With SLAM mode"
echo "  4) RViz only"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo "Launching navigation without RViz..."
        ros2 launch lunabot_navigation nav_single.launch.py
        ;;
    2)
        echo "Launching full system with RViz..."
        ros2 launch lunabot_navigation lunabot_full.launch.py
        ;;
    3)
        echo "Launching with SLAM mode..."
        ros2 launch lunabot_navigation lunabot_full.launch.py use_slam:=true
        ;;
    4)
        echo "Launching RViz only..."
        rviz2 -d $(ros2 pkg prefix lunabot_navigation)/share/lunabot_navigation/rviz/navigation.rviz
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac
