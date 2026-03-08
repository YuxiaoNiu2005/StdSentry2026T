
#!/usr/bin/env bash

##############################################
# 哨兵机器人自动启动脚本
# 在4个独立终端窗口中启动：串口通信 -> 视觉系统 -> 导航系统 -> 哨兵导航控制
# 停止方式：直接关闭对应的终端窗口即可
##############################################

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "========================================"
echo "启动哨兵机器人四大系统"
echo "========================================"
echo "工作空间: $SCRIPT_DIR"
echo ""

# 检测可用的终端模拟器
if command -v gnome-terminal &> /dev/null; then
    TERMINAL="gnome-terminal"
elif command -v xfce4-terminal &> /dev/null; then
    TERMINAL="xfce4-terminal"
elif command -v konsole &> /dev/null; then
    TERMINAL="konsole"
elif command -v xterm &> /dev/null; then
    TERMINAL="xterm"
else
    echo "错误: 未找到可用的终端模拟器"
    echo "请安装: gnome-terminal, xfce4-terminal, konsole 或 xterm"
    exit 1
fi

echo "使用终端: $TERMINAL"
echo ""

# 1. 启动串口通信（裁判系统）
echo "[1/4] 启动串口通信系统..."
if [ "$TERMINAL" = "gnome-terminal" ]; then
    gnome-terminal --title="串口通信 - Serial Comm" -- bash -c "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        echo '========================================='
        echo '串口通信系统 - Serial Communication'
        echo '========================================='
        ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
        exec bash
    " &
elif [ "$TERMINAL" = "xfce4-terminal" ]; then
    xfce4-terminal --title="串口通信 - Serial Comm" -e "bash -c '
        cd \"$SCRIPT_DIR\"
        source install/setup.bash
        echo \"串口通信系统启动中...\"
        ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
        exec bash
    '" &
elif [ "$TERMINAL" = "konsole" ]; then
    konsole --title="串口通信 - Serial Comm" -e bash -c "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
        exec bash
    " &
else
    xterm -T "串口通信 - Serial Comm" -e "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py
        bash
    " &
fi
sleep 2

# 2. 启动视觉系统
echo "[2/4] 启动视觉系统..."
if [ "$TERMINAL" = "gnome-terminal" ]; then
    gnome-terminal --title="视觉系统 - Vision" -- bash -c "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        echo '========================================='
        echo '视觉系统 - Vision System'
        echo '========================================='
        ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py
        exec bash
    " &
elif [ "$TERMINAL" = "xfce4-terminal" ]; then
    xfce4-terminal --title="视觉系统 - Vision" -e "bash -c '
        cd \"$SCRIPT_DIR\"
        source install/setup.bash
        echo \"视觉系统启动中...\"
        ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py
        exec bash
    '" &
elif [ "$TERMINAL" = "konsole" ]; then
    konsole --title="视觉系统 - Vision" -e bash -c "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py
        exec bash
    " &
else
    xterm -T "视觉系统 - Vision" -e "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py
        bash
    " &
fi
sleep 3

# 3. 启动导航系统
echo "[3/4] 启动导航系统..."
if [ "$TERMINAL" = "gnome-terminal" ]; then
    gnome-terminal --title="导航系统 - Navigation" -- bash -c "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        echo '========================================='
        echo '导航系统 - Navigation System'
        echo '========================================='
        ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=7fs slam:=False use_robot_state_pub:=True
        exec bash
    " &
elif [ "$TERMINAL" = "xfce4-terminal" ]; then
    xfce4-terminal --title="导航系统 - Navigation" -e "bash -c '
        cd \"$SCRIPT_DIR\"
        source install/setup.bash
        echo \"导航系统启动中...\"
        ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=7fs slam:=False use_robot_state_pub:=True
        exec bash
    '" &
elif [ "$TERMINAL" = "konsole" ]; then
    konsole --title="导航系统 - Navigation" -e bash -c "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=7fs slam:=False use_robot_state_pub:=True
        exec bash
    " &
else
    xterm -T "导航系统 - Navigation" -e "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=7fs slam:=False use_robot_state_pub:=True
        bash
    " &
fi

sleep 2

# 4. 启动哨兵导航控制系统
echo "[4/4] 启动哨兵导航控制系统..."
if [ "$TERMINAL" = "gnome-terminal" ]; then
    gnome-terminal --title="哨兵导航控制 - Sentry Nav Ctrl" -- bash -c "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        echo '========================================='
        echo '哨兵导航控制系统 - Sentry Navigation Controller'
        echo '========================================='
        ros2 launch sentry_nav_controller sentry_nav_real_launch.py
        exec bash
    " &
elif [ "$TERMINAL" = "xfce4-terminal" ]; then
    xfce4-terminal --title="哨兵导航控制 - Sentry Nav Ctrl" -e "bash -c '
        cd \"$SCRIPT_DIR\"
        source install/setup.bash
        echo \"哨兵导航控制系统启动中...\"
        ros2 launch sentry_nav_controller sentry_nav_real_launch.py
        exec bash
    '" &
elif [ "$TERMINAL" = "konsole" ]; then
    konsole --title="哨兵导航控制 - Sentry Nav Ctrl" -e bash -c "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        ros2 launch sentry_nav_controller sentry_nav_real_launch.py
        exec bash
    " &
else
    xterm -T "哨兵导航控制 - Sentry Nav Ctrl" -e "
        cd '$SCRIPT_DIR'
        source install/setup.bash
        ros2 launch sentry_nav_controller sentry_nav_real_launch.py
        bash
    " &
fi

sleep 2

echo ""
echo "========================================"
echo "✓ 四个系统已在独立终端中启动"
echo "========================================"
echo ""
echo "终端窗口标题："
echo "  1. 串口通信 - Serial Comm"
echo "  2. 视觉系统 - Vision"
echo "  3. 导航系统 - Navigation"
echo "  4. 哨兵导航控制 - Sentry Nav Ctrl"
echo ""
echo "停止方式: 直接关闭对应的终端窗口"
echo "或在终端内按 Ctrl+C 停止"
echo "========================================"