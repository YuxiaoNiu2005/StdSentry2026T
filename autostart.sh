#!/usr/bin/env bash

##############################################
# 哨兵机器人自动启动脚本
# 在4个独立终端窗口中启动：串口通信 -> 视觉系统 -> 导航系统 -> 哨兵导航控制
# 停止方式：直接关闭对应的终端窗口即可
##############################################

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

# 在新终端窗口中启动 ROS2 进程
# 用法: launch_in_terminal <窗口标题> <ros2 launch 命令>
launch_in_terminal() {
    local title="$1"
    local ros_cmd="$2"
    local script="cd $SCRIPT_DIR && source install/setup.bash && $ros_cmd; exec bash"

    case "$TERMINAL" in
        gnome-terminal)
            gnome-terminal --title="$title" -- bash -c "$script" &
            ;;
        xfce4-terminal)
            xfce4-terminal --title="$title" -e "bash -c \"$script\"" &
            ;;
        konsole)
            konsole --title="$title" -e bash -c "$script" &
            ;;
        *)
            xterm -T "$title" -e "bash -c \"$script\"" &
            ;;
    esac
}

echo "[1/4] 启动串口通信系统..."
launch_in_terminal "串口通信 - Serial Comm" \
    "ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py"
sleep 2

echo "[2/4] 启动视觉系统..."
launch_in_terminal "视觉系统 - Vision" \
    "ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py"
sleep 3

echo "[3/4] 启动导航系统..."
launch_in_terminal "导航系统 - Navigation" \
    "ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=race slam:=False use_robot_state_pub:=True"
sleep 2

echo "[4/4] 启动哨兵导航控制系统..."
launch_in_terminal "哨兵导航控制 - Sentry Nav Ctrl" \
    "ros2 launch sentry_nav_controller sentry_nav_real_launch.py"

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
