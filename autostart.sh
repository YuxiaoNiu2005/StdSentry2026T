
#!/bin/bash

##############################################
# 哨兵机器人自动启动脚本
# 按顺序启动：串口通信 -> 视觉系统 -> 导航系统
##############################################

set -e  # 遇到错误立即退出

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 日志文件
LOG_DIR="$HOME/.ros/log/autostart"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/autostart_$(date +%Y%m%d_%H%M%S).log"

# 日志函数
log() {
    echo "[$(date +%Y-%m-%d\ %H:%M:%S)] $1" | tee -a "$LOG_FILE"
}

# 错误处理
error_exit() {
    log "ERROR: $1"
    exit 1
}

# 检查 ROS2 进程
wait_for_launch() {
    local launch_name=$1
    local max_wait=$2
    local wait_time=0
    
    log "Waiting for $launch_name to start (max ${max_wait}s)..."
    
    while [ $wait_time -lt $max_wait ]; do
        if ros2 node list 2>/dev/null | grep -q .; then
            log "$launch_name appears to be running"
            return 0
        fi
        sleep 1
        wait_time=$((wait_time + 1))
    done
    
    error_exit "$launch_name failed to start within ${max_wait}s"
}

log "========================================"
log "Starting Sentry Robot System"
log "========================================"

# 设置环境变量
log "Setting up ROS2 environment..."
source install/setup.bash || error_exit "Failed to source ROS2 environment"

# 1. 启动串口通信（裁判系统）
log "Step 1/3: Launching serial communication..."
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py > "$LOG_DIR/serial_comm.log" 2>&1 &
SERIAL_PID=$!
log "Serial communication launched (PID: $SERIAL_PID)"
sleep 3
wait_for_launch "serial_communication" 10

# 2. 启动视觉系统
log "Step 2/3: Launching vision system..."
ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py > "$LOG_DIR/vision.log" 2>&1 &
VISION_PID=$!
log "Vision system launched (PID: $VISION_PID)"
sleep 5
wait_for_launch "vision_system" 15

# 3. 启动导航系统
log "Step 3/3: Launching navigation system..."
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
    slam:=True \
    use_robot_state_pub:=True \
    > "$LOG_DIR/navigation.log" 2>&1 &
NAV_PID=$!
log "Navigation system launched (PID: $NAV_PID)"
sleep 3

log "========================================"
log "All systems launched successfully!"
log "========================================"
log "Process IDs:"
log "  - Serial Communication: $SERIAL_PID"
log "  - Vision System: $VISION_PID"
log "  - Navigation System: $NAV_PID"
log ""
log "Log files location: $LOG_DIR"
log "Main log: $LOG_FILE"
log "========================================"

# 保持脚本运行，监控子进程
log "Monitoring launched processes..."

while true; do
    # 检查进程是否还在运行
    if ! kill -0 $SERIAL_PID 2>/dev/null; then
        error_exit "Serial communication process died unexpectedly"
    fi
    
    if ! kill -0 $VISION_PID 2>/dev/null; then
        error_exit "Vision system process died unexpectedly"
    fi
    
    if ! kill -0 $NAV_PID 2>/dev/null; then
        error_exit "Navigation system process died unexpectedly"
    fi
    
    sleep 5
done