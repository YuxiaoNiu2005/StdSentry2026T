# 哨兵机器人开机自启动设置指南

## 方式一：使用 systemd 服务（推荐）

### 1. 创建 systemd 服务文件

创建文件 `/etc/systemd/system/sentry-robot.service`：

```bash
sudo nano /etc/systemd/system/sentry-robot.service
```

### 2. 添加以下内容

```ini
[Unit]
Description=Sentry Robot Autonomous System
After=network.target

[Service]
Type=simple
User=zckj
WorkingDirectory=/home/zckj/Documents/StdSentry2026T
ExecStart=/home/zckj/Documents/StdSentry2026T/autostart.sh
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

# 环境变量
Environment="ROS_DOMAIN_ID=0"

[Install]
WantedBy=multi-user.target
```

### 3. 启用并启动服务

```bash
# 重新加载 systemd 配置
sudo systemctl daemon-reload

# 启用开机自启动
sudo systemctl enable sentry-robot.service

# 立即启动服务（测试用）
sudo systemctl start sentry-robot.service

# 查看服务状态
sudo systemctl status sentry-robot.service

# 查看日志
journalctl -u sentry-robot.service -f
```

### 4. 管理服务

```bash
# 停止服务
sudo systemctl stop sentry-robot.service

# 禁用开机自启动
sudo systemctl disable sentry-robot.service

# 重启服务
sudo systemctl restart sentry-robot.service
```

---

## 方式二：使用 crontab（备选方案）

### 1. 编辑 crontab

```bash
crontab -e
```

### 2. 添加以下行

```bash
@reboot sleep 30 && /home/zckj/Documents/StdSentry2026T/autostart.sh
```

### 3. 查看和管理

```bash
# 查看当前 crontab
crontab -l

# 删除 crontab
crontab -r
```

---

## 日志文件位置

所有启动日志保存在：
- 主日志：`~/.ros/log/autostart/autostart_YYYYMMDD_HHMMSS.log`
- 串口通信：`~/.ros/log/autostart/serial_comm.log`
- 视觉系统：`~/.ros/log/autostart/vision.log`
- 导航系统：`~/.ros/log/autostart/navigation.log`

---

## 测试脚本

在设置开机自启动之前，建议先手动测试：

```bash
cd /home/zckj/Documents/StdSentry2026T
./autostart.sh
```

---

## 故障排查

### 查看哪个进程出错

```bash
# systemd 方式
journalctl -u sentry-robot.service --since today

# 查看自定义日志
tail -f ~/.ros/log/autostart/autostart_*.log
tail -f ~/.ros/log/autostart/serial_comm.log
tail -f ~/.ros/log/autostart/vision.log
tail -f ~/.ros/log/autostart/navigation.log
```

### 检查 ROS2 节点

```bash
ros2 node list
ros2 topic list
```

### 手动停止所有进程

```bash
pkill -f "ros2 launch"
```

---

## 注意事项

1. **权限**：确保脚本有执行权限（已设置）
2. **路径**：确保所有路径正确（脚本中的绝对路径）
3. **环境变量**：systemd 服务中可能需要设置额外的环境变量
4. **启动顺序**：脚本已经设置了合理的等待时间
5. **网络依赖**：如果需要网络，确保 `After=network.target`
