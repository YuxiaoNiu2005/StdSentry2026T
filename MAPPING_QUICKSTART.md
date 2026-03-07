# 快速建图流程

## 前置要求

- 如修改过 LiDAR 位置，需更新机器人描述文件和 `point_lio` 的 `gravity` 参数
- 实车部署需配置 `livox_ros_driver2` 的 IP 地址

## 建图步骤

### 1. 启动建图

```bash
ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True
```

### 2. 遥控机器人走完区域

控制机器人遍历需要建图的所有区域。

### 3. 保存地图

**新开终端**，执行以下命令（不要先终止建图程序）：

```bash
# 使用 namespace 时
ros2 run nav2_map_server map_saver_cli -f <MAP_NAME> --ros-args -r __ns:=/red_standard_robot1

# 不使用 namespace 时
ros2 run nav2_map_server map_saver_cli -f <MAP_NAME>
```

将生成 `<MAP_NAME>.pgm` 和 `<MAP_NAME>.yaml` 文件。

### 4. 终止建图程序

按 `Ctrl+C` 终止，将自动保存 `scan.pcd` 到 `point_lio/PCD` 文件夹。

### 5. 整理文件

将文件重命名并移动到指定位置：

```bash
# 假设 MAP_NAME 为 my_map
cd point_lio/PCD
mv scan.pcd <MAP_NAME>.pcd
mv <MAP_NAME>.pcd ../../pb2025_nav_bringup/pcd/reality/  # 实车
# mv <MAP_NAME>.pcd ../../pb2025_nav_bringup/pcd/simulation/  # 仿真

cd ../../
mv <MAP_NAME>.pgm pb2025_nav_bringup/map/reality/
mv <MAP_NAME>.yaml pb2025_nav_bringup/map/reality/

# 修改 yaml 文件中的 image 字段
sed -i "s|image:.*|image: <MAP_NAME>.pgm|g" pb2025_nav_bringup/map/reality/<MAP_NAME>.yaml
```

### 6. 编译并测试

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 启动导航测试
ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py \
  world:=<MAP_NAME>
```

## 可选：精修地图

使用 GIMP 打开 `.pgm` 文件，用橡皮擦工具清理噪点，用画笔添加围挡。

## 备注

- `<MAP_NAME>` 替换为实际地图名称
- 仿真环境将 `reality` 替换为 `simulation`
- 预览点云：`pcl_viewer -fc 255,255,255 -ax 3 scan.pcd`
