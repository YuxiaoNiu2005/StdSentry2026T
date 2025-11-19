# Nav2 导航系统调参指南（ai瞎写的）

## 一、障碍物检测参数

### 高度参考坐标系

所有高度参数都是**相对于地面**的绝对高度,而非相对于雷达本体。

**坐标变换流程**:

```text
激光点(雷达坐标系)
  ↓ [Point-LIO 里程计提供姿态]
  ↓ (roll/pitch 补偿)
水平坐标系点
  ↓ [减去 vehicleHeight 偏移]
相对地面高度
  ↓ [在 min/max_obstacle_height 范围内过滤]
最终障碍物点
```

### 关键参数配置

#### terrain_analysis 节点参数

```yaml
# 高度范围过滤 (单位: 米, 相对地面)
min_obstacle_height: 0.0    # 最低检测高度
max_obstacle_height: 2.0    # 最高检测高度

# 机器人底盘高度 (用于地面基准校准)
vehicleHeight: 0.5          # 底盘离地高度

# 地形分析参数
useSorting: true            # 启用高度排序
quantileZ: 0.2              # 分位数比例 (z轴排序用)
clearDyObs: true            # 清除动态障碍历史记录
maxRelZ: 0.5                # 相对高度上限 (地形突变阈值)
```

### 实际检测行为

#### 场景 1: 0.2 米高台

- **当前配置**: [0.0, 2.0] 范围
- **检测结果**: ✅ **被识别为障碍物**
- **机器人行为**: 绕开高台,不尝试爬上去
- **何时修改**: 如果想允许上去,设置 `min_obstacle_height: 0.25`

#### 场景 2: 17 毫米小弹丸

- **物理限制**: 激光雷达垂直分辨率 ~1-2°,近距离间距 2-4 厘米
- **检测结果**: ❌ **不会被检测** (超出分辨率)
- **机器人行为**: **直接碾过去**
- **原因**: 激光束直接越过 + 噪声过滤 + 栅格化稀释

#### 场景 3: 0.5 米机器人

- **检测结果**: ✅ **被识别为障碍物**
- **机器人行为**: 绕开躲避
- **参数关键**: 需要保持 `max_obstacle_height >= 0.5`

---

## 二、局部控制器 (OmniPidPursuitController) 调参

### 数据流向

#### 输入数据 (实时回传)

| 数据源 | Topic | 内容 | 用途 |
|--------|-------|------|------|
| **里程计** | `/odometry` | pose (x,y,yaw) + velocity (vx,vy,wz) | 位姿反馈 + 速度反馈 |
| **全局规划** | setPlan() | 全局路径 | 提供目标路径 |
| **代价地图** | costmap | 障碍物分布 | 碰撞检测 |
| **TF树** | `/tf` | 坐标变换 | 坐标系转换 |

#### 输出数据 (发布)

```
cmd_vel (geometry_msgs::msg::TwistStamped)
├── linear.x   → X方向线速度 (前后)
├── linear.y   → Y方向线速度 (左右,全向轮特有)
└── angular.z  → 角速度 (旋转)
```

### 控制算法流程

```
输入: pose(当前位姿) + velocity(当前速度)
  ↓
1️⃣ 全局路径转换 → 转换到机器人坐标系
  ↓
2️⃣ 动态前瞻距离 → lookahead_dist = f(|velocity|)
  ↓
3️⃣ 计算前瞻点 → 在路径上找目标点
  ↓
4️⃣ 误差计算 → 距离误差 + 角度误差
  ↓
5️⃣ PID 控制
  ├─ 线速度 = move_pid->calculate(distance_error)
  └─ 角速度 = heading_pid->calculate(angle_error)
  ↓
6️⃣ 曲率限制 → 弯道自动减速
  ↓
7️⃣ 接近目标减速 → 目标附近降低速度
  ↓
8️⃣ 碰撞检测 → 规划轨迹检查
  ↓
9️⃣ 全向分解 → vx = v·cos(θ), vy = v·sin(θ)
  ↓
输出: cmd_vel 发布到底盘
```

---

## 三、底盘跟随延迟的 PID 调参

### 问题现象

**底盘相对于 gimbal_yaw 慢半拍**:
- 导航系统发出速度指令
- 底盘需要先旋转对齐云台方向
- 产生响应延迟

### 调参策略

#### 原则
响应滞后系统需要增加**阻尼和积分补偿**,同时给予更多反应时间。

#### 参数调整表

| 参数 | 当前值 | 调整建议 | 目的 |
|------|--------|---------|------|
| `translation_kp` | 3.0 | **1.5~2.0** | 降低比例增益,减少过冲 |
| `translation_kd` | 0.3 | **0.8~1.2** | **增加微分增益**,阻尼振荡 |
| `translation_ki` | 0.1 | **0.3~0.5** | **增加积分增益**,补偿滞后 |
| `rotation_kp` | 3.0 | **1.5~2.0** | 降低旋转比例增益 |
| `rotation_kd` | 0.3 | **0.8~1.0** | 增加旋转阻尼 |
| `rotation_ki` | 0.1 | **0.2~0.3** | 增加旋转积分 |
| `lookahead_dist` | 2.0 | **2.5~3.0** | 更早开始转向准备 |
| `min_lookahead_dist` | 0.5 | **0.8** | 提高最小前瞻距离 |
| `controller_frequency` | 20.0 | **15.0** | 减少控制频率,给底盘跟随时间 |
| `transform_tolerance` | 0.1 | **0.2~0.3** | 容忍更大的时间延迟 |

### 调参步骤

#### 第一阶段: 降低过冲 (Week 1)
```yaml
translation_kp: 3.0 → 2.0     # 减半 P 值
translation_kd: 0.3 → 0.6     # 加倍 D 值
```
**测试**: 直线跟踪,观察是否还有振荡

#### 第二阶段: 补偿滞后 (Week 2)
```yaml
translation_ki: 0.1 → 0.3     # 增加 I 值
lookahead_dist: 2.0 → 2.5     # 增加前瞻距离
```
**测试**: 弯道跟踪,观察转向平滑性

#### 第三阶段: 微调 (Week 3)
```yaml
controller_frequency: 20.0 → 15.0
transform_tolerance: 0.1 → 0.2
```
**测试**: 整体导航稳定性

### 调参经验法则

| PID 参数 | 增大效果 | 减小效果 | 注意事项 |
|---------|---------|---------|---------|
| **Kp** | 响应快 | 响应慢 | 过大→振荡,过小→跟踪误差大 |
| **Ki** | 消除误差 | 误差持续 | 过大→积分饱和,过小→稳态误差 |
| **Kd** | 减少过冲 | 增加过冲 | 过大→噪声放大,过小→响应欠阻尼 |

---

## 四、新地图适配 (0.2m 高台场景)

### 场景背景
- **旧地图**: 2m 墙 + 0.5m 机器人
- **新地图**: + 0.2m 高台

### 参数保持

```yaml
# ✅ 保持不变 - 能正确识别所有障碍物
min_obstacle_height: 0.0
max_obstacle_height: 2.0
vehicleHeight: 0.5
```

**原因**:
- 0.2m 高台在 [0.0, 2.0] 范围内 → 自动检测
- 0.5m 机器人在范围内 → 正常绕避
- 17mm 弹丸超出分辨率 → 自动忽略

### 可选微调 (精细化)

如果高台边缘检测过于敏感:

```yaml
maxRelZ: 0.5 → 0.25    # 更严格的地形陡峭判定
quantileZ: 0.2 → 0.25  # 更保守的地形分析
```

---

## 五、参数调试工作流

### 环境准备

```bash
# 源代码环境
source /home/niuyuxiao/Documents/StdSentry2026T/install/setup.zsh

# 启动仿真
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

# 单独查看代价地图
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/map.yaml
```

### 参数快速修改

配置文件位置:
```
/home/niuyuxiao/Documents/StdSentry2026T/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml
```

修改后立即生效(需要 `use_sim_time: True`):
```bash
# 不需要重新编译,直接更新参数服务器
ros2 param set /controller_server translation_kp 2.0
```

### 调试技巧

#### 1. 可视化前瞻点
订阅 `/carrot_pose` marker 话题,在 RViz 中显示

#### 2. 监看 PID 误差
```bash
ros2 topic echo /controller_server/debug/trajectory_error
```

#### 3. 记录性能指标
```bash
# 记录底盘速度响应延迟
ros2 bag record /cmd_vel /odom -o delay_test
```

#### 4. 单步测试
```bash
# 直线导航测试
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 5, y: 0}}}}"

# 转圈测试 (观察旋转响应)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 0, y: 0}, orientation: {w: 0, z: 1}}}}"
```

---

## 六、常见问题排查

### Q1: 机器人路径跟踪震荡
**症状**: 在直线上左右摇摆

**排查步骤**:
1. 增加 `Kd` (微分增益) → 0.8~1.2
2. 减少 `Kp` (比例增益) → 1.5~2.0
3. 如果还是震荡,降低 `controller_frequency` 至 10.0

### Q2: 转弯时进度缓慢
**症状**: 接近目标时速度大幅下降

**排查步骤**:
1. 检查 `curvature_max` 设置 (高曲率阈值)
2. 增加 `reduction_ratio_at_high_curvature` → 0.7
3. 检查障碍物膨胀半径是否过大

### Q3: 底盘跟随云台抖动
**症状**: 底盘相对云台抖动,不稳定

**排查步骤**:
1. **减少** `rotation_kp` → 1.5
2. **增加** `rotation_kd` → 1.0
3. 检查 `gimbal_yaw_fake` TF 发布频率

### Q4: 0.2m 高台被识别为深沟
**症状**: 路径规划绕开高台,但应该通过

**排查步骤**:
1. 检查 `maxRelZ: 0.5` (相对高度)
2. 如果高台边缘陡峭,调小至 0.25
3. 或调大 `min_obstacle_height: 0.25`

### Q5: 激光点云缺失导致障碍物漏检
**症状**: 某些障碍物不出现在代价地图

**排查步骤**:
1. 检查 `terrain_analysis` 节点是否运行
2. 查看 Point-LIO 里程计质量
3. 验证 TF 树完整性: `ros2 run tf2_tools view_frames`

---

## 七、参考配置示例

### 保守配置 (优先安全性)
```yaml
translation_kp: 1.5
translation_ki: 0.2
translation_kd: 1.0
rotation_kp: 1.5
rotation_ki: 0.2
rotation_kd: 0.8
lookahead_dist: 3.0
controller_frequency: 15.0
transform_tolerance: 0.2
```

### 激进配置 (优先速度)
```yaml
translation_kp: 3.0
translation_ki: 0.5
translation_kd: 0.6
rotation_kp: 3.0
rotation_ki: 0.3
rotation_kd: 0.5
lookahead_dist: 2.0
controller_frequency: 20.0
transform_tolerance: 0.1
```

### 均衡配置 (推荐)
```yaml
translation_kp: 2.0
translation_ki: 0.3
translation_kd: 0.8
rotation_kp: 2.0
rotation_ki: 0.2
rotation_kd: 0.8
lookahead_dist: 2.5
controller_frequency: 15.0
transform_tolerance: 0.15
```

---

## 八、关键配置文件位置

| 文件 | 路径 | 用途 |
|------|------|------|
| **Nav2 参数** | `pb2025_nav_bringup/config/simulation/nav2_params.yaml` | 所有导航系统参数 |
| **局部控制器代码** | `pb_omni_pid_pursuit_controller/src/omni_pid_pursuit_controller.cpp` | 控制算法实现 |
| **地形分析代码** | `terrain_analysis/src/terrain_analysis.cpp` | 障碍物检测算法 |
| **启动文件** | `rmu_gazebo_simulator/launch/bringup_sim.launch.py` | 仿真启动脚本 |

---

**最后更新**: 2025-11-15  
**作者**: AI 调参指南生成器
