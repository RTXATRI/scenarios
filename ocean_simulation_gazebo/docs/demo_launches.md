# 通过 Launch 放置无人船与无人机

两类地图均只保存环境、码头和障碍物。无人船和无人机由 Demo 的 `gazebo_ros/spawn_model` 节点生成，世界文件中没有机器人实例，也没有机器人出生位姿。

当前六张地图和两份基础Xacro中的环境物体全部静态放置：码头、浮标、小球已移除浮力及运动关节，平台、岸线和地面设施继续复用静态模型。船和无人机保留原动力学，海浪场仍供船体插件使用。

2018码头的两块红墙和两块绿块均为静态实体障碍，参与船体碰撞、相机成像和雷达检测；`std_usv` 的已知地图同步包含这些几何，绿色区域不可通航。2022码头原本未启用的示意区域保持现状。修改模型后需重新启动仿真，已运行的Gazebo不会自动重载SDF。

静态浮标可以由位姿服务显式移动。本包提供 `navigation_dynamic_buoys.py`，由 `std_usv demo_astar_dwa.launch dynamic_obstacles:=true` 在原支持地图上启用，默认0.7m/s、20Hz，只改变XY并保留地图初始高度和姿态；暂停时停止移动，关闭脚本后停留在最后位置。该参数不切换模型的物理静态属性。

## 仿真辅助工具与包职责

本包提供世界与模型、浮标脚本运动、一次性倍率设置和基于SDF的导航资源生成；`std_usv` 负责无人船的感知、定位、决策、控制、地图发布及实验编排。仿真工具不查找或依赖 `std_usv`。

| 工具 | 用途 |
| --- | --- |
| `navigation_dynamic_buoys.py` | 在支持地图上按仿真时间更新静态浮标的XY，保留高度和姿态 |
| `set_simulation_speed.py` | 设置并回读Gazebo物理更新率，保留步长、重力、ODE和暂停状态，完成后退出 |
| `generate_navigation_maps.py` | 从既有世界和模型生成两套已知栅格、来源清单及艇体轮廓／惯性配置 |

三个脚本已从 `std_usv` 迁入本包，文件名不变，旧包不保留兼容入口。倍率工具仍只由完整导航实验调用；本包的纯无人船、协同和环境预览入口不自动启用倍率或浮标运动。

生成工具要求显式指定两个输出路径。例如维护 `std_usv` 现有导航资源：

```bash
rosrun ocean_simulation_gazebo generate_navigation_maps.py \
  --output-dir "$(rospack find std_usv)/maps" \
  --model-output "$(rospack find std_usv)/config/wamv_model.yaml" \
  --check
```

调用方决定文件放置位置，两个路径也可以指向临时目录；`--check` 不创建目录、不改写文件，去掉它才生成。几何实现同时作为只读资源安装到 `share/ocean_simulation_gazebo/scripts/`，供导航发布节点在源码和安装环境中加载；ROS可执行入口按catkin规则安装在 `lib/ocean_simulation_gazebo/`。

| 入口 | 默认地图 | 内容 |
| --- | --- | --- |
| `demo_usv.launch` | `usv_path_planning/map1-3.world` | Gazebo、一个 `wamv` |
| `demo_uav_usv.launch` | `uav_usv_coop/map1.world` | Gazebo、一个带顶部 AprilTag 的 `wamv`、一个 `uav_1`、PX4、MAVROS |
| `demo_uav_usv_circle.launch` | `uav_usv_coop/map1_circle.world` | 清空水面的场景、绕圈及摇晃的 `wamv`、岸边 `uav_1`、PX4和根命名空间MAVROS |
| `world_text.launch` | `usv_path_planning/map1-3.world` | 仅环境预览，不生成机器人 |

原 `simulation_dock_world.launch`、`simulation_empty_world.launch`、`uav_camera_test.launch` 和旧 `core/usv_spawn.launch` 保留为历史测试文件。本次没有整理它们；其中引用已删除 `worlds/extra` 文件的旧入口仍不可直接使用。新场景使用上表中的 Demo。

## 清空水面的绕圈目标场景

新场景保留原岸线、三个岸上柱子、天线、三个地面站及海面，共9个环境模型；移除水中码头及红绿块、平台、浮标和小球。船与无人机仍由launch生成，启动后共有11个顶层模型。原有地图和Demo保持原样。

第一个终端加载下方“环境与启动”中的工作区环境，然后启动：

```bash
roslaunch ocean_simulation_gazebo demo_uav_usv_circle.launch

# 关闭Gazebo窗口，或调整轨迹及摇晃。
roslaunch ocean_simulation_gazebo demo_uav_usv_circle.launch \
  gui:=false circle_radius:=4 circle_speed:=0.4 clockwise:=true \
  roll_amplitude:=0.02 pitch_amplitude:=0.01 heave_amplitude:=0.02
```

两条场景命令择一运行。新入口只启动仿真、PX4、一个MAVROS和船运动脚本，不启动无人船导航、无人机控制系统，也不自动解锁或起飞。

第二个终端加载相同工作区环境，独立启动无人机控制核心：

```bash
roslaunch lite_quad_uav uav_core_start.launch \
  start_mavros:=false mavros_ns:=/mavros
```

需要现有方形巡航任务时，第二个终端改用以下命令；它自身包含控制核心，不与上一条同时运行：

```bash
roslaunch lite_quad_uav square_patrol.launch \
  start_sim:=false start_mavros:=false \
  mavros_ns:=/mavros auto_start_task:=false
```

等待控制系统就绪后，任务仍由用户显式启动：

```bash
rosservice call /uav/task_manager/start_task \
  "task_type: 'square_patrol'
task_params_json: ''"
```

新场景将MAVROS直接置于 `/mavros`，对齐lite控制包已有的 `mavros_ns` 参数；无需修改其C++源码或增加话题中转。`/uav/...` 继续作为控制系统内部接口。

| 用途 | 新场景接口 |
| --- | --- |
| 飞控状态 | `/mavros/state` |
| 本地位置／速度 | `/mavros/local_position/pose`、`/mavros/local_position/velocity_local` |
| IMU／电池 | `/mavros/imu/data`、`/mavros/battery` |
| 速度指令 | `/mavros/setpoint_velocity/cmd_vel_unstamped` |
| 模式／解锁服务 | `/mavros/set_mode`、`/mavros/cmd/arming` |
| 下视相机 | `/uav_1/usb_cam/image_raw`、`/uav_1/usb_cam/camera_info` |

原 `demo_uav_usv.launch` 的 `/uav_1/mavros/...` 接口保持不变。新场景默认飞控URL仍为 `udp://:14540@localhost:14557`，`gcs_url` 为空；一次运行一个仿真。

| 轨迹和出生参数 | 默认值 |
| --- | --- |
| `circle_center_x / circle_center_y` | `-570 / 180` m |
| `circle_radius / circle_speed` | `5 m / 0.5 m/s`，一圈约62.83秒仿真时间 |
| `clockwise` | `false`，逆时针 |
| `usv_z` | `0.1 m` |
| `roll_amplitude / pitch_amplitude` | `0.0349066 / 0.0174533 rad`，约2°／1° |
| `heave_amplitude` | `0.03 m` |
| `roll_period / pitch_period / heave_period` | `6 / 8 / 7 s` |
| `motion_rate` | `50 Hz`，监督节点的墙钟心跳频率；轨迹由插件按物理步更新 |
| `uav_x / uav_y / uav_z` | `-584.75 / 177 / 0.75 m`，岸边地面上方 |
| `uav_roll / uav_pitch / uav_yaw` | `0 / 0 / 0 rad` |
| `world / gui / paused / verbose` | 新世界文件／`true / false / false` |

默认船从圆的东侧 `(-565,180,0.1)` 开始，朝向切线；修改圆心、半径或方向时，出生位姿由同一组参数自动推导。默认完整艇体轮廓加2m余量已检查不与岸线重叠；自定义轨迹应避开岸线。

绕圈入口通过 `-s libwamv_circle_motion_plugin.so` 加载本包的Gazebo 11系统插件。插件在每个物理步计算位置、四元数及解析速度，不发送推进器指令。`/wamv/simulation/circle_motion` 保留为Python监督节点，负责参数校验、启动、墙钟心跳和正常停止。这是规则运动目标，保留共用船模和传感器；真实推进器或水动力控制测试仍使用原Demo。`circle_speed:=0` 可保持水平位置，仅摇晃；三种幅度设为0可关闭摇晃。参数必须有限，半径／周期／频率为正，速度和幅度非负。

首次使用修复后的入口需要构建插件：

```bash
catkin_make --pkg ocean_simulation_gazebo -j2
source devel/setup.bash --extend
roslaunch ocean_simulation_gazebo demo_uav_usv_circle.launch
```

插件接管时缓存所有link相对船体的装配位姿和原有运动学模式，随后在物理更新锁保护下统一重建每个link的世界位姿。各部件质心速度分别设置为 `v_ref + ω × (p_CoG - p_ref)`，避免整模型速度接口把相同线速度广播到全部link而造成错位。暂停、恢复和运动均不修改磁盘上的共用SDF；其他Demo不加载该插件。

更新发生在模型插件执行之后、ODE物理积分之前。接管期间同步清除各link的外力和力矩，防止水动力插件在时钟复位时产生的非有限力污染运动学部件。正常停止后恢复原模式和原模型插件的物理运动，不修改水动力源码或船模配置。

监督节点内部使用三个 `std_srvs/Trigger` 服务：`/wamv/simulation/circle_motion/start`、`keepalive`、`stop`。插件读取同一节点命名空间下的原有轨迹参数，因而启动命令和参数名称保持兼容。

`paused:=true` 保持场景暂停，插件和监督节点均不会解除暂停；恢复后运动继续。启动就绪和接管共用最多30秒墙钟时间；健康检查及停止RPC上限为1秒。暂停时心跳仍按墙钟运行，停止服务可以直接归零速度并恢复原link模式。仿真时钟回退后，轨迹从起点重新计时。

插件的时间回退行为在独立无人船实例中验证。带PX4的完整场景调用 `/gazebo/reset_simulation` 后，本环境出现外部锁步时钟不再推进的现象；需要从头开始完整船机场景时，应重启该场景及SITL。本次不修改PX4或无人机控制代码。

正常关闭监督节点、监督节点失联或插件卸载时，插件归零各link速度并恢复接管前模式，随后船恢复原模型物理运动。心跳超时为 `max(2 秒, 3/motion_rate)`，默认2秒；强制终止监督进程也会触发恢复。恢复过程不改变Gazebo暂停状态。

```bash
# 正常停止船的脚本运动；不会关闭Gazebo或无人机控制系统。
rosnode kill /wamv/simulation/circle_motion
```

出生位姿使用Gazebo世界坐标，无人机控制目标使用MAVROS本地ENU坐标，两者原点不同。本场景不自动启动AprilTag检测、跟船或降落算法。船仍使用 `wamv_apriltag`，标记为tag36h11、ID 3、黑色外边0.40m。移除独立弹丸后，原射球插件会提示 `blue_projectile` 不存在并退出初始化；该未启用功能不影响船体、相机或无人机。

### 推进器装配修复验收

数学测试随C++插件编译，Python回归继续覆盖launch、监督节点、地图和原有工具：

```bash
cmake --build build --target test_wamv_circle_motion_math -j2
devel/lib/ocean_simulation_gazebo/test_wamv_circle_motion_math
python3 -m unittest discover -s src/scenarios/ocean_simulation_gazebo/test -p 'test_*.py' -v
```

launch解析测试需要本节下方配置的PX4资源路径。catkin缓存的测试环境可能覆盖额外的 `ROS_PACKAGE_PATH`；出现 `ResourceNotFound: px4` 时，在已加载PX4路径的终端运行上面的Python命令。

以下显式运行的Gazebo检查使用独立ROS／Gazebo master，保存截图、每次采样和汇总报告，退出时仅清理自己创建的进程。需要已加载上述工作区环境、OpenGL显示及提供AprilTag字典的OpenCV ArUco模块：

```bash
# 默认两圈：同一world、船模、插件与监督节点，不加载PX4；覆盖时钟复位。
python3 src/scenarios/ocean_simulation_gazebo/test/circle_motion_runtime_check.py \
  --output-dir /tmp/wamv-circle-check --duration 126

# 完整协同入口：另验证PX4、MAVROS及监督节点重启；不对PX4执行时间归零。
python3 src/scenarios/ocean_simulation_gazebo/test/circle_motion_runtime_check.py \
  --output-dir /tmp/wamv-circle-full-check --full-demo --duration 15

# --package-path 可指定 install/share/ocean_simulation_gazebo 验证已安装资源。
```

检查对全部13个link采样，比较相对装配、轨迹半径和姿态；从SDF读取惯性质心偏移，把Gazebo `LinkStates` 的link原点速度换算为质心速度后，再与解析刚体速度比较。不能直接把link原点速度与质心速度混比。实测数值、外观、船顶AprilTag图像及压缩采样记录见 [修复验收记录](circle_motion_validation/README.md)。

### 绕圈场景早期验证记录（2026-11-09，修复前）

以下记录检查的是整船轨迹，没有覆盖各link相对船体的装配位姿。后续已复现推进器相对船体的累积漂移，不能以这组整船指标作为装配正确性的依据。

- 本包59项自动检查通过；构建、安装及devel／install下的入口定位、默认／自定义launch解析、SDF检查通过。原有世界、模型和Demo与本轮开始时逐文件一致，`std_usv`、`lite_quad_uav` 源码未改动。
- 无界面、暂停启动后恢复，按 `GetModelState` 返回的仿真时间戳采样4557次，连续运行126.042秒仿真时间，完成两圈；暂停期间时钟和位姿保持不变。

| 两圈实测指标 | 最大误差 |
| --- | --- |
| 半径 | `0.000218 m`（验收上限 `0.2 m`） |
| 切线航向 | `0.000136 rad`（验收上限 `0.05 rad`） |
| roll／pitch正弦相位对应角度 | `0.009083 / 0.003773 rad` |
| heave高度 | `0.005825 m` |

- 早期仅设置位姿的版本在启动控制进程时出现约24cm的瞬时高度偏差；最终版本采用上述临时运动学接管。正常停止运动脚本后，回读确认13个link均恢复原来的动态模式，Gazebo及11个模型继续保留。
- 无人机未接入控制系统时保持未解锁，岸边稳定落地高度为世界坐标 `z=0.531191 m`；MAVROS连接正常，下视图像为 `640×360 rgb8`，相机标定有效。
- 独立启动方形巡航入口后，确认 `/velocity_manager` 为MAVROS速度话题的唯一发布者；显式启动任务，完成低高度起飞、暂停任务后悬停5秒、取消任务并通过 `/uav/vehicle/command` 请求 `AUTO.LAND`。悬停时本地高度约 `1.85 m`、世界高度约 `2.83 m`，最终落地世界高度约 `0.53219 m` 并解除解锁。已保存下视图像作为本次测试产物。
- install环境另完成15秒仿真时间短测，验证暂停、恢复、船运动、MAVROS和相机。所有自有测试进程均已关闭。

## 环境与启动

在项目的 `catkin_ws` 目录执行：

```bash
source /opt/ros/noetic/setup.bash
source ../depend_ws/devel/setup.bash
source devel/setup.bash --extend

# 纯无人船：不查找、不启动 PX4 或 MAVROS
roslaunch ocean_simulation_gazebo demo_usv.launch
```

依赖工作区需提供 WAM-V、VRX、海浪及动力学插件；三路相机使用 `gazebo_plugins`。GPS、IMU 和 GPU 雷达使用原 Noetic 二进制插件，安装或恢复方式见 [地图1依赖说明](map1_worlds.md#没有-sudo-权限时的本地插件安装)。

协同 Demo 还需要已经构建的 PX4 `px4_sitl_default`、Gazebo Classic 插件和 MAVROS。若当前 ROS 环境尚不能定位项目内 PX4，可在同一终端执行：

```bash
demo_px4="$(realpath ../external/PX4-Autopilot)"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$demo_px4:$demo_px4/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
rospack find px4

roslaunch ocean_simulation_gazebo demo_uav_usv.launch
```

两个 Demo 每次运行一个；协同 Demo 使用现有单机飞控端口。它启动飞控和 MAVROS，不自动解锁、起飞或运行定位、建图、路径规划和协同任务。

相机、GPU 雷达和海面显示需要可用的 OpenGL 环境；`gui:=false` 关闭客户端窗口，仍会渲染传感器图像。GUI 保留原地图总览视角，查看细节时可聚焦模型列表中的 `wamv` 或 `uav_1`。

## 地图与出生位姿

```bash
# 选择其他无人船地图
roslaunch ocean_simulation_gazebo demo_usv.launch \
  world:="$(rospack find ocean_simulation_gazebo)/worlds/usv_path_planning/map1-1.world"

# 修改无人船位置和姿态
roslaunch ocean_simulation_gazebo demo_usv.launch \
  usv_x:=-580 usv_y:=170 usv_z:=0.2 \
  usv_roll:=0 usv_pitch:=0 usv_yaw:=0.5

# 协同场景可分别调整两台机器人
roslaunch ocean_simulation_gazebo demo_uav_usv.launch \
  usv_x:=-580 usv_yaw:=0.5 \
  uav_x:=-585 uav_y:=178 uav_z:=2.2 uav_yaw:=1.57
```

位置相对于 Gazebo 世界坐标系，单位为米；所有角度均为弧度。出生位姿是初始状态，恢复物理后机器人会受动力学影响，无人机不会自动保持出生高度。

| 参数 | 两个 Demo 的默认值 |
| --- | --- |
| `world` | 各自对应的默认地图 |
| `gui`、`paused`、`verbose` | `true`、`false`、`false` |
| `usv_x`、`usv_y`、`usv_z` | `-570`、`167`、`0.1` |
| `usv_roll`、`usv_pitch`、`usv_yaw` | `0`、`0`、`1` |

| 协同入口的额外参数 | 默认值 |
| --- | --- |
| `usv_model` | `wamv_apriltag`；可设为 `wamv` 使用原船 |
| `uav_x`、`uav_y`、`uav_z` | `-584.75`、`177`、`1.75` |
| `uav_roll`、`uav_pitch`、`uav_yaw` | `0`、`0`、`0` |
| `interactive` | `false`，PX4 非交互模式 |
| `fcu_url` | `udp://:14540@localhost:14557` |
| `gcs_url` | 空字符串 |

实例名固定为 `wamv`、`uav_1`，本次不提供多机端口或实例名切换。船使用已有四推进器及全传感器 SDF，无人机使用 `lite_quad_uav` SDF，PX4 机型保持 `iris`。

协同场景默认使用新资源 `wamv_apriltag`：在中央较高设备箱的箱盖上增加朝上的 `tag36h11`、ID 3 视觉靶标。完整贴图为 `0.50 × 0.50 m`，黑色外边为 `0.40 × 0.40 m`，四周各留 `0.05 m` 白边。标记中心相对 `wamv/base_link` 为 `(0.033, 0, 1.7611) m`，仅增加 visual，沿用原船动力学、碰撞体和全部 `/wamv/...` 接口。

```bash
# 切回原船；纯无人船 demo_usv.launch 的默认模型仍为原船。
roslaunch ocean_simulation_gazebo demo_uav_usv.launch usv_model:=wamv
```

识别船顶标记时使用 `tag_size:=0.40`；现有地面标签和感知 launch 的 `0.20 m` 默认值不变。安装尺寸、局部位姿、纹理生成方法和验证图见 [新模型说明](../models/vehicle/wamv_apriltag/README.md)。该改动不启动降落控制算法。

| 地图 | 环境模型数 | 对应 Demo 启动后的模型数 |
| --- | ---: | ---: |
| `usv_path_planning/map1-1.world` | 22 | 23 |
| `usv_path_planning/map1-2.world` | 28 | 29 |
| `usv_path_planning/map1-3.world` | 27 | 28 |
| `usv_path_planning/map2-1.world`，含附加障碍物 | 45 | 46 |
| `usv_path_planning/map2-2.world`，无附加障碍物 | 25 | 26 |
| `uav_usv_coop/map1.world` | 45 | 47 |

仅预览环境时可运行 `world_text.launch world:=...`。直接在 Gazebo 打开这些 `.world` 同样不会出现机器人。

## 公共模块与 Xacro

`core/gazebo_world.launch` 加载世界并配置模型、材质和插件路径；`core/wamv_spawn.launch` 与 `core/uav_spawn.launch` 分别放置机器人。生成节点通过 Gazebo 服务等待就绪，成功后退出，不设置自动重生，也不会解除暂停。`paused:=true` 时需手动点击运行，或调用 `/gazebo/unpause_physics`，传感器和飞控才能随仿真时间运行。

资源通过 ROS 包位置及已加载的工作区环境定位，没有用户名或固定工作区路径。协同场景把 PX4 模型目录放在 WAM-V 依赖之前：`model://gps` 解析为无人机 GPS 模型，`model://gps/mesh/gps.dae` 仍解析为 WAM-V 的网格。已有资源路径继续保留，第三方文件不变。

世界中保留 `ocean_waves` 及原风插件对 `wamv` 的绑定；风插件会在更新时找到后来生成的船。WAM-V 生成请求不重写 SDF 内已配置的 ROS 命名空间，保持全部 `/wamv/...` 接口。

两份基础 `.world.xacro` 也只生成环境。原 `boat_x`、`boat_y`、`boat_z`、`boat_yaw` 参数已经移除；对应位置现在通过 Demo 的 `usv_*` 参数传入。

```bash
rosrun xacro xacro \
  "$(rospack find ocean_simulation_gazebo)/worlds/usv_path_planning/map1_base.world.xacro" \
  -o /tmp/map1-environment.world

roslaunch ocean_simulation_gazebo demo_usv.launch \
  world:=/tmp/map1-environment.world usv_x:=-580 usv_yaw:=0.5
```

启动普通 `.world` 不执行 Xacro，不会覆盖地图文件。

## 检查命令

静态回归检查不需要启动 ROS master：

```bash
cd "$(rospack find ocean_simulation_gazebo)"
python3 -m unittest discover -s test -p 'test_*.py' -v
```

启动对应 Demo 后，在另一个已加载相同 ROS 环境的终端检查：

```bash
rosservice call /gazebo/get_world_properties
rosservice call /gazebo/get_model_state "{model_name: 'wamv', relative_entity_name: 'world'}"
rostopic hz /wamv/sensors/cameras/front_left_camera/image_raw
rostopic echo -n 1 /wamv/sensors/cameras/front_left_camera/camera_info
rostopic hz /wamv/sensors/lidars/lidar_wamv/points
rostopic echo -n 1 /wamv/sensors/gps/gps/fix
rostopic echo -n 1 /wamv/sensors/gps/gps/fix_velocity
rostopic echo -n 1 /wamv/sensors/imu/imu/data
rostopic echo -n 1 /wamv/sensors/pingers/pinger/range_bearing

# 以下用于协同 Demo，恢复物理后检查
rosservice call /gazebo/get_model_state "{model_name: 'uav_1', relative_entity_name: 'world'}"
rostopic echo -n 1 /uav_1/mavros/state
rostopic echo -n 1 /uav_1/mavros/local_position/pose
rostopic hz /uav_1/usb_cam/image_raw
rostopic echo -n 1 /uav_1/usb_cam/camera_info
```

船体其他两路相机为 `front_right_camera` 和 `middle_right_camera`，图像均为 1280×720、30 Hz；雷达 10 Hz，GPS/IMU 15 Hz，pinger 1 Hz。无人机下视图像为 640×360、30 Hz。验收按消息中的仿真时间戳统计频率；窗口渲染速度和墙钟运行速度取决于仿真实时倍率。

完整船体控制和传感器接口见 [地图1说明](map1_worlds.md#控制和传感器接口)，无人机模型参数见 [相机模型说明](uav_camera_model.md)。

### 四推进器运动检查

在已恢复物理的无人船 Demo 中执行以下脚本，向四个推进器发送 `0.2` 命令 6 个仿真秒，再发送零命令 8 个仿真秒。预期船沿船艏前进约 1 米，归零后速度下降；不要同时运行其他推力发布节点。

```bash
python3 - <<'PY'
import rospy
from std_msgs.msg import Float32

rospy.init_node('demo_thrust_check')
publishers = [rospy.Publisher('/wamv/thrusters/' + side + '_thrust_cmd',
                              Float32, queue_size=1)
              for side in ('left_front', 'right_front', 'left_rear', 'right_rear')]
rospy.sleep(1)
rate = rospy.Rate(20)
for command, seconds in ((0.2, 6), (0, 8)):
    end = rospy.Time.now() + rospy.Duration(seconds)
    while not rospy.is_shutdown() and rospy.Time.now() < end:
        for publisher in publishers:
            publisher.publish(Float32(command))
        rate.sleep()
PY
```

## 本次验证记录（2026-11-04）

- 15 项 Python unittest 通过，涵盖纯环境地图、当前地图文件名、两个 Demo 的实际 launch 图、默认及自定义六自由度参数、暂停参数、固定实例名和协同资源优先级。纯无人船入口也通过了禁止解析 PX4/MAVROS 包的检查。
- 六张世界及两份 Xacro 的生成结果通过本机 libsdformat 的解析和 DOM 检查。与修改前逐一比较，环境 XML 只移除了机器人引用及模板参数；全部既有 launch、船机 SDF 的校验值保持一致。新入口、地图、模板及文档的 CMake 安装检查通过。
- `demo_usv.launch` 加载 `map1-1.world` 已实际运行：23 个顶层模型、唯一 `wamv`、全部初始位姿及暂停状态正确，船正常漂浮。三路相机及标定约 30.30 Hz，雷达 10 Hz，GPS及速度、IMU 约 14.93 Hz，pinger 约 1 Hz，均收到有效连续数据。
- 同次实测四推进器发送 `0.2` 命令 6 个仿真秒，船沿船艏前进 1.017 米；再发送零命令 8 个仿真秒，速度由 0.260 m/s 降至 0.054 m/s。测试进程随后关闭。
- 其余四张无人船地图和协同场景的实跑、自定义位姿的运行检查及 PX4/MAVROS 联通检查，按用户要求改为本地手动验证；上文给出了检查命令。本次未将这些场景标为实跑通过。

额外的严格 DOM 检查发现，共用 WAM-V 的 `wamv/imu_wamv_link` 原有惯性定义会触发 invalid inertia 诊断；已对照修改前文件确认同样存在。本次按计划保留模型质量与惯性，Gazebo 实跑中的 IMU 数据有效。该项与世界文件检查分开记录。

## 船顶 AprilTag 模型验证（2026-11-07）

协同入口现默认使用 `wamv_apriltag`。新旧模型通过 SDF 检查，除新模型资源名和增加的标签 visual 外，原船结构与配置一致；现有 19 项场景检查通过。已实际运行协同地图，确认 `wamv`、`uav_1` 存在且 MAVROS 连接正常。与无人机相同内参的固定相机在名义 3 m、5 m 视距均识别出 ID 3；船体移动约 0.360 m 后仍能识别。检测参数、截图和实测数据见 [新模型验证记录](../models/vehicle/wamv_apriltag/README.md#gazebo-验证记录2026-11-07)。
