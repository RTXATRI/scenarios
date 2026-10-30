# lite_quad_uav 仿真模型与验证

模型、网格、测试世界和仿真入口统一归本包维护。`ocean` 是实验室名称。
控制与感知程序仍在 `lite_quad_uav` 包；本包加载模型不需要启动该控制包。

## 环境与启动

使用已构建的 ROS Noetic、依赖工作空间和 PX4 Gazebo Classic。以下示例在
`ocean_lab_robotics/` 根目录执行；只设置当前终端环境，不修改 shell 配置：

```bash
source /opt/ros/noetic/setup.bash
source depend_ws/devel/setup.bash
source catkin_ws/devel/setup.bash --extend
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PWD/external/PX4-Autopilot:$PWD/external/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

rospack find ocean_simulation_gazebo
rospack find px4
rospack find wamv_gazebo
```

`--extend` 保留依赖工作空间环境，避免旧的主工作空间缓存使 WAM-V/VRX 包不可见。
需要已构建的 `px4_sitl_default` 和 `build_gazebo-classic` 插件，以及 Gazebo ROS 相机插件。
这些入口显式设置模型、插件及共享库路径并保留既有路径，不自动下载或编译依赖。
修改 SDF、网格或 launch 后重新启动仿真即可，不需要为静态资源修改全量编译。

先退出其他 Gazebo/PX4 仿真实例，避免服务及 TCP 4560 等端口冲突。三个入口分别运行：

```bash
# 码头：保留无人船、MAVROS 和原放置位置
roslaunch ocean_simulation_gazebo simulation_dock_world.launch

# 最简验证：仅 Gazebo、PX4 和无人机
roslaunch ocean_simulation_gazebo uav_camera_test.launch

# 空场景：保留原 AprilTag，使用同一份新无人机模型
roslaunch ocean_simulation_gazebo simulation_empty_world.launch
```

每次只运行其中一个入口。码头和空场景继续支持 `uav_sdf:=...` 覆盖默认模型。

| 名称或参数 | 含义 |
|---|---|
| `lite_quad_uav` | Gazebo 模型资源名，完整机体包含下视相机 |
| `uav_1` | 默认 Gazebo 实例名，也是相机及码头 MAVROS 的 ROS 前缀 |
| `iris` | PX4 机型配置；不要因模型资源改名而改为 `lite_quad_uav` |
| 码头位置 | `(-584.75, 177, 1.75)`，RPY 为零 |
| 最简验证位置 | `(0, 0, 0.3)` |
| 空场景位置 | `(0, 0, 0.2)`，原 AprilTag 在 `(5, 5, 0.055)` |

Gazebo 模型列表显示 `uav_1` 是预期行为，其加载的资源来自 `lite_quad_uav`。
最简入口保留参数：`gui=true`、`verbose=false`、`uav_namespace=uav_1`、`uav_z=0.3`、
`px4_dir=$(find px4)`。`px4_dir` 设置数据和插件路径；ROS 仍须能定位 PX4 可执行文件。
修改实例名不会自动分配多机端口。

## 相机与位姿检查

在加载相同 ROS 环境的另一终端运行：

```bash
rostopic echo -n 1 /uav_1/usb_cam/camera_info
rostopic hz /uav_1/usb_cam/image_raw
rosrun rqt_image_view rqt_image_view
```

在图像窗口选择 `/uav_1/usb_cam/image_raw`。预期 640×360、目标频率 30 Hz；实际墙钟频率
受仿真速度、渲染负载与订阅状态影响。CameraInfo 的主点为 `(320, 180)`，焦距
`477.296554`，零畸变，图像帧标识沿用 `robot_camera_link`。

```bash
rosservice call /gazebo/get_link_state "{link_name: 'uav_1::fpv_cam::link', reference_frame: 'uav_1::iris::base_link'}"
rostopic hz /clock
```

相机相对机体平移应为零，姿态约为绕 Y 轴旋转 π/2；四元数约为
`(0, 0.707107, 0, 0.707107)`，整体取反也等价。相机链路 +X 朝向机体 -Z。
这不是可以直接套用到感知程序的完整 ROS 光学外参；本次没有调整感知参数、TF 或控制程序。
另检查 PX4 日志中的仿真连接，以及码头入口 `/uav_1/mavros/state` 的连接状态。

入口不自动解锁或起飞。未解锁模型会自然落地；近裁剪距离保留为 0.1 米，落地后近处地面
可能不可见。可使用 Gazebo 平移工具临时抬高观察，动态模型仍会下落；持续悬空需后续接入控制。

## 模型维护与清理

完整模型位于 `models/vehicle/lite_quad_uav/`：

- `model.sdf`：组合关系和唯一相机安装 pose，当前零平移、下视。
- `base/model.sdf` 与 `base/meshes/`：机体与网格，保持原有动力学和插件。
- `camera/model.sdf`：640×360、30 Hz 相机及内参。
- 内部实例名为 `iris`、`fpv_cam`；组件 URI 为 `model://lite_quad_uav/base`、
  `model://lite_quad_uav/camera`，网格 URI 为 `model://lite_quad_uav/base/meshes/...`。

直接维护 SDF，不接入 Jinja。来源、版本、历史摘要及许可证见
[`UPSTREAM.md`](../models/vehicle/lite_quad_uav/UPSTREAM.md)。
外部 `model://gps` 和已编译的 PX4 插件仍为依赖。GPS 的 `Gazebo/Black` 材质脚本缺少 URI，
本机 SDFormat 完整 DOM 检查会报告这条已有问题，离线展开成功不代表完整校验无告警。

旧控制包中的三份测试模型、测试 launch、专用 world 和说明，以及场景包旧 `iris`、
`iris_fusion`、`fpv_cam` 副本在新入口检查后删除；未删除 PX4 默认模型快照、外部 PX4
或其他机器人资源。原 `roslaunch lite_quad_uav uav_camera_test.launch` 命令已退役，不留转发入口。
旧 `square_patrol.launch` 的可选仿真分支本次保留兼容，故控制包暂留该分支的 Gazebo/PX4 依赖。

验收需分别记录离线模型与 launch 检查，以及实际 Gazebo 渲染、PX4 连接、相机图像检查。

## 本次验证记录（2026-10-29）

- 离线模型展开、两种搜索路径顺序、三个入口的参数解析和场景包实际安装检查通过。
  旧资源删除后再次复查；机体网格、控制代码、控制参数、码头世界和其他机器人资源保持一致。
- 最简入口和码头入口均以 `gui:=false`、独立 ROS/Gazebo master 端口实际运行。
  两者都完成 PX4 启动并收到多个不同时间戳的 640×360 图像，输出 K 与上述参数一致。
  实测相机平移接近零，成像方向为机体 -Z；码头中 `uav_1`、`usv_1` 均存在，MAVROS 已连接。
  测试结束后已关闭本次启动的进程。GUI 显示和长时间运行可按上面的正常启动命令继续验收。
- 当前环境的原有 WAM-V 配置还报告三个缺失插件：`libgazebo_ros_velodyne_gpu_laser.so`、
  `libhector_gazebo_ros_gps.so`、`libhector_gazebo_ros_imu.so`。本次保留无人船配置，未安装新依赖；
  无人船传感器的完整验收还需要补齐这些库。
- 相机插件有焦距/HFOV 一致性提示：展开后的 HFOV 为 `1.18122`，对应焦距约 `477.298866`，
  与保留值 `477.296554` 相差约 `0.0023` 像素。输出 K 已核对，本次按约定保留相机参数，
  后续标定时可一并核对这一精度提示。
