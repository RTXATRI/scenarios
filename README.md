<!--
    文档说明

    Author: WYB
    Email: 854459949@qq.com
    Year: 2026

    Copyright (C) 2026 Ocean Information and Intelligent Equipment Laboratory
-->

# scenarios — 仿真场景资源说明

`catkin_ws/src/scenarios` 是 ocean_lab_robotics 项目的仿真场景层，存放两款仿真器的场地、机器人模型、场景描述与启动入口。本目录为独立 git 仓库并计划发布到 GitHub，第三方资源复用声明见第 3 节。

| 包 | 仿真器 | 用途 |
| --- | --- | --- |
| `ocean_simulation_gazebo` | Gazebo classic（ROS Noetic） | 无人船（WAM-V）路径规划场地、无人机（PX4 SITL）及船机协同场景 |
| `ocean_simulation_stonefish` | StoneFish | 浅水下潜器 under_usv 的水下仿真与姿态控制 |

## 1. 目录内容

### ocean_simulation_gazebo

| 路径 | 内容 |
| --- | --- |
| `worlds/usv_path_planning/` | 地图1 / 地图2 系列无人船路径规划场地 |
| `worlds/uav_usv_coop/` | 船机（USV + UAV）协同场地 |
| `worlds/px4-default/` | PX4 SITL 官方世界文件副本（19 个） |
| `launch/` | 仿真入口，`core/` 为 Gazebo、PX4、机器人 spawn 的组装件 |
| `models/vehicle/` | `wamv`、`lite_quad_uav`、`typhoon_h480`、`usv` 模型 |
| `models/sensor/`、`models/other/` | 传感器与场地辅助模型（波场、码头、AprilTag 等） |
| `models/px4_default/` | PX4 模型库副本（89 个模型） |
| `test/` | 地图与模型回归测试（unittest） |
| `docs/` | 场景与 demo 说明文档 |

### ocean_simulation_stonefish

| 路径 | 内容 |
| --- | --- |
| `worlds/ocean.scn.xacro` | 水下场景：NED 坐标、海水与波浪参数、`under_usv` 实例化 |
| `models/` | `under_usv` 场景描述与材质定义 |
| `meshes/` | 机器人与场地网格（hull、prop、duct、tank 等） |
| `src/`、`include/pid/` | USV 控制节点（手动 / 自稳 / 定深 / 定点 / 回坞 PID 控制） |
| `launch/simulation.launch` | StoneFish 仿真入口 |
| `scripts/` | 键盘直控推进器测试脚本 |

## 2. 运行与入口

本仓库不是自包含项目，运行依赖父项目的 `depend_ws/`（VRX 与 mvp 系列包）、`external/`（PX4-Autopilot、StoneFish）及系统 apt 包（Gazebo、gazebo_ros、mavros、hector/velodyne 插件等），source 顺序与安装命令见 [ocean_simulation_gazebo/docs/](ocean_simulation_gazebo/docs/)。

| 入口 | 内容 |
| --- | --- |
| `demo_usv.launch` | Gazebo + `wamv`（默认 `map1-3.world`） |
| `demo_uav_usv.launch` | Gazebo + `wamv` + `uav_1`（PX4 SITL + MAVROS） |
| `world_text.launch` | 仅加载环境预览 |
| `simulation.launch` | StoneFish 水下仿真 |

## 3. 第三方资源复用声明

### 3.1 复用原则

所有第三方资源的著作权及相关权利归其原作者/组织所有。本仓库在各自许可证（BSD-3-Clause、Apache-2.0、GPL-3.0）允许的范围内复制、修改与使用，保留原始版权与许可声明，不以修改为由变更归属；对上游资源的本地修改由实验室完成并就此负责。

### 3.2 随本仓库分发的第三方资源

| 本仓库位置 | 上游来源 | 许可证 | 复用方式 |
| --- | --- | --- | --- |
| `ocean_simulation_gazebo/models/px4_default/`、`worlds/px4-default/`、`ocean_simulation_stonefish/include/pid/pid.h` | [PX4-SITL_gazebo-classic](https://github.com/PX4/PX4-SITL_gazebo-classic)、PX4 Development Team | BSD-3-Clause | 原样副本，版权声明保留 |
| `ocean_simulation_gazebo/models/vehicle/lite_quad_uav/`、`models/vehicle/typhoon_h480/` | PX4 `iris`、`fpv_cam`、`typhoon_h480`（来源细节见 `lite_quad_uav/UPSTREAM.md` 与 `LICENSE.PX4`） | BSD-3-Clause | 修改后自维护（相机参数、图像发布等） |
| `ocean_simulation_gazebo/models/vehicle/wamv/`、`models/other/map1_ocean/`、`models/other/map1_dock_2022/`、`worlds/usv_path_planning/`、`worlds/uav_usv_coop/` | VRX（[osrf/vrx](https://github.com/osrf/vrx)）`wamv_description`、`wave_gazebo`、`vrx_gazebo` | Apache-2.0 | 派生 SDF 与场地布局改造；网格与子模型经 `model://` 引用外部包，不随本仓库分发 |

### 3.3 运行时依赖（不随本仓库分发）

| 上游项目 | 许可证 | 在本项目中提供 |
| --- | --- | --- |
| [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | BSD-3-Clause | SITL 飞控固件 |
| VRX 全套（`vrx_gazebo`、`wamv_*`、`wave_gazebo*`、`usv_*`） | Apache-2.0 | WAM-V 网格、波场、浮力/推进动力学、`sydney_regatta` 环境模型 |
| [StoneFish](https://github.com/patrykcieslak/stonefish)（本地使用 uri-ocean-robotics fork） | GPL-3.0 | 水下仿真本体 |
| `stonefish_mvp`、`mvp_*`、`alpha_std_auv`（URI SOS Lab） | GPL-3.0 | StoneFish 的 ROS 包装与控制栈 |
| Gazebo、`gazebo_ros`、hector/velodyne 插件、`mavros` 等 apt 包 | 各自许可证 | 仿真与传感运行时 |

StoneFish 与 `mvp_*` 系列按 GPL-3.0 发布。本仓库不包含其源码，仅以独立进程经 ROS 接口启动，不被 GPL-3.0 传染；若将其代码链接进自有程序或随仓库分发其二进制，须遵守 GPL-3.0。

### 3.4 实验室自有资源

以下内容为实验室自制，版权归实验室：launch 入口、回归测试、`docs/` 文档与截图；`models/vehicle/usv/`、`models/sensor/`、`models/other/` 下除 3.2 表所列外的模型；`ocean_simulation_stonefish` 的场景描述、机器人模型、网格、控制节点（`pid.h` 除外）与测试脚本。

### 3.5 责任与免责声明

1. 第三方资源的著作权归其原作者/组织所有，本仓库的复用不改变其归属，再分发按各自许可证履行条件。
2. 对上游资源的修改由实验室完成并就此负责，修改版不代表上游官方版本。
3. 本仓库以学习与科研为目的，按"现状"提供，不作任何明示或默示担保，使用者自行承担使用风险。
4. 如有权利人认为本仓库内容侵犯其权益，请通过 GitHub Issue 或文首邮箱联系，核实后将及时处理。
5. 仓库整体许可证以根目录 `LICENSE` 为准，仅约束实验室自有部分，第三方资源始终遵循其原始许可证。

### 3.6 致谢

- [PX4 Development Team](https://github.com/PX4)（BSD-3-Clause）
- [VRX / osrf](https://github.com/osrf/vrx)（Apache-2.0）
- StoneFish 作者 Patryk Cieślak 及 [URI Smart Ocean Systems Laboratory](https://github.com/uri-ocean-robotics)（GPL-3.0）；使用 StoneFish 的研究请引用其 OCEANS 2019 论文
- ROS 与 Gazebo 社区
