# 通过 Launch 放置无人船与无人机

两类地图均只保存环境、码头和障碍物。无人船和无人机由 Demo 的 `gazebo_ros/spawn_model` 节点生成，世界文件中没有机器人实例，也没有机器人出生位姿。

| 入口 | 默认地图 | 内容 |
| --- | --- | --- |
| `demo_usv.launch` | `usv_path_planning/map1-3.world` | Gazebo、一个 `wamv` |
| `demo_uav_usv.launch` | `uav_usv_coop/map1.world` | Gazebo、一个 `wamv`、一个 `uav_1`、PX4、MAVROS |
| `world_text.launch` | `usv_path_planning/map1-3.world` | 仅环境预览，不生成机器人 |

原 `simulation_dock_world.launch`、`simulation_empty_world.launch`、`uav_camera_test.launch` 和旧 `core/usv_spawn.launch` 保留为历史测试文件。本次没有整理它们；其中引用已删除 `worlds/extra` 文件的旧入口仍不可直接使用。新场景使用上表中的 Demo。

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
| `uav_x`、`uav_y`、`uav_z` | `-584.75`、`177`、`1.75` |
| `uav_roll`、`uav_pitch`、`uav_yaw` | `0`、`0`、`0` |
| `interactive` | `false`，PX4 非交互模式 |
| `fcu_url` | `udp://:14540@localhost:14557` |
| `gcs_url` | 空字符串 |

实例名固定为 `wamv`、`uav_1`，本次不提供多机端口或实例名切换。船使用已有四推进器及全传感器 SDF，无人机使用 `lite_quad_uav` SDF，PX4 机型保持 `iris`。

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
