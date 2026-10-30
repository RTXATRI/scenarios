# 无人机模型来源与维护

这些模型是项目独立维护的资源副本，`model.sdf` 是可直接编辑的源文件。
没有复制 Jinja 模板或 `.last_generated` 文件，也没有新增自动生成或下载任务。

## 来源快照

整理日期：2026-10-29。以下路径相对 `ocean_lab_robotics/`。

| 最初副本名称 | 最初复制来源（源目录现已退役） |
|---|---|
| `ocean_iris_base` | `catkin_ws/src/scenarios/ocean_simulation_gazebo/models/vehicle/iris/iris.sdf` 及 `meshes/` |
| `ocean_down_camera` | `catkin_ws/src/scenarios/ocean_simulation_gazebo/models/sensor/fpv_cam/fpv_cam.sdf` |
| `ocean_iris_camera` | 参照现有 `iris_fusion` 的组合关系重新编写 |

基础 Iris SDF 和三份网格与以下外部快照一致：

- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)，本地版本 `v1.15.1`，提交 `93eef0c78719a9ac972d46593262c76c978ed3c4`。
- [PX4-SITL_gazebo-classic](https://github.com/PX4/PX4-SITL_gazebo-classic)，提交 `da7206e057703cc645770f02437013358b71e1c0`，模型位于 `models/iris/` 和 `models/fpv_cam/`。
- Iris 原始作者信息来自上游 `model.config`：Thomas Gubler、Lorenz Meier。

本地相机已有定制，并非外部 `fpv_cam` 的原样副本：640×360、30 Hz，
水平视场角 `1.181224482`，焦距 `477.296554`，主点 `(320, 180)`，零畸变。
外部快照的相机为 320×240，另有 GStreamer 插件；本副本保留本地定制版本。

## 当前结构与维护方式

2026-10-29：模型从控制包迁入 `ocean_simulation_gazebo/models/vehicle/lite_quad_uav/`。
完整资源名为 `lite_quad_uav`，Gazebo 默认实例仍为 `uav_1`。

- 顶层 `model.sdf` 通过 `model://lite_quad_uav/base` 和 `model://lite_quad_uav/camera` 引用组件。
- 基础机体和网格位于 `base/`，相机位于 `camera/`；网格 URI 为 `model://lite_quad_uav/base/meshes/...`。
- 保留基础机体的链接、旋翼、关节及 PX4 插件配置；网格字节内容保持一致。
- 相机自身 pose 为零，安装 pose 仅在顶层声明：零平移、绕 Y 轴旋转 π/2。
- 组合内部实例名为 `iris`、`fpv_cam`，固定关节沿用这两个实例的链接。
- 元数据统一使用 SDF 1.6 和 `model.sdf` 文件名，直接维护 SDF，不接入 Jinja 生成。

基础机体仍依赖外部 `model://gps` 和已编译的 PX4 Gazebo 插件。
最初的控制包测试副本及场景包旧 `iris`、`iris_fusion`、`fpv_cam` 目录在新入口验证后删除。
下方来源路径和摘要保留作历史溯源，不是当前运行入口。

## 许可证

这些模型从原控制包迁出时保留原有许可证和作者信息。复制资源的上游 `PX4-SITL_gazebo-classic/package.xml`
声明 BSD，父仓库 `PX4-Autopilot/LICENSE` 为 BSD 3-Clause；后者原文保存在
[`LICENSE.PX4`](LICENSE.PX4)。保留原始作者信息，模型副本不改标为 MIT。
上游子仓库当前根目录没有单独的 LICENSE 文件；此处记录两层来源及原始声明，
后续更新模型时应一并核对来源说明和许可证文件。


## 本次复制来源的 SHA-256

| 来源（相对场景包 models） | SHA-256 |
|---|---|
| `vehicle/iris/iris.sdf` | `141ac823262a56813605b7d7512d225277691478401ca0e1cb81029bfbfcf7f4` |
| `vehicle/iris/meshes/iris.stl` | `11a9a1453723182f812e4ca0def829b18272a0dfc1145d2c3e28f8c6210cea7b` |
| `vehicle/iris/meshes/iris_prop_ccw.dae` | `5f8b01668ee24a5b663ca8c4dd56e294fd6480db7eec1e5a7c11e45ba9004e99` |
| `vehicle/iris/meshes/iris_prop_cw.dae` | `6cbc686772dccd46253fb65ece000e7ffa6a74b9c097bda349884bd1e78cd879` |
| `sensor/fpv_cam/fpv_cam.sdf` | `f1c090e97831fa0d361a5fc37b995b298ca459759e8e18db43889817c42648a0` |
