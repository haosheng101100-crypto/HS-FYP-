# 基于官方 Gazebo 的车机协同仿真

使用 ArduPilot 官方 `ardupilot_gazebo` 插件、Iris 四旋翼模型和 `iris_runway.sdf` 跑道世界，在 Gazebo Harmonic 原生界面中加入轮式车辆和车顶降落平台。第一阶段已有的预测、PID、速度前馈与降落状态机已经通过 MAVLink 接到 ArduCopter SITL。

## 已保存的运行结果

截至 2026-10-14，已取回的运行记录确认完成：

- 静止车辆车顶降落：`results/stationary_land_cooperation_status.json` 中 `completed: true`。
- 低速跟车、车辆停车、车顶降落：`results/follow_stop_cooperation_status.json` 中 `completed: true`。
- 车辆持续运动时的车顶降落：`results/moving_land_cooperation_status.json` 中 `completed: true`。接触完成后车辆停车，由 LAND 模式正常停转。

两次流程均记录了车顶接触、进入 LAND 模式和飞控正常解除解锁。跟车示例以 0.35 m/s 直行，行驶指令持续 12 秒仿真时间。

2026-10-14 已恢复虚拟机并取回此前运动中降落的最终记录。旧的 `results/moving_landing.mp4` 因录制被中断，缺少 MP4 索引，不能播放，不能作为可观看的成果交付。本次已重新现场演示并录制跟车后停车降落，保存为 `results/follow_demo_20261014.mp4`，H.264、1280×720、3 帧/秒、107.334 秒。视频来自实际 Gazebo 相机，在任务完成后结束录制并确认 MP4 收尾有效；对应任务记录是 `results/follow_20261014_cooperation_status.json`。

当前使用 Gazebo 真值观测，尚未加入视觉识别、观测噪声、通信延迟、风或车辆转弯。结果仅说明这些仿真条件下的协同流程已运行。

2026-10-14 新增自己的三轴姿态角 PID，并通过角速度与纯推力指令完成一次实际飞行：起飞、悬停、20° 偏航目标跟踪、跟车、车顶下降和正常停转。`results/attitude_demo_status.json` 为 `completed: true`，记录了 869 次 PID 控制计算。此模式的角速度内环和电机混控仍由 ArduCopter 执行。

项目保留两个入口：`run_cooperation.sh` 是原来的位置 PID + 飞控姿态控制；`run_attitude.sh` 使用自己的位置、速度与姿态角控制链。新增的姿态 PID 位于 `control/attitude_pid.py`，实际连接与起降流程位于 `tools/attitude_demo.py`。

可观看成果：`results/attitude_pid_demo_20261014.mp4`（109.667 秒）及实际反馈图 `results/attitude_pid_response.png`。原理详见 [姿态 PID 与起降原理](姿态PID与起降原理.md)。当前为 Gazebo 仿真结果，实体机的飞控和硬件适配仍待确认。

## 在已有 Ubuntu 虚拟机中运行

已部署目录为 `~/hs_ardupilot_stage1`，环境为 Ubuntu 22.04、Gazebo Harmonic、ArduCopter SITL 和 pymavlink。

```bash
cd ~/hs_ardupilot_stage1
bash scripts/start_scene.sh
```

场景启动后，在虚拟机桌面查看 Gazebo。需要重新打开图形客户端时运行：

```bash
bash scripts/view_scene.sh
```

运行协同演示，选择其中一条：

```bash
bash scripts/run_cooperation.sh --task stationary-land
bash scripts/run_cooperation.sh --task follow --car-speed 0.35 --drive-seconds 12
bash scripts/run_cooperation.sh --task moving-land --car-speed 0.35 --drive-seconds 12
```

`moving-land` 在下降时保持车辆行驶，接触完成后停车并交给 LAND 模式停转。每次演示覆盖当前的 `results/cooperation.log`、CSV 和状态 JSON；要保留结果，应在下一次运行前另存。

使用自己的姿态 PID 演示时，先停止并重新加载场景，使无人机回到地面，再运行：

```bash
bash scripts/stop_scene.sh
bash scripts/start_scene.sh
bash scripts/run_attitude.sh --task follow --car-speed 0.25 --drive-seconds 10
tail -f results/attitude_demo.log
```

此模式起飞不调用 `NAV_TAKEOFF`。消息掩码 128 将我们算出的三轴机体系角速度交给飞控，`GUID_OPTIONS` 第 3 位开启纯推力字段。演示结束并解除解锁后恢复原参数值。输出保存到 `results/attitude_demo.csv` 和 `results/attitude_demo_status.json`，与旧入口的结果文件分开。绘制实际 CSV 曲线的 `tools/plot_attitude.py` 需要 matplotlib，飞行脚本不依赖绘图库。

```bash
# 查看进度
 tail -f results/cooperation.log
# 结束场景
bash scripts/stop_scene.sh
```

单独加载官方原始跑道场景使用 `bash scripts/start_scene.sh --official`，切换前先停止当前场景。

## 文件与控制关系

- `tools/create_scene.py`：从官方跑道世界和 Iris 包装模型生成自己的场景，并加入车辆、位姿发布与接触反馈。
- `models/landing_vehicle/`：物理轮式车辆及 2.0×1.4 m 车顶平台，由 DiffDrive 驱动轮关节。
- `first_stage/core.py`、`integration.py`：复用的第一阶段控制代码。
- `tools/cooperation_demo.py`：获取模型位姿和接触反馈，调用第一阶段控制器，向飞控发送速度及朝向指令。
- `tools/capture_scene.py`、`record_scene.py`：保存实际 Gazebo 渲染画面与录像。
- `scripts/env.sh`：配置隔离安装的 Harmonic 库、插件和软件渲染。
- `results/source_snapshot.json`：官方来源版本与文件哈希。

位置控制使用世界 ENU，发送给飞控的 NED 速度为 `(vy, vx, -vz)`。目标点包含车顶高度及无人机起落架参考点的姿态旋转偏移。下降条件同时检查相对位置、相对速度、无人机倾斜与相对朝向。姿态服务于协同降落，当前不增加车辆姿态控制课题。

详细依据见 [研究与实施说明](研究与实施说明.md)。官方代码与资源保留原许可；本项目新增内容放在上游目录之外。

## 上游依据

- [ArduPilot 官方 Gazebo 仓库](https://github.com/ArduPilot/ardupilot_gazebo)
- [ArduCopter Guided 控制指令](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
- [Gazebo DiffDrive 接口](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
