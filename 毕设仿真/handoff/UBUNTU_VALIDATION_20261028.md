# Ubuntu 数值验证与物理环境接续（2026-10-28）

验证来源：`97f3cb0e7b69d09abffaa51e3a11315328522a18`，本轮未修改算法、控制参数、场景或门限。

## 已完成

- 通过已连接的 GitHub 插件同步全部 408 个仓库文件，逐文件核对 Git blob SHA；407 个清单文件通过原 `scripts/verify_source_package.py`。
- 在 Ubuntu 26.04.1 amd64、Python 3.12.14 的独立虚拟环境安装原 requirements.txt。
- 实际依赖：NumPy 2.5.4、SciPy 1.18.1、CVXPY 1.9.3、Clarabel 0.11.1。
- 在仓库根目录运行 `python -m unittest discover -s tests -v`：130 项通过，16.337 秒，退出码 0。
- 测试日志 SHA256：`1faaa7f4a066a01458fb09ae6028e56fe148a4a8096b44cade9516db393a56b1`。原始本次数值测试日志留在本机工作目录。

这些结果支持源码和所测数值/逻辑路径在本机运行，不证明 Gazebo 物理闭环、真实接触、实时求解期限或起降复现成功。现有九场景报告仍是原机器的历史验收。

## 当前物理环境

当前系统未发现 Gazebo、ArduCopter SITL、构建工具 CMake/G++ 或 Docker/Podman。原环境为 Ubuntu 22.04、Gazebo Harmonic、ArduCopter SITL、ardupilot_gazebo，以及原模型、飞控参数、控制监督器和 MAVLink/Gazebo Transport。

检索 landing/、tools/ 和 simulation/ 未发现当前控制链调用 rospy/rclpy/rclcpp/MAVROS；匹配项为旧环境说明、Windows 的 D:/ROS 路径和不依赖 ROS 的辅助模块说明。原机器装有 ROS 2 Humble，并不意味着当前算法以 ROS 为必需运行时。ArduPilot 官方也说明 ardupilot_gazebo 不依赖 ROS：
https://ardupilot.org/dev/docs/sitl-with-gazebo.html

Gazebo 官方 Harmonic 二进制安装支持 Ubuntu 22.04 和 24.04；当前主机 26.04 不在该安装说明范围：
https://gazebosim.org/docs/harmonic/install_ubuntu/

原 `simulation/reference_cycle/scripts/prepare_harmonic.sh` 写死 jammy 软件源，并包含系统级 apt 操作。没有在这台 26.04 主机执行它。

## 接续顺序

1. 先恢复兼容运行环境：复现优先考虑恢复原 Ubuntu 22.04 环境；若采用本机隔离环境，Ubuntu 24.04 + Harmonic + Python 3.12 是另一个受支持的候选，但属于迁移，需独立验证。不要混用主机 26.04 与 jammy 软件源，也不要仅为恢复本链路先安装 ROS 全套。
2. 固定原插件版本和飞控二进制哈希，确认模型、世界、Gazebo Python 绑定、MAVLink 和目录映射。版本依据见 `simulation/environment_reference/source_snapshot.json`。不要把上游 stable 的浮动下载当成原固定版本。
3. 先验收 Gazebo 世界加载、Iris 与平台、SITL 心跳、仿真时钟和控制链路；再做基础起飞、悬停、落地。保留原物理参数。
4. 新建独立 session，先复现 constant 落台与 surge 安全退出，再复现完整九场景；保留原始位姿、控制与飞控日志，运行原因果和物理审计。

## 实际阻塞与同步方式

当前执行终端启用了 no-new-privileges，`sudo -n true` 在提权前被拒绝，尚未完成系统级安装。此状态不是密码错误，也不是环境已经恢复。

本轮使用 GitHub 插件同步仓库；外部 Chrome 窗口当前不能被操作工具接管，本机 GitHub CLI 未完成认证。因此本地目前是经 Git SHA 校验的文件快照，不是已配置好 pull/push 的 Git clone。GitHub 插件的仓库读写和本机 Git CLI 登录是独立的接续事项。
