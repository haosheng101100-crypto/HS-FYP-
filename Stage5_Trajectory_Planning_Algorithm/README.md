# 第五阶段：规划算法

2026-11-05：本阶段作为已有着陆算法基线保留，原“毕设仿真”目录改名为 `Stage5_Trajectory_Planning_Algorithm`。新工作转入[第六阶段：车辆平台运动预测](../Stage6_Platform_Prediction_Algorithm/README.md)，先完成预测模块，再研究无人机择机下降。下方记录保留各自日期和版本的结论，不作为恢复旧待办的指令。

---

# 当前演示交付状态

2026-11-02功能演示已完成：X/Y/XY三场真实物理录制均严格通过，18份审计与3份请求历史通过；三份1倍墙钟单轴视频和1份2倍墙钟汇总已完整解码检查。控制仍固定748b27e，最终入口经独立审查补齐保护，211项Linux测试通过。首次录制主机重启中断保留，3个有效场景来自4次物理尝试。

[演示视频与验收](experiments/ubuntu_demo_delivery_20261102/README.md)；[本机演示启动](演示启动说明.md)。

---

# 移动小平台着陆算法

2026-11-02最新用户范围：只做功能实现与演示，取消92场大样本/压力计划。已保留5场重新跟随复测、5/5严格通过；原二维5场回归5/5符合各自声明（4落台、1安全退出）。A其余25场及其余压力/边界/旧九矩阵取消；前次主机重启中断及基础左右场通信中断各1次保留；基础有效5场来自6次尝试。代码748b27e28be42ad294769105c5c899d3e9ecfb25，200项测试通过，控制算法/原门限保持。见[本轮功能验收](handoff/UBUNTU_FUNCTIONAL_ACCEPTANCE_20261102.md)。当前不再要求继续30/92场；下方大样本步骤仅作历史记录，最新用户范围优先。


2026-11-02 Linux实测：高净空X/Y/XY三个专项全部通过严格独立验收（3/3）。每场一次实测下降换向、一次真实PLANAR_REACQUIRE，其后再次对齐、新计划激活、真实落台、正常原生LAND及停转后支撑均通过；18份审计通过。190项测试在Linux实际通过，包含Gazebo解析，无跳过。控制算法及安全门限保持；4m脚底初始跟随净空是专项独立初态。[本轮结果](handoff/UBUNTU_REACQUISITION_RUN_20261102.md) · [曲线和逐场证据](experiments/ubuntu_reacquisition_20261102_session001/README.md)。


**Linux接续入口：先读 [Linux接续工作说明](LINUX接续工作说明.md)。** 已确定Linux负责代码与仿真，GitHub同步版本和实验数据，Windows负责看结果、讨论和材料整理。

2026-11-02：新增“换向后重新跟随并落台”专项配置、实测下降触发器和严格验收入口，见 [专项仿真说明](重新跟随专项仿真说明.md)。此专项采用独立4米跟随净空，原五场配置及安全门限保持；新专项物理结果尚待运行，不以旧五场成功替代重新跟随分支验证。

2026-11-01：二维导轨平台五场基础矩阵已完成本机独立物理审计：前后、左右、斜向、二维连续变速四场真正落台，换向场景安全退出，5/5 满足声明、24 份审计通过。模型生成器另修正 SDF 轴坐标系，控制算法与安全门限保持；本轮未触发重新跟随，该分支仍待专项物理验证。见 [本机结果](handoff/UBUNTU_PLANAR_RUN_20261101.md) 与 [曲线和审计](experiments/ubuntu_planar_20261101_session004/README.md)。

2026-10-30：新增二维连续运动算法，支持前后、左右运动的因果换向识别、接触窗口筛选、安全高度跟随和有净空条件的重新跟随。完整源码、场景配置、测试及运行入口已同步；本地回归与数值检查通过，**五场基础矩阵已新增物理结果，未触发功能仍待专项验证**，历史九场景结果不作为本版验收结论。见 [二维移动平台算法说明](二维移动平台算法说明.md)。


2026-10-30：新版数值求解初始化（773994d）已在独立 session_003 完成九场物理仿真及 36 次独立审计，6 场真正落台、3 场安全退出，9/9 满足声明；安全门限未放宽。结果见 [新版九场景复核](handoff/UBUNTU_SOLVER_RUN_20261030.md)，曲线和逐场资料见 [新版轻量实验数据](experiments/ubuntu_solver_20261030_session003/README.md)。

当前主规划器修正末段数值求解初始化：复用 CVXPY DPP 模板、重新建立 Clarabel 数值求解器，保持原目标、约束和安全检查。部署与测试说明见 [算法调度说明](算法调度说明.md)。此更新完成了离线数值回归和同输入对照，该固定版本已完成本机独立物理仿真，见上方记录；下方九场景记录对应各自固定的历史代码版本。

2026-10-30：新版评分下界剪枝（922f9d8）已在独立 session_001 完成九场物理仿真及 36 次独立审计，6 场真正落台、3 场安全退出，9/9 满足声明；安全门限未放宽。结果见 [新版九场景复核](handoff/UBUNTU_PRUNED_RUN_20261030.md)，曲线和逐场资料见 [新版轻量实验数据](experiments/ubuntu_pruned_20261030_session001/README.md)。

当前主规划器增加候选评分下界剪枝，保留原评分、候选集合与安全检查；部署方法和完整搜索对照开关见 [算法调度说明](算法调度说明.md)。历史冻结源码及下方 Ubuntu 复现记录保留原版本。

2026-10-29 更新：曲线、绘图抽样数据、36 份逐场审计及关键回放样本见 [轻量实验资料](experiments/ubuntu_20261029_session002/README.md)。完整原始数据保留本机并另做归档。

2026-10-29 更新：本机已完成新的九场景物理仿真与独立复核，六场真正落台、三场安全退出，9/9 满足原协议。环境、数据边界和结果见 [本机复现记录](handoff/UBUNTU_PHYSICAL_RUN_20261029.md)。

这是本次对话最终通过九场景仿真验收的算法源码与交接包（2026-10-28 上传整理）。实际验收版本为 batch 082/083、协议 v30；包含六个真实落台、三个安全退出，退出不计落台。

算法用因果平台状态估计、分段七阶 Bézier SOCP、矩形整机包络、末段实测跟踪偏差补偿和原生 LAND 接管，实现移动平台接近、下降与安全退出。详见 [算法与验收](新算法与仿真验收.md)。

## 下载和算法测试

```bash
git clone https://github.com/haosheng101100-crypto/HS-FYP-.git
cd HS-FYP-/Stage5_Trajectory_Planning_Algorithm
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python scripts/verify_source_package.py
python -m unittest discover -s tests -v
```

私有仓库需要使用有权限的 GitHub 账号登录。后续更新使用 `git pull --ff-only`。依赖版本来自本次 Windows 已测试环境；Linux 平台安装和求解器数值差异仍需在目标 Ubuntu 核对。

## 目录

- `landing/`：算法源码。主优化器 `receding_landing.py`，异步执行、捕获与恢复 `receding_live.py`。
- `config/`：飞机几何、平台与控制配置。
- `tests/`：本算法相关回归测试；附带所需的历史失败记录摘录，逐行保留，不冒充完整飞行证据。
- `tools/`：规划示例、工作进程、审计、回放和原机器启动工具。
- `simulation/reference_cycle/`：实际 batch 082/1 使用的完整源程序、模型、世界、原生飞控参数和前置参考输入。保留其自身引用与历史哈希。
- `simulation/operator_package/`：实际自动仿真的监督器、控制后端、面板及源清单。
- `outputs/uncertainty_landing_20261024/`：最终冻结源码、九场景报告与协议。完整原始物理日志和飞控二进制日志仍保存在原电脑，未上传；云端摘要不足以单独复算所有物理验收。
- `handoff/`：当前进度、来源与跨机器交接说明。

## Ubuntu 物理仿真

上传包含模型、场景与控制源码，但不包含虚拟机磁盘、Gazebo 安装目录、ArduCopter 二进制或依赖缓存。原环境为 Ubuntu 22.04、Gazebo Harmonic、ArduCopter SITL、ardupilot_gazebo；依赖安装和模型引用见 `simulation/environment_reference/`。

**原监督器是原机器适配版本**，使用 `/home/haosheng/hs_ardupilot_stage1`、`reproduction_20261018/fixed_gimbal_terminal_004`、`paper_experiments/moving_rapid_v1`、独立版本包与 session 目录，并要求 `vendor/`、`gz_prefix/`、`venv/`、Gazebo Python 绑定及图形桌面。`tools/run_receding_simulation.py` 则是原 Windows + VMware 入口，不能直接作为另一台 Ubuntu 的启动命令。

另一台 Ubuntu 需要先安装上述运行时，将 `simulation/reference_cycle/` 作为场景/参考源码，按监督器的 `SOURCE`、`FROZEN`、`BASE`、`PKG` 路径关联部署；新 session 必须独立，不能指向已有验收记录。不要删除前置校验或修改原飞控参数来跳过错误。初始上传时新机器的完整物理启动尚未验证；2026-10-29 已在 Ubuntu 24.04 隔离环境完成本机新一轮物理运行与独立审计，详见上方复现记录。

## 当前进度与边界

本轮声明任务已完成，九个场景全部通过原门限。模型为水平固定高度小平台，面积为整机占用外接矩形的 1.5 倍。当前成果属于仿真；没有真机或任意运动平台保证。

跨机器继续时，先读 `handoff/CURRENT_STATUS.md` 和算法验收说明，拉取最新代码，再创建新实验版本。保存失败和退出记录，不把退出当落台，不放宽原门限。

