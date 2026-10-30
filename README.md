# 移动小平台着陆算法

当前主规划器修正末段数值求解初始化：复用 CVXPY DPP 模板、重新建立 Clarabel 数值求解器，保持原目标、约束和安全检查。部署与测试说明见 [算法调度说明](算法调度说明.md)。此更新完成了离线数值回归和同输入对照，新代码仍需在独立 session 进行物理仿真；下方九场景记录对应各自固定的历史代码版本。

2026-10-30：新版评分下界剪枝（922f9d8）已在独立 session_001 完成九场物理仿真及 36 次独立审计，6 场真正落台、3 场安全退出，9/9 满足声明；安全门限未放宽。结果见 [新版九场景复核](handoff/UBUNTU_PRUNED_RUN_20261030.md)，曲线和逐场资料见 [新版轻量实验数据](experiments/ubuntu_pruned_20261030_session001/README.md)。

当前主规划器增加候选评分下界剪枝，保留原评分、候选集合与安全检查；部署方法和完整搜索对照开关见 [算法调度说明](算法调度说明.md)。历史冻结源码及下方 Ubuntu 复现记录保留原版本。

2026-10-29 更新：曲线、绘图抽样数据、36 份逐场审计及关键回放样本见 [轻量实验资料](experiments/ubuntu_20261029_session002/README.md)。完整原始数据保留本机并另做归档。

2026-10-29 更新：本机已完成新的九场景物理仿真与独立复核，六场真正落台、三场安全退出，9/9 满足原协议。环境、数据边界和结果见 [本机复现记录](handoff/UBUNTU_PHYSICAL_RUN_20261029.md)。

这是本次对话最终通过九场景仿真验收的算法源码与交接包（2026-10-28 上传整理）。实际验收版本为 batch 082/083、协议 v30；包含六个真实落台、三个安全退出，退出不计落台。

算法用因果平台状态估计、分段七阶 Bézier SOCP、矩形整机包络、末段实测跟踪偏差补偿和原生 LAND 接管，实现移动平台接近、下降与安全退出。详见 [算法与验收](新算法与仿真验收.md)。

## 下载和算法测试

```bash
git clone https://github.com/haosheng101100-crypto/HS-FYP-.git
cd HS-FYP-
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
