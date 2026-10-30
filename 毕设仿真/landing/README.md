# 真实脚底接触几何接口

`contact_geometry.py` 只依赖 Python 标准库，计算接触前刚性几何与速度；无飞控、Gazebo 连接或控制指令副作用。`trajectory_audit.py` 将此接口接到规划器提供的位姿/速度参考，输出采样捕获的首次几何接触与终端脚底状态。

所有长度/时间/角度使用米、秒、弧度；四元数顺序为 xyzw，表示局部坐标到世界坐标。飞机机体系向前/向左/向上，后倾角 θ 为正时对应标准欧拉 pitch 为负（零滚转、零偏航时）。速度与角速度分量均在世界坐标系表达。

```python
from landing.contact_geometry import AirframeGeometry, Pose, RigidMotion, Deck, evaluate_airframe

# 路径由调用方明确选择；待定真机模板缺少必要几何，会报错。
geometry = AirframeGeometry.load("config/airframes/iris_fixed_gimbal_sim.json")
aircraft = RigidMotion(Pose((0, 0, .195), (0, 0, 0, 1)),
                       (0, 0, 0), (0, -.5, 0))
deck = Deck(RigidMotion.stationary(Pose((0, 0, 0), (0, 0, 0, 1))), (1, .7))
feet = evaluate_airframe(geometry, aircraft, deck)
```

`RigidMotion` 的线速度对应 `Pose` 原点。若输入是质心位置/速度，先使用 `RigidMotion.from_com`，明确提供质心相对机体原点的偏移。不能把两种原点混用。上述数值仅是接口示例，尚不代表可执行飞行指令。

甲板 Pose 的原点是顶面中心，局部 +z 是外法向；半尺寸给出有限矩形范围。模块只支持沿机体 z 轴的圆柱腿。

返回项包括净空、支撑点材料坐标和世界坐标、脚底材料点速度、相对甲板的法向/切向速度、整根圆柱在甲板上的投影覆盖裕度。闭合速度正值代表接近甲板。平台平面之外的零净空不会自动被当成着陆；矩形边缘/侧面碰撞需另行处理。

平放圆柱的整个底面同样低，侧放圆柱则整条母线同样低。接口明确标记这些退化情况，给出一个代表材料点和支撑集合上的保守速度上界。规划约束不能用代表点的速度代替整个支撑集合的上界。

最低点会随姿态改变，直接对最低点位置差分可能包含支撑点切换；材料点速度使用 `v原点 + ω×Rr`。模块不求摩擦、冲击、形变、支撑稳定性或机体/云台网格碰撞。

验证入口：`python -m unittest discover -s tests -v`。离线实测数据核对：`python tools/replay_contact_geometry.py`。说明与结果在 outputs/model_audit/step05。

## 触地边界反算

`touchdown_boundary.py` 给定相对甲板的姿态、角速度、活动脚底代表支撑点中心的位置及相对速度，反算机体原点的位置和线速度。`TouchdownRequest.planar` 中正后倾角速度对应甲板 -y 方向角速度。

`ContactDesignLimits` 必须显式提供，不含默认“已测能力”。求解器对所有活动支撑集合检查速度上下界，并检查所有起落架的甲板投影裕度；不只匹配中心平均速度。不可行请求抛出带原因的 `InfeasibleTouchdown`。

输出 `TouchdownBoundary.aircraft` 明确以机体原点为参考；接触支撑中心放在甲板中心并不意味着机体原点也在那里。该边界只解决接触前的代数运动学一致性，不证明全程可达、电机能力或触地后稳定性。

候选配置：config/experiments/touchdown_boundary_v1.json。运行 `python tools/check_touchdown_boundaries.py`，结果在 outputs/model_audit/step06。

## 完整平面进场参考

`septic_approach.py` 提供 `PlanarSepticApproach`（七次）与 `PlanarSnapMatchedApproach`（九次）。输入为明确的质心初态、`TouchdownBoundary`、机型质心偏移、重力、质量和持续时间。九次变体额外指定终端质心 snap，初始 snap 为零。当前仅支持静止水平平台、零滚转/偏航、恒定质心 y，以及零终端角速度；不支持的边界会报错。

参考以质心加速度及重力确定所需推力方向，jerk 确定角速度，snap 确定角加速度；再转换为机体原点的位置、速度和加速度。输出包含点质量模型的合推力需求，未与已测电机能力比较，也未解转动动力学或飞控跟踪。

`check_approach_reference` 采样检查速度、加速度、jerk/snap、姿态与角速度、各脚净空、低空平台覆盖、近触地脚底材料点速度及终端边界。`check_height_band_entries` 对采样捕获的高度区间交叉作二分细化，并检查边界两侧，避免仅凭普通采样漏掉近触地速度限制刚激活时的违反。主速度峰后不允许超过 0.02 m/s 的速度反弹；最终低水平速度区采用 0.12 m/s 阈值。它不作连续时间可行性证明，仍不能排除未括住的采样间违反，也不处理平台边缘/侧面网格碰撞、机身碰撞或冲击稳定性。

运行 `python tools/plan_contact_approach.py`，按 config/experiments/snap_approach_v2.json 搜索时长及终端竖直 snap；然后运行 `python tools/verify_contact_approach.py`，从保存系数重建，进行 20001 点检查和另一组高度边界括区。每项限制均为仿真参考设计条件，不是硬件辨识结果。结果、可复算系数、CSV 与图在 outputs/model_audit/step07；不会向飞控发指令，且参考到首次计划触地为止。失败记录分别保存在 step07/septic_attempt_001 和 ninth_attempt_001。

## 准备、执行与交接原型

`reference_loader.py` 根据固定配置和已通过的第七步证据重建参考，并核对源码哈希与保存系数。`startup.VerticalLeadIn` 提供单独计时的竖直准备段；当前为从质心 z=3.95 m 悬停参考，用 1.50 s 下降到主参考的 z=3.65 m、vz=-0.4 m/s，匹配位置导数直到 snap。准备参考与实际飞行状态的一致程度必须由 `execution.assess_entry` 核对，不能把参考连续性当作实际飞行连续性。

`LandingExecutor` 输出结构化决策，未接通 Gazebo/网络。未就绪时保持 WAIT_ENTRY，外部起飞/准备控制器仍拥有飞行控制权。准备完成再次核对主参考入口；失败时请求 LAND。首次实际甲板接触、参考到期无接触、状态/模式失效或跟踪更新中断会不可逆地退出空中参考。`ContactHandover` 要求模式回执、持续多腿接触、运动学支撑检查、正常停转及停转后观察，不强制停转，不求解接触动力学。

`ContactHistory.poll` 返回 `(current_support_frame, first_new_active_event)`；二者必须分别传入 `executor.step(now, observation, frame, event)`。不可只用最新接触帧代替首次事件。队列只接收一个规范来源的有序消息；重复且相同的帧可忽略，冲突/乱序或未消费队列覆盖会报错，在线调用方需执行异常接管。观测使用仿真时间；上游还必须检查接收端墙钟超时、模式心跳新鲜度以及模拟时钟停滞。

`mavlink_reference.dispatch` 只对显式传入的 transport 和 LAND 回调产生副作用：机体原点 PVA 或仅速度目标，LAND 确认后不再发外部目标。它不创建连接、不解锁、不停转、不写仿真位姿。当前通过 mock 验证参数，真实飞控能力与模式交接待新飞验证。

运行 `python tools/verify_execution.py`，保存结果在 outputs/model_audit/step08。该工具只做准备参考检查、单元测试和已有 run_004 的接触交接反事实回放；模式缺失明确使用 UNKNOWN，未运行新飞行。运行说明和剩余 Gazebo 桥接工作见该目录报告。配置为 config/experiments/execution_v1.json，均为仿真软件条件。

## 实时 Gazebo 状态接口

`observations.py` 使用 NumPy 对最多约 81 ms 的过去机体位置与旋转矩阵做二次拟合，估计机体速度、角速度、角加速度，再换算质心加速度。窗口不足、消息间隔过大、未来/陈旧样本、接收墙钟失效或心跳失效会拒绝估计。它使用仿真真值，不是视觉/LIO 定位器或真机传感器滤波器。读取墙钟时必须在锁定样本之后，避免回调新收到的样本被误判为“来自未来”。

虚拟机使用 `source scripts/env.sh` 暴露现有 NumPy/Gazebo 运行库；Windows 本机可使用已有 NumPy 环境，或先把 `work/plot_dependencies` 加到 PYTHONPATH。实时脚本同时按仿真时间检查模式心跳，并保留接收墙钟上限，区别慢速仿真与实际消息停滞。

实际桥接代码、启动脚本和每轮源码保存在 `work/model_step09_v*`、`work/model_step09_launch_v*_20261018.sh` 及 `outputs/model_audit/step09` 的独立运行目录。启动前查询接触话题的真实发布者，不能把存在订阅者当作存在发布者。对当前模型实际发布的是完整 scoped sensor 话题，单独的 `/vehicle/deck/contact` 声明无发布端。

`LandingExecutor.experiment_can_finish` 将实验采集结束与触地目标是否通过分开：即使姿态或速度超限，仍需完成正常 LAND 和停转后验证，才收尾保存。总体失败不会被覆盖为成功。第九步的飞行结论应查看原始接触日志与 `analysis/flight_verification.json`，不要用旧步骤的离线通过代替实际飞行通过。


## 加速平台：当前新增物理验证

`accelerating_platform.py`实现带时间戳的因果共享状态、加速度外推、有界snap优化及固定截止时刻的滚动修正；`accelerating_feedback.py`实现捕获段位置反馈转移和曲率修正的自身参考预览。实时入口为`tools/accelerating_platform_demo.py`，实验配置`config/experiments/accelerating_platform_v1.json`。

初速3 m/s、加速度0.5 m/s²工况的规范物理结果为`outputs/model_audit/step18/run_007`；使用`tools/analyze_accelerating_contact.py`独立审计，接触后相对位置使用同时间对齐的原始数据。新增算法是解析滚动轨迹修正与混合控制，不是完整NMPC、学习模型或真机部署方案。视觉识别、真实通信、任意变加速度及批量鲁棒性尚未验证。参见第十八步正式报告和各失败记录。
## 2026-10-23：三维Bézier滚动着陆规划器（数值准备）

新增 `receding_landing.py`：有限接触时间搜索、参数化SOCP、真实脚底端点、连续平移约束审核、滚动参考与接触交接请求。配置在 `config/experiments/receding_landing_v1.json`；运行 `tools/receding_landing_demo.py` 与 `tools/check_receding_closed_loop.py`。完整结果和局限在 `outputs/receding_landing_20261023/算法实现与验证说明.md`。新代码无网络/飞控副作用，没有改动原生操作面板；不把质点穿平面结果作为Gazebo落台或支撑成功。当前计算性能尚未达到10Hz，实时控制接入待完成。

