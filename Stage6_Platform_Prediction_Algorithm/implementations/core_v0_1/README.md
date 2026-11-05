# 第六阶段：车辆平台预测核心算法 v0.1

日期：2026-11-05。

第1—4步的核心实现已经写齐：车辆运动模型、UKF状态估计、IMM多模型交互、保留轨迹关联的未来分支预测。另附平面甲板刚体映射算子。此包不运行Gazebo、飞控或新车辆仿真，不改第五阶段。

这是一版可运行、可单测的算法实现，尚未经过第5—8步的独立仿真、误差校准、失信监测和着陆联调验收。当前预测范围是有限样本包络，不是校准概率区域或确定性可达外包。

先读 [算法说明](算法说明.md)；详细公式和具体参数见 [设计与推导](docs/superpowers/specs/2026-11-05-vehicle-core-design.md)。

## 安装与运行

Python 3.12+，NumPy和SciPy。已核验环境：Python 3.12.14、NumPy 2.5.4、SciPy 1.18.1。核心没有神经网络训练，也不需要车辆未来控制、路线或轴距。

在本目录创建独立环境后运行（不要装进第五阶段环境）：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m stage6 --input examples/straight_pose_fixture.jsonl --output my_forecasts.jsonl --particles 12 --seed 0
```

这里记录的是安装方法；本轮核验借用已有科学计算解释器只读运行，没有安装依赖或修改旧环境。第三方依赖精确版本在 requirements-tested.txt；默认requirements允许兼容版本，其他组合仍需重跑测试。

附带的straight_pose_fixture是手工构造的直线位置样例（x=t,y=0,psi=0），仅用于接口/解析核对，不是新仿真或实测数据。verification/fixture_forecasts.jsonl是此输入的可重复输出。相同依赖、参数和种子下结果应相同。输出文件必须不存在，防止意外覆盖；坏输入会删除本次新建的不完整输出。

## 输入

每行一个JSON对象，仅允许以下五个字段：

```json
{"stamp":0.0,"received_at":0.0,"position_xy":[0.0,0.0],"yaw":0.0,"covariance":[[0.0025,0,0],[0,0.0025,0],[0,0,0.0004]]}
```

stamp是采样时间，received_at是该样本实际可用时间，同一时间基准、单位秒；position_xy单位米、yaw单位弧度；covariance对应[x,y,yaw]，含相关项，须有限、对称正定。没有未来真值、未来控制或路线字段。strict reader拒绝额外字段，避免无意传入未来信息。

观测必须按stamp严格递增，received_at非递减且不早于stamp。乱序或重复样本显式报错，尚未实现乱序重放平滑。断测超过1秒重置；至少5样本/0.4秒跨度后才就绪；采样年龄超过0.2秒时无有效预测。

观测点与运动学参考点存在已知车体偏移时，用 `--offset DX DY`，单位米。默认零偏移只适用于已明确对应参考点的观测，不能把未知安装参数当作零。

命令行在收到新观测时检查发出周期，默认0.2秒；没有观测时它不会伪造新的观测/定时事件，独立调度器可通过Python API调用forecast(now)检查过期状态。由于命令行按实际receipt触发，发出时刻可能有抖动，不回填过去时刻的预测。

## Python接口

```python
import numpy as np
from stage6.observation import Observation
from stage6.predictor import VehiclePredictor
from stage6.model import Limits

predictor = VehiclePredictor(offset=(0.0, 0.0))
# 对每一个已经收到的观测执行：
predictor.update(Observation(
    stamp=0.0, received_at=0.03, position_xy=(0.0, 0.0), yaw=0.0,
    covariance=np.diag([0.0025, 0.0025, 0.0004]),
))
packet = predictor.forecast(
    now=0.03, horizon=2.0, step_s=0.1, particles_per_mode=12, seed=7,
    limits=Limits(),
)
# 只有一个观测时packet为warming_up；继续收到足够观测才有轨迹。
```

forecast复制后验、补偿观测年龄，不修改在线滤波器，不修改全局随机数状态。种子由调用者管理；文件入口每次发出预测依次增加种子。

## 输出

每个预测包含configuration快照、观测年龄、观测跨度、样本数、status、trust及4个models。

- warming_up：历史不足，没有未来轨迹。
- stale：观测过期，没有未来轨迹。
- ready：程序成功生成每模式请求数量的样本，不等于已通过统计或着陆验收。
- degraded：某模式样本不足，或从观测时刻传播到now时超出局部角度估计支持范围。对应模式保留posterior_weight；无效状态明确为null，不删掉后把剩余分支归一成“全部可能”。

每模式保存当前状态/协方差、后验模型权重、trajectory样本、sample_envelope及拒绝数。8维状态顺序固定为[x,y,psi,v,a,kappa,j,eta]。每条trajectory保留initial_state和完整states时间序列，以及velocity_xy。horizons_s为相对issued_at的未来时间。state_time与issued_at一致；角度传播失败时state/covariance/state_time均null。

权重是观测条件下滤波模式的后验标签，未声称已校准。对整条物理不合规轨迹进行拒绝之后，这些标签不能当作严格的未来分支概率。样本包络的角度以中心+圆周半径表达，避免跨±pi的区间错误；包络包含的中间点不保证满足车辆约束。

attempted表示顺序考虑到获得请求数量时所用的候选路径数（不足时为上限20*N），rejected=attempted-accepted。实现为向量化计算，会先计算完整20*N候选池，不能把attempted当作实际运算量或耗时依据。

physical_reachable_set=null。当前没有确定性可达外包实现，也没有覆盖率声明。

## 数学与数值边界

UKF使用局部角度坐标；为了防止sigma点经过2pi环绕后错误消去不确定性，在任何wrap前要求sqrt(8*Ppsi,psi)<pi/2。过宽初始先验或滤波传播会抛LocalAngleError(ValueError子类)；状态预测失败不提交部分更新。未来查询的年龄补偿失败则返回degraded并保留模式失败原因。接收调用者应处理异常，不应继续用失败的本次更新当成成功估计。更广的圆周多峰分布需要不同的状态表示，不在本版范围。

过程噪声只在j、eta驱动上加入，滤波与未来采样使用相同谱密度和sqrt(q*dt)离散增量。未识别的结构误差没有独立建模，更没有宣称全覆盖；后续残差校准另做。RK4子步不超过0.05秒，噪声离散仍是一阶近似。

默认物理限制属于公开的测试配置，不是实车标定值；通过Limits传入已识别的参数。估计高斯不会被随意裁剪，状态均值可违反约束并在输出记录。未来样本则整条拒绝越界路径，不改变老的概率声明。约束只在起点、子步中点/末端和噪声更新后检查，不是严格连续时间证明。

模式参数和过程噪声在stage6/model.py的冻结Mode定义中；configuration.json为默认快照（不是会被程序自动加载的外部配置）。改变模型定义后应版本化源码并重跑测试，不能只修改快照文件。

## 甲板映射

stage6/deck.py提供DeckGeometry、body_to_deck和map_forecast。对于固定平面安装，包含中心偏移的角速度速度项和旋转矩形角点，沿每条轨迹分别转换；不含侧倾、俯仰、悬架和垂直运动，也尚未接入第五阶段。

## 测试与文件

36项测试涵盖解析直行/加速/倒车/圆弧、实际jerk边界、UT协方差重构、角度环绕/过宽拒绝、偏移观测Jacobian、正定协方差、IMM权重和混合离差、时序/断测/过期、不改变历史与随机状态、整路径拒绝、甲板偏移速度、命令行和文件回放。

verification/tests.txt保存最终测试输出；docs/ledger.md记录设计取舍和审查问题修复。源码与测试均为独立新文件，第五阶段未改动。本轮没有上传GitHub。

研究来源和具体公式链接见设计稿；参数是本项目初步工程选择。算法有效性、比较优势和可信覆盖需要后续独立实验，不能从这些单测推出。
