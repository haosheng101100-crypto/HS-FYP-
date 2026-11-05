# 第六阶段车辆预测算法设计 v0.1

本轮按用户确认的8步路线推进：先实现第1—4步完整核心，再独立检验。禁止把旧XY外推称为车辆预测算法。旧代码和实验只读，本地新包开发，不推送、不接管飞控。

## 目标与方案

UKF + IMM 同状态多模型，输出不合并的未来样本分支。相比纯XY外推，显式满足车辆非完整运动形式；相比大型网络，不需训练集。UKF/IMM是成熟估计方法，本项目组合不是理论创新。参数是公开的初步工程选择，尚未调参和验证。本设计补充GitHub Stage6要求，不表示最终验收通过。

## 状态、参考点与输入

s=[x,y,psi,v,a,kappa,j,eta]，单位[m,m,rad,m/s,m/s²,1/m,m/s³,1/(m*s)]。前6维对应GitHub模型，新增 j/eta 是为未来输入随时间连续。v有符号，可倒车，psi为车身航向；不由速度方向替代。参考点为低侧滑运动学参考点（虚拟后轴中心）；观测位姿可在已知车体偏移d处。不能默认轴距、转角、控制或未来路线已知。

Observation(stamp,received_at,position_xy,yaw, covariance3x3) 是唯一在线输入，时间单位仿真秒同一时钟。h(s)=[x+cos(psi)dx-sin(psi)dy,y+sin(psi)dx+cos(psi)dy,psi]。观测来自甲板需提供真实安装偏移；目前未知时不能冒充已知，默认d=0仅用于明确的参考点位姿。矩阵必须有限、对称正定。

严格按stamp递增输入，received_at递增且stamp<=received_at；晚到但仍按采样时序的观测在其采样时刻更新，然后按年龄外推至发出时刻。乱序包显式拒绝，不偷偷重排。发出时刻必须>=最新received_at。首观测初始化宽先验；至少5次、0.4秒观测跨度且观测年龄<=0.2秒才ready。断测>1秒重置，不跨越长时间空洞声称已跟踪。

## 连续模型与数值积分

所有模型：xdot=v cos psi, ydot=v sin psi, psidot=v*kappa。
vdot=a; adot=j-lambda_a*a; kappadot=eta-lambda_k*kappa;
jdot=-j/tau_j; etadot=-eta/tau_eta。

四模型参数（名称表示近期运动倾向，不是已知动作）：
CV: lambda_a=3,lambda_k=3,tau_j=.4,tau_eta=.4；
CA: lambda_a=0,lambda_k=3,tau_j=.7,tau_eta=.4；
CT: lambda_a=3,lambda_k=0,tau_j=.4,tau_eta=.5；
CTVAR: lambda_a=0,lambda_k=0,tau_j=1,tau_eta=1。

因此实际纵向jerk为j-lambda_a*a，实际曲率变化率eta-lambda_k*kappa；限制和审计必须用实际导数而不是只检查隐藏驱动j/eta。

RK4，子步<=.05秒；所有角度残差用wrap。UKF高斯的sigma点使用未裁剪的状态方程，防止剪裁后仍声称原高斯覆盖；物理约束独立检查。

## UKF

n=8，scaled UT alpha=1,beta=2,kappa_UT=0；lambda=alpha²(n+kappa_UT)-n；Wm0=lambda/(n+lambda)，Wc0=Wm0+1-alpha²+beta，其余1/[2(n+lambda)]。sigma点为均值±sqrt(n+lambda)*Cholesky(P)列。

每个子步传播sigma点，圆周角均值采用以中心角为锚的wrap残差加权（局部角分布假设）。任何sigma点wrap前检查sqrt(8*Ppsi,psi)<pi/2；否则抛LocalAngleError，拒绝将宽角度误差误算成精确分布。predict失败不提交部分状态，forecast年龄传播失败时保留模式但返回degraded和null状态；P=sum Wc dx dx^T+diag(q)*dt。q仅j/eta两维非零，四模型(qj,qeta)分别(.03,.03),(.25,.03),(.03,.10),(.25,.25)，单位为对应驱动状态单位²/秒。它是未来输入未知的工程先验，尚未识别的结构误差不能声称已覆盖，后续独立残差校准处理。粒子每子步同样加sqrt(q*dt)*Normal(0,I)，只影响j/eta，保持位置/速度/加速度/曲率连续演化；过程噪声为一阶离散近似，子步上限公开，需做步长敏感性检查。

观测预测由h作用于重新采样的sigma点（包含最新Q）。S=sum Wc dz dz^T+R, C=sum Wc dx dz^T, K=solve(S,C^T)^T。创新航向wrap；m+=K*innovation, P-=KSK^T。高斯log likelihood=-.5*(3log2pi+logdetS+innovation^T solve(S,innovation))。仅修复舍入级PSD误差；明显负特征值抛错；禁止静默掩盖数值发散。

初始位置由测量点减旋转偏移得到。初始速度0，Pvv=1, Paa=.5, Pkk=.5, Pjj=.5,Petaeta=.5，航向位置协方差通过offset逆变换Jacobian传播。低速不以yaw_rate/v估曲率，不人为缩小曲率方差。

## IMM

连续时间跳转率矩阵G，对角-0.45/s，其余0.15/s；T(dt)=expm(G*dt)，因此适配非均匀观测间隔。
c_j=sum_i mu_i*Tij；mix_ij=mu_i*Tij/c_j；每目标模型用混合均值/协方差初始化，再predict/update。协方差包含模型间均值离差；角度用wrap残差。

后验mu_j=softmax(log(c_j)+logLikelihood_j)。保留全部4模型，无未来分支删减或平均成唯一行驶路线。混合只用于当前滤波交互，未来输出各分支。

## 未来样本与接口

forecast(now,horizon=2,step=.1,particles_per_mode=12,seed=...)复制当前后验并按相同UT和Q传播至now，不修改在线滤波器，使用局部RNG。每模型抽样初态与每.05秒连续j/eta随机驱动；输出20时刻的8维轨迹、位置/速度/航向联合样本、模式后验权重、样本数及拒绝数。保持完整轨迹样本关联，不独立按时间画圆。

物理测试默认边界：|v|<=3, |a|<=1, |kappa|<=1, |adot|<=2, |kappadot|<=1, v²|kappa|<=2.5。都是测试配置，非真实车辆标定。初始不合规或积分检查不合规的样本拒绝，最多每模型尝试20*N次。不能修改通过范围或剪掉分支仍保留原概率解释。接收样本的模式权重仅为观测后验标签，拒绝采样后不作为经过校准的未来概率；采样包络不是确定性可达外包。零接收模型也保留其权重和失败状态；任一模型接收数量不足N则总状态degraded，不把剩余模型重新归一化伪装全部覆盖。

输出status=[warming_up,stale,ready,degraded]、observation_age、observed_duration、estimate_time、issued_at、models[{name,posterior_weight,state,covariance,trajectories,attempted,rejected}]。统计预测集合=尚未校准；physical_reachable_set=null，scope明确。估计中超过物理边界的均值记录violations，不能宣称均值一定物理可行。物理采样只在离散子步核查，不能宣称严格连续时间保证。

## 对外扩展

甲板中心pd=p+Rpsi*d，vd=v[cospsi,sinpsi]+omega*J*Rpsi*d，omega=v*kappa。角点用同一样本同一时刻旋转，保留全轨迹关联。当前只需算子单测；整体模块后续接入。

误差校准、失信监测、独立车辆仿真、着陆联调为第5—8步；核心实现前不得先挑情景调参数。不得将核心单测称为仿真验收。后续独立运行分组拆分development/calibration/test，再比较CV/CA、单CT、多模型和校准多模型。

## 核验

解析直行/加速/倒车/恒曲率圆弧；物理边界、实际jerk而非隐藏j；角度跨pi、低速不辨识、偏移观测；线性测量更新；PSD和极小概率log运算；IMM混合方差/跳转矩阵半群；未来预测不修改滤波器、不调用全局RNG；未来/乱序/过期/断测；粒子拒绝诚实输出；甲板偏移角速度项。

来源：GitHub Stage6说明（README blob 60bf1b5824f54183667a300649dd8f11a4da4646）；Julier & Uhlmann 2004 UKF；Särkkä/Hartikainen EKF/UKF toolbox §4 IMM公式。数值与模型参数是本项目选择，不借用文献保证。


## 交付接口补充

Python CLI: python -m stage6 --input poses.jsonl --output predictions.jsonl。严格五字段观测，每次收到观测后按receipt检查默认0.2秒发出周期；独立调用forecast(now)可用于无观测时的时钟查询。新输出独占创建；不覆盖既有文件，坏输入清除本次新建的不完整输出。每个packet带完整默认配置、物理边界、UT参数和随机种子。

引用：
- 项目要求：https://github.com/haosheng101100-crypto/HS-FYP-/blob/main/Stage6_Platform_Prediction_Algorithm/README.md （读取blob 60bf1b5824f54183667a300649dd8f11a4da4646）
- UKF原论文：Julier & Uhlmann (2004), Unscented Filtering and Nonlinear Estimation, DOI 10.1109/JPROC.2003.823141；https://www.fs.isy.liu.se/Edu/Courses/ObserversPhD/CourseMaterial/Julier_UKF.pdf
- 工具作者技术手册：Hartikainen, Solin, Särkkä, Optimal Filtering with Kalman Filters and Smoothers (2011), §3.3、§4.1.1；https://www.cic.ipn.mx/~pescamilla/Sensor_Fusion/software/ekfukf_1_3_manual.pdf

上述来源支持通用方法；实际噪声、松弛模型、物理拒绝采样和数值阈值均为项目选择，未声称继承来源的性能保证。
