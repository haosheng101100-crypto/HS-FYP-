# 第六阶段参考文献与方法来源

整理日期：2026-11-05

本轮针对车辆运动预测、物理约束、多模态预测和不确定性校准，阅读了以下20篇文章的方法正文及相关实验、结论，包括18篇研究文章或预印本和2篇综述。阅读未包含代码复现或全部证明的独立复核。

表中编号与 [阶段说明](README.md) 的引用一致。链接指向出版页或实际阅读的原文版本；采用预印本时，以对应版本的方程和条件为准。机械臂可达集验证、自车模型学习等相邻研究单列用途，不作为已经完成车辆平台预测的证据。

|序号|文章与原文来源|阅读发现、适用条件及本项目取舍|
|---|---|---|
|01|[A survey on motion prediction and risk assessment for intelligent vehicles，2014](https://link.springer.com/article/10.1186/s40648-014-0001-z)|区分物理、机动和交互预测；比较CV/CA、CTRV/CTRA、转角模型、滤波、混合模型与采样。单模型高效但难描述未来行为切换；本项目不再把XY自由加速当作正常车辆完整运动约束。|
|02|[A Survey on Vehicle Trajectory Prediction Procedures for Intelligent Driving，2025](https://www.mdpi.com/1424-8220/25/16/5129)|从感知、预测、决策和应用组织方法，涉及IMM、滤波、贝叶斯、深度预测；只作路线和引用索引，二手转述不能替代被引算法的原文与实现验证。|
|03|[Multimodal Trajectory Prediction for Diverse Vehicle Types in Autonomous Driving with Heterogeneous Data and Physical Constraints，2024](https://www.mdpi.com/1424-8220/24/22/7323)|MTP-HPC以CVAE/GRU预测加速度和转角分布，再经自行车模型积分，结合地图、车辆类型和交互；nuScenes验证。借鉴在控制量空间建模不确定性；有运动学解码器不自动满足所有轮胎、加速度及执行器限制。|
|04|[Trajectron++: Dynamically-Feasible Trajectory Forecasting With Heterogeneous Data，2020](https://arxiv.org/html/2001.03093)|图结构、CVAE、控制分布与动力学积分。正文明确：外部车辆的轴距、质心和转角难以在线估计，采用动态扩展独轮模型。对本项目最关键的是非完整约束和不依赖未知转角传感器的建模思路。|
|05|[Deep Stochastic Kinematic Models for Probabilistic Motion Forecasting in Traffic，2024](https://arxiv.org/html/2406.01431)|对速度、加速度、速度/航向、加速度/转角四种参数化推导概率传播；关联跨时刻的均值与方差。借鉴不确定性随动力学传播，不能每个时刻独立画圆。线性化和独立性假设需检查，不能将近似方差称为最坏情况边界。|
|06|[Motion Transformer with Global Intention Localization and Local Movement Refinement，2022](https://arxiv.org/html/2209.13508)|局部注意力编码道路/历史，静态意图查询与动态查询迭代细化多模态预测，Waymo验证。借鉴保留多个未来分支；地图、大训练集和算力要求使其不宜直接当现阶段最小实现。|
|07|[Query-Centric Trajectory Prediction，2023](https://openaccess.thecvf.com/content/CVPR2023/papers/Zhou_Query-Centric_Trajectory_Prediction_CVPR_2023_paper.pdf)|QCNet以各元素局部时空坐标缓存编码，循环无锚提议后有锚细化，Argoverse验证。借鉴流式历史缓存及多分支输出；它解决预测架构效率，不提供本任务所需的接触保证。|
|08|[MotionDiffuser: Controllable Multi-Agent Motion Prediction using Diffusion，2023](https://arxiv.org/html/2306.03083)|联合未来轨迹扩散分布、置换等变去噪、PCA压缩和代价引导采样。能表达复杂多峰，但引导采样不是严格物理可达外包；第一版不引入完整扩散网络。|
|09|[Towards Generalizable Trajectory Prediction Using Dual-Level Representation Learning And Adaptive Prompting，2025](https://arxiv.org/pdf/2501.04815)|PerReg+通过掩码重建、自蒸馏、register查询和提示微调改善多模态与跨域适应。阅读的是2025-01-08预印本PDF，正式CVPR版需另核对。不能把道路大数据上的泛化直接当本项目车台数据的覆盖保证。|
|10|[GoIRL: Graph-Oriented Inverse Reinforcement Learning for Multimodal Trajectory Prediction，2025](https://arxiv.org/html/2506.21121)|道路图转网格、最大熵IRL推断策略，采样路线后以Bézier控制点生成并细化轨迹。说明Bézier也可用于预测，但平滑不等于物理或统计保证。地图和训练需求较重，作为备选而非首版。|
|11|[Safe Planning in Dynamic Environments Using Conformal Prediction，2023](https://arxiv.org/html/2210.10254)|使用独立校准轨迹的预测误差构造区域，结合MPC；通过风险分配/并集界处理多时刻事件。校准对象交换性等条件不能删掉；随时间重叠的窗口不能直接当独立样本。|
|12|[Adaptive Conformal Prediction for Motion Planning among Dynamic Agents，2023](https://arxiv.org/html/2212.00278)|按预测步长延迟评价历史预测，再在线更新误覆盖参数。重要限制：正文的理论主要针对一步平均覆盖，多步延迟误差应用不自动具有相同保证；窗口突增也可能导致MPC不可行。借鉴严格因果的误差队列。|
|13|[Conformalized Adaptive Forecasting of Heterogeneous Trajectories，2024起](https://arxiv.org/html/2402.09623)|CAFHT在ACI上增加轨迹级校准，使用整段最大超出量、可乘性分数、额外划分选择学习率。依赖校准轨迹与测试轨迹交换性；正文主要是逐步揭示观测的一步预测，不直接等同一次发出未来3秒的整条管。借鉴联合轨迹评分和数据划分。|
|14|[Safe, Out-of-Distribution-Adaptive MPC with Conformalized Neural Network Ensembles，2024起](https://arxiv.org/html/2406.02436)|SODA-MPC用集成分歧检测OOD，校准阈值，触发可达集控制备用。读取版本更新于2025。检测器误报保证不等于必然检测全部失效；保守备用也可能损害任务效率。借鉴预测失信状态和显式降级机制。|
|15|[Safety Evaluation of Motion Plans Using Trajectory Predictors as Forward Reachable Set Estimators，2025](https://arxiv.org/html/2507.22389v1)|FORCE-OPT从GMM提取椭圆并集，用模式概率分配面积预算，再通过校准修正尺度，贝叶斯监测/备用处理失信。nuScenes及合成不安全计划验证。借鉴保留分支并集，不把多峰强行压成单个大高斯。统计FRS与包含所有允许控制的确定性FRS必须分开。|
|16|[Verification of Neural Reachable Tubes via Scenario Optimization and Conformal Prediction，2024](https://proceedings.mlr.press/v242/lin24a/lin24a.pdf)|讨论HJ神经可达管验证、容忍离群违规的场景方法与共形方法关系。目标是验证学习的安全集合，并非直接预测一辆车未来位置。借鉴明确违规率、置信度、样本预算及验证条件，避免把有限抽样当严格可达性证明。|
|17|[Adaptive Conformal Inference Under Distribution Shift，2021](https://arxiv.org/html/2106.00170)|通过alpha更新适应误覆盖，实现长时间覆盖频率控制。长期平均覆盖不能解释为每个时刻或每种机动都达到目标；分布突变时仍有适应滞后，原文也指出延迟标签是后续问题。|
|18|[Conformal Time-Series Forecasting，2021](https://papers.nips.cc/paper_files/paper/2021/file/312f1ba2a72318edaaa995a67835fad5-Paper.pdf)|CF-RNN把多步点预测转成区间，按步长误差及风险分配取得整段覆盖，依赖交换性。明确多时刻联合覆盖；不能从逐点95%直接得出整段95%。|
|19|[Conformalized Reachable Sets for Obstacle Avoidance With Spheres，2024起](https://arxiv.org/html/2410.09924)|CROWS学习机械臂球形前向占用及梯度，以校准缓冲包住原集合，含仿真与硬件实验。这是机械臂而非车辆预测，仅借鉴学习集合的误差校准和整体几何传播；不照搬多事件概率组合。|
|20|[Vehicle Prediction Model for Enhanced MPC Path Tracking in Formula Student Driverless，2026](https://arxiv.org/html/2606.10732)|名义自行车模型、离线贝叶斯线性残差、在线稀疏GP修正，实际赛车验证。它使用自车转角和控制命令，不能把已知控制条件下的系统模型预测误认为外部未知驾驶意图预测。借鉴小数据残差修正，首版不直接套用其输入。|

## 使用这些文献时的约定

综述用于了解方法和寻找原始引用，具体实现仍需核对原论文。大型网络的方法可作为比较对象，是否引入取决于数据和计算条件。各论文的概率保证都有相应条件，不能省略后移用到本项目。

阶段说明中的曲率模型、轻量多模型组合、接口和实现顺序，是本项目据此提出的设计选择。实现端应记录实际采用的公式及来源，不能把方法组合本身直接写成新的理论贡献。
