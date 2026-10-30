# 新版评分下界剪枝：Ubuntu 九场景实验

代码提交 `922f9d8478a5727f50f6067bb6ed0e1ad7983bc8`，新声明 `score_pruning_20261030_v1`；独立 session_001。历史原版九场景数据保留在 ../ubuntu_20261029_session002/。

本轮 6 场真正落台、3 场安全退出，9/9 满足声明；退出不计落台。完整结论、原始位姿曲线与剪枝统计见 [结果归纳](结果归纳.md)。全部控制配置、安全门限与原模型沿用原版。

- cases/：逐场独立审计、控制运行状态、原始日志 SHA256、绘图抽样与关键回放。
- 每场 plot_samples.csv：约 25 Hz 的实际机体原点与甲板顶面坐标；用于绘图，不能代替全频率原始位姿进行严格运动审计。
- replay/：选取的原始规划器请求响应和接触/退出转换附近的三条控制记录；selection.json 标明源文件哈希与行号。这不是完整飞行回放。
- configuration/、declared_protocol.json、environment.json：实际实验配置、当前源码声明及运行环境。
- planner_statistics_and_results.json：逐场结果、solve() 调用数与剪枝数量。solve() 调用数包含预检拒绝，不等同于底层锥优化器调用数。

完整位姿、DataFlash、控制历史与全部求解器请求响应保留在 Ubuntu，本机另做全量归档；GitHub 使用精简同步范围。目录文件由 SHA256SUMS.json 校验。
