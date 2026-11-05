# 二维20工况执行计划

用户已在当前对话批准执行GitHub4342b77的《二维多工况仿真要求》。原控制748b27e、既有环境和全部历史数据保留。只处理毕设仿真。

设计：独立世界端场景模块产生有界XY目标，原整形与物理关节力控制执行；控制/预测端看不到未来。原审计保持门限，新覆盖审计区分落台目标、安全退出和覆盖不足。每case一次，失败记录保留。先验证源码与动力学，集中提交场景支持，随后固定commit跑矩阵，再归档和同步证据。

隔离目录work/multicondition_20261103/毕设仿真，是连接器快照，不是Git工作树。原canonical不先修改。GitHub发布采用expected_sha非强制更新。

接口（各任务共享、实现前固定）：
- config/experiments/multicondition_matrix_v1.json 顶层cases列表，20条case_id M01..M20、scenario multi_m01..multi_m20，kind、parameters、trigger、coverage、expected、seed、initial、budget、auditor_version等精确字段。
- tools/multicondition_scene.py: ScenarioProgram(case)，step(elapsed, measured) -> (target_xy, events_list)。measured包含sim_t_s、clearance_m、vertical_speed_m_s、phase、reacquisitions、activated_plan_count；阶段来自实际reference_session，计数来自实际事件。不得给控制器未来指令。
- tools/audit_multicondition.py: CLI RUN --case CASE_JSON，输出results/multicondition_audit.json；分别physics/safety、coverage、landing_objective、classification；环境缺证失败封闭。
- tools/run_multicondition.py: prepare/run/audit，矩阵冻结+不可覆盖的尝试索引、固定源码SHA、逐场声明；运行只能由主代理在提交后启动。适配本机环境参数通过CLI传入。

任务：
- [ ] 场景模块/20精确配置，针对真实运动、测量触发、预算和无未来泄露先测试失败后实现；离线检查曲率/加速度/场地。
- [ ] 审计适配，保持原检查与退出语义；新增曲线/多事件/晚触发覆盖测试。
- [ ] 批次入口与证据/归档/绘图；已有结果不可覆盖，环境中断和算法失败分开，未来无自动重试算法失败。
- [ ] 主代理集成backend/supervisor部署、核对环境、全套Linux测试、独立复查、集中发布实现commit。
- [ ] 20场按序实际运行，留全频率原始数据/请求响应/事件、逐场验收并归纳20行；本机全量归档、精简证据同步，不重复已取消计划。

关键审查：曲线接触前仍弯曲；8字真实过交叉；M15第二反向在重新对齐+新计划后触发；M18实物接触前真实变化才覆盖；最终LAND接触优先，场景变化不能迫使控制保护违规。任何未实现条件均显式记录，不能改工况名假通过。
