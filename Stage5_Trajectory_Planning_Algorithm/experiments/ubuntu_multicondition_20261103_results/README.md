# 二维20工况实测与复核结果（2026-11-03）

20个不同工况各尝试一次。复核分类：{'complete_function_success': 15, 'coverage_failure': 2, 'environment_or_insufficient_evidence': 1, 'algorithm_control_or_physical_failure': 1, 'correct_safe_exit': 1}。另保留batch001一次初始化中断，合计21次物理尝试；没有恢复旧92场计划。

原运行固定caab35f1ab49ff9a21bf3dae49026b43caa440e9，控制固定748b27e28be42ad294769105c5c899d3e9ecfb25；原255项Linux测试通过。物理运行没有改控制/接触门限、预算、几何或增加未来运动信息。HS_NO_GUI=1仅关闭查看器；Gazebo服务器/传感器/物理/飞控真实运行。

补充复核修正了两个检查器问题：ContactFrame序列化有leg_names但无active；M15第二次是重新对齐/新计划触发，不再要求首次下降的负垂速，且核对已激活计数后允许同一仿真时刻的新计划。6项新增回归与全套261项Linux测试通过、无跳过。原始审计和result.json保留；独立副本对原数据重新审计，不伪称修正后入口重新飞过。

|工况|原结论|复核结论|实际落台|安全检查|覆盖|
|---|---|---|---|---|---|
|M01|完整通过|完整通过|True|True|True|
|M02|完整通过|完整通过|True|True|True|
|M03|完整通过|完整通过|True|True|True|
|M04|完整通过|完整通过|True|True|True|
|M05|完整通过|完整通过|True|True|True|
|M06|完整通过|完整通过|True|True|True|
|M07|物理/控制验收未通过|覆盖不足|True|True|False|
|M08|完整通过|完整通过|True|True|True|
|M09|完整通过|完整通过|True|True|True|
|M10|完整通过|完整通过|True|True|True|
|M11|完整通过|完整通过|True|True|True|
|M12|环境中断/回执不足|环境中断/回执不足|True|True|True|
|M13|完整通过|完整通过|True|True|True|
|M14|完整通过|完整通过|True|True|True|
|M15|覆盖不足|完整通过|True|True|True|
|M16|完整通过|完整通过|True|True|True|
|M17|覆盖不足|覆盖不足|False|True|False|
|M18|物理/控制验收未通过|物理/控制验收未通过|False|False|True|
|M19|完整通过|完整通过|True|True|True|
|M20|正确安全退出|正确安全退出|False|True|True|

M07：实际落台，接触策略回放的误报经隔离复核排除，但落台前尚未完成八字交叉，仍为覆盖不足，不能算完整通过。
M12：已落台、停转，原生进程正常关闭；系统关机中断后处理，监督器退出码缺失。恢复后原物理数据审计通过，但仍单列环境回执不足，不捏造退出码、不重复飞行。前一启动末尾有正常关机标记，触发原因未建立，不能当作已诊断GPU/OOM/自动重启。
M15：原覆盖报告因检查器判定有误失败；两次实际反向、两条重新跟随→对齐→新计划证据链完整，修正后独立复核结论见表。
M17：中段触发后安全退出，平台还未完成所要求的完整反向；安全退出不等于落台或完整覆盖。
M18：临近接触触发退出后仍接触平台，原始失败包含contact_during_failed_attempt_recovery、first_contact_leg_speed、first_contact_contact_time。属于必须修复的实际边界问题，不能用检查器修正掩盖。原物理报告完整保留。

下一步先定位M18近地退出与接触交接的竞争，制定剩余净空/相对运动不足以安全退出时的可验证处理，保留全部原门限；实现后只补该边界及少量基础回归。M07另需在不增加预算、不泄露未来指令的前提下调整世界端激励，使落台前真实完成交叉。M17如需验收完整反向，要明确相应可行净空和扰动条件。不要为凑成功数扩大重复。

精简交付：20张轨迹/高度/净空/速度曲线及注明抽样的数据；case_evidence.tar.gz包含完整逐场审计、原结果、补充复核、实际配置/模型、事件链、全部编号规划请求/响应压缩历史及文件哈希。曲线参考虚线是目标速度积分，不是物理位置；实际初态、首控制与首次下降时刻分开记录。

全频率原始位姿、控制/反馈、DataFlash留本机，没有全量上传GitHub。完整本机同盘归档：/home/sheng-hao/Documents/Codex/2026-10-28/new-chat/outputs/二维多工况归档_20261103/ubuntu_multicondition_full_20261103.tar.gz；5888普通文件，865525423字节；SHA256 `a04aac2501925fa93e90d97723f5f46bfb868b78ed110c2f9b69a008a2e9e5b2`；逐文件回读校验通过，符号链接未展开，未异地备份。

这是声明的理想真值观测、二维定高平台仿真，不代表任意不规则运动、真实视觉或真机已验证。
