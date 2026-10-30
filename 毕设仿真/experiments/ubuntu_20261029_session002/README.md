# Ubuntu 本机九场景：轻量实验资料

本目录用于跨电脑修改算法、查看结果和定位关键行为。六场真正落台、三场安全退出，九场均满足原协议；安全退出不计作落台。

- 三张 PNG：实际 Gazebo 场景与原始位姿曲线。
- cases/：36 份逐场独立审计报告、运行状态、原始日志哈希和关键回放片段。
- 每场 plot_samples.csv：约 25 Hz 抽样的实际机体原点及甲板顶面坐标，可重新绘图。它是绘图数据，不代替全频率位姿记录进行严格速度/加速度审计。
- replay/：首次/末次接受及最后求解响应，以及接触接管或退出转换附近的三个原始控制记录；selection.json 标明来源行号和完整源文件哈希。它不是完整飞行回放。
- configuration/：本轮实际控制、执行、几何、整机包络、平台与世界配置。
- declared_protocol.json、results.json、environment.json 与安装包版本清单：声明、结果和环境接续依据。算法源码和原审计程序仍在仓库 landing/、tools/ 中。

完整 Gazebo 位姿、DataFlash、全部控制历史和求解器请求保留在 Ubuntu；另有本地全量归档。该全量归档没有上传到 GitHub。此前被主机重启中断的 session_001 和环境检查失败记录也保留在全量归档中，不计入本目录九场结果。

目录内文件通过 SHA256SUMS.json 核对。可在仓库根目录运行 `python scripts/verify_source_package.py` 校验整个源码及轻量资料清单。
