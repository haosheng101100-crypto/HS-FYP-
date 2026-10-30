# 论文计算对照入口

仅回放已记录的规划请求，不启动飞机，不代表新的闭环仿真。复用当前主规划器，保留候选、评分及审计，仅切换评分下界剪枝。研究记录与输出留在运行机器，不上传原始数据。

Ubuntu 在已安装仓库依赖的环境运行：

```bash
git pull --ff-only
python -m unittest discover -s 论文仿真/paper_20261030/tests -p test_paper_search_benchmark.py -v
python 论文仿真/paper_20261030/search_benchmark.py --help
sha256sum landing/receding_landing.py
python 论文仿真/paper_20261030/search_benchmark.py \
  --inputs /absolute/path/to/recorded/replay \
  --config /absolute/path/to/controller.json \
  --expected-planner-sha PASTE_THE_ACTUAL_SHA256 \
  --output /absolute/path/to/new-paper-run --repeats 3
```

输入为含 `request` 的 `response_*.json`；不是随意生成的成绩。首次可加 `--limit 2` 检查兼容性。输入目录和配置取自本轮实际部署包及日志，而非旧冻结版本。输出目录必须不存在。异常立即停止，已完成配对逐条保存；重跑使用新目录，不覆写旧记录。

每份请求两种模式各预热一次，计时不含构建和预热，交替AB/BA顺序。记录机器环境、源码／配置／几何哈希、请求来源、审计、评分、选择时长及耗时。去重以完整请求哈希为准；重复计时用于测计算波动，不扩大独立飞行样本数。`solve_calls` 不等于底层锥求解器调用次数。

`all_equivalent` 仅要求两种模式有解状态一致，有解时审计通过、选择时长一致且评分容差内一致；两者无解只算无解一致，绝不算落台。请逐项统计候选报错，不把有返回值或剪枝数量当作实时性能证明。

## 之后需要的闭环试验

同一版本下两组分别跑匀速／变速／转弯／组合机动，各10个预声明匹配初态或扰动种子。另一组只关闭剪枝，其余约束、平台动作、参数不变。需先在Ubuntu执行后端显式接通 `prune_score_dominated` 参数并记录实际值；本入口本身不提供这个闭环开关，禁止仅改离线脚本就声称完成闭环对照。

另做延迟、噪声等级与急变退出检查。落台、正常退出、任务失败和环境中断分开统计，接触后支撑与正常停转继续独立审核。原有九场景是功能检查，不是大样本成功率。正式批次启动前冻结种子、版本与指标，不在看到结果后择优换条件。
