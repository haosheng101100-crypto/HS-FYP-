# HS-FYP

本仓库按研究阶段组织。第五阶段保留移动平台着陆的规划算法与已有实验，第六阶段开始研究车辆平台的短期运动预测。

| 目录 | 内容 |
| --- | --- |
| [Stage5_Trajectory_Planning_Algorithm](Stage5_Trajectory_Planning_Algorithm/README.md) | 第五阶段：规划算法、仿真环境、配置、测试和历史实验资料 |
| [Stage6_Platform_Prediction_Algorithm](Stage6_Platform_Prediction_Algorithm/README.md) | 第六阶段：车辆平台预测算法的研究说明、文献和后续实现 |

当前新工作从 [第六阶段说明](Stage6_Platform_Prediction_Algorithm/README.md) 开始。先实现并验证预测模块，再研究无人机择机下降；具体测试情景在算法实现后安排。

## 目录调整

2026-11-05，原“毕设仿真”目录改名为“Stage5_Trajectory_Planning_Algorithm”，新增第六阶段目录。第五阶段源码和实验文件随目录迁移，历史结果仍对应各自的实际运行版本。

旧记录中的“毕设仿真/”指向第五阶段目录。引用历史提交的链接保留原路径；新版本的工作目录使用英文名称。Linux 上已有部署路径和旧实验归档无需跟随本次仓库改名批量迁移，运行前按实际环境核对。

## 下载与第五阶段检查

```bash
git clone https://github.com/haosheng101100-crypto/HS-FYP-.git
cd HS-FYP-/Stage5_Trajectory_Planning_Algorithm
python scripts/verify_source_package.py
python -m unittest discover -s tests -v
```

依赖与仿真部署见第五阶段原有说明。第六阶段目前只有研究说明和引用，尚无可运行的新预测程序。
