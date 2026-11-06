# HS-FYP

本仓库按研究阶段组织。第五阶段保留移动平台着陆的规划算法与已有实验，第六阶段开始研究车辆平台的短期运动预测。

| 目录 | 内容 |
| --- | --- |
| [Stage5_Trajectory_Planning_Algorithm](Stage5_Trajectory_Planning_Algorithm/README.md) | 第五阶段：规划算法、仿真环境、配置、测试和历史实验资料 |
| [Stage6_Platform_Prediction_Algorithm](Stage6_Platform_Prediction_Algorithm/README.md) | 第六阶段：车辆预测算法、独立评价与四轮平台原生降落联调 |

第六阶段已实现 UKF/IMM 车辆预测与甲板映射，并完成选定匀速、变速的前台原生甲板降落，以及声明预测中断后的安全退出核验。结果范围、复现限制及完整下载见[原生闭环交付说明](Stage6_Platform_Prediction_Algorithm/releases/native_closed_loop_20261107.md)。两次降落与一次退出分别统计，不能据此推断总体成功率。

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

依赖与仿真部署见第五阶段原有说明。第六阶段源码、测试和所选原生实跑证据见[最新交付](Stage6_Platform_Prediction_Algorithm/releases/native_closed_loop_20261107.md)。原生运行还需说明中列出的 Gazebo、飞控及模型环境。

