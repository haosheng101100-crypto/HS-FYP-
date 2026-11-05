# 仿真项目

用户指定的毕设主工程已统一移入 [毕设仿真](毕设仿真/README.md)。算法、配置、模型、测试、历史数据与交接记录均在该目录内。

```bash
git clone https://github.com/haosheng101100-crypto/HS-FYP-.git
cd HS-FYP-/毕设仿真
python scripts/verify_source_package.py
python -m unittest discover -s tests -v
```
