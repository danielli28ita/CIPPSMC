# 第三方来源

- PX4-Autopilot v1.15.4，BSD 3-Clause；已保留源码版权头和根 LICENSE。完整上游及子模块由固定提交获取，各依赖保留其原始许可证。
- 冻结控制器、机载任务和集成文件来自当前项目工作树，来源/基线/文件 SHA256 位于 firmware/BASELINE.json。没有从其他实飞日志导入参数或样例结果。
- 离线轨迹测试中的 LemniscateTrajectory 和 TrajectorySample 来自项目既有复现参考生成器；原始包许可证副本见 LICENSES/BPPSMC-reference.txt。
- Python 库和 Gazebo 通过系统/Python 包管理器安装，没有将其二进制库冒充本项目源码。版本锁位于 environments/，其许可证以相应上游分发内容为准。
- 唯一 ULog 由用户提供，用于本包声明的飞行样例复算；原始字节和校验和保留。

本包不包含历史论文汇总表、其他控制器实飞样例或其他 ULog。新加入的打包/分析脚本使用根 LICENSE 的 BSD 3-Clause 条款；现有文件的原始声明保留。
