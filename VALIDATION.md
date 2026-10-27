# 复现包验证记录

日期：2026-10-26，Ubuntu 22.04 amd64。完整环境见 environments/versions.json，Python 依赖通过新建 venv 安装，没有使用作者系统的 NumPy 兼容路径。

## 已执行

| 项目 | 结果 | 证据 |
|---|---|---|
| 冻结源码来源 | 精确 v1.15.4 基线、32 个依赖提交、28 个 payload 校验通过 | firmware/BASELINE.json |
| 唯一样例与参数 | 仅 log_19，原始 SHA256 一致；1153 项完整参数和 36 项 profile 精确导出 | data/PROVENANCE.json、params/ |
| 离线轨迹与状态机 | C++ 参考几何、数值导数、边界、完整阶段、多圈、中止和 EKF 平移检查通过 | validation/offline_tests.log |
| 来源/参数测试 | 5 项通过，包含完整参数 JSON/QGC 文本往返和 payload 校验 | validation/offline_tests.log |
| 样例离线复算 | 14 个指标满足 rtol=1e-6、atol=1e-8 | validation/sample_recompute.log、data/expected_metrics.json |
| 实机目标构建 | px4_fmu-v6c_onboard 成功；默认 PID、任务关闭；编译 BPPSMC 调用和轨迹模块检查通过 | validation/px4_fmu-v6c_onboard.build.* |
| SITL 目标构建 | px4_sitl_default 成功；编译分支检查通过 | validation/px4_sitl_default.build.* |
| 本地模拟完整任务 | 校准模型映射后 BPPSMC 两圈、降落、普通上锁完成，参数 profile 检查无差异 | validation/sitl_result.json、sitl_lifecycle.json |

本轮构建采用本机 Git 对象镜像加速，在独立新 checkout 中恢复固定提交和包内 payload。没有复制原开发目录的构建结果或脏依赖工作树。默认网络获取入口和镜像入口共用同一套基线/依赖/文件检查；没有在全新 Ubuntu 系统上重新执行 apt 安装或完整网络下载流程。

## 模拟机映射校验及结果

第一轮保持原 X500 启动配置 MPC_THR_HOVER=0.60，任务流程完成，但八字 RMSE 约 **0.5086 m**、最大约 **0.7647 m**，未满足本包工程精度门槛。该结果保留于 validation/sitl_native_mapping_result.json，没有隐藏为“全通过”。

从模型近稳输出确认映射失配后，第二轮仅在模拟启动脚本中使用 X500 专用 MPC_THR_HOVER=0.73，保持 log_19 的 BPP/F8 参数，重新运行完整任务：

- 八字误差 RMSE：**0.05983 m**，最大：**0.13236 m**。
- 两圈完整执行，记录 **9901** 个 BPPSMC 调试样本，任务落地普通上锁。
- 满足本包工程门槛 RMSE <0.15 m、最大 <0.30 m。
- 模拟机为 SYS_AUTOSTART=4001、GPS/模拟传感器，EV_CTRL=0；实机样例为 4050、外部定位，不能混为同一工况。
- 个别 F8 默认参数在 logger 启动后模块构造，没有作为 ULog 初始值写入；验证程序明确记录哪些值从对应编译参数元数据补齐，不冒充独立日志观测。

仿真专用 0.73 不写入实机参数文件，也不修改冻结控制器源码。本轮生成的两个模拟 ULog 留在被排除的 workspace，**压缩包中仍只有 log_19 一份 ULog 样例**。

## 实机固件产物

板型 ID=56，FLASH 1,900,252 / 1,966,080 bytes（约 96.65%）。附 .px4、ELF、parameters.xml、airframes.xml 和构建清单。固件默认 MPC_CTRL_METHOD=0、F8_ENABLE=0；样例参数通过独立 profile 配置。

原始样例确实记录了实机完整任务；本轮重建没有连接或刷写真实飞控。未证明本轮固件与当时历史飞行二进制逐字节一致，不声称本轮新实机飞行已验证。

## 未覆盖

- 新硬件刷写、实体 HITL、不同动力/电池/载荷的重新实飞。
- ROS/动捕/外部定位上游发布器的设备级重建，其型号和源码未提供。
- 额外 `--extra-checks` 的实际 RC/Kill/失联回归；本轮仅执行默认完整模拟任务，相关状态机部分有离线测试。
- 多次随机扰动统计、论文全部实验、其他控制器性能对照或字节级固件构建一致性。
- 样例 5 V 低压文本的物理电源根因；记录被保留并明确说明。

候选压缩包已在独立解压目录中新建 Python venv，执行完整性检查、离线测试和样例复算，全部通过；结果见 validation/release_checks.json。最终包加入这些验证记录后重新生成，并再次检查归档完整性。
