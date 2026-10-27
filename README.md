# Cycloid-Inspired Prescribed Performance Sliding-Mode Position Control of Quadrotors With Bounded Time-Varying Gains

简称 **BIPPSMC**。本仓库是该论文位置控制器的实飞记录：PX4 v1.15.4 源码修改、唯一实飞日志，以及按论文误差图和加速度图的版式重绘的曲线。

**唯一实飞样例是 `log_19_2026-10-19-10-01-54.ulg`。** 下面三张图、样例参数和 `data/REPORT.md` 都只来自这一条日志。原始日志按字节保留，SHA256 为：

```text
34d4087e9f9f53c8364c87fd2501edbf2f88131b98b14af4910c027afb3eea7e
```

## 图

时间零点是这条日志的起点，不是论文软件在环实验的解锁时刻。位置误差是估计位置减去参考。垂向面板里的灰色带是该轴记录的走廊 \(\rho_c\pm b\)；平面面板不画走廊，避免半宽盖住误差曲线。短竖线是走廊中心重置。加速度图中的点线是该轴指令限幅。水平轨迹只取八字段。

![Per-axis position error](figs/error_time.png)

*分轴位置误差。\(e_x\)、\(e_y\)、\(e_z\) 分别是北、东、下。*

![Commanded acceleration](figs/acceleration_time.png)

*位置环加速度指令 \(a_{sp,x}\)、\(a_{sp,y}\)、\(a_{sp,z}\)。*

![Horizontal trajectory](figs/trajectory_xy.png)

*八字段水平轨迹。虚线是参考，实线是估计位置。*

重新出图：

```bash
python scripts/make_paper_figures.py --log data/log_19_2026-10-19-10-01-54.ulg --out figs
```

## 源码、数据和文档

| 内容 | 位置 |
|---|---|
| 固件修改与基线 | [firmware/](firmware/) |
| 实飞日志 | [data/log_19_2026-10-19-10-01-54.ulg](data/log_19_2026-10-19-10-01-54.ulg) |
| 样例参数 | [params/flight_profile.params](params/flight_profile.params) |
| 指标与口径 | [data/REPORT.md](data/REPORT.md) |
| 论文版式图 | [figs/](figs/) |
| 离线复算 | `scripts/analyze_sample.py --verify` |
| 实机构建与操作 | [docs/REAL_AIRCRAFT.md](docs/REAL_AIRCRAFT.md) |

日志里的参数名 `BPP_*` 和话题 `bppsmc_debug` 保持记录时的名字，这样离线复算能对上这条 ULog。

## 先选择要复现的内容

| 目标 | 入口 | 所需条件 |
|---|---|---|
| 查看样例和结果 | [README 图](figs/error_time.png)、[样例报告](data/REPORT.md) | 解压即可 |
| 重新计算实飞指标 | `scripts/analyze_sample.py --verify` | Python 3.10 与锁定依赖，不需要飞控、ROS 或 Gazebo |
| 轨迹与任务离线测试 | `scripts/test_offline.py` | 上述 Python 环境及 g++ |
| 从源码构建实机固件 | `scripts/reproduce.py build real` | Ubuntu 22.04、ARM 编译器、固定上游源码 |
| 本地仿真完整任务 | `scripts/run_sitl.py --execute-sitl` | 已编译 SITL、Gazebo Harmonic |
| 配置实体飞机 | [硬件与操作说明](docs/REAL_AIRCRAFT.md) | 与实物匹配的动力、传感器、外部定位及人工接管配置 |

本包包含完整修改集、精确基线和依赖提交、获取/构建程序，以及本轮重建的实机固件。**它不是离线系统镜像**：首次安装系统/Python 依赖及获取完整 PX4 上游源码需要网络。固件源代码恢复不依赖作者电脑的目录、旧构建缓存或已有 ROS 工作区。

## 1. 最短离线复算流程

以下命令在解压后的本包根目录执行。使用 Linux 本地文件系统，路径不含空格。

```bash
# 标准库检查，不安装依赖，不启动飞机
python3 scripts/verify_package.py

python3 -m venv workspace/venv
source workspace/venv/bin/activate
python -m pip install -r environments/requirements.lock.txt

python scripts/test_offline.py
python scripts/analyze_sample.py --verify
```

复算结果写入 `reports/log_19/`，不会覆盖 `data/` 中的原始数据及归档结果。指标容差为相对 1e-6、绝对 1e-8，见 [expected_metrics.json](data/expected_metrics.json)。图像不要求逐像素相同。

样例任务完成两圈、落地和普通上锁，八字三维位置 RMSE 约 **0.1026 m**、水平 RMSE 约 **0.0959 m**。这些是飞控估计位置相对参考的误差，不是独立定位真值，也不是多次飞行的统计性能结论。

## 2. 独立获取源码并构建

测试环境为 Ubuntu 22.04 amd64、Python 3.10、ARM GCC 10.3.1、Gazebo Harmonic 8.15.0。精确软件清单见 [environments/versions.json](environments/versions.json)。本地 Harmonic 适配已包含在冻结修改集中。

如需安装完整构建及仿真依赖，显式执行以下脚本；它使用 sudo/apt，不刷固件、不运行仿真：

```bash
bash scripts/setup_ubuntu22.sh --gazebo
source workspace/venv/bin/activate

python scripts/reproduce.py prepare
python scripts/reproduce.py build all --jobs 2
python scripts/reproduce.py check --binary real
python scripts/reproduce.py check --binary sitl
```

仅构建实机时可省略安装脚本的 `--gazebo`，然后执行 `build real`。仅仿真可执行 `build sitl`。`prepare` 获取固定 PX4 基线和列出的 32 个依赖仓库，再逐文件验证并应用 28 文件修改集。它只写本包 `workspace/`，遇到文件冲突拒绝覆盖，不 reset/clean 用户的开发目录。

PX4 基线为 `99c40407ffd7ac184e2d7b4b293f36f10fe561ef`，即 v1.15.4。完整提交、依赖 URL 和修改文件校验和见 [firmware/BASELINE.json](firmware/BASELINE.json)。修改后的完整文件位于 `firmware/overlay/`，审查用差分位于 `firmware/*.patch`。

本机有同一基线的 Git 镜像时，可以用 `prepare --mirror-source /path/to/PX4-Autopilot` 加速；它只从镜像取得已提交对象，工作树改动由本包 payload 决定。这是可选入口，不是运行前提。镜像模式工作区引用本机对象，不应作为独立源码副本另行搬走。

实机目标是 `px4_fmu-v6c_onboard`，输出位于 `workspace/artifacts/px4_fmu-v6c_onboard/`。构建日志和清单位于 `workspace/*.build.log`、`workspace/*.build.json`。

## 3. 明确启动本地 SITL

```bash
python scripts/run_sitl.py --execute-sitl
```

该命令只启动本包的本地 PX4/Gazebo，通过 localhost 模拟 RC 开关，自动解锁模拟飞机，执行一次完整任务并等待落地上锁。主机不发布轨迹；轨迹由飞控内部 `figure8_trajectory` 生成。没有 `--execute-sitl` 时拒绝启动；发现已有 PX4 进程或 UDP 18570 占用时退出，不清理他人的进程。

默认结果写入 `workspace/results/sitl_<时间>/`。失败会保留日志和 `result.json`，不会把超时当作任务完成。`--controller pid` 可作为用户自行运行的控制分支检查；它不是额外的随包样例。`--extra-checks` 可执行额外接管/中断流程，其验证状态见 [VALIDATION.md](VALIDATION.md)。

SITL 使用 log_19 的 BPP/F8 参数，但采用 X500 自身传感器和机型配置，并显式使用仿真专用悬停映射（`--sim-hover`，记录在 `validation/sitl_result.json`）。该值来自本轮模型平衡校验，不是实机参数，也不是实机建议。原始 X500 启动配置的另一轮结果保留在 `validation/sitl_native_mapping_result.json`。仿真悬停映射、外部定位配置和实机 `flight_profile` 互不导入。仿真用于验证代码、任务和本包工程精度门槛。

可检查自己生成的模拟日志（将路径替换为控制台打印的结果目录）：

```bash
python scripts/check_sitl_result.py workspace/results/sitl_具体时间
```

该检查会报告实际参数、任务完成、圈数、控制器调试样本及跟踪误差。工程门槛为八字 RMSE <0.15 m、最大误差 <0.30 m，属于本包仿真检查，不是论文性能保证。

## 4. 参数和固件的关系

- [params/flight_profile.params](params/flight_profile.params)：36 个从 log_19 精确提取的算法/任务参数，包含 MPC_THR_HOVER=0.21、MPC_CTRL_METHOD=1。
- [params/all_parameters.json](params/all_parameters.json)：1153 个原始初始参数的完整记录。
- `params/all_parameters_REFERENCE_ONLY.params`：完整 QGC 格式参考，包含机体与传感器校准，**不作为其他飞机的通用导入文件**。
- [参数说明](docs/PARAMETERS.md)：默认值、样例值、坐标与生效时机。

本包保持当前源码原样，**没有把样例调参值写入所有固件默认值**。重建固件默认 `MPC_CTRL_METHOD=0`、`F8_ENABLE=0`；选择该位置控制器和启用任务需要在适配后的机体上显式设置参数。刷写保留旧参数时，也必须检查飞控实际回显。

`artifacts/` 中的固件来自本轮冻结源码重建，附 ELF、参数元数据和哈希。未证明它与 log_19 飞行时的历史固件逐字节相同；ULog 的 Git 基线哈希不能证明未提交改动的完整二进制身份。没有为制作本包重新刷写或执行实体飞行。

## 5. 文档与范围

- [算法与源码对应](docs/ALGORITHM.md)
- [实机、外部定位及操作说明](docs/REAL_AIRCRAFT.md)
- [样例指标与分析口径](data/REPORT.md)
- [复现验证记录](VALIDATION.md)
- [变更与来源](CHANGELOG.md)、[第三方声明](THIRD_PARTY_NOTICES.md)

机载任务本身不需要 ROS/DDS；如果实际外部定位链路依赖动捕软件、ROS 或 MAVLink 桥接，这些仍是该机体的外部依赖。电机、桨叶、ESC、电池、载荷和定位设备型号未随样例提供，未知项在硬件说明中明确列出，不能由日志虚构。

HITL、论文多控制器批量实验、其他实飞架次和旧起飞保护修正版不属于本包的复现承诺。本包不复制旧文档中与当前 log_19 不一致的“未实飞”或历史故障结论，也不把一架次成功扩展为所有机体和工况均已验证。
