# ET-BPPSMC for PX4

Compact research implementation of an event-triggered brachistochrone
prescribed-performance sliding-mode position controller for PX4 multicopters.

本仓库保存 ET-BPPSMC 的核心实现和复现入口。它不是完整 PX4 fork，也不包含
构建产物、ULog、论文源文件或内部项目台账。固件部分以可校验 overlay 的形式
发布：先获取精确的 PX4 v1.15.4 基线，再由脚本应用 9 个配套修改文件。

> 当前仓库为论文完成前的私有研究快照。公开版本、论文引用信息和归档数据将在
> 论文定稿及最终复现审核后补充。

## 版本锁定

| 组件 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04 LTS |
| PX4 Autopilot | v1.15.4, commit `99c40407ffd7ac184e2d7b4b293f36f10fe561ef` |
| ROS 2 | Humble |
| `px4_msgs` | `release/1.15` |
| Micro XRCE-DDS Agent | v2.4.2 |
| SITL | Gazebo Garden, `gz_x500` |

PX4 v1.15 使用 v2.x Micro XRCE-DDS Client，因此不要用 v3.x Agent 替代
上表版本。

## 仓库内容

```text
firmware/
  changed/                 PX4 相对路径下的 9 个完整修改文件
  against-v1.15.4.patch    可审阅的统一补丁
  BASELINE.json            基线与逐文件 SHA-256
reference_generator/       ROS 2 期望位置、速度、加速度生成器
analysis/                  ULog 时间对齐与核心指标函数
scripts/apply_overlay.py   基线检查、应用和结果校验
tests/test_repository.py   仓库离线一致性测试
```

固件修改包括：

- `PositionControl.cpp/.hpp`：ET-BPPSMC、事件触发状态和控制器选择；
- `MulticopterPositionControl.cpp/.hpp`：起飞状态、解锁时刻和控制器使能；
- `BppsmcDebug.msg` 及消息注册；
- logger 话题注册；
- X500 SITL airframe 参数；
- 默认未启用的对比控制器所需 `etimr_gains.h`。

`BASELINE_CONTROLLER=0` 选择 ET-BPPSMC。当前源码还保留研究期间使用的比较
控制器实现，因此 9 个文件必须作为整体使用，不能只复制
`PositionControl.cpp`。

## 恢复固件

```bash
git clone --branch v1.15.4 --single-branch --recursive \
  https://github.com/PX4/PX4-Autopilot.git PX4-Autopilot

git clone https://github.com/danielli28ita/BPPSMC.git

python3 BPPSMC/scripts/apply_overlay.py PX4-Autopilot --check-only
python3 BPPSMC/scripts/apply_overlay.py PX4-Autopilot
```

恢复脚本会拒绝：错误的 PX4 commit、非干净工作树、基线文件 SHA 不匹配、
仓库快照损坏或重复应用。它只修改指定的独立 PX4 checkout。

## 编译

SITL：

```bash
cd PX4-Autopilot
make -j4 px4_sitl_default
make px4_sitl gz_x500
```

Pixhawk 6C 仅编译、不上传：

```bash
export PATH=/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH
cd PX4-Autopilot
make -j4 px4_fmu-v6c_default
```

2026-09-20 的隔离构建中，SITL 与 `px4_fmu-v6c_default` 均通过。该板级配置的
FLASH 余量仅约 820 B；修改固件后必须重新检查构建报告。构建通过不等于飞行
验收通过。

## 构建参考指令生成器

先准备 ROS 2 Humble 和与 PX4 v1.15.4 对应的 `px4_msgs`：

```bash
mkdir -p reference_ws/src
ln -s "$(pwd)/BPPSMC/reference_generator" reference_ws/src/px4_reference_generator
ln -s /path/to/px4_msgs reference_ws/src/px4_msgs

source /opt/ros/humble/setup.bash
cd reference_ws
colcon build --symlink-install --packages-select px4_reference_generator
source install/setup.bash
```

示例 SITL 运行：

```bash
ros2 launch px4_reference_generator reference_generator.launch.py \
  mission_overlay:=/path/to/BPPSMC/reference_generator/config/spawn_relative_mission.yaml
```

历史任务配置会自动请求 Offboard 和解锁，并以 5 m 高度运行，仅用于独立 SITL。
不得直接用于室内实机。新实验建议将 `clock_source` 显式设为
`local_position`；保留 `vehicle_status` 仅用于复现早期约 2 Hz 阶梯参考条件。

## 离线检查

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile analysis/*.py scripts/*.py
```

分析工具依赖：

```bash
python3 -m pip install -r requirements.txt
```

ULog、ROS bag 和批量实验数据不进入 Git。分析时通过命令行或本地数据目录传入，
正式公开数据应作为带校验和的独立归档发布。

## 已验证范围

- 9 文件 overlay 已从干净 PX4 v1.15.4 恢复并逐文件校验；
- PX4 SITL 与 Pixhawk 6C 固件编译通过；
- ROS 2 参考生成器在 Humble 下编译通过；
- 当前仓库离线清单和本地绝对路径检查通过。

尚未由本仓库独立完成的范围包括 Q250 实机飞行、动捕融合验收和公开数据集的
端到端复现。请勿把软件构建结果解释为飞行安全保证。

## 许可

PX4 派生文件保留原版权头并遵循 BSD 3-Clause License，见 [LICENSE](LICENSE)
和 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。公开发布前还将完成一次
文件级版权与引用审核。
