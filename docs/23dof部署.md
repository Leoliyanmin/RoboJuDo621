# G1 23-DoF 速度任务真机部署

| 项 | 内容 |
|---|---|
| 生产时间 | 2026-06-29 |
| 生产人 | claude |
| 审计人 | 李晏敏 |

部署目标：在 `RoboJuDo621` 中用 `g1_ih_velocity_23dof_real_keyboard` 把 AGILE 23-DoF
速度策略（`assets/models/g1/unitree/velocity_history_23dof.pt`）部署到真实 23-DoF G1，
键盘驱动，仿真（RoboJuDo sim2sim）不晃、真机要稳。

---

## 零、安装与环境准备

> 固定工作目录：
> - RoboJuDo 部署仓库：`/home/unitree/23dof_deployment/RoboJuDo621`
> - 训练 / sim2sim 参考仓库：`/home/unitree/23dof_deployment/Agile_with_ih`
> - Python 版 Unitree SDK：`/home/unitree/unitree_sdk2_python`（注意：不在 `RoboJuDo621/` 里面）

### 0.1 创建并进入 conda 环境

`RoboJuDo621/pyproject.toml` 要求 `Python >= 3.11`，本机建议统一使用 `robojudo` 环境：

```bash
conda create -n robojudo python=3.11 -y
conda activate robojudo
python -V
```

### 0.2 安装 RoboJuDo621 本体

如果直接运行 `python scripts/run_pipeline.py ...` 报
`ModuleNotFoundError: No module named 'robojudo'`，说明当前环境还没安装仓库本体。

```bash
conda activate robojudo
cd /home/unitree/23dof_deployment/RoboJuDo621
pip install -e .
python -c "import robojudo; print('robojudo OK')"
```

### 0.3 安装 `unitree_sdk2_python`

`unitree_sdk2_python` 目录在 `/home/unitree/unitree_sdk2_python`。如果该目录不存在，先单独 clone；
不要在 `RoboJuDo621/` 下面找它。

```bash
conda activate robojudo
cd /home/unitree
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd /home/unitree/unitree_sdk2_python
pip install -e .
python -c "import unitree_sdk2py; print('unitree_sdk2py OK')"
```

如果目录已经存在，不要重复 clone，直接更新并安装：

```bash
conda activate robojudo
cd /home/unitree/unitree_sdk2_python
git pull
pip install -e .
```

如果验证时报：

```text
ModuleNotFoundError: No module named 'cyclonedds'
```

则在同一个 conda 环境里补装：

```bash
conda activate robojudo
pip install cyclonedds
python -c "import unitree_sdk2py; print('unitree_sdk2py OK')"
```

如果 `pip install cyclonedds` 在 Jetson/aarch64 上找不到预编译包或编译失败，按官方 README 的方式先编译
CycloneDDS，再安装 Python 包：

```bash
conda activate robojudo
cd /home/unitree
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd /home/unitree/cyclonedds
mkdir -p build install
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target install

cd /home/unitree/cyclonedds
export CYCLONEDDS_HOME=/home/unitree/cyclonedds/install
pip install cyclonedds --no-binary cyclonedds

cd /home/unitree/unitree_sdk2_python
pip install -e .
python -c "import unitree_sdk2py; print('unitree_sdk2py OK')"
```

### 0.4 安装 / 检查 C++ 版 `unitree_sdk2`

`UnitreeCppEnv` 依赖官方 C++ SDK 的头文件和库。执行 `sudo make install` 后，正常应存在
`/usr/local/include/unitree/idl/hg/`。

若本机还没有 C++ SDK 源码：

```bash
cd /home/unitree
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd /home/unitree/unitree_sdk2
```

若本机已有源码（例如 `/home/unitree/unitree_sdk2-main/build/unitree_sdk2`），进入包含
`CMakeLists.txt` 的 SDK 源码根目录即可。然后编译安装：

```bash
mkdir -p build
cd build
cmake ..
sudo make install
sudo ldconfig
ls /usr/local/include/unitree/idl/hg/
```

如果 `ls /usr/local/include/unitree/idl/hg/` 不存在，说明 C++ SDK 版本不对或安装失败；
此时需要重新确认源码目录和 `sudo make install` 是否成功。若该目录已经存在且 SDK 未更新，
通常不需要重复编译。

### 0.5 安装 `unitree_cpp` 子模块

`unitree_cpp` 是 RoboJuDo 里用于 `UnitreeCppEnv` 的 Python binding。必须先完成 C++ SDK 安装，
再安装它：

```bash
conda activate robojudo
cd /home/unitree/23dof_deployment/RoboJuDo621
python submodule_install.py unitree_cpp
python -c "import unitree_cpp; print('unitree_cpp OK')"
```

### 0.6 检查 `UnitreeCppEnv` 注册

当前 `RoboJuDo621` 使用 registry 注册环境，`robojudo/environment/__init__.py` 中应包含：

```python
env_registry.add("UnitreeEnv", ".unitree_env")
env_registry.add("UnitreeCppEnv", ".unitree_cpp_env")
```

因此不需要再手工改成旧式：

```python
from .unitree_cpp_env import UnitreeCppEnv
```

验证：

```bash
conda activate robojudo
cd /home/unitree/23dof_deployment/RoboJuDo621
python -c "from robojudo.environment import UnitreeCppEnv; print('UnitreeCppEnv OK')"
python -c "import robojudo, unitree_cpp, unitree_sdk2py; from robojudo.environment import UnitreeCppEnv; print('ALL OK')"
```

### 0.7 运行前环境清理与网口确认

为了避免 ROS/CycloneDDS/旧环境变量污染当前部署进程，运行前建议：

```bash
conda activate robojudo
cd /home/unitree/23dof_deployment/RoboJuDo621

unset PYTHONPATH RMW_IMPLEMENTATION CYCLONEDDS_URI CYCLONEDDS_HOME
export LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
```

> 说明：以上环境清理对**当前 lowcmd 部署不是硬性前提**（实际能跑通的那些运行并未做此清理）。
> 它属于运行期卫生操作，主要在你**重新尝试 arm_sdk** 时才重要（缓解 C++/Python cyclonedds
> 冲突）；当前 `arm_sdk_motor_idx=None`，运行期不调用 Python cyclonedds，腿走 C++ DDS。
> 此清理只影响运行期、不影响第 0.3 节的安装（`CYCLONEDDS_HOME` 仅安装期需要）。

本机实测 G1 网口为 `enP8p1s0`。如果换机器或换网口，先用：

```bash
ip addr
```

确认带 `192.168.123.x` 的接口，并同步检查 `g1_ih_velocity_23dof_real_keyboard`
对应配置里的 `net_if`。

### 0.8 当前 23-DoF 实机配置要点

- 实机运行 config：`g1_ih_velocity_23dof_real_keyboard`
- 环境类型：`UnitreeCppEnv`
- 策略文件：`assets/models/g1/unitree/velocity_history_23dof.pt`
- 机器人：`g1`
- 消息类型：`hg`
- 23-DoF 关节映射：腿 + waist_yaw + 双臂，HG 稀疏槽位见第五节。

快速检查：

```bash
cd /home/unitree/23dof_deployment/RoboJuDo621
python scripts/inspect_real_gains.py -c g1_ih_velocity_23dof_real_keyboard
```

若这一步能打印最终生效的 Kp/Kd 和关节映射，再进入真机运行。

---

运行命令：
```bash
python scripts/run_pipeline.py -c g1_ih_velocity_23dof_real_keyboard
```

---

## 一、遇到的问题 与 解决思路

> 核心症状：真机站立/行走时基座"前后晃"，**最严重出现在"移动→停止"的瞬间**，有节奏、
> 甚至会扩大、长时间不收敛；而同一策略在 RoboJuDo 仿真里不晃。

| # | 遇到的问题 | 解决思路 / 结论 |
|---|---|---|
| 1 | 怀疑是奖励里没设质心晃动惩罚 | 排除。速度任务里本就有 `lin_vel_z`/`base_height`/`root_acc` 等惩罚；且**同一冻结策略在仿真不晃、真机晃 → 是 sim2real 物理差异，不是奖励/训练问题**。 |
| 2 | 不确定真机增益/关节顺序对不对 | 写 `scripts/inspect_real_gains.py` 打印某 config **最终生效**的 Kp/Kd/关节→电机映射；对照 SDK 电机枚举逐一核对，确认腿+waist_yaw→电机 0-12、增益经 `merge_dof_cfgs` 用训练值覆盖，**配置层与仿真一致**。 |
| 3 | 晃动本体定位 | 用宇树自带水平仪观察：实为**矢状面(前后)摆动，主动关节是膝**（此策略膝主导平衡，踝几乎不动）。判定为**控制延迟驱动的膝部共振/极限环**（日志也见控制环 frame drop）。 |
| 4 | 怎么压住共振 | **固件 PD 的 kd 是最稳的杠杆**（在 ~1kHz 上作用于真实关节速度，不引入 50Hz 策略环延迟）。逐步加阻尼：踝 kd 0.2/0.1→0.5/0.3（大幅好转）、膝 kd 5→8、髋pitch kd 2.5→4。 |
| 5 | 加阻尼后仍残留、且是延迟型 | 延迟型失稳要**降回路增益换相位裕度**：膝 kp 200→150→130→115。前进停下变为即时收敛，后退停下收敛时间大幅缩短。 |
| 6 | "移动→停止"瞬间最不稳 | 触发源是**指令阶跃**：松键时 vx 从 0.x 一步跳到 0。新增 `cmd_smooth_alpha`（指令 EMA 斜坡），设 0.1（约 0.2s），消除急停冲击。注意：调更小(0.06)反而让后退滑行更久，0.1 是甜点。 |
| 7 | 试过的无效杠杆 | **观测侧低通(dof_vel / IMU)对延迟型振荡无效**，IMU 滤波甚至因增加滞后放大振荡 → 已废除（见第四节）。 |
| 8 | 手臂软垂、看起来没控制 | 实为肉眼误判：默认臂姿(肩0.2/肘0.6)很像下垂。确认**手臂走 `rt/lowcmd`（kp=40）**即可控制；`rt/arm_sdk` 在本架构不可用（`run_pipeline` 的 MotionSwitcher 释放了订阅 arm_sdk 的高层服务，matched=False）。设 `arm_sdk_motor_idx=None` 让手臂走 lowcmd。 |
| 9 | 调手臂姿态后前后晃复发 | 屈肘使前臂前伸→**质心前移**→重新激出后退停下的膝共振。手臂与腿平衡耦合：改臂姿后需对腿做小幅再调（膝 kp 130→115）。 |
| 10 | arm_sdk 的 DDS 崩溃 | 历史排查：C++(unitree_cpp) 与 Python(cyclonedds) 同进程共用 `/usr/local/lib/libddsc.so` → 类型注册表冲突 `BAD_PARAMETER`。因最终弃用 arm_sdk 而变为无关；`_init_arm_sdk` 仍加了 try/except 防崩护栏。 |

---

## 二、最终改动清单（均在 `RoboJuDo621/` 内）

> 训练仓库 `Agile_with_ih/`、SDK `unitree_sdk2_python/`、C++ `packages/unitree_cpp/` 均**未改动**（仅只读参考）。

| 文件 | 改动 |
|---|---|
| `robojudo/config/g1/g1_cfg.py` | `g1_agile_velocity_23dof_real` 增加 policy 覆盖（腿部增益+指令平滑）、`arm_sdk_motor_idx=None`、import `G1AgileVelocity23DoF` |
| `robojudo/config/g1/env/g1_env_cfg.py` | `G1_23RaisedArmsDoF.default_pos` 调整手臂保持姿态（肘） |
| `robojudo/policy/policy_cfgs.py` | 新增 `cmd_smooth_alpha` 字段（默认 1.0=关，全局行为不变） |
| `robojudo/policy/unitree_policy.py` | `_get_commands` 增加指令 EMA 平滑 |
| `robojudo/environment/unitree_cpp_env.py` | `_init_arm_sdk` 加 try/except 防崩护栏；移除废弃的观测滤波（见第四节） |
| `scripts/inspect_real_gains.py`（新增） | 打印 config 最终生效的 Kp/Kd/关节顺序，纯只读自查工具 |

---

## 三、最终生效参数（真机专用，仅 `g1_agile_velocity_23dof_real`，sim2sim/训练不受影响）

腿部（`policy.action_dof`，对比训练值）：

| 关节 | Kp | Kd |
|---|---|---|
| 膝 | 200 → **115** | 5 → **8** |
| 髋 pitch | 100（不变） | 2.5 → **4** |
| 踝 pitch | 20（不变） | 0.2 → **0.5** |
| 踝 roll | 20（不变） | 0.1 → **0.3** |
| 其余（髋roll/yaw、waist_yaw） | 不变 | 不变 |

其它：
- `cmd_smooth_alpha = 0.1`（指令斜坡，消除急停阶跃；勿调更小，否则后退滑行更久）。
- `arm_sdk_motor_idx = None`（手臂走 `rt/lowcmd`，kp 40 / 腕 20、kd 2）。
- 手臂保持姿态：`G1_23RaisedArmsDoF.default_pos`，**改这里即可调臂姿**。**当前值：肘 = 0.0、肩pitch = 0.2**。肘符号：**值越大越直**（≈1.5 直、≈0.0 屈 90°、限位 [-1.047, 2.094]）；第 1 个值是肩pitch（调大整条手臂抬高）。
- 里程计保持默认开启（`UNITREE`）：曾试关闭（`DUMMY` + `enable_odometry=False`）以省去每步多余的 sport_state 查询，但收益未证实，故保持默认。（注：关闭里程计与 arm_sdk 的 DDS 报错**无关**——实测里程计开着也照样报 `BAD_PARAMETER`，真正原因见第一节第 10 条的 libddsc 冲突。）

> 只降了膝 kp，其余全是"增阻尼/指令平滑"，稳定性只增不减；全部改动可逆（注释里写了恢复值）。

调参口诀：
- 还想更稳/后退更快：膝 kp 再降一档（115→105，注意膝是否变软下沉）或膝 kd 8→10/11（不影响站姿）。
- 改了手臂姿态导致前后晃复发：质心变了，腿要随之小幅再调（同上）。

---

## 四、废除项（写明废除日期）

| 项 | 位置 | 废除日期 | 原因 |
|---|---|---|---|
| dof_vel EMA 低通 | `unitree_cpp_env.py`（已移除，留废除标注） | 2026-06-29 | 对前后晃实测无效 |
| IMU(ang_vel/quat) EMA 低通 | `unitree_cpp_env.py`（已移除，留废除标注） | 2026-06-29 | 无效，且增加相位滞后放大延迟型振荡 |
| `scripts/hold_arms.py` | 文件保留并加废除横幅 | 2026-06-29 | arm_sdk 在本架构不可用；手臂已改走 lowcmd。保留以备高层场景参考 |

> 通用结论：**观测侧低通不是治延迟/共振型极限环的工具**（只会加滞后）。该用固件 PD 的 kd（增阻尼）和降 kp（增裕度）。

---

## 五、附：与训练/仿真的对应关系（便于排查）

- 策略冻结 JIT；观测 = `[ang_vel·0.2, gravity·1.0, cmd·1.0, (q−q0)·1.0, q̇·0.05, last_action]`，5 帧历史；**不含 base 线速度**。
- 控制 50Hz（`policy.freq=50`，与训练 `sim_dt0.005×decimation4` 一致）。
- 13 个受控关节 = 12 腿 + waist_yaw；其余 10 个手臂关节在 lowcmd 中保持 `G1_23RaisedArmsDoF.default_pos`。
- 关节→电机映射 `joint2motor_idx=[0..12,15..19,22..26]`（HG 稀疏槽位），已对照 SDK 枚举核对无误。

---

## 六、概念澄清：`rt/lowcmd` 是什么（无需"安装"）

`rt/lowcmd` **不是一个要安装的软件/包，而是一个 DDS 话题名（字符串）**——相当于"广播频道/信箱地址"。机器人固件开机就一直在监听它，你把 `LowCmd`（29 个电机的 q/kp/kd/...）发到这个地址，固件就驱动电机。**机器人侧不需要装任何东西。**

控手臂的两条通路也都是 DDS 话题：`rt/lowcmd`（底层，控全 29 电机含手臂，本部署用）、`rt/arm_sdk`（高层叠加，本架构下无订阅端、不可用）。

它在代码里的位置：
- 话题名定义：`robojudo/environment/env_cfgs.py` → `lowcmd_topic = "rt/lowcmd"`、`lowstate_topic = "rt/lowstate"`。
- 发布方（本部署，C++）：`packages/unitree_cpp/src/unitree_controller.cpp` 建 `ChannelPublisher<LowCmd_>(cfg_.lowcmd_topic)`，`LowCommandWriter()` 每个控制周期 `Write`。
- 发布方（Python SDK 版）：`robojudo/environment/unitree_env.py` → `ChannelPublisher(lowcmd_topic, LowCmdHG)`。

你真正"安装"的是**承载它的传输与 SDK**（第 0.3~0.5 节）：
- **cyclonedds（libddsc）**：DDS 传输层，把消息经网络发出去；
- **Unitree SDK**（C++ `unitree_sdk2` / Python `unitree_sdk2py`）：定义 `LowCmd` 消息结构 + 发布 API；
- **unitree_cpp**：RoboJuDo 的 binding，其 `UnitreeController` 往 `rt/lowcmd` 发。

完整链路：
```
策略 → ChannelPublisher("rt/lowcmd", LowCmd) → cyclonedds → 网口 enP8p1s0 → 机器人固件订阅 rt/lowcmd → 电机
```

可用 DDS 工具自查话题是否在线（可选）：`ddsperf`/`cyclonedds` 命令或 `ros2 topic list`（若装了 ROS）能看到 `rt/lowcmd`、`rt/lowstate`。
