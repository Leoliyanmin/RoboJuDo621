# G1 29-DoF 速度-高度（velheight）循环策略真机部署

| 项 | 内容 |
|---|---|
| 生产时间 | 2026-07-01 |
| 生产人 | claude |
| 审计人 | 李晏敏 |

部署目标：在 `RoboJuDo621`（分支 `ih/agile-velocity-sim2sim`）中，把 AGILE 的
**velocity-HEIGHT frozen-hands wrist20 循环（LSTM）学生策略**
（`assets/models/g1/agile/velheight_frozenhands_wrist20_recurrent.pt`）部署到**真实 29-DoF G1**，
键盘驱动，支持前后左右转 + **站高/下蹲**（height 指令）。仿真（RoboJuDo MuJoCo，见
`AGILE_VELHEIGHT_SETUP.md`）已验证站立/行走/下蹲；本文覆盖真机侧的落地。

> 与 23-DoF 部署（`docs/23dof部署.md`）的关系：那是速度-历史 MLP 策略、控制 13 关节（腿+waist_yaw）、
> 原生 23-DoF 硬件；本文是**循环（LSTM）+ 高度指令**策略、控制 **12 条腿关节**、跑在 **29-DoF 硬件**上，
> 其余 17 个躯干关节（腰 3 + 双臂/腕 14）保持默认位姿、与腿同走 `rt/lowcmd`。24 个灵巧手关节在 obs 中补零（训练时冻结）。

---

## 零、安装与环境准备

> 固定工作目录：
> - RoboJuDo 部署仓库：`/home/unitree/29_dof_locomotion/RoboJuDo621`（GitHub `Leoliyanmin/RoboJuDo621`，分支 `ih/agile-velocity-sim2sim`）
> - Python 版 Unitree SDK：`/home/unitree/unitree_sdk2_python`
> - C++ 版 Unitree SDK：`/home/unitree/unitree_sdk2`（`sudo make install` 后应有 `/usr/local/include/unitree/idl/hg/`）

conda 环境、`unitree_sdk2_python` / `cyclonedds` / C++ `unitree_sdk2` / `unitree_cpp` 的安装步骤
与 `docs/23dof部署.md` 第 0.1–0.6 节**完全相同**，此处不重复。

### ⚠️ 0.1 本机的 robojudo 环境指向另一个仓库（重要）

本机 `robojudo` conda 环境是对 `/home/unitree/RoboJudo_Real`（另一个仓库 `Andisyc/RoboJudo_Real`）做的
`pip install -e .`，所以直接 `import robojudo` 解析到的是那个旧仓库，**不是本 29-DoF 仓库**，`run_pipeline`
会报 `Unknown type: g1_agile_velheight_real`。两种解决方式：

1. **不改共享环境（推荐做只读检查/临时运行）**：用 `PYTHONPATH` 让本仓库优先：
   ```bash
   conda activate robojudo
   cd /home/unitree/29_dof_locomotion/RoboJuDo621
   PYTHONPATH=/home/unitree/29_dof_locomotion/RoboJuDo621 python scripts/inspect_real_gains.py -c g1_ih_velheight_29dof_real_keyboard
   ```
2. **重装本仓库（会把共享环境重新指向本仓库，影响 RoboJudo_Real 的使用者，慎用）**：
   ```bash
   conda activate robojudo
   cd /home/unitree/29_dof_locomotion/RoboJuDo621
   pip install -e .
   ```

### 0.2 网口确认

本机 G1 网口为 `enP8p1s0`（`192.168.123.164/24`）。换机器/网口时先 `ip addr` 找带 `192.168.123.x` 的接口，
并同步改配置里的 `net_if`。

### 0.3 实机配置要点

- 实机运行 config：`g1_ih_velheight_29dof_real_keyboard`（键盘）/ `g1_agile_velheight_real`（Unitree 手柄）
- 环境类型：`UnitreeCppEnv`（C++ DDS 发 `rt/lowcmd`）；`*_py` 变体走 `unitree_sdk2py`
- 策略文件：`assets/models/g1/agile/velheight_frozenhands_wrist20_recurrent.pt`
- 机器人：`g1`，消息类型：`hg`，`motor_cmd_num_dofs=29`，`joint2motor_idx=None`（0..28 直通）
- 受控关节：12 条腿（policy `action_dof=G1AgileVHLegsDoF`，AGILE 训练增益）；
  其余 17 关节（腰 yaw/roll/pitch + 双臂/腕）保持 `G1_29DoF.default_pos`（全 0）、env 增益、走 `rt/lowcmd`
- `arm_sdk_motor_idx=None`：手臂**不**走 `rt/arm_sdk`（`run_pipeline` 的 MotionSwitcher 释放了订阅它的高层服务，不可用），
  与腿同走 `rt/lowcmd`；同时让 `_init_arm_sdk` 变空操作，避免 Python/C++ cyclonedds 冲突

上机前先做只读自查（打印最终生效的 Kp/Kd、关节→电机映射）：
```bash
PYTHONPATH=/home/unitree/29_dof_locomotion/RoboJuDo621 \
  python scripts/inspect_real_gains.py -c g1_ih_velheight_29dof_real_keyboard
```
应看到：腿 0–11 为 `TRAIN` 源（膝 kp99.1/kd6.31、髋 40/99、踝 28.5/1.81）；waist 12–14 为 env（kp200/kd6）；
双臂 15–28 为 env（肩肘 kp40/kd2、腕 kp20/kd2）；`arm_sdk_motor_idx` 为空。

---

## 一、运行

```bash
conda activate robojudo
cd /home/unitree/29_dof_locomotion/RoboJuDo621
# 运行期卫生（可选，重试 arm_sdk 时才重要；本部署 arm_sdk=None 可不做）：
unset PYTHONPATH RMW_IMPLEMENTATION CYCLONEDDS_URI CYCLONEDDS_HOME
export LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
# 若未 pip install 本仓库，则临时加回 PYTHONPATH（注意别和上面的 unset 冲突，二选一）：
PYTHONPATH=/home/unitree/29_dof_locomotion/RoboJuDo621 \
  python scripts/run_pipeline.py -c g1_ih_velheight_29dof_real_keyboard
```

> 通过 SSH 跑时必须是交互 TTY（正常 `ssh` / `tmux`；不要 `nohup`、后台脚本或无 TTY 的服务方式），
> 否则键盘读不到（会打印 `KeyboardCtrl needs a TTY or DISPLAY`）。

### 启动流程

1. 上电、机器人吊起/有人扶稳。运行上面的命令。
2. `prepare` 分两段：先 6s 缓慢斜坡到默认站姿，再 blend 进策略，最后打印：
   ```text
   prepare done — holding default pose, press R to start motion
   ```
   此时策略已在跑（零指令=站立/保持 height=0.72）。
3. 用 `w/a/s/d/q/e` 给速度、`r/f` 调站高，即开始运动。（本策略无独立 "motion 模式"，
   零指令下就是站立；`|` 键 `[MOTION_RESET]` 用于需要时重新 blend。）

### 键盘键位

| 键 | 含义 | 指令量 |
|---|---|---|
| `w` / `s` | 前进 / 后退 | `vx = ±0.8 m/s` |
| `a` / `d` | 左移 / 右移 | `vy = ±0.5 m/s` |
| `q` / `e` | 左转 / 右转 | `wz = ±1.0 rad/s` |
| `r` | 站更高 | `height += 0.01 m`（上限 0.72） |
| `f` | 下蹲更低 | `height -= 0.01 m`（下限 0.40） |
| `\|`（Shift+`\`） | `[MOTION_RESET]` | 重新从默认位姿 blend 进策略 |
| `o`/`O`/`Esc`/`Ctrl-C` | `[SHUTDOWN]` | 退出并关闭控制 |

> 关键点：**`r`/`f` 是策略的高度键，绝不能设成 trigger**——被触发的按键会从事件流里移除、传不到策略
> （`KeyboardCtrl.process_triggers` 会 `remove` 该事件）。所以 `[MOTION_RESET]` 用 `|`，不用 `r`。
> 速度键靠带超时的 `keys_pressed` 维持：终端后端无松开事件，按住靠系统连发，松手约 0.25s（`terminal_key_timeout`）内归零。

---

## 二、改动清单（本次 29-DoF 落地，均在 `RoboJuDo621/` 内）

| 文件 | 改动 |
|---|---|
| `robojudo/policy/agile_velheight_policy.py` | `_get_commands` 改读带超时的 `keys_pressed`（SSH 终端安全，等价 `UnitreePolicy`）；新增速度指令 EMA 平滑（`cmd_smooth_alpha`，默认 1.0=关）；`reset` 去掉 `_held_keys`、加 `_cmd_smoothed` |
| `robojudo/config/g1/g1_cfg.py` | 充实 `g1_agile_velheight_real`（prepare 斜坡、`arm_sdk=None`、`cmd_smooth_alpha=0.1`、safety）；新增 `*_keyboard` / `*_py` / `*_py_keyboard`；新增 `g1_ih_velheight_29dof_real*` 别名 |
| `docs/29dof部署.md` | 本文 |

> 仿真侧 `g1_ih_29dof_velheight`（MuJoCo）与策略/配置的 obs、增益、动作 scale 未改；平滑默认关闭，仿真行为不变。

---

## 三、最终生效参数（真机）

腿部 12 关节（AGILE 训练增益，与仿真一致；顺序见 `G1AgileVHLegsDoF`）：

| 关节 | Kp | Kd |
|---|---|---|
| 膝 | 99.098 | 6.309 |
| 髋 roll | 99.098 | 6.309 |
| 髋 pitch / yaw | 40.179 | 2.558 |
| 踝 pitch / roll | 28.501 | 1.814 |

其余（env 值，保持位姿用）：腰 yaw/roll/pitch kp200/kd6；肩肘 kp40/kd2；腕 kp20/kd2；默认位姿全 0。

其它：
- `cmd_smooth_alpha = 0.1`（速度指令 EMA，约 0.2s；消除急停阶跃；高度指令不平滑）。
- `arm_sdk_motor_idx = None`（手臂走 `rt/lowcmd`）。
- 手臂保持位姿：`G1_29DoF.default_pos`（双臂全 0，即自然下垂）——与策略 obs 默认一致（`joint_pos_rel ≈ 0`，即冻结手训练位姿）。
  要改臂姿可覆盖 env `dof` 的 `default_pos`（改后腰腿平衡会变，可能需按下条重新调腿）。
- height 指令范围 `[0.40, 0.72]`（`G1AgileVelHeightPolicyCfg.height_min/max`），默认 0.72，步进 0.01。

调参口诀（若真机出现 23-DoF 式前后晃，先小步试，吊着调）：
- 抬矢状面主动关节的固件 PD `kd`（膝、髋 pitch、踝），再不行降膝 `kp` 换相位裕度——
  覆盖 `action_dof=G1AgileVHLegsDoF(stiffness=[...], damping=[...])`。
- 别用观测侧低通治延迟型振荡（只加滞后）。
- 后退停下收敛慢多半是减速/滑行时间，`cmd_smooth_alpha` 调更小反而更久，保持 0.1。

---

## 四、与训练/仿真的对应关系（排查用）

- 策略：AGILE `Velocity-Height-G1-Dev-FrozenHands-Wrist20-Distillation-Recurrent-v0` 学生，冻结 JIT，**循环（LSTM）**，
  隐状态在 JIT 内部 buffer（`hidden_state`/`cell_state`）携带，`reset()` 清零；调用输入 1D `(128,)`。
- obs（128，**无帧历史**）顺序：
  `[commands(4: vx,vy,wz,height), ang_vel(3), gravity(3), joint_pos_rel(53), joint_vel_rel(53)·0.1, last_action(12)]`。
  其中 `joint_pos_rel/joint_vel_rel = 29 躯干关节 + 24 冻结手关节（补 0）`。obs scale：ang_vel 1.0、gravity 1.0、dof_pos 1.0、dof_vel 0.1。
- 动作：12 条腿，**逐关节 scale**（`[0.5475×2, 0.3507×2, 0.5475×2, 0.3507×2, 0.4386×4]`）+ 腿默认偏置；
  `last_action` 用的是**原始网络输出**（scale 前）。
- 控制 50Hz（`policy.freq=50`），与训练 `sim_dt0.005×decimation4` 一致。
- 完整链路：`策略 → rt/lowcmd(LowCmd, 29 电机) → cyclonedds → enP8p1s0 → 机器人固件 → 电机`。

---

## 五、附：概念澄清

`rt/lowcmd` 是 DDS 话题名（不是要安装的软件），固件常驻订阅；把 29 个电机的 q/kp/kd 发到这个地址即可驱动。
承载它的是 cyclonedds（`libddsc`）+ Unitree SDK（C++/Python）+ RoboJuDo 的 `unitree_cpp` binding。详见 `docs/23dof部署.md` 第六节。
