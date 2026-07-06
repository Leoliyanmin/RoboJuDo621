# Deep-squat × arm-swing 最终策略对比（2026-07-04）

G1 29DoF 冻手 deep-squat 线的四个最终策略 + 对比图。单 seed，IsaacLab。
**结论：PIN 全能，三个"加摆臂"变体皆负优化 → 部署直接用 PIN + 独立手臂 PD target。**

## 文件

> **格式 = TorchScript jit**（可直接 `torch.jit.load`，供 RoboJuDo 等部署用），in_dim=131 / out=12 leg joints。
> 不是 rsl_rl 训练 checkpoint。每个的 `.onnx` 与源 `exported/policy.onnx` 在对应 `logs/.../exported/` 下。

| 文件 | 策略 | 训练 | 原始 checkpoint |
|---|---|---|---|
| `0_PIN_baseline_model_3750.pt` | **PIN**（repro29 深蹲基线，无臂 DR） | 从0 4000it | `logs/.../deepsquat_repro29/2026-07-02_07-14-37.../model_3750.pt` |
| `1_RandArms_scratch_model_3999.pt` | **#1** 任意随机上肢 | 从0 4000it | `logs/.../deepsquat_repro29_randarms/2026-07-03_09-59-04.../model_3999.pt` |
| `2_ArmSwing_scratch_model_3999.pt` | **#2** 部署匹配规律摆臂 | 从0 4000it | `logs/.../deepsquat_repro29_armswing/2026-07-03_09-59-04.../model_3999.pt` |
| `3_ArmSwing_finetune_model_4749.pt` | **#3** 规律摆臂微调 | 从 PIN 微调 +1000it | `logs/.../deepsquat_repro29_armswing_ft/2026-07-03_03-44-42.../model_4749.pt` |
| `comparison_figure.png` | 支配性对比图 | — | `1temp/exp12_pin_dominance.png` |

## eval 结果（eval2 零速深蹲 cmd0.20 + walking eval 带摆臂）

| 策略 | 深蹲 cmd0.20 pelvis / 摔率 | 走路+摆臂 摔率 | 判定 |
|---|---|---|---|
| **PIN** | **0.31m / 2%** | **0-1%** | ✅ 深且稳 + 走路摆臂稳 |
| #1 RandArms | 全崩(100%) | 31-47% | ✗ 灾难 |
| #2 ArmSwing | 0.44m / 5% | 0-1% | ~ 走路稳但蹲浅 12cm |
| #3 ArmSwing-Ft | 崩(99%) | 20-57% | ✗ 崩 |

---

## 具体代码差别

四者**共享同一个策略网络结构**（rsl_rl PPO，131维teacher obs，12维腿动作），差别**全在环境配置 `env_cfg`**（决定训练时上肢怎么动）+ **#3 的训练过程**。全部继承同一个 base：

**Base = `G1LowerVelocityHeightFrozenHandsDeepSquatRepro29EnvCfg`**（PIN 的环境）
= 冻手29dof base + WBC-AGILE 的 8 处深蹲改动。上肢用 `random_upper_body_pos`(RandomActionCfg)，继承 `no_random_when_walking=True` → **走路时手臂钉在默认位；零速(站/蹲)时手臂跳到随机静态姿**。

### PIN（基线）——直接用 base，无任何上肢改动
- 走路：臂钉默认（不动）
- 零速蹲：臂随机静态姿

### #1 RandArms —— 相对 PIN 只改 **1 行**
`G1LowerVelocityHeightFrozenHandsDeepSquatRepro29RandArmsEnvCfg`：
```python
self.actions.random_upper_body_pos.no_random_when_walking = False
```
→ 手臂在**所有步态**（含走路）都跳随机静态姿。腿策略被要求"任意臂姿下都走稳/蹲稳"。**扰动太猛 → 4000it 学不出 → 灾难性退化。**

### #2 ArmSwing —— 相对 PIN 改 **两处**
`G1LowerVelocityHeightFrozenHandsDeepSquatRepro29ArmSwingEnvCfg`：
```python
# (1) 把两个肩关节从"随机静态姿"里排除，交给 ArmSwing 接管
excl += [".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"]
self.actions.random_upper_body_pos.joint_names_exclude = excl
# (2) 新增一个脚本化(非策略输出)的 arm_swing 动作项
self.actions.arm_swing = mdp.ArmSwingActionCfg(
    asset_name="robot",
    joint_names=[".*_shoulder_pitch_joint", ".*_shoulder_roll_joint"],
    command_name="base_velocity")
```
**关键 = 全新的 `ArmSwingAction`（脚本化、部署匹配、速度门控）**，和 #1 的"随机静态姿"本质不同——它是**有节奏的正弦摆 + 外展**，复刻 RoboJuDo 部署摆臂：
```python
# ArmSwingAction.process_actions (random_actions.py)
speed = ||command[:, :3]||                         # 速度命令模长
walk_alpha = clamp((speed-lo)/(hi-lo), 0, 1)       # 0=站/蹲, 1=全速走
swing  = amp * sin(phase) * walk_alpha             # shoulder_pitch 正弦摆(反相 L/R)
spread = spread_amp * walk_alpha                   # shoulder_roll 外展(手离大腿)
target = default + pitch_sign*swing + roll_sign*spread
```
→ **走路时肩规律摆+外展；零速时 walk_alpha=0 → 臂回默认（深蹲不被摆臂碰）**。其余上肢(waist/elbow/wrist)仍照旧零速随机。
**结果：走路+摆臂学得和 PIN 一样好，但共享网络把深蹲练浅了 12cm（多任务干扰）。**

`ArmSwingActionCfg` 的可调 DR 范围（actions_cfg.py）：amp 0.06-0.14rad / freq 0.9-1.4Hz / spread 0.12-0.18rad / speed_lo 0.05 / speed_hi 0.20，每 episode 随机。

### #3 ArmSwing-Ft —— **环境和 #2 完全相同**，差别只在训练过程
- **env 零差别**（同 `...ArmSwingEnvCfg`）。
- 差别 = **训练方式**：#2 从0随机初始化训 4000it；#3 用 `--resume True --checkpoint <PIN model_3750.pt> --max_iterations 1000` **从 PIN 微调 1000it**（runner 用 `...ArmSwingFtRunnerCfg`：max_iterations 1000 / save_interval 100，vs #2 的 4000 / 250）。
- **结果：ft250 即崩**。后经 vanilla 对照（从 #2 m3999 零 cfg 改动续训也崩）证明 = **续训/继续 PPO 的 resume 冲击本身**，与改动无关。

### 小结（代码差别一句话）
- PIN → #1：`no_random_when_walking=False`（1 行，随机化扩到走路）。
- PIN → #2：排除双肩 + 新增脚本化 `ArmSwingAction`（规律摆臂、速度门控）。
- #2 → #3：**环境一字不差**，只是"从0训 vs 从PIN微调续训"。

## 边界（诚实标注）
单 seed；sim（IsaacLab eval2 + walking eval）；#3/续训崩 = resume 冲击（vanilla 对照证），非奖励改动；PIN 对注入摆臂的鲁棒来自 push/外力 DR 余量。**RoboJuDo sim2sim（真实注入延迟/gain）仍未测 = open**。**未试**：从0 + 深度优先加权 + 摆臂 DR（可能是让"深+摆臂"共存的正确路，但 PIN 已满足部署需求）。
