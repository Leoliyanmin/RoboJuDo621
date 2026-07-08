# 清空手臂 obs（arm-obs mask）

**一句话**：部署时为了自然让手臂摆动（`walk_elbow`/`walk_squat_box`），但手臂关节角会进入运动策略的观测（obs）；策略训练时手臂几乎钉在 0（伸展位），所以驱动手臂等于给策略灌**分布外（OOD）**的手臂 obs，害它原地下蹲约 9 cm（cmd 0.72 → 实际 0.65）。`b` 键打开 mask 后，喂给策略的手臂 obs 强制为训练默认值（0），**手臂照样物理摆动**，但策略不再被它扰动——高度拿回来，摆臂对转向的物理帮助保留。

面向 `AgileVelHeightRecurrentPolicy` / `AgileVelHeightTeacherPolicy`（G1 29DoF 冻手 velocity-height 系列，含深蹲 PIN student）。

---

## 背景现象

viewer 里同一条 `height cmd = 0.72`：

| 手臂模式 | 手肘关节值 | 实际 pelvis 高度 |
|---|---|---|
| `off`（钉训练默认位，≈0） | ~0.05 rad | ~0.71–0.74 m |
| `walk_elbow`（垂手，肘 ~1.06 rad） | ~1.06 rad | **~0.65 m** |

换手臂模式、命令不变，实际高度差 ~9 cm。

## 机制：两条独立通路

obs 里 `joint_pos_rel` / `joint_vel_rel` 包含 **29 个身体关节，含 14 个手臂关节**（肩/肘/腕）。`joint_pos_rel = 实际角 − default`，而 RoboJuDo 与训练侧的手臂 default 都是 0（训练 `init_state` 只设了腿）。所以：

- `off`：肘 ≈0 → 手臂 obs ≈0 → **匹配训练分布**。
- `walk_elbow`：肘 ≈1.06 → 手臂 obs ≈1.06 → **训练从没见过（OOD）**。

用 `--zero_arm_obs` 拆解实验（手臂物理照 `walk_elbow` 摆，但 obs 里 14 个手臂位/速强制清零），student，静止纯偏航 wz=-0.6，`turn_prime` off：

| 条件 | 手臂物理 | 手臂 obs | cmd0.72 高度 | cmd0.72 转向 ratio |
|---|---|---|---|---|
| off | 训练默认 | 真实(≈0) | 0.741 | 0.09 |
| walk_elbow | 摆动 | 真实(≈1.06) | 0.656 | 0.42 |
| **walk_elbow + mask** | **摆动** | **清零** | **0.738** | **0.41** |

三高度（0.62/0.72/0.80）一致：mask 后高度**回到 off 水平**，转向 ratio **保持 walk_elbow 水平**。据此拆出：

- **高度掉 = obs 通路**（策略对 OOD 手臂 obs 的反应）。清零 obs、手臂仍摆，高度弹回 → 物理 CoM 贡献 <1 cm，那 9 cm 几乎全是策略自己蹲下去的。
- **转向变好 = 物理通路**（摆臂的偏航角动量帮脚挣脱蹲定 basin）。清零 obs 后转向不变 → 与 obs 无关。cmd0.80 更明显：mask 后 ratio 0.94 > walk_elbow 0.75（高度不掉 + 摆臂助力，两头都占）。

**结论**：`walk_elbow` 掉的高度是"白掉的"副作用——摆臂的转向好处来自物理、和 obs 无关，obs 那份只带来掉高。把手臂 obs 钉在训练默认位即可既拿回高度又保住转向。

## 用法

- 运行时按 **`b`** 切换（需 config `zero_arm_obs_keyboard=True`）。viewer 的 `arms:` 行末显示 `obs=MASKED (b)` / `obs=real (b)`。
- config 字段（`G1AgileVelHeightPolicyCfg`）：
  - `zero_arm_obs_default: bool = False` — 启动即开
  - `zero_arm_obs_keyboard: bool = False` — 允许 `b` 键切换
  - 已在 `G1DeepSquatPINStudentPolicyCfg` 打开 keyboard（`zero_arm_obs_keyboard=True`）。
- 默认全 off：不改 config 的 stock checkpoint 保持 obs bit-for-bit 兼容。

## 副作用与修复：上升时后倒（CoM）

mask 是"高度 vs CoM 一致性"的**本质权衡**：mask 骗策略"手臂在训练默认位（质量偏前）"，策略据此往后压平衡；实际 walk_elbow/box 手臂在别处 → 净 CoM 偏后。稳态影响小（每档前倾少 ~2°、不摔），但有一个会摔的瞬态：

**walk_squat_box 下蹲(→box reach 前伸) 再快速上升(→hang 回摆)时**，手臂从 box 快速回 hang 是一次大 CoM 突变；mask on 让策略**看不见这个突变、补不了** → 上升段后仰。实测（squat 0.72→0.20→rise，mask on）：上升段最小 pitch rise=0.010 时 **−7.8°**（mask off 仅 −4.4°），升得慢（0.004）则不后仰。

**修复（默认已开）**：让下蹲手臂 blend（`squat_alpha`）跟随**平滑后的实测 pelvis 高度**而非高度命令 → 命令一升、身体没起来时手臂**滞后留在 box**，等身体真升上去才回 hang（"完全升上去再变状态"），box→hang 突变消失。实测修复后上升段最小 pitch **−7.8° → +0.4°**。
- cfg：`arm_squat_use_measured: bool`（默认 False；`g1_ih_29dof_deepsquat_pin` 已开 True）、`arm_squat_measured_alpha: float = 0.08`（实测高度平滑系数，越小越滞后）。
- 备选（未实现）：条件 mask（仅站立高档 mask、下蹲/上升解 mask 让策略看得见手臂平衡）；或减小手臂物理垂/伸幅度、降 `arm_motion_rate_dps`。

## 实现位置

- `robojudo/policy/agile_velheight_policy.py`：`_arm_obs_idx`（14 个肩/肘/腕）、`_zero_arm_obs` 状态、`b` 键切换（`_get_commands`）、`_mask_arm_obs()`（两个 `get_observation` 都调用）、extras 上报 `zero_arm_obs`。
- `robojudo/config/g1/policy/g1_agile_velheight_cfg.py`：两个 cfg 字段 + student 开 keyboard。
- `robojudo/environment/mujoco_env.py`：`arms:` 行末显示 mask 状态（读 `_cmd_extras["zero_arm_obs"]`）；`_apply_arm_motion` 的 `arm_squat_use_measured` 实测高度跟随（修上升后倒）。
- `robojudo/environment/env_cfgs.py` + `config/g1/g1_custom_cfg.py`：`arm_squat_use_measured/alpha` 字段 + student env 开 True。

## 复现工具

```
# 拆解实验（Isaac 侧 scripts/）：静止纯偏航，比较 off / walk_elbow / walk_elbow+mask
python scripts/ih_turn_height_sweep_mujoco.py g1_ih_29dof_deepsquat_pin \
    --heights=0.62,0.72,0.80 --arms=walk_elbow --zero_arm_obs=on
```

## 注意范围

- 结论口径：MuJoCo（RoboJuDo）、student（deepsquat_pin），静止纯偏航、arms 物理摆。未外推 PhysX / 真机 / 全速。
- mask 只动 obs 的手臂 14 槽（肩/肘/腕），不动腰（waist）3 关节，不动腿。
- 打开 mask 后手臂仍受 PD 驱动摆动，只是策略"看不见"——不改变手臂的实际运动/外观。
