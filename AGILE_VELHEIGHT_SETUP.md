# [ih] AGILE velocity-HEIGHT frozen-hands recurrent policy → RoboJuDo sim2sim

Deploy the AGILE `Velocity-Height-G1-Dev-FrozenHands-Wrist20-Distillation-Recurrent-v0`
student (LSTM recurrent, height-command squatting, trained with 20 N wrist load) in
RoboJuDo's MuJoCo sim, keyboard-driven. Validated end-to-end: stands, walks, squats on
command. This is the recurrent counterpart to `g1_agile_velocity` (which is MLP, config-only);
the recurrence + different obs layout needs a dedicated policy class.

## What this adds

| path | kind |
|------|------|
| `robojudo/policy/agile_velheight_policy.py` | NEW — `AgileVelHeightRecurrentPolicy` (obs assembly, LSTM state, height cmd) |
| `robojudo/policy/__init__.py` | EDIT — register the policy |
| `robojudo/config/g1/policy/g1_agile_velheight_cfg.py` | NEW — DoF (29 body / 12 legs) + `G1AgileVelHeightPolicyCfg` |
| `robojudo/config/g1/g1_cfg.py` | EDIT — import + register `g1_agile_velheight` pipeline |
| `assets/models/g1/agile/velheight_frozenhands_wrist20_recurrent.pt` | NEW — the JIT policy |

## Run

```bash
SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c g1_agile_velheight
```

Keyboard (pynput global listener):
- `w`/`s` = forward/back (vx), `a`/`d` = strafe (vy), `q`/`e` = turn (wz)
- **`r` = stand taller, `f` = squat lower** (height command, range 0.50–0.74 m)

## How it maps

- **obs (128, NO history)** in order: `[commands(4: vx,vy,wz,height), ang_vel(3),
  gravity(3), joint_pos_rel(53), joint_vel_rel(53)·0.1, last_action(12)]`.
- **joint_pos_rel / joint_vel_rel = 53** = 29 body joints + 24 DFQ hand joints. The hands are
  **frozen** in training → their pos_rel/vel are always 0, so the policy **zero-pads** the 24
  hand slots. Hence the robot mjcf needs only the 29 body joints (`g1_29dof_rev_1_0.xml`); no
  dexterous-hand model required. (If you run a 53-joint dex-hand mjcf instead, the frozen hands
  read ≈0 anyway — same result.)
- **recurrent**: the exported JIT carries the LSTM hidden state internally (buffers
  `hidden_state` / `cell_state`); call it with a **1D (128,)** tensor; `reset()` zeros those buffers.
- **action**: 12 leg joints, **per-joint scale** `[0.5475×2, 0.3507×2, 0.5475×2, 0.3507×2,
  0.4386×4]`, + leg default offset. `last_action` obs = raw network output (pre-scale).
- **physics** 200 Hz (sim_dt 0.005 × decimation 4) to match AGILE training.
- The 12 leg PD gains come from the IO descriptor (stiffness ~40/99/28, damping ~2.6/6.3/1.8);
  the other 17 body joints (waist_yaw + arms + wrists) hold at the env default with RoboJuDo's
  `G1_29DoF` gains.

## Ground truth

Exported IO descriptor:
`logs/rsl_rl/velocity_height_g1_lower_frozenhands_distillation/2026-06-17_16-30-39_D4_wrist20_recurrent_1000_s42/exported/velocity_height_g1_dev_frozenhands_wrist20_distillation_recurrent_v0_IO_descriptors.yaml`
(re-export with `eval.py --task Velocity-Height-G1-Dev-FrozenHands-Wrist20-Distillation-Recurrent-v0 --checkpoint model_999.pt --export_io`).

## Validation (in-sim, agile_env + xvfb)

| cmd | result |
|-----|--------|
| stand h=0.72 | root_z ≈ 0.718 (stable) |
| walk vx=0.4   | +1.21 m in 3 s (~0.4 m/s), upright |
| squat h=0.55  | root_z ≈ 0.596 (−0.12 m) — height command works |

## Dexterous-hand mass (default)

The policy was trained on exp03 (DFQ inspire hands, frozen). The hands aren't articulated
or observed (zero-padded), but their **mass** affects the dynamics. So the default robot is
`g1_29dof_rev_1_0_handmass.xml` = the stock 29-DOF mjcf + a lumped **0.1918 kg fixed inertial
body at each wrist** (the exp03 training hand mass, total +0.384 kg → 33.725 kg). No joints /
actuators added, so it stays a 29-torque-actuator model (RoboJuDo-compatible). This makes the
hand mass physically present in sim2sim without needing articulated fingers or a position-actuator
dex-hand mjcf (which RoboJuDo's torque-PD env can't drive). To run massless instead, point the
env `xml` back to `g1_29dof_rev_1_0.xml`.

## Tuning

`max_cmd=[0.8,0.5,1.0]` scales the velocity keys; `height_default/min/max/step` in the policy
cfg set the squat range. The dexterous-hand exp03 robot is optional (frozen hands aren't in the
effective obs); to show them, swap the env mjcf to a 53-joint dex-hand model (see the dex-hand
note in AGILE_VELOCITY_SETUP.md) — no policy change needed.
