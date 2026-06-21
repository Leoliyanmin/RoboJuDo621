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

## Dexterous-hand mass & visuals (default = `g1_29dof_rev_1_0_handmass.xml`)

**Mass is already faithful in the stock model.** RoboJuDo's `g1_29dof_rev_1_0.xml` has NO separate
head/hand bodies — it LUMPS them in: torso 7.818 = exp03 torso 6.78 + head 1.036; each wrist 0.255 ≈
exp03 wrist 0.085 + hand 0.192. Total 33.341 kg ≈ exp03's 33.385 kg (with hands). So the hand mass
already acts in sim2sim (lumped into the wrist) — no mass needs adding (an earlier attempt to add a
lumped 0.1918 kg/wrist body DOUBLE-COUNTED it; reverted).

So `g1_29dof_rev_1_0_handmass.xml` = the stock mjcf with **UNCHANGED masses** + the exp03 DFQ dexterous
hands grafted on **for visuals only** (welded, density=0, no collision/joints/sites; frozen open pose).
The robot looks like the dex-hand G1 and stays a 29-torque-actuator RoboJuDo-compatible model. The 16
finger STL meshes live in `assets/robots/g1/meshes/{left,right}_*.stl`.

Caveat (why a full conversion is still better): the lumping makes the mass slightly more central
(head in torso) and slightly less distal (hand at the wrist, ~4 cm closer than the real hand COM).
For the EXACT exp03 distribution + inertia tensors + real DFQ hands, convert the exp03 URDF
(`g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf`) to MJCF (weld the 24 hand joints, add 29 torque
motors, add a floor). The lumped/stock model captures ~95% of the fidelity for locomotion.

## Tuning

`max_cmd=[0.8,0.5,1.0]` scales the velocity keys; `height_default/min/max/step` in the policy
cfg set the squat range. The dexterous-hand exp03 robot is optional (frozen hands aren't in the
effective obs); to show them, swap the env mjcf to a 53-joint dex-hand model (see the dex-hand
note in AGILE_VELOCITY_SETUP.md) — no policy change needed.

## Wrist load (box carry) — default 10 N

This is the wrist20-trained policy, so the config applies a sustained **10 N world-down force
per wrist** by default (, applied at  in MujocoEnv.step).
Set  for no load, or up to ~20 (its training max). Validated: stands (z~0.70,
slightly compressed) and walks ~0.43 m/s under 10 N/wrist.
