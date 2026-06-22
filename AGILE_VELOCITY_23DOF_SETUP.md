# [ih] AGILE 23-DOF velocity-history policy → RoboJuDo sim2sim

Deploy the AGILE `Velocity-G1-History-23DOF-Wrist20-v0` policy (23-DOF G1, MLP +
5-frame history, trained with a 0/5/10/20 N wrist-load curriculum) in RoboJuDo's MuJoCo
sim, keyboard-driven. Sibling of `g1_agile_velocity` (29-DOF); same family, config-only.

## What this adds

| path | kind |
|------|------|
| `robojudo/config/g1/policy/g1_agile_velocity_23dof_cfg.py` | NEW — 13-joint DoF + `G1AgileVelocity23DOFPolicyCfg` |
| `robojudo/config/g1/g1_cfg.py` | EDIT — import + register `g1_agile_velocity_23dof` pipeline |
| `assets/models/g1/unitree/velocity_history_23dof_wrist20.pt` | NEW — the JIT policy |

No new policy class: reuses `UnitreeWoGaitPolicy` (its obs assembly == AGILE velocity-history).

## Run

```bash
SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c g1_agile_velocity_23dof
```

Keyboard: `w`/`s` = vx, `a`/`d` = vy, `q`/`e` = yaw (wz).

## How it maps

- **robot**: 23-DOF `g1_23dof_rev_1_0.xml` (`G1_23MujocoEnvCfg`). 200 Hz physics
  (sim_dt 0.005 × decimation 4) to match AGILE training.
- **controlled / observed = 13 joints** = 12 legs + `waist_yaw` (the 29-DOF task used
  14 = legs + waist_roll + waist_pitch; this one swaps to waist_yaw). The other 10 joints
  (arms / wrists) hold at the env default; `DoFAdapter` slices 23→13 for obs and expands
  13→23 for PD.
- **obs (240)** = 5-frame history, per-term concat, of:
  `[ang_vel·0.2, gravity·1.0, cmd·1.0, (dof_pos−default)·1.0, dof_vel·0.05, last_action]`
  (48/frame). Scales are the `UnitreeWoGaitPolicy` defaults and match the IO descriptor.
- **action**: 13 joints, `clip(raw, ±10)·0.5 + default_pos`. Joint order, scale (0.5),
  clip (±10) and offset are from the IO descriptor (IsaacLab articulation order —
  `waist_yaw` at index 2). PD gains (stiffness/damping) are the IO-descriptor subset
  (hip 100, waist_yaw 300, knee 200, ankle 20; damping 2.5/5/5/0.2-0.1); torque limits are
  the physical G1 motor limits (hip/waist 200, knee 300, ankle 40).
- **wrist load**: wrist20-trained, so a sustained **10 N world-down force per wrist** is on
  by default (`wrist_load_n=10.0` on the env, applied at `left/right_wrist_roll_rubber_hand`).
  Set `wrist_load_n=0.0` to drop it, up to ~20 (training max).

## Ground truth

Exported IO descriptor:
`logs/rsl_rl/velocity_g1_lower_23dof_wrist20/2026-06-21_07-33-47_velocity_g1_lower_23dof_wrist20/exported/velocity_g1_history_23dof_wrist20_v0_IO_descriptors.yaml`
(re-export: `eval.py --task Velocity-G1-History-23DOF-Wrist20-v0 --checkpoint model_3298.pt --export_io`).
JIT obs (1,240) → action (1,13).

## Validation (headless, agile_env + xvfb)

Built the pipeline, forced cmd vx=0.4 with the 10 N load + training perturbations on, ran
300 steps (6 s): base height stayed 0.719 m (upright), walked ~1.6 m, feet lifted ~5 cm
each step (proper stepping). DoFAdapter 23↔13 slice/expand verified.

## Note (shared upstream keyboard quirk)

`UnitreeWoGaitPolicy`'s keyboard handler is edge-triggered and scales by 1.5 (same as the
29-DOF `g1_agile_velocity`). If q/e turning feels jumpy, it's the upstream handler, not this
config — the latched/clamped fix lives only in the velheight policy class. Tell me if you
want it ported here too (it's an upstream file → would be tagged `# [ih]`).
