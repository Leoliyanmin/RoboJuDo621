# [ih] AGILE 29-DOF velocity policy → RoboJuDo sim2sim

Deploy the AGILE `Velocity-G1-History-v0` (29-DOF G1) locomotion policy in RoboJuDo's
MuJoCo sim, keyboard-driven. Validated end-to-end (stands at zero command, walks under
`vx` command). Config-only — reuses RoboJuDo's stock `UnitreeWoGaitPolicy`, whose obs
assembly matches the AGILE velocity-history layout byte-for-byte.

## What this adds (3 things)

| # | path | kind |
|---|------|------|
| 1 | `robojudo/config/g1/policy/g1_agile_velocity_cfg.py` | NEW — 14-joint DoF + `G1AgileVelocityPolicyCfg` |
| 2 | `robojudo/config/g1/g1_cfg.py` | EDIT — import + register `g1_agile_velocity` pipeline |
| 3 | `assets/models/g1/unitree/velocity_history_29dof.pt` | NEW — the AGILE JIT policy |

### Applying to a fresh RoboJuDo clone

(1) and (3) are new files — copy them in. (2) is a 2-hunk edit to `g1_cfg.py`:

```diff
 from .env.g1_real_env_cfg import G1RealEnvCfg, G1UnitreeCfg  # noqa: F401
+from .policy.g1_agile_velocity_cfg import G1AgileVelocityPolicyCfg  # noqa: F401  [ih]
 from .policy.g1_amo_policy_cfg import G1AmoPolicyCfg  # noqa: F401
```
```diff
 # ======================== Configs for supported Policy ======================== #


+@cfg_registry.register
+class g1_agile_velocity(RlPipelineCfg):
+    """[ih] AGILE Velocity-G1-History-v0 (29-DOF) deployed via RoboJuDo sim2sim."""
+
+    robot: str = "g1"
+    env: G1MujocoEnvCfg = G1MujocoEnvCfg(sim_dt=0.005, sim_decimation=4)
+    ctrl: list[KeyboardCtrlCfg] = [KeyboardCtrlCfg()]
+    policy: G1AgileVelocityPolicyCfg = G1AgileVelocityPolicyCfg()
+
+
 @cfg_registry.register
 class g1_h2h(RlPipelineCfg):
```

The JIT policy (3) is `agile/data/policy/velocity_g1/unitree_g1_velocity_history.pt`
from the AGILE repo, copied to `assets/models/g1/unitree/velocity_history_29dof.pt`
(the `policy_name` in the cfg points here).

## Local setup

```bash
# python deps
pip install python-box onnxruntime joblib easydict pygame pyzmq pynput msgpack-numpy colorlog

# REQUIRED: RoboJuDo's forked mujoco_viewer (PyPI mujoco-python-viewer lacks the
# `diable_key_callbacks` arg the env passes — without the fork it crashes on launch).
python submodule_install.py mujoco_viewer
```

## Run

```bash
# SDL_AUDIODRIVER=dummy silences harmless ALSA "no soundcard" spam (headless boxes)
SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c g1_agile_velocity
```

Keyboard (pynput global listener — works regardless of window focus):
`w`/`s` = forward/back (vx), `a`/`d` = strafe (vy), `q`/`e` = turn (wz).
Startup ramps to default pose, blends in the policy, then you drive.

## How it maps (for reference / debugging)

- **obs (255)** = per-term 5-frame history of `[ang_vel·0.2, gravity, cmd, (q-q0), q̇·0.05, last_action]`
  → `UnitreeWoGaitPolicy.get_observation` produces this exact layout/order. JIT: obs(1,255)→action(1,14).
- **14 controlled joints** = legs(12) + waist_roll + waist_pitch (NOT waist_yaw). The 29-DOF env keeps
  all joints; `DoFAdapter`/`merge_dof_cfgs` slice obs to these 14 by name and expand 14 actions back to
  29 (arms/wrists/waist_yaw held at env default pose + gains).
- **action** = `clip(raw, ±10)·0.5 + default_pos`, no smoothing (`action_beta=1.0`).
- **gains** for the 14 come from the AGILE IO descriptor (stiffness 100/200/300, etc.); the rest keep
  RoboJuDo's `G1_29DoF` gains.
- **physics** overridden to 200 Hz (sim_dt 0.005 × decimation 4) to match AGILE training.

## Known tuning point

Observed command = `raw × max_cmd` (default `[0.8, 0.5, 1.57]`), so commanded `vx=0.5` reaches the
policy as ~0.4 m/s → it tracks ~0.34 m/s. For faithful m/s, override in `G1AgileVelocityPolicyCfg`:
`max_cmd: list[float] = [1.0, 1.0, 1.57]`.

## Status / TODO

- 29-DOF: ✅ validated (stand + walk).
- 23-DOF: TODO — `assets/robots/g1/g1_23dof_rev_1_0.xml` already ships; once the AGILE 23-DOF policy is
  trained + exported (JIT), add a `g1_agile_velocity_23dof` the same way (13-joint DoF = legs + waist_yaw).
