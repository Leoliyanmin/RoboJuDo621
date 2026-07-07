from robojudo.config import ASSETS_DIR, cfg_registry
from robojudo.controller.ctrl_cfgs import (
    JoystickCtrlCfg,  # noqa: F401
    KeyboardCtrlCfg,  # noqa: F401
    UnitreeCtrlCfg,  # noqa: F401
)
from robojudo.pipeline.pipeline_cfgs import (
    RlLocoMimicPipelineCfg,  # noqa: F401
    RlMultiPolicyPipelineCfg,  # noqa: F401
    RlPipelineCfg,  # noqa: F401
)

from .ctrl.g1_beyondmimic_ctrl_cfg import G1BeyondmimicCtrlCfg  # noqa: F401
from .ctrl.g1_motion_ctrl_cfg import (  # noqa: F401
    G1MotionCtrlCfg,
    G1MotionH2HCtrlCfg,
    G1MotionKungfuBotCtrlCfg,
    G1MotionTwistCtrlCfg,
)
from .ctrl.g1_twist_redis_ctrl_cfg import G1TwistRedisCtrlCfg  # noqa: F401
from .env.g1_dummy_env_cfg import G1DummyEnvCfg  # noqa: F401
from .env.g1_mujuco_env_cfg import G1_12MujocoEnvCfg, G1_23MujocoEnvCfg, G1MujocoEnvCfg  # noqa: F401
from .env.g1_real_env_cfg import G1RealEnvCfg, G1UnitreeCfg  # noqa: F401
from .policy.g1_agile_velheight_cfg import (  # noqa: F401
    G1DeepSquatArmSwingPolicyCfg,
    G1DeepSquatPINPolicyCfg,
    G1DeepSquatPINStudentPolicyCfg,
    G1DeepSquatRandArmsPolicyCfg,
)
from .policy.g1_amo_policy_cfg import G1AmoPolicyCfg  # noqa: F401
from .policy.g1_asap_policy_cfg import G1AsapLocoPolicyCfg, G1AsapPolicyCfg  # noqa: F401
from .policy.g1_beyondmimic_policy_cfg import G1BeyondMimicPolicyCfg  # noqa: F401
from .policy.g1_h2h_policy_cfg import G1H2HPolicyCfg  # noqa: F401
from .policy.g1_kungfubot_policy_cfg import G1KungfuBotGeneralPolicyCfg, G1KungfuBotPolicyCfg  # noqa: F401
from .policy.g1_smooth_policy_cfg import G1SmoothPolicyCfg  # noqa: F401
from .policy.g1_twist_policy_cfg import G1TwistPolicyCfg  # noqa: F401
from .policy.g1_unitree_policy_cfg import G1UnitreePolicyCfg, G1UnitreeWoGaitPolicyCfg  # noqa: F401

# ======================== Custom Configs ======================== #
"""
Add your custom config here.
"""


@cfg_registry.register
class g1_dev(RlPipelineCfg):
    robot: str = "g1"
    env: G1_23MujocoEnvCfg = G1_23MujocoEnvCfg()

    ctrl: list[KeyboardCtrlCfg] = [
        KeyboardCtrlCfg(),
    ]

    policy: G1UnitreePolicyCfg = G1UnitreePolicyCfg()


# ── Deep-squat × arm-swing ablation sim2sim display configs (2026-07-04) ──────────────────
# All three run on g1_29dof_rev_1_0_handmass.xml at 200 Hz (sim_dt=0.005, decimation=4).
# Keys: w/a/s/d = vx/vy, q/e = yaw, r/f = stand taller / squat lower, ,/. = waist lean.
# See deepsquat_armswing_final_policies/README.md for eval results and env differences.
# Run: SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c <config_name>

_DEEPSQUAT_ENV = dict(
    sim_dt=0.005,
    sim_decimation=4,
    xml=(ASSETS_DIR / "robots/g1/g1_29dof_rev_1_0_handmass.xml").as_posix(),
    visualize_extras=False,
    wrist_load_n=0.0,
    waist_manual_keyboard=True,
    # elbow-primary arm swing: elbow flexes/extends sinusoidally, shoulder pitch stays at hang.
    arm_motion_mode="walk_elbow",
    arm_swing_amp=0.22,        # elbow swing amplitude (rad); j/k to adjust at runtime
    arm_swing_scale=1.0,       # multiplier; j/k steps by arm_swing_step
    arm_stride_ref=0.18,
    arm_stride_filter_alpha=0.55,
    arm_walk_spread_amp=0.12,  # per-unit shoulder-roll spread (rad)
    arm_spread_scale=2.2,      # default: fully spread (arm_spread_max); c/v to adjust
    arm_motion_rate_dps=260.0,
    arm_swing_keyboard=True,   # j = swing up / k = swing down
    arm_spread_keyboard=True,  # v = spread up / c = spread down
)


# [ih] Student display env: two demo motions in one mode (walk_squat_box).
#   • walking  -> shoulder-pitch arm swing (走路摆手)
#   • squatting-> arms blend forward into the box-carry reach pose (下蹲前伸手臂),
#     gated by the height command (arm_stand_height 0.66 -> arm_squat_height 0.52).
# z/x tune forward-reach depth, c/v tune spread, j/k tune walk-swing.
_DEEPSQUAT_STUDENT_ENV = {
    **_DEEPSQUAT_ENV,
    "arm_motion_mode": "walk_squat_box",
    "arm_reach_keyboard": True,  # z/x = reach forward less/more when squatting
}


@cfg_registry.register
class g1_ih_29dof_deepsquat_pin(RlPipelineCfg):
    """[ih] Deep-squat PIN — DEPLOYABLE recurrent STUDENT (distilled from the PIN teacher).

    128-dim proprioceptive obs (no privileged base_lin_vel), LSTM-recurrent, 12-leg actions;
    distilled from repro29 model_3750 (behavior loss 0.0097). This is the deployable policy.
    Arm demo (walk_squat_box): walking -> shoulder-pitch swing; squatting -> arms reach forward.
    Run: SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c g1_ih_29dof_deepsquat_pin
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(**_DEEPSQUAT_STUDENT_ENV)
    ctrl: list[KeyboardCtrlCfg] = [KeyboardCtrlCfg()]
    policy: G1DeepSquatPINStudentPolicyCfg = G1DeepSquatPINStudentPolicyCfg()


@cfg_registry.register
class g1_ih_29dof_deepsquat_pin_teacher(RlPipelineCfg):
    """[ih] Deep-squat PIN TEACHER (privileged, non-recurrent, 131-dim obs incl. base_lin_vel).

    Sim-only diagnostic (base_lin_vel is unavailable on hardware). model_3750: 0.31 m pelvis at
    cmd 0.20, 2 % fall (deep+stable); walking+arm-swing 0–1 % fall. Reference for the ablations.
    Run: SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c g1_ih_29dof_deepsquat_pin_teacher
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(**_DEEPSQUAT_ENV)
    ctrl: list[KeyboardCtrlCfg] = [KeyboardCtrlCfg()]
    policy: G1DeepSquatPINPolicyCfg = G1DeepSquatPINPolicyCfg()


@cfg_registry.register
class g1_ih_29dof_deepsquat_randarms(RlPipelineCfg):
    """[ih] Deep-squat #1 RandArms (random arm pose at ALL gaits incl. walking).

    Training note: no_random_when_walking=False — arms jump to random static poses even while
    walking. Result: catastrophic degradation (100 % squat fall, 31–47 % walk fall). Included
    for ablation reference only; do NOT deploy.
    Run: SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c g1_ih_29dof_deepsquat_randarms
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(**_DEEPSQUAT_ENV)
    ctrl: list[KeyboardCtrlCfg] = [KeyboardCtrlCfg()]
    policy: G1DeepSquatRandArmsPolicyCfg = G1DeepSquatRandArmsPolicyCfg()


@cfg_registry.register
class g1_ih_29dof_deepsquat_armswing(RlPipelineCfg):
    """[ih] Deep-squat #2 ArmSwing (scripted sinusoidal arm swing, speed-gated).

    Shoulder pitch/roll driven by ArmSwingAction (sine swing + outward spread when walking,
    returns to default at zero speed). Result: walk+arm-swing as stable as PIN (0–1 % fall),
    but squat depth 12 cm shallower than PIN (multi-task interference).
    Run: SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c g1_ih_29dof_deepsquat_armswing
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(**_DEEPSQUAT_ENV)
    ctrl: list[KeyboardCtrlCfg] = [KeyboardCtrlCfg()]
    policy: G1DeepSquatArmSwingPolicyCfg = G1DeepSquatArmSwingPolicyCfg()
