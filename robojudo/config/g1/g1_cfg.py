from robojudo.config import ASSETS_DIR, cfg_registry  # [ih] ASSETS_DIR for handmass xml
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
from .policy.g1_agile_velocity_cfg import G1AgileVelocityPolicyCfg  # noqa: F401  [ih]
from .policy.g1_agile_velocity_23dof_cfg import G1AgileVelocity23DOFPolicyCfg  # noqa: F401  [ih]
from .policy.g1_agile_velheight_cfg import G1AgileVelHeightPolicyCfg  # noqa: F401  [ih]
from .policy.g1_amo_policy_cfg import G1AmoPolicyCfg  # noqa: F401
from .policy.g1_asap_policy_cfg import G1AsapLocoPolicyCfg, G1AsapPolicyCfg  # noqa: F401
from .policy.g1_beyondmimic_policy_cfg import G1BeyondMimicPolicyCfg  # noqa: F401
from .policy.g1_h2h_policy_cfg import G1H2HPolicyCfg  # noqa: F401
from .policy.g1_kungfubot_policy_cfg import G1KungfuBotGeneralPolicyCfg, G1KungfuBotPolicyCfg  # noqa: F401
from .policy.g1_protomotions_tracker_cfg import ProtoMotionsTrackerPolicyCfg  # noqa: F401
from .policy.g1_smooth_policy_cfg import G1SmoothPolicyCfg  # noqa: F401
from .policy.g1_twist_policy_cfg import G1TwistPolicyCfg  # noqa: F401
from .policy.g1_unitree_policy_cfg import G1UnitreePolicyCfg, G1UnitreeWoGaitPolicyCfg  # noqa: F401


# ======================== Basic Configs ======================== #
@cfg_registry.register
class g1(RlPipelineCfg):
    """
    Unitree G1 robot configuration, Unitree Policy, Sim2Sim.
    You can modify to play with other policies and controllers.
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg()
    # env: G1_23MujocoEnvCfg = G1_23MujocoEnvCfg()
    # env: G1_12MujocoEnvCfg = G1_12MujocoEnvCfg()

    ctrl: list[JoystickCtrlCfg | KeyboardCtrlCfg] = [  # note: the ranking of controllers matters
        JoystickCtrlCfg(),
        # KeyboardCtrlCfg(),
    ]

    policy: G1UnitreePolicyCfg = G1UnitreePolicyCfg()
    # policy: G1UnitreeWoGaitPolicyCfg = G1UnitreeWoGaitPolicyCfg()
    # policy: G1AmoPolicyCfg = G1AmoPolicyCfg()

    # run_fullspeed: bool = env.is_sim


@cfg_registry.register
class g1_real(g1):
    """
    Unitree G1 robot, Unitree Policy, Sim2Real.
    To extend the sim2sim config to sim2real, just need to change the env to real env.
    """

    # env: G1DummyEnvCfg = G1DummyEnvCfg()
    env: G1RealEnvCfg = G1RealEnvCfg(
        # env_type="UnitreeEnv",  # For unitree_sdk2py
        env_type="UnitreeCppEnv",  # For unitree_cpp, check README for more details
        unitree=G1UnitreeCfg(
            net_if="eth0",  # note: change to your network interface
        ),
    )

    ctrl: list[UnitreeCtrlCfg] = [
        UnitreeCtrlCfg(),
    ]

    do_safety_check: bool = True  # enable safety check for real robot


@cfg_registry.register
class g1_switch(RlMultiPolicyPipelineCfg):
    """
    Example of multi-policy pipeline configuration.
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg()

    ctrl: list[KeyboardCtrlCfg | JoystickCtrlCfg] = [
        # KeyboardCtrlCfg(
        #     triggers_extra={
        #         "Key.tab": "[POLICY_TOGGLE]",
        #     }
        # ),
        JoystickCtrlCfg(
            triggers_extra={
                "RB+Down": "[POLICY_SWITCH],0",
                "RB+Up": "[POLICY_SWITCH],1",
            }
        ),
    ]

    policies: list[G1UnitreePolicyCfg | G1AmoPolicyCfg] = [
        G1UnitreePolicyCfg(),
        G1AmoPolicyCfg(),
    ]


@cfg_registry.register
class g1_locomimic(RlLocoMimicPipelineCfg):
    """
    Example of loco mimic pipeline configuration.
    You can switch between loco and mimic policies during runtime, with interpolation.
    === Check more fancy locomimic examples in g1_loco_mimic_cfg.py ===
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg()

    ctrl: list[KeyboardCtrlCfg | JoystickCtrlCfg] = [
        KeyboardCtrlCfg(
            triggers_extra={
                "]": "[POLICY_LOCO]",
                "[": "[POLICY_MIMIC]",
            }
        ),
        JoystickCtrlCfg(
            triggers_extra={
                "RB+Down": "[POLICY_LOCO]",
                "RB+Up": "[POLICY_MIMIC]",
            }
        ),
    ]

    loco_policy: G1UnitreePolicyCfg = G1UnitreePolicyCfg()
    mimic_policies: list[G1AsapPolicyCfg] = [
        G1AsapPolicyCfg(),
    ]


# ======================== Configs for supported Policy ======================== #


# [ih] AGILE 29-DOF velocity-history policy, sim2sim. Keyboard velocity teleop
# (w/a/s/d = vx/vy, q/e = yaw). env physics overridden to 200 Hz (sim_dt 0.005 x
# decimation 4) to match AGILE training (RoboJuDo default is 1000 Hz). Uses the
# stock g1_29dof_rev_1_0.xml. Run: python scripts/run_pipeline.py -c g1_agile_velocity
@cfg_registry.register
class g1_agile_velocity(RlPipelineCfg):
    """[ih] AGILE Velocity-G1-History-v0 (29-DOF) deployed via RoboJuDo sim2sim."""

    robot: str = "g1"
    # [ih] visualize_extras=False: suppress UnitreeWoGaitPolicy.debug_viz's command
    # arrows (red/green/white), which flicker with the keyboard command. No viewer markers.
    # wrist load is keyboard-adjustable ('[' / ']') + shown above the robot; starts at 0
    # since the 29-DOF velocity policy is NOT wrist20-trained (any load is out-of-distribution).
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(
        sim_dt=0.005,
        sim_decimation=4,
        visualize_extras=False,
        wrist_load_n=0.0,
        wrist_load_bodies=["left_wrist_yaw_link", "right_wrist_yaw_link"],
        wrist_load_keyboard=True,
    )
    # [ih] keyboard-only (w/a/s/d=vx/vy, q/e=wz). Add JoystickCtrlCfg() back if a
    # gamepad is plugged in — otherwise it just logs a harmless "No joystick" error.
    ctrl: list[KeyboardCtrlCfg] = [
        KeyboardCtrlCfg(),
    ]
    policy: G1AgileVelocityPolicyCfg = G1AgileVelocityPolicyCfg()


# [ih] AGILE Velocity-G1-History-23DOF-Wrist20 (23-DOF G1, 13 controlled = legs+waist_yaw)
# via RoboJuDo sim2sim. Same velocity-history family as g1_agile_velocity (config-only,
# UnitreeWoGaitPolicy). Uses the 23-DOF robot/mjcf (g1_23dof_rev_1_0.xml). 200 Hz physics.
# wrist20-trained, so a sustained 10 N/wrist box-carry load is on by default (set 0 to drop).
# Run: SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c g1_agile_velocity_23dof
@cfg_registry.register
class g1_agile_velocity_23dof(RlPipelineCfg):
    """[ih] AGILE Velocity-G1-History-23DOF-Wrist20 deployed via RoboJuDo sim2sim."""

    robot: str = "g1"
    env: G1_23MujocoEnvCfg = G1_23MujocoEnvCfg(
        sim_dt=0.005,
        sim_decimation=4,
        wrist_load_n=10.0,
        wrist_load_bodies=["left_wrist_roll_rubber_hand", "right_wrist_roll_rubber_hand"],
        # [ih] suppress UnitreeWoGaitPolicy.debug_viz command arrows (flicker w/ keys).
        visualize_extras=False,
        # [ih] '[' / ']' adjust the box-carry load at runtime; readout shown above the robot.
        wrist_load_keyboard=True,
    )
    ctrl: list[KeyboardCtrlCfg] = [
        KeyboardCtrlCfg(),
    ]
    policy: G1AgileVelocity23DOFPolicyCfg = G1AgileVelocity23DOFPolicyCfg()


# [ih] AGILE velocity-HEIGHT frozen-hands RECURRENT (LSTM) policy, sim2sim.
# Keyboard: w/a/s/d=vx/vy, q/e=yaw, r/f=stand taller/squat lower. obs 128 (no history),
# 12 leg joints controlled, 24 DFQ hand joints zero-padded (frozen). 200Hz physics.
# Run: SDL_AUDIODRIVER=dummy python scripts/run_pipeline.py -c g1_agile_velheight
@cfg_registry.register
class g1_agile_velheight(RlPipelineCfg):
    """[ih] AGILE Velocity-Height FrozenHands Wrist20 distillation recurrent student."""

    robot: str = "g1"
    # [ih][GPT-5.5 audit 2026-06-22] Historical note: this file used to be described
    # as adding a separate 0.1918 kg DFQ hand mass per wrist, but that description is
    # now stale. The current ``g1_29dof_rev_1_0_handmass.xml`` keeps the stock MJCF
    # inertials unchanged and adds only welded DFQ visual meshes (density=0, no hand
    # joints/collisions). Evidence: both the stock and handmass MJCF sum to
    # 33.341142 kg with 30 inertials and 29 motors; the current handmass MJCF contains
    # no ``left_hand_mass``/``right_hand_mass`` body or 0.1918 kg inertial.
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(
        sim_dt=0.005,
        sim_decimation=4,
        xml=(ASSETS_DIR / "robots/g1/g1_29dof_rev_1_0_handmass.xml").as_posix(),
        # [ih] default 10 N down per wrist — this is a wrist20-trained policy, so show it
        # carrying a load. Set 0 for no load, up to ~20 (the max it was trained on).
        wrist_load_n=10.0,
        # [ih] '[' / ']' adjust the box-carry load at runtime; readout shown above the robot.
        # (r/f stay mapped to height in the policy — no key clash.)
        wrist_load_keyboard=True,
        # [ih] manual waist_pitch test: ',' lean back / '.' lean forward (full ±30° limit),
        # angle shown in readout — hand-test whether the policy tolerates / benefits from lean.
        waist_manual_keyboard=True,
    )
    ctrl: list[KeyboardCtrlCfg] = [
        KeyboardCtrlCfg(),
    ]
    policy: G1AgileVelHeightPolicyCfg = G1AgileVelHeightPolicyCfg()


@cfg_registry.register
class g1_h2h(RlPipelineCfg):
    """
    Human2Humanoid
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg()
    ctrl: list[KeyboardCtrlCfg | G1MotionH2HCtrlCfg] = [
        KeyboardCtrlCfg(),
        G1MotionH2HCtrlCfg(),
    ]

    policy: G1H2HPolicyCfg = G1H2HPolicyCfg()


@cfg_registry.register
class g1_beyondmimic(RlPipelineCfg):
    """
    BeyondMimic Policy, support both with and without state estimator.
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg()
    ctrl: list[KeyboardCtrlCfg] = [
        KeyboardCtrlCfg(),
    ]

    policy: G1BeyondMimicPolicyCfg = G1BeyondMimicPolicyCfg(
        policy_name="Jump_wose",
        without_state_estimator=True,
        use_modelmeta_config=True,  # use robot dof config from modelmeta
        use_motion_from_model=True,  # use motion from onnx model
        max_timestep=140,
    )


@cfg_registry.register
class g1_beyondmimic_with_ctrl(RlPipelineCfg):
    """
    BeyondMimic with External BeyondMimicCtrl as motion source.
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg()
    ctrl: list[KeyboardCtrlCfg | G1BeyondmimicCtrlCfg] = [
        KeyboardCtrlCfg(),
        G1BeyondmimicCtrlCfg(
            motion_name="dance1_subject2",  # you can put your own motion file in assets/motions/g1
        ),
    ]

    policy: G1BeyondMimicPolicyCfg = G1BeyondMimicPolicyCfg(
        policy_name="Dance_wose",
        use_motion_from_model=False,  # use motion from BeyondmimicCtrl instead of the onnx
    )


@cfg_registry.register
class g1_asap(RlPipelineCfg):
    """
    Unitree G1 robot configuration, ASAP Policy, Sim2Sim.
    You can modify to play with other policies and controllers.
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(forward_kinematic=None, update_with_fk=False, born_place_align=True)

    ctrl: list[JoystickCtrlCfg | KeyboardCtrlCfg] = [  # note: the ranking of controllers matters
        # JoystickCtrlCfg(),
        KeyboardCtrlCfg(triggers={"i": "[SIM_REBORN]", "o": "[SHUTDOWN]", "r": "[MOTION_RESET]"}),
    ]

    policy: G1AsapPolicyCfg = G1AsapPolicyCfg()
    """You can also try other models, from ASAP, RoboMimic, KungfuBot(PBHC)"""
    # policy: G1KungfuBotPolicyCfg = G1KungfuBotPolicyCfg() # KungfuBot horse_squat
    # # fmt: off
    # policy: G1AsapPolicyCfg = G1AsapPolicyCfg(
    #     policy_name="robomimic",
    #     relative_path="dance_0605.onnx",
    #     motion_length_s=18.0,
    #     start_upper_body_dof_pos = [
    #         0, 0, 0,
    #         0.35, 0.18, 0, 0.87,
    #         0.35, -0.18, 0, 0.87,
    #     ],
    # )
    # # fmt: on


@cfg_registry.register
class g1_asap_loco(RlPipelineCfg):
    """
    Unitree G1 robot configuration, ASAP Locomotion Policy, Sim2Sim.
    You can modify to play with other policies and controllers.
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(forward_kinematic=None, update_with_fk=False, born_place_align=False)

    ctrl: list[JoystickCtrlCfg | KeyboardCtrlCfg] = [  # note: the ranking of controllers matters
        # JoystickCtrlCfg(),
        KeyboardCtrlCfg(
            triggers={
                "i": "[SIM_REBORN]",
                "o": "[SHUTDOWN]",
            }
        ),
    ]

    policy: G1AsapLocoPolicyCfg = G1AsapLocoPolicyCfg()


@cfg_registry.register
class g1_kungfubot2(RlPipelineCfg):
    """
    PBHC KungfuBot2 General Policy
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg()
    ctrl: list[KeyboardCtrlCfg | G1MotionKungfuBotCtrlCfg] = [
        KeyboardCtrlCfg(),
        G1MotionKungfuBotCtrlCfg(
            motion_name="kungfubot/Horse-stance_pose",  # put motion files in assets/motions/g1/phc/kungfubot
        ),
    ]

    policy: G1KungfuBotGeneralPolicyCfg = G1KungfuBotGeneralPolicyCfg(
        policy_name="horse_test_43000",  # this is a test model trained with only one motion
        compatibility_old_version=True,  # for old version of kungfubot general policy (before 2025-11-13 bugfix #68)
    )


@cfg_registry.register
class g1_twist(RlPipelineCfg):
    """
    Unitree G1 robot configuration, TWIST Policy, Sim2Sim.
    TwistRedisCtrl for the original repo of high level motion stream over redis.
    MotionTwistCtrl for built-in motion control.
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(forward_kinematic=None, update_with_fk=False, born_place_align=False)

    ctrl: list[G1TwistRedisCtrlCfg | G1MotionTwistCtrlCfg] = [  # note: the ranking of controllers matters
        G1TwistRedisCtrlCfg(redis_host="localhost"),  # with hign level motion lib through redis
        # G1MotionTwistCtrlCfg(), # with built-in motion ctrl
    ]

    policy: G1TwistPolicyCfg = G1TwistPolicyCfg()


# ======================== Fancy Example Configs ======================== #


@cfg_registry.register
class g1_switch_beyondmimic(RlMultiPolicyPipelineCfg):
    """
    Switch between multiple BeyondMimic policies. Withour Interpolation.
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg()
    ctrl: list[KeyboardCtrlCfg | JoystickCtrlCfg] = [
        KeyboardCtrlCfg(
            triggers_extra={
                "Key.tab": "[POLICY_TOGGLE]",
                "!": "[POLICY_SWITCH],0",  # note: with shift
                "@": "[POLICY_SWITCH],1",  # note: with shift
                "#": "[POLICY_SWITCH],2",  # note: with shift
                "$": "[POLICY_SWITCH],3",  # note: with shift
            }
        ),
        JoystickCtrlCfg(
            triggers_extra={
                "RB+Down": "[POLICY_SWITCH],0",
                "RB+Left": "[POLICY_SWITCH],1",
                "RB+Up": "[POLICY_SWITCH],2",
                "RB+Right": "[POLICY_SWITCH],3",
            }
        ),
    ]

    policies: list[G1AmoPolicyCfg | G1BeyondMimicPolicyCfg] = [
        G1AmoPolicyCfg(),
        G1BeyondMimicPolicyCfg(policy_name="Violin", without_state_estimator=False, max_timestep=500),
        G1BeyondMimicPolicyCfg(policy_name="Waltz", without_state_estimator=False, max_timestep=850),
        G1BeyondMimicPolicyCfg(policy_name="Dance_wose", without_state_estimator=True),
    ]


# ======================== ProtoMotions Tracker ======================== #


@cfg_registry.register
class g1_protomotions_tracker(RlPipelineCfg):
    """ProtoMotions tracker with cached 50fps motion.

    Uses the standard RoboJuDo G1 MuJoCo environment with ``born_place_align``
    disabled (our policy handles heading alignment itself). ``random_heading``
    is on so we exercise the policy's heading-alignment recompute on each spawn.

    Use ``scripts/run_tracker_pipeline.py`` — it parses ``--onnx-path`` /
    ``--motion-path`` / ``--motion-index``, which the generic ``run_pipeline.py``
    does not.

    Usage::

        python scripts/run_tracker_pipeline.py -c g1_protomotions_tracker \\
            --motion-path assets/motions/g1/g1_bones_seed_mini.pt \\
            --motion-index 0
    """

    robot: str = "g1"
    env: G1MujocoEnvCfg = G1MujocoEnvCfg(
        born_place_align=False,
        random_heading=True,
    )
    ctrl: list[KeyboardCtrlCfg] = [
        KeyboardCtrlCfg(
            triggers={
                "r": "[MOTION_RESET]",
                "i": "[SIM_REBORN]",
                "o": "[SHUTDOWN]",
                "<": "[MOTION_FADE_IN]",
                ">": "[MOTION_FADE_OUT]",
            },
        ),
    ]

    policy: ProtoMotionsTrackerPolicyCfg = ProtoMotionsTrackerPolicyCfg()


@cfg_registry.register
class g1_protomotions_tracker_real(g1_protomotions_tracker):
    """ProtoMotions tracker on real G1 hardware.

    Use ``scripts/run_tracker_pipeline.py`` — it parses ``--onnx-path`` /
    ``--motion-path`` / ``--motion-index``, which the generic ``run_pipeline.py``
    does not.

    Usage::

        python scripts/run_tracker_pipeline.py -c g1_protomotions_tracker_real \\
            --motion-path assets/motions/g1/g1_bones_seed_mini.pt \\
            --motion-index 0
    """

    env: G1RealEnvCfg = G1RealEnvCfg(
        env_type="UnitreeCppEnv",
        unitree=G1UnitreeCfg(
            net_if="eth0",  # note: change to your network interface
        ),
        born_place_align=False,
    )
    ctrl: list[UnitreeCtrlCfg] = [
        UnitreeCtrlCfg(),
    ]
    do_safety_check: bool = True


# TIPS: check g1_loco_mimic_cfg.py for more complex examples
