from typing import Literal

from robojudo.environment.env_cfgs import UnitreeEnvCfg

# from robojudo.tools.tool_cfgs import ZedOdometryCfg
from .g1_env_cfg import G1_23EnvCfg, G1EnvCfg


G1_23_ARM_SDK_MOTOR_IDX = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26]


class G1UnitreeCfg(UnitreeEnvCfg.UnitreeCfg):
    robot: Literal["h1", "g1"] = "g1"

    msg_type: Literal["hg", "go"] = "hg"
    hand_type: Literal["Dex-3", "Inspire", "NONE"] = "NONE"

    enable_odometry: bool = True
    motor_cmd_num_dofs: int | None = 29


class G1RealEnvCfg(G1EnvCfg, UnitreeEnvCfg):
    # env_type: str = UnitreeEnvCfg.model_fields["env_type"].default
    env_type: str = "UnitreeCppEnv"
    # ====== ENV CONFIGURATION ======
    unitree: UnitreeEnvCfg.UnitreeCfg = G1UnitreeCfg(
        net_if="eth0",
    )

    odometry_type: Literal["NONE", "DUMMY", "UNITREE", "ZED"] = "UNITREE"

    joint2motor_idx: list[int] | None = None  # list(range(0, 29))


class G1_23RealEnvCfg(G1_23EnvCfg, UnitreeEnvCfg):
    env_type: str = "UnitreeCppEnv"
    # ====== ENV CONFIGURATION ======
    odometry_type: Literal["NONE", "DUMMY", "UNITREE", "ZED"] = "UNITREE"

    # HG lowcmd/lowstate keeps the 29-slot G1 layout. Native 23-DoF G1 removes
    # waist_roll, waist_pitch, and wrist pitch/yaw joints, so env DoFs are compact
    # but real motor slots are not.
    joint2motor_idx: list[int] | None = [
        *range(0, 13),
        15,
        16,
        17,
        18,
        19,
        22,
        23,
        24,
        25,
        26,
    ]
    unitree: UnitreeEnvCfg.UnitreeCfg = G1UnitreeCfg(
        net_if="eth0",
        arm_sdk_motor_idx=G1_23_ARM_SDK_MOTOR_IDX,
    )


class G1WithHandRealEnvCfg(G1EnvCfg, UnitreeEnvCfg):
    # env_type: str = UnitreeEnvCfg.model_fields["env_type"].default
    env_type: str = "UnitreeCppEnv"
    # ====== ENV CONFIGURATION ======
    unitree: UnitreeEnvCfg.UnitreeCfg = G1UnitreeCfg(
        net_if="eth0",
        hand_type="Dex-3",
    )

    odometry_type: Literal["NONE", "DUMMY", "UNITREE", "ZED"] = "DUMMY"
    # zed_cfg: ZedOdometryCfg | None = ZedOdometryCfg(
    #     server_ip="192.168.123.167",
    #     pos_offset=[0.0, 0.0, 0.9],
    #     zero_align=True,
    # )

    joint2motor_idx: list[int] | None = None  # list(range(0, 29))
