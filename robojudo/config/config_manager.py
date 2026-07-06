from robojudo.config import cfg_registry

# [ih] On an unknown `-c`, only suggest the clean 29dof SIM task family (g1_<ih|official>_29dof_*)
# instead of dumping all ~50 configs. Hidden for now: 23dof, real-robot deploy (`_real`), and the
# legacy `agile`-named / velocity-history configs — they don't fit the current sim focus.
_RELEVANT_CFG_KW = ("official", "ih", "agile", "velheight", "velocity", "deepsquat", "29dof", "unitree")
_HIDE_CFG_KW = ("23dof", "_real", "agile")


class ConfigManager:
    def __init__(self, config_name: str, override_cfg: dict | None = None):
        self.config_name = config_name
        self.override_cfg = override_cfg

        self.cfg = self.parse_config()

    def get_cfg(self):
        return self.cfg

    def parse_config(self):
        # [ih] friendly unknown-config error: list only the relevant task family, not all ~50.
        # Meaning is carried by the naming convention itself (g1_<ih|official>_29dof_velheight...),
        # so no per-config description is printed.
        if self.config_name not in cfg_registry.types:
            suggestions = sorted(
                n
                for n in cfg_registry.types
                if any(k in n for k in _RELEVANT_CFG_KW) and not any(h in n for h in _HIDE_CFG_KW)
            )
            raise SystemExit(
                f"[config] unknown -c config: {self.config_name!r}\n"
                f"  available 29dof configs (ih_* = our AGILE retrain, official_* = stock):\n"
                + "\n".join(f"    {n}" for n in suggestions)
            )
        # cfg_class = getattr(robojudo.config, self.config_name)
        cfg_class = cfg_registry.get(self.config_name)
        cfg_raw = cfg_class()
        # cfg_raw = make_g1_pipeline_cfg(
        #     env="g1_mujoco_env",
        #     policy="g1_amo_policy",
        #     ctrl=["keyboard_ctrl"],
        # )
        # cfg_class = type(cfg_raw)

        cfg = cfg_raw

        # TODO: to be deleted
        #  # override from command line or external dict
        # if self.override_cfg is not None:
        #     cfg_intp = OmegaConf.create(cfg_raw.to_dict())
        #     override_cfg = OmegaConf.create(self.override_cfg)
        #     print("Override cfg:", override_cfg)
        #     cfg_intp = OmegaConf.merge(cfg_intp, override_cfg)
        #     cfg_dict = OmegaConf.to_container(cfg_intp, resolve=True)
        #     cfg = cfg_class.model_validate(cfg_dict)  # FOR TEST

        return cfg
        # return Box(cfg_dict)


if __name__ == "__main__":
    from pprint import pprint

    config_manager = ConfigManager("G1PipelineCfg")
    cfg = config_manager.get_cfg()

    pprint(cfg)
