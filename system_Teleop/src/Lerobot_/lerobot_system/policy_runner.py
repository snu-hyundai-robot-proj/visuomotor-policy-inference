import math
from dataclasses import dataclass
from typing import Any, Dict

import numpy as np


@dataclass
class PolicyRunnerConfig:
    policy_path: str
    device: str = "cuda"
    task: str = ""
    robot_type: str = "hyundai"
    local_files_only: bool = False
    use_amp: bool = False
    mock_policy: bool = False
    mock_action_size: int = 6


class LeRobotPolicyRunner:
    def __init__(self, config: PolicyRunnerConfig):
        self.config = config
        self.policy = None
        self.preprocessor = None
        self.postprocessor = None
        self.policy_cfg = None
        self.torch = None
        self.device = None
        self.policy_type = "mock"
        self.input_features: Dict[str, Any] = {}
        self.output_features: Dict[str, Any] = {}
        self.state_dim = 0
        self.action_dim = config.mock_action_size
        self.image_features: Dict[str, tuple[int, ...]] = {}

        if config.mock_policy:
            return

        if not config.policy_path:
            raise ValueError("policy_path is required unless mock_policy is true.")

        try:
            import torch
            from lerobot.configs.policies import PreTrainedConfig
            from lerobot.policies import get_policy_class, make_pre_post_processors
        except Exception as exc:
            raise RuntimeError(
                "Failed to import LeRobot. Install lerobot and the policy extras in this ROS environment."
            ) from exc

        self.torch = torch
        self.device = torch.device(config.device if torch.cuda.is_available() or config.device == "cpu" else "cpu")

        policy_cfg = PreTrainedConfig.from_pretrained(
            config.policy_path,
            local_files_only=config.local_files_only,
        )
        policy_cfg.pretrained_path = config.policy_path
        policy_cfg.device = str(self.device)
        policy_cfg.use_amp = bool(config.use_amp or getattr(policy_cfg, "use_amp", False))
        self.policy_cfg = policy_cfg
        self.policy_type = getattr(policy_cfg, "type", "unknown")
        self.input_features = dict(getattr(policy_cfg, "input_features", {}) or {})
        self.output_features = dict(getattr(policy_cfg, "output_features", {}) or {})
        self.state_dim = self._feature_dim("observation.state", self.input_features)
        self.action_dim = self._feature_dim("action", self.output_features)
        self.image_features = {
            key: tuple(getattr(feature, "shape", ()))
            for key, feature in self.input_features.items()
            if key.startswith("observation.images.")
        }

        policy_cls = get_policy_class(policy_cfg.type)
        try:
            self.policy = policy_cls.from_pretrained(
                pretrained_name_or_path=config.policy_path,
                config=policy_cfg,
            )
        except TypeError:
            self.policy = policy_cls.from_pretrained(
                config.policy_path,
                config=policy_cfg,
            )
        self.policy.to(self.device)
        self.policy.eval()
        if hasattr(self.policy, "reset"):
            self.policy.reset()

        preprocessor_overrides = {
            "device_processor": {"device": str(self.device)},
        }
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=policy_cfg,
            pretrained_path=config.policy_path,
            preprocessor_overrides=preprocessor_overrides,
        )

    def select_action(self, observation: Dict[str, np.ndarray]) -> np.ndarray:
        if self.config.mock_policy:
            state = observation.get("observation.state")
            if state is not None and len(state) >= self.config.mock_action_size:
                return np.asarray(state[: self.config.mock_action_size], dtype=np.float32)
            return np.zeros(self.config.mock_action_size, dtype=np.float32)

        batch = self._to_torch_batch(observation)
        if self.preprocessor is not None:
            batch = self.preprocessor(batch)
        else:
            batch = {
                key: value.to(self.device) if hasattr(value, "to") else value
                for key, value in batch.items()
            }

        amp_context = self.torch.autocast(device_type=self.device.type) if self.config.use_amp else _NullContext()
        with self.torch.inference_mode(), amp_context:
            action = self.policy.select_action(batch)

        if self.postprocessor is not None:
            action_for_postprocessor = action
            if hasattr(action_for_postprocessor, "ndim") and action_for_postprocessor.ndim >= 2:
                action_for_postprocessor = action_for_postprocessor.squeeze(0)
            action = self.postprocessor(action_for_postprocessor)

        return self._action_to_numpy(action)

    def _to_torch_batch(self, observation: Dict[str, np.ndarray]) -> Dict[str, Any]:
        batch: Dict[str, Any] = {}

        for key, value in observation.items():
            if value is None:
                continue

            arr = np.asarray(value)
            tensor = self.torch.from_numpy(arr)

            if "image" in key:
                if tensor.ndim != 3:
                    raise ValueError(f"{key} must be HWC image data, got shape {tuple(tensor.shape)}.")
                tensor = tensor.to(dtype=self.torch.float32) / 255.0
                tensor = tensor.permute(2, 0, 1).contiguous()
            else:
                tensor = tensor.to(dtype=self.torch.float32)

            batch[key] = tensor.unsqueeze(0)

        batch["task"] = self.config.task
        batch["robot_type"] = self.config.robot_type
        return batch

    def _action_to_numpy(self, action: Any) -> np.ndarray:
        if isinstance(action, dict):
            action = action.get("action", next(iter(action.values())))

        if hasattr(action, "to"):
            action = action.detach().to("cpu").numpy()
        else:
            action = np.asarray(action)

        action = np.asarray(action, dtype=np.float32)
        if action.ndim >= 2:
            action = action[0]
        return action.reshape(-1)

    @staticmethod
    def _feature_dim(key: str, features: Dict[str, Any]) -> int:
        feature = features.get(key)
        if feature is None:
            return 0
        shape = getattr(feature, "shape", None)
        if not shape:
            return 0
        total = 1
        for dim in shape:
            total *= int(dim)
        return int(total)


def convert_units(values: np.ndarray, source_unit: str, target_unit: str) -> np.ndarray:
    source = source_unit.lower()
    target = target_unit.lower()
    out = np.asarray(values, dtype=np.float64)

    if source == target:
        return out
    if source == "rad" and target == "deg":
        return out * 180.0 / math.pi
    if source == "deg" and target == "rad":
        return out * math.pi / 180.0
    return out


class _NullContext:
    def __enter__(self):
        return None

    def __exit__(self, exc_type, exc, traceback):
        return False
