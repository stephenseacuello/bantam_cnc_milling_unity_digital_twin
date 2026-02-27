"""
ML Model Manager Node.

Manages ML model lifecycle: loading, versioning, A/B testing,
and hot-swapping models without downtime.
"""

from typing import Any, Dict, List, Optional, Tuple
from dataclasses import dataclass, field
import os
import json
import re
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import ModelUpdate


def _parse_semver(version: str) -> Tuple[int, int, int]:
    """Parse a semantic version string into (major, minor, patch).

    Handles formats like '1.2.3', 'v1.2.3', '1.2', and '1'.
    """
    version = version.lstrip('vV')
    match = re.match(r'^(\d+)(?:\.(\d+))?(?:\.(\d+))?', version)
    if not match:
        return (0, 0, 0)
    major = int(match.group(1))
    minor = int(match.group(2)) if match.group(2) else 0
    patch = int(match.group(3)) if match.group(3) else 0
    return (major, minor, patch)


def compare_versions(v1: str, v2: str) -> int:
    """Compare two semantic version strings.

    Returns:
        -1 if v1 < v2, 0 if v1 == v2, 1 if v1 > v2.
    """
    parsed1 = _parse_semver(v1)
    parsed2 = _parse_semver(v2)
    if parsed1 < parsed2:
        return -1
    elif parsed1 > parsed2:
        return 1
    return 0


@dataclass
class ModelInfo:
    """Information about a managed ML model."""
    model_id: str
    model_type: str
    version: str
    path: str
    status: str = 'LOADED'
    accuracy: float = 0.0
    is_active: bool = False
    load_time_sec: float = 0.0


class ModelManagerNode(MiracleLifecycleNode):
    """Manages ML model lifecycle and versioning.

    Parameters:
        model_directory (str): Base directory for model files.
        max_models_loaded (int): Maximum models in memory.
        auto_switch_threshold (float): Accuracy threshold for auto-switch.

    Published Topics:
        ~/model_updates (ModelUpdate): Model lifecycle events.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'model_manager',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._models: Dict[str, ModelInfo] = {}
        self._model_history: Dict[str, List[ModelInfo]] = {}
        self._models_lock = threading.Lock()
        self._model_dir: str = ''
        self._update_pub = None

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure model manager."""
        params = self.declare_and_validate_parameters({
            'model_directory': {
                'default': '/tmp/miracle_models',
                'type': str,
            },
            'max_models_loaded': {
                'default': 10,
                'type': int,
                'range': (1, 100),
            },
            'auto_switch_threshold': {
                'default': 0.05,
                'type': float,
                'range': (0.0, 1.0),
            },
        })

        self._model_dir = params['model_directory']
        os.makedirs(self._model_dir, exist_ok=True)

        self._update_pub = self.create_publisher(
            ModelUpdate,
            'model_updates',
            QoSProfiles.state_data(),
        )

        self.get_logger().info(
            f"Model manager configured (dir={self._model_dir})"
        )
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate model manager."""
        self._scan_model_directory()
        self.get_logger().info("Model manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate model manager."""
        return TransitionCallbackReturn.SUCCESS

    def _scan_model_directory(self) -> None:
        """Scan model directory for available models."""
        try:
            for filename in os.listdir(self._model_dir):
                if filename.endswith('.json'):
                    filepath = os.path.join(self._model_dir, filename)
                    with open(filepath, 'r') as f:
                        metadata = json.load(f)
                    model_id = metadata.get('model_id', filename)
                    self._models[model_id] = ModelInfo(
                        model_id=model_id,
                        model_type=metadata.get('type', 'unknown'),
                        version=metadata.get('version', '0.0.0'),
                        path=filepath,
                    )
                    self.get_logger().info(f"Found model: {model_id}")
        except Exception as exc:
            self.get_logger().debug(f"Model scan: {exc}")

    def register_model(
        self,
        model_id: str,
        model_type: str,
        version: str,
        path: str,
    ) -> bool:
        """Register a new model.

        Args:
            model_id: Unique model identifier.
            model_type: Type of model (anomaly, phm, wear).
            version: Model version string.
            path: Path to model file.

        Returns:
            True if registered successfully.
        """
        max_loaded = self.get_parameter('max_models_loaded').value

        with self._models_lock:
            if len(self._models) >= max_loaded:
                # Evict oldest inactive model
                inactive = [
                    m for m in self._models.values()
                    if not m.is_active
                ]
                if inactive:
                    evict = inactive[0]
                    del self._models[evict.model_id]
                else:
                    return False

            new_info = ModelInfo(
                model_id=model_id,
                model_type=model_type,
                version=version,
                path=path,
            )
            # Track version history for rollback support
            if model_id not in self._model_history:
                self._model_history[model_id] = []
            if model_id in self._models:
                self._model_history[model_id].append(self._models[model_id])
            self._models[model_id] = new_info

        self.get_logger().info(
            f"Registered model: {model_id} v{version} ({model_type})"
        )
        return True

    def activate_model(self, model_id: str) -> bool:
        """Activate a model for inference.

        Args:
            model_id: Model to activate.

        Returns:
            True if activated.
        """
        with self._models_lock:
            if model_id not in self._models:
                return False

            # Deactivate other models of same type
            model = self._models[model_id]
            for m in self._models.values():
                if m.model_type == model.model_type:
                    m.is_active = False

            model.is_active = True
            model.status = 'ACTIVE'

        self.get_logger().info(f"Activated model: {model_id}")
        return True

    def rollback_model(self, model_id: str) -> bool:
        """Rollback a model to its previous version.

        Args:
            model_id: Model to roll back.

        Returns:
            True if rolled back successfully, False if no history available.
        """
        with self._models_lock:
            history = self._model_history.get(model_id, [])
            if not history:
                self.get_logger().warn(
                    f"Cannot rollback {model_id}: no version history"
                )
                return False

            previous = history.pop()
            was_active = False
            if model_id in self._models:
                was_active = self._models[model_id].is_active

            self._models[model_id] = previous
            if was_active:
                previous.is_active = True
                previous.status = 'ACTIVE'

        self.get_logger().info(
            f"Rolled back model {model_id} to v{previous.version}"
        )
        return True

    def compare_models(self, model_id_a: str, model_id_b: str) -> Dict[str, Any]:
        """Compare two models by version and accuracy.

        Args:
            model_id_a: First model identifier.
            model_id_b: Second model identifier.

        Returns:
            Dictionary with comparison results including version_comparison,
            accuracy_delta, and recommendation.
        """
        with self._models_lock:
            model_a = self._models.get(model_id_a)
            model_b = self._models.get(model_id_b)

            if model_a is None or model_b is None:
                missing = []
                if model_a is None:
                    missing.append(model_id_a)
                if model_b is None:
                    missing.append(model_id_b)
                return {
                    'error': f"Model(s) not found: {', '.join(missing)}",
                    'comparable': False,
                }

            version_cmp = compare_versions(model_a.version, model_b.version)
            accuracy_delta = model_a.accuracy - model_b.accuracy

            if version_cmp > 0:
                version_label = f"{model_id_a} is newer"
            elif version_cmp < 0:
                version_label = f"{model_id_b} is newer"
            else:
                version_label = "same version"

            # Recommendation: prefer newer version unless accuracy is worse
            if accuracy_delta > 0:
                recommendation = f"Use {model_id_a} (higher accuracy)"
            elif accuracy_delta < 0:
                recommendation = f"Use {model_id_b} (higher accuracy)"
            else:
                if version_cmp >= 0:
                    recommendation = f"Use {model_id_a} (newer or same version)"
                else:
                    recommendation = f"Use {model_id_b} (newer version)"

            return {
                'comparable': True,
                'model_a': {
                    'id': model_id_a,
                    'version': model_a.version,
                    'accuracy': model_a.accuracy,
                    'type': model_a.model_type,
                },
                'model_b': {
                    'id': model_id_b,
                    'version': model_b.version,
                    'accuracy': model_b.accuracy,
                    'type': model_b.model_type,
                },
                'version_comparison': version_label,
                'accuracy_delta': accuracy_delta,
                'recommendation': recommendation,
            }


def main(args=None):
    """Entry point for the model manager node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = ModelManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
