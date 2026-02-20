"""
ML Model Manager Node.

Manages ML model lifecycle: loading, versioning, A/B testing,
and hot-swapping models without downtime.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
import os
import json
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import ModelUpdate


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

            self._models[model_id] = ModelInfo(
                model_id=model_id,
                model_type=model_type,
                version=version,
                path=path,
            )

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
