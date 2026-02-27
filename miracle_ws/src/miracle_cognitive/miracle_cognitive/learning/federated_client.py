"""Federated Client - local training client for federated learning."""

import os
from typing import Any, Optional
import numpy as np
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FederatedModel, ModelUpdate


class FederatedClientNode(MiracleLifecycleNode):
    """Local federated learning client with actual gradient descent training."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('federated_client', criticality=self.CRITICALITY_LOW, **kwargs)
        self._model_sub = None
        self._update_pub = None
        self._local_data: Optional[np.ndarray] = None
        self._local_labels: Optional[np.ndarray] = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'client_id': {'default': 'cnc1', 'type': str},
            'local_epochs': {'default': 5, 'type': int, 'range': (1, 100)},
            'learning_rate': {'default': 0.01, 'type': float, 'range': (0.0001, 1.0)},
            'local_data_path': {'default': '', 'type': str},
        })
        self._model_sub = self.create_subscription(
            FederatedModel, '/miracle/cognitive/global_model',
            self._on_global_model, QoSProfiles.bulk_data(),
        )
        self._update_pub = self.create_publisher(
            ModelUpdate, '/miracle/cognitive/model_updates', QoSProfiles.bulk_data(),
        )
        # Attempt to load local training data
        self._load_local_data()
        self.get_logger().info("Federated client configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _load_local_data(self) -> None:
        """Load local training data from the configured path.

        Expected format: NumPy .npz file with 'features' and 'labels' arrays.
        Falls back to synthetic data if file is not available.
        """
        data_path = self.get_parameter('local_data_path').value
        if data_path and os.path.exists(data_path):
            try:
                loaded = np.load(data_path)
                self._local_data = loaded['features'].astype(np.float64)
                self._local_labels = loaded['labels'].astype(np.float64)
                self.get_logger().info(
                    f"Loaded local data: {self._local_data.shape[0]} samples "
                    f"from {data_path}"
                )
                return
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to load local data from {data_path}: {exc}. "
                    "Falling back to synthetic data."
                )

        # Generate synthetic local training data as fallback
        rng = np.random.default_rng(seed=hash(self.get_name()) & 0xFFFFFFFF)
        n_samples = 200
        n_features = 10
        self._local_data = rng.standard_normal((n_samples, n_features))
        # Synthetic labels: simple linear relationship with noise
        true_weights = rng.standard_normal(n_features)
        self._local_labels = (
            self._local_data @ true_weights
            + rng.standard_normal(n_samples) * 0.1
        )
        self.get_logger().info(
            f"Using synthetic local data: {n_samples} samples, "
            f"{n_features} features"
        )

    def _on_global_model(self, msg: FederatedModel) -> None:
        """Receive global model and perform local gradient descent training."""
        client_id = self.get_parameter('client_id').value
        local_epochs = self.get_parameter('local_epochs').value
        learning_rate = self.get_parameter('learning_rate').value

        weights = np.array(msg.global_weights, dtype=np.float64) \
            if msg.global_weights else np.zeros(10, dtype=np.float64)

        # Ensure data dimensions match model weights
        if self._local_data is None or self._local_labels is None:
            self._load_local_data()

        n_features = len(weights)
        data = self._local_data
        labels = self._local_labels

        # Resize data if feature count doesn't match
        if data.shape[1] != n_features:
            if data.shape[1] > n_features:
                data = data[:, :n_features]
            else:
                padding = np.zeros((data.shape[0], n_features - data.shape[1]))
                data = np.hstack([data, padding])

        # Local gradient descent training loop
        local_weights = weights.copy()
        total_loss = 0.0
        n_samples = data.shape[0]

        for epoch in range(local_epochs):
            # Forward pass: predictions = X @ w
            predictions = data @ local_weights

            # Compute residuals
            residuals = predictions - labels

            # MSE loss
            loss = float(np.mean(residuals ** 2))
            total_loss = loss

            # Gradient: dL/dw = (2/n) * X^T @ residuals
            gradients = (2.0 / n_samples) * (data.T @ residuals)

            # Gradient clipping to prevent explosion
            grad_norm = np.linalg.norm(gradients)
            max_grad_norm = 10.0
            if grad_norm > max_grad_norm:
                gradients = gradients * (max_grad_norm / grad_norm)

            # Weight update
            local_weights -= learning_rate * gradients

        self.get_logger().info(
            f"Local training complete: {local_epochs} epochs, "
            f"final_loss={total_loss:.6f}, samples={n_samples}"
        )

        # Publish updated weights
        update = ModelUpdate()
        update.timestamp = self.get_clock().now().to_msg()
        update.model_id = msg.model_id
        update.client_id = client_id
        update.round_number = msg.round_number
        update.local_weights = local_weights.tolist()
        update.local_loss = float(total_loss)
        update.num_samples = n_samples
        self._update_pub.publish(update)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = FederatedClientNode()
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
