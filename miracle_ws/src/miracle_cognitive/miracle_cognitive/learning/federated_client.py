"""Federated Client - local training client for federated learning."""

from typing import Any
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FederatedModel, ModelUpdate


class FederatedClientNode(MiracleLifecycleNode):
    """Local federated learning client."""
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('federated_client', criticality=self.CRITICALITY_LOW, **kwargs)
        self._model_sub = None
        self._update_pub = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'client_id': {'default': 'cnc1', 'type': str},
            'local_epochs': {'default': 5, 'type': int, 'range': (1, 100)},
        })
        self._model_sub = self.create_subscription(
            FederatedModel, '/miracle/cognitive/global_model',
            self._on_global_model, QoSProfiles.bulk_data(),
        )
        self._update_pub = self.create_publisher(
            ModelUpdate, '/miracle/cognitive/model_updates', QoSProfiles.bulk_data(),
        )
        self.get_logger().info("Federated client configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_global_model(self, msg: FederatedModel) -> None:
        """Receive global model and train locally."""
        import numpy as np
        client_id = self.get_parameter('client_id').value
        weights = np.array(msg.global_weights) if msg.global_weights else np.zeros(10)
        # Simulate local training
        local_weights = weights + np.random.normal(0, 0.01, len(weights))
        update = ModelUpdate()
        update.timestamp = self.get_clock().now().to_msg()
        update.model_id = msg.model_id
        update.client_id = client_id
        update.round_number = msg.round_number
        update.local_weights = local_weights.tolist()
        update.local_loss = float(np.random.uniform(0.01, 0.1))
        update.num_samples = 100
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
