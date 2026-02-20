"""Federated Learning Coordinator - orchestrates federated learning across machines."""

from typing import Any, Dict, List
import numpy as np
import threading
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import FederatedModel, ModelUpdate
from miracle_msgs.action import FederatedRound


class FederatedCoordinatorNode(MiracleLifecycleNode):
    """Coordinates federated learning across CNC machines.

    Parameters:
        min_participants (int): Minimum participants per round.
        aggregation_strategy (str): FedAvg, FedProx, etc.
    """
    def __init__(self, **kwargs: Any) -> None:
        super().__init__('federated_coordinator', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._round_number: int = 0
        self._client_updates: Dict[str, ModelUpdate] = {}
        self._global_weights: List[float] = []
        self._lock = threading.Lock()
        self._update_sub = None
        self._model_pub = None
        self._round_server = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'min_participants': {'default': 2, 'type': int, 'range': (1, 100)},
            'aggregation_strategy': {'default': 'FedAvg', 'type': str,
                                      'choices': ['FedAvg', 'FedProx']},
        })
        self._update_sub = self.create_subscription(
            ModelUpdate, '/miracle/cognitive/model_updates',
            self._on_client_update, QoSProfiles.bulk_data(),
        )
        self._model_pub = self.create_publisher(
            FederatedModel, '/miracle/cognitive/global_model', QoSProfiles.bulk_data(),
        )
        self._round_server = ActionServer(
            self, FederatedRound, 'federated_round',
            execute_callback=self._execute_round,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Federated coordinator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Federated coordinator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_client_update(self, msg: ModelUpdate) -> None:
        with self._lock:
            self._client_updates[msg.client_id] = msg

    def _aggregate_fedavg(self) -> List[float]:
        """Federated averaging."""
        with self._lock:
            if not self._client_updates:
                return self._global_weights
            all_weights = [list(u.local_weights) for u in self._client_updates.values()]
            samples = [u.num_samples for u in self._client_updates.values()]
        total = sum(samples) or 1
        weight_arrays = [np.array(w) for w in all_weights if len(w) > 0]
        if not weight_arrays:
            return self._global_weights
        max_len = max(len(w) for w in weight_arrays)
        padded = [np.pad(w, (0, max_len - len(w))) for w in weight_arrays]
        averaged = sum(w * (s / total) for w, s in zip(padded, samples))
        return averaged.tolist()

    async def _execute_round(self, goal_handle):
        """Execute one federated learning round."""
        import asyncio
        request = goal_handle.request
        self._round_number = request.round_number
        result = FederatedRound.Result()

        with self._lock:
            self._client_updates.clear()

        # Wait for updates
        timeout = request.timeout_sec
        elapsed = 0.0
        min_p = request.min_participants
        while elapsed < timeout:
            await asyncio.sleep(1.0)
            elapsed += 1.0
            with self._lock:
                received = len(self._client_updates)
            feedback = FederatedRound.Feedback()
            feedback.updates_received = received
            feedback.updates_expected = min_p
            feedback.elapsed_sec = elapsed
            feedback.current_phase = 'COLLECTING'
            goal_handle.publish_feedback(feedback)
            if received >= min_p:
                break

        self._global_weights = self._aggregate_fedavg()
        model_msg = FederatedModel()
        model_msg.timestamp = self.get_clock().now().to_msg()
        model_msg.model_id = request.model_id
        model_msg.round_number = self._round_number
        model_msg.global_weights = self._global_weights
        with self._lock:
            model_msg.num_participants = len(self._client_updates)
        self._model_pub.publish(model_msg)

        goal_handle.succeed()
        result.success = True
        result.num_participants = model_msg.num_participants
        return result


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = FederatedCoordinatorNode()
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
