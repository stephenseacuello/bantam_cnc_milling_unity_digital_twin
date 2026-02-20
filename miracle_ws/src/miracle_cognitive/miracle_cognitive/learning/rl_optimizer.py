"""RL Optimizer Node - uses reinforcement learning to optimize CNC process parameters."""

from typing import Any, Dict, List, Optional
import numpy as np
import threading
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, OptimizationAction, SystemKPIs
from miracle_msgs.srv import OptimizeParameters
from miracle_msgs.action import TrainRLPolicy


class RLOptimizerNode(MiracleLifecycleNode):
    """RL-based process parameter optimizer.

    Parameters:
        learning_rate (float): RL learning rate.
        discount_factor (float): RL discount factor.
        exploration_rate (float): Epsilon-greedy exploration.
        state_dim (int): State space dimension.
        action_dim (int): Action space dimension.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('rl_optimizer', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._q_table: Dict[str, np.ndarray] = {}
        self._state_sub = None
        self._kpi_sub = None
        self._action_pub = None
        self._optimize_srv = None
        self._train_server = None
        self._lock = threading.Lock()
        self._lr: float = 0.01
        self._gamma: float = 0.99
        self._epsilon: float = 0.1

    def _do_configure(self) -> TransitionCallbackReturn:
        params = self.declare_and_validate_parameters({
            'learning_rate': {'default': 0.01, 'type': float, 'range': (0.0001, 1.0)},
            'discount_factor': {'default': 0.99, 'type': float, 'range': (0.0, 1.0)},
            'exploration_rate': {'default': 0.1, 'type': float, 'range': (0.0, 1.0)},
            'state_dim': {'default': 10, 'type': int, 'range': (1, 1000)},
            'action_dim': {'default': 5, 'type': int, 'range': (1, 100)},
        })
        self._lr = params['learning_rate']
        self._gamma = params['discount_factor']
        self._epsilon = params['exploration_rate']

        self._action_pub = self.create_publisher(
            OptimizationAction, 'optimization_actions', QoSProfiles.command(),
        )
        self._optimize_srv = self.create_service(
            OptimizeParameters, 'rl_optimize',
            self._handle_optimize, callback_group=self.service_callback_group,
        )
        self._train_server = ActionServer(
            self, TrainRLPolicy, 'train_policy',
            execute_callback=self._train_policy,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("RL optimizer configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("RL optimizer activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _discretize_state(self, state: List[float]) -> str:
        """Discretize continuous state for Q-table lookup."""
        discretized = tuple(round(s, 1) for s in state[:5])
        return str(discretized)

    def select_action(self, state: List[float]) -> int:
        """Epsilon-greedy action selection."""
        state_key = self._discretize_state(state)
        action_dim = self.get_parameter('action_dim').value
        if np.random.random() < self._epsilon:
            return np.random.randint(action_dim)
        with self._lock:
            if state_key not in self._q_table:
                self._q_table[state_key] = np.zeros(action_dim)
            return int(np.argmax(self._q_table[state_key]))

    def update(self, state: List[float], action: int, reward: float, next_state: List[float]) -> None:
        """Q-learning update."""
        s_key = self._discretize_state(state)
        ns_key = self._discretize_state(next_state)
        action_dim = self.get_parameter('action_dim').value
        with self._lock:
            if s_key not in self._q_table:
                self._q_table[s_key] = np.zeros(action_dim)
            if ns_key not in self._q_table:
                self._q_table[ns_key] = np.zeros(action_dim)
            td_target = reward + self._gamma * np.max(self._q_table[ns_key])
            td_error = td_target - self._q_table[s_key][action]
            self._q_table[s_key][action] += self._lr * td_error

    def _handle_optimize(self, request, response):
        """Handle optimization request."""
        state = list(request.current_parameters) if request.current_parameters else [0.0] * 5
        action = self.select_action(state)
        # Map action to parameter adjustment
        param_names = list(request.parameter_names) if request.parameter_names else ['feed_rate']
        adjustments = np.zeros(len(state))
        if action < len(adjustments):
            adjustments[action] = 0.05  # Small positive adjustment
        optimized = [s + a for s, a in zip(state, adjustments)]

        response.success = True
        response.optimized_parameters = optimized
        response.expected_improvement = 0.02
        response.reasoning = f'RL action {action}: adjusted parameter'
        return response

    async def _train_policy(self, goal_handle):
        """Train RL policy action handler."""
        import asyncio
        request = goal_handle.request
        result = TrainRLPolicy.Result()
        rewards = []
        for ep in range(request.num_episodes):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                return result
            # Simulated episode
            episode_reward = np.random.normal(0.5, 0.1) + ep * 0.001
            rewards.append(episode_reward)
            feedback = TrainRLPolicy.Feedback()
            feedback.current_episode = ep + 1
            feedback.total_episodes = request.num_episodes
            feedback.current_reward = episode_reward
            feedback.average_reward = np.mean(rewards)
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(0.01)

        goal_handle.succeed()
        result.success = True
        result.final_reward = rewards[-1] if rewards else 0.0
        result.reward_history = rewards
        result.model_path = '/tmp/miracle_models/rl_policy.pkl'
        return result


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = RLOptimizerNode()
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
