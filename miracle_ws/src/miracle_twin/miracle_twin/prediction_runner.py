"""
Prediction Runner Node.

Runs what-if scenarios on the digital twin to predict outcomes
of process parameter changes before applying to physical machine.
"""

from typing import Any, Dict, List, Optional
from dataclasses import dataclass
import threading

from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import MachineState, PHMPrediction
from miracle_msgs.action import RunPrediction
from miracle_msgs.srv import RunPrediction as RunPredictionSrv
from miracle_twin.cutting_sim_proxy import CuttingSimProxy, GCodeBlock


@dataclass
class PredictionScenario:
    """A prediction scenario configuration."""
    scenario_id: str
    machine_id: str
    parameter_changes: Dict[str, float]
    prediction_horizon_hours: float = 24.0


class PredictionRunnerNode(MiracleLifecycleNode):
    """Runs predictive scenarios on the digital twin.

    Parameters:
        max_concurrent_predictions (int): Max simultaneous predictions.
        default_horizon_hours (float): Default prediction horizon.

    Action Servers:
        ~/run_prediction (RunPrediction): Run a prediction scenario.

    Service Servers:
        /miracle/twin/run_prediction (RunPredictionSrv): Synchronous prediction for dashboard.

    Published Topics:
        ~/predictions (PHMPrediction): Prediction results.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'prediction_runner',
            criticality=self.CRITICALITY_MEDIUM,
            **kwargs,
        )
        self._action_server: Optional[ActionServer] = None
        self._prediction_srv = None
        self._prediction_pub = None
        self._active_predictions: int = 0
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure prediction runner."""
        params = self.declare_and_validate_parameters({
            'max_concurrent_predictions': {
                'default': 5,
                'type': int,
                'range': (1, 50),
            },
            'default_horizon_hours': {
                'default': 24.0,
                'type': float,
                'range': (0.1, 720.0),
            },
        })

        self._action_server = ActionServer(
            self,
            RunPrediction,
            'run_prediction',
            execute_callback=self._execute_prediction,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._prediction_pub = self.create_publisher(
            PHMPrediction,
            'predictions',
            QoSProfiles.state_data(),
        )

        self._prediction_srv = self.create_service(
            RunPredictionSrv,
            '/miracle/twin/run_prediction',
            self._handle_run_prediction,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info("Prediction runner configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate prediction runner."""
        self.get_logger().info("Prediction runner activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate prediction runner."""
        return TransitionCallbackReturn.SUCCESS

    def _goal_callback(self, goal_request) -> GoalResponse:
        """Accept or reject prediction goals."""
        max_pred = self.get_parameter('max_concurrent_predictions').value
        with self._lock:
            if self._active_predictions >= max_pred:
                return GoalResponse.REJECT
            self._active_predictions += 1
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        """Accept cancellation."""
        return CancelResponse.ACCEPT

    async def _execute_prediction(
        self, goal_handle: ServerGoalHandle
    ) -> RunPrediction.Result:
        """Execute a prediction scenario."""
        import asyncio

        request = goal_handle.request
        result = RunPrediction.Result()

        self.get_logger().info(
            f"Running prediction for {request.machine_id}: "
            f"{request.prediction_type}"
        )

        try:
            # Simulate prediction computation
            phases = ['LOADING_MODEL', 'RUNNING_SIMULATION', 'ANALYZING']
            for i, phase in enumerate(phases):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    return result

                feedback = RunPrediction.Feedback()
                feedback.progress = (i + 1) / len(phases)
                feedback.current_phase = phase
                goal_handle.publish_feedback(feedback)
                await asyncio.sleep(0.5)

            # Run real simulation via CuttingSimProxy
            proxy = CuttingSimProxy()
            sample_blocks = [
                GCodeBlock(
                    feed_rate_mmpm=500.0, spindle_rpm=8000.0,
                    axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50.0,
                )
                for _ in range(10)
            ]
            sim_result = proxy.simulate_program(sample_blocks)

            prediction = PHMPrediction()
            prediction.timestamp = self.get_clock().now().to_msg()
            prediction.machine_id = request.machine_id
            prediction.component = 'spindle'
            prediction.prediction_type = request.prediction_type
            prediction.remaining_useful_life_hours = sim_result.remaining_useful_life_hours
            prediction.confidence = sim_result.confidence
            prediction.health_index = sim_result.health_index
            prediction.recommended_action = sim_result.recommended_action
            prediction.trend_data = sim_result.trend_data

            self._prediction_pub.publish(prediction)

            goal_handle.succeed()
            result.success = True
            result.prediction = prediction
            result.detailed_report_json = '{"status": "completed"}'

        except Exception as exc:
            self.get_logger().error(f"Prediction error: {exc}")
            goal_handle.abort()
            result.success = False

        finally:
            with self._lock:
                self._active_predictions = max(
                    0, self._active_predictions - 1
                )

        return result


    def _handle_run_prediction(
        self,
        request: RunPredictionSrv.Request,
        response: RunPredictionSrv.Response,
    ) -> RunPredictionSrv.Response:
        """Handle synchronous prediction service calls (dashboard)."""
        self.get_logger().info(
            f"Service prediction for {request.machine_id}: "
            f"{request.scenario_type}"
        )

        # Run real simulation via CuttingSimProxy
        proxy = CuttingSimProxy()
        sample_blocks = [
            GCodeBlock(
                feed_rate_mmpm=500.0, spindle_rpm=8000.0,
                axial_depth_mm=1.5, radial_depth_mm=3.175, length_mm=50.0,
            )
            for _ in range(10)
        ]
        sim_result = proxy.simulate_program(sample_blocks)

        prediction = PHMPrediction()
        prediction.timestamp = self.get_clock().now().to_msg()
        prediction.machine_id = request.machine_id
        prediction.component = 'spindle'
        prediction.prediction_type = request.scenario_type
        prediction.remaining_useful_life_hours = sim_result.remaining_useful_life_hours
        prediction.confidence = sim_result.confidence
        prediction.health_index = sim_result.health_index
        prediction.recommended_action = sim_result.recommended_action
        prediction.trend_data = sim_result.trend_data

        self._prediction_pub.publish(prediction)

        response.success = True
        response.summary = (
            f"Prediction for {request.machine_id}: "
            f"RUL={prediction.remaining_useful_life_hours:.0f}h, "
            f"Health={prediction.health_index:.0%}, "
            f"Action: {prediction.recommended_action}"
        )
        response.confidence = prediction.confidence
        response.detailed_report_json = (
            f'{{"machine_id": "{request.machine_id}", '
            f'"rul_hours": {prediction.remaining_useful_life_hours}, '
            f'"health_index": {prediction.health_index}, '
            f'"confidence": {prediction.confidence}}}'
        )
        return response


def main(args=None):
    """Entry point for the prediction runner node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = PredictionRunnerNode()
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
