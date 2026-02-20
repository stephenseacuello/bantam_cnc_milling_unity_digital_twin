"""
Reasoning Engine Node.

Performs logical reasoning over the knowledge graph.
Supports forward chaining, backward chaining, and rule evaluation.
"""

from typing import Any, Dict, List, Tuple
from dataclasses import dataclass
import threading

from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import KnowledgeUpdate


@dataclass
class Rule:
    """An inference rule."""
    name: str
    conditions: List[Tuple[str, str, str]]  # List of (s, p, o) patterns
    conclusion: Tuple[str, str, str]
    confidence: float = 0.9


class ReasoningEngineNode(MiracleLifecycleNode):
    """Logical reasoning over the knowledge graph.

    Parameters:
        max_inference_depth (int): Maximum chaining depth.
        reasoning_interval_sec (float): Periodic reasoning cycle.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('reasoning_engine', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._rules: List[Rule] = []
        self._facts: List[Tuple[str, str, str]] = []
        self._inferred: List[Tuple[str, str, str, float]] = []
        self._lock = threading.Lock()
        self._update_sub = None
        self._inference_pub = None
        self._reason_timer = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_inference_depth': {'default': 10, 'type': int, 'range': (1, 100)},
            'reasoning_interval_sec': {'default': 5.0, 'type': float, 'range': (1.0, 60.0)},
        })

        self._update_sub = self.create_subscription(
            KnowledgeUpdate, '/miracle/cognitive/knowledge_events',
            self._on_knowledge_update, QoSProfiles.state_data(),
        )
        self._inference_pub = self.create_publisher(
            KnowledgeUpdate, 'inferences', QoSProfiles.state_data(),
        )

        self._load_manufacturing_rules()
        self.get_logger().info("Reasoning engine configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        interval = self.get_parameter('reasoning_interval_sec').value
        self._reason_timer = self.create_timer(
            interval, self._run_inference, callback_group=self.service_callback_group,
        )
        self.get_logger().info("Reasoning engine activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        if self._reason_timer is not None:
            self._reason_timer.cancel()
            self._reason_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _load_manufacturing_rules(self) -> None:
        """Load manufacturing domain rules."""
        self._rules = [
            Rule(
                name='tool_wear_causes_quality_drop',
                conditions=[('?tool', 'hasWearLevel', 'HIGH')],
                conclusion=('?tool', 'causesRisk', 'QualityDrop'),
            ),
            Rule(
                name='high_vibration_indicates_chatter',
                conditions=[('?machine', 'hasVibrationLevel', 'HIGH')],
                conclusion=('?machine', 'mayHave', 'Chatter'),
            ),
            Rule(
                name='overload_requires_feedrate_reduction',
                conditions=[('?machine', 'hasSpindleLoad', 'OVERLOAD')],
                conclusion=('?machine', 'requires', 'FeedRateReduction'),
            ),
        ]

    def _on_knowledge_update(self, msg: KnowledgeUpdate) -> None:
        """Receive knowledge graph updates as facts."""
        with self._lock:
            self._facts.append((msg.subject, msg.predicate, msg.object_value))

    def _run_inference(self) -> None:
        """Run forward-chaining inference."""
        with self._lock:
            new_inferences = []
            for rule in self._rules:
                bindings = self._match_rule(rule)
                for binding in bindings:
                    s = self._apply_binding(rule.conclusion[0], binding)
                    p = self._apply_binding(rule.conclusion[1], binding)
                    o = self._apply_binding(rule.conclusion[2], binding)
                    if (s, p, o) not in [(i[0], i[1], i[2]) for i in self._inferred]:
                        new_inferences.append((s, p, o, rule.confidence))
                        self._inferred.append((s, p, o, rule.confidence))

            for s, p, o, c in new_inferences:
                msg = KnowledgeUpdate()
                msg.timestamp = self.get_clock().now().to_msg()
                msg.update_type = 'INFERRED'
                msg.subject = s
                msg.predicate = p
                msg.object_value = o
                msg.confidence = c
                msg.source = 'reasoning_engine'
                msg.reasoning = 'forward_chaining'
                self._inference_pub.publish(msg)

    def _match_rule(self, rule: Rule) -> List[Dict[str, str]]:
        """Find variable bindings that satisfy rule conditions."""
        bindings_list: List[Dict[str, str]] = [{}]
        for pattern in rule.conditions:
            new_bindings = []
            for binding in bindings_list:
                matches = self._match_pattern(pattern, binding)
                new_bindings.extend(matches)
            bindings_list = new_bindings
        return bindings_list

    def _match_pattern(
        self, pattern: Tuple[str, str, str], binding: Dict[str, str]
    ) -> List[Dict[str, str]]:
        """Match a pattern against facts with variable bindings."""
        results = []
        s_pat, p_pat, o_pat = pattern
        for s, p, o in self._facts:
            new_binding = dict(binding)
            if self._matches(s_pat, s, new_binding) and \
               self._matches(p_pat, p, new_binding) and \
               self._matches(o_pat, o, new_binding):
                results.append(new_binding)
        return results

    @staticmethod
    def _matches(pattern: str, value: str, binding: Dict[str, str]) -> bool:
        """Check if pattern matches value, updating bindings."""
        if pattern.startswith('?'):
            if pattern in binding:
                return binding[pattern] == value
            binding[pattern] = value
            return True
        return pattern == value

    @staticmethod
    def _apply_binding(template: str, binding: Dict[str, str]) -> str:
        """Apply variable bindings to a template."""
        if template.startswith('?') and template in binding:
            return binding[template]
        return template


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = ReasoningEngineNode()
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
