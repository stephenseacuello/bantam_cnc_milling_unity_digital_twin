"""Task Allocator Node - distributes tasks to agents based on capabilities and load."""

from typing import Any, Dict, List
import threading
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import TaskAnnouncement, AgentBid, TaskAward


class TaskAllocatorNode(MiracleLifecycleNode):
    """Allocates tasks to agents using configurable strategies.

    Parameters:
        allocation_strategy (str): Strategy (auction, round_robin, least_loaded).
        bid_timeout_sec (float): Time to wait for bids.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('task_allocator', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._pending_auctions: Dict[str, List] = {}
        self._lock = threading.Lock()
        self._bid_sub = None
        self._award_pub = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'allocation_strategy': {'default': 'auction', 'type': str,
                                     'choices': ['auction', 'round_robin', 'least_loaded']},
            'bid_timeout_sec': {'default': 5.0, 'type': float, 'range': (1.0, 60.0)},
        })
        self._bid_sub = self.create_subscription(
            AgentBid, '/miracle/cognitive/bids', self._on_bid, QoSProfiles.command(),
        )
        self._award_pub = self.create_publisher(
            TaskAward, '/miracle/cognitive/task_awards', QoSProfiles.command(),
        )
        self.get_logger().info("Task allocator configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Task allocator activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_bid(self, msg: AgentBid) -> None:
        with self._lock:
            if msg.auction_id not in self._pending_auctions:
                self._pending_auctions[msg.auction_id] = []
            self._pending_auctions[msg.auction_id].append(msg)

    def resolve_auction(self, auction_id: str) -> None:
        with self._lock:
            bids = self._pending_auctions.pop(auction_id, [])
        if not bids:
            return
        # Select lowest cost bid
        winner = min(bids, key=lambda b: b.proposed_cost)
        award = TaskAward()
        award.timestamp = self.get_clock().now().to_msg()
        award.auction_id = auction_id
        award.awarded_agent_id = winner.agent_id
        award.agreed_cost = winner.proposed_cost
        award.agreed_completion_time = winner.proposed_completion_time
        self._award_pub.publish(award)
        self.get_logger().info(f"Task {auction_id} awarded to {winner.agent_id}")


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = TaskAllocatorNode()
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
