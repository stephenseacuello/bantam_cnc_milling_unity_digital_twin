"""Auction Manager Node - manages multi-agent auction protocol for task allocation."""

from typing import Any, Dict, List
import threading
from rclpy.lifecycle import TransitionCallbackReturn
from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import TaskAnnouncement, AgentBid, TaskAward


class AuctionManagerNode(MiracleLifecycleNode):
    """Manages Contract Net Protocol auctions.

    Parameters:
        auction_duration_sec (float): Duration of bidding window.
        min_bidders (int): Minimum bidders before resolving.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('auction_manager', criticality=self.CRITICALITY_MEDIUM, **kwargs)
        self._auctions: Dict[str, Dict] = {}
        self._lock = threading.Lock()
        self._announcement_pub = None
        self._bid_sub = None
        self._award_pub = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'auction_duration_sec': {'default': 10.0, 'type': float, 'range': (1.0, 120.0)},
            'min_bidders': {'default': 1, 'type': int, 'range': (1, 50)},
        })
        self._announcement_pub = self.create_publisher(
            TaskAnnouncement, '/miracle/cognitive/task_announcements', QoSProfiles.command(),
        )
        self._bid_sub = self.create_subscription(
            AgentBid, '/miracle/cognitive/bids', self._on_bid, QoSProfiles.command(),
        )
        self._award_pub = self.create_publisher(
            TaskAward, '/miracle/cognitive/task_awards', QoSProfiles.command(),
        )
        self.get_logger().info("Auction manager configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self.get_logger().info("Auction manager activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _on_bid(self, msg: AgentBid) -> None:
        with self._lock:
            if msg.auction_id in self._auctions:
                self._auctions[msg.auction_id].setdefault('bids', []).append(msg)

    def start_auction(self, announcement: TaskAnnouncement) -> str:
        with self._lock:
            self._auctions[announcement.auction_id] = {
                'announcement': announcement, 'bids': [],
            }
        self._announcement_pub.publish(announcement)
        duration = self.get_parameter('auction_duration_sec').value
        self.create_timer(
            duration, lambda: self._close_auction(announcement.auction_id),
            callback_group=self.service_callback_group,
        )
        return announcement.auction_id

    def _close_auction(self, auction_id: str) -> None:
        with self._lock:
            auction = self._auctions.pop(auction_id, None)
        if not auction or not auction.get('bids'):
            return
        winner = min(auction['bids'], key=lambda b: b.proposed_cost)
        award = TaskAward()
        award.timestamp = self.get_clock().now().to_msg()
        award.auction_id = auction_id
        award.awarded_agent_id = winner.agent_id
        award.agreed_cost = winner.proposed_cost
        award.agreed_completion_time = winner.proposed_completion_time
        self._award_pub.publish(award)


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = AuctionManagerNode()
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
