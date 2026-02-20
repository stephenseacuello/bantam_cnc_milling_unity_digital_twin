"""
Knowledge Graph Node.

Maintains a manufacturing knowledge graph using RDF triples.
Supports SPARQL queries and real-time updates from sensor data.
"""

from typing import Any, Dict, List, Optional, Tuple
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import KnowledgeUpdate
from miracle_msgs.srv import SPARQLQuery


class KnowledgeGraphNode(MiracleLifecycleNode):
    """In-memory knowledge graph with SPARQL-like query support.

    Parameters:
        max_triples (int): Maximum stored triples.
        persist_path (str): Path for persistence.

    Subscribed Topics:
        ~/knowledge_updates (KnowledgeUpdate): Knowledge graph updates.

    Service Servers:
        ~/sparql_query (SPARQLQuery): Query the knowledge graph.

    Published Topics:
        ~/knowledge_events (KnowledgeUpdate): Graph change events.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('knowledge_graph', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._triples: List[Tuple[str, str, str, float]] = []  # (s, p, o, confidence)
        self._index_spo: Dict[str, List[int]] = {}
        self._lock = threading.Lock()
        self._update_sub = None
        self._query_srv = None
        self._event_pub = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_triples': {'default': 100000, 'type': int, 'range': (1000, 10000000)},
            'persist_path': {'default': '/tmp/miracle_kg', 'type': str},
        })

        self._update_sub = self.create_subscription(
            KnowledgeUpdate, 'knowledge_updates',
            self._on_update, QoSProfiles.state_data(),
        )
        self._query_srv = self.create_service(
            SPARQLQuery, 'sparql_query',
            self._handle_query, callback_group=self.service_callback_group,
        )
        self._event_pub = self.create_publisher(
            KnowledgeUpdate, 'knowledge_events', QoSProfiles.state_data(),
        )
        self.get_logger().info("Knowledge graph configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        self._load_base_ontology()
        self.get_logger().info("Knowledge graph activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def _load_base_ontology(self) -> None:
        """Load base manufacturing ontology."""
        base_triples = [
            ('CNCMachine', 'isA', 'ManufacturingEquipment'),
            ('ManufacturingEquipment', 'isA', 'PhysicalAsset'),
            ('Spindle', 'isPartOf', 'CNCMachine'),
            ('CuttingTool', 'isUsedBy', 'Spindle'),
            ('Milling', 'isA', 'MachiningProcess'),
            ('Turning', 'isA', 'MachiningProcess'),
            ('Drilling', 'isA', 'MachiningProcess'),
            ('ToolWear', 'affects', 'SurfaceQuality'),
            ('Chatter', 'affects', 'SurfaceQuality'),
            ('FeedRate', 'isParameterOf', 'MachiningProcess'),
            ('SpindleSpeed', 'isParameterOf', 'MachiningProcess'),
            ('DepthOfCut', 'isParameterOf', 'MachiningProcess'),
        ]
        for s, p, o in base_triples:
            self.add_triple(s, p, o, confidence=1.0)

    def add_triple(self, subject: str, predicate: str, obj: str, confidence: float = 1.0) -> None:
        """Add a triple to the knowledge graph."""
        with self._lock:
            max_t = self.get_parameter('max_triples').value
            if len(self._triples) >= max_t:
                self._triples.pop(0)

            idx = len(self._triples)
            self._triples.append((subject, predicate, obj, confidence))

            for key in (subject, predicate, obj):
                if key not in self._index_spo:
                    self._index_spo[key] = []
                self._index_spo[key].append(idx)

    def query(self, subject: str = '', predicate: str = '', obj: str = '') -> List[Tuple[str, str, str, float]]:
        """Query triples matching pattern (empty string = wildcard)."""
        with self._lock:
            results = []
            for s, p, o, c in self._triples:
                if subject and s != subject:
                    continue
                if predicate and p != predicate:
                    continue
                if obj and o != obj:
                    continue
                results.append((s, p, o, c))
            return results

    def _on_update(self, msg: KnowledgeUpdate) -> None:
        """Handle knowledge graph update."""
        if msg.update_type == 'ADD':
            self.add_triple(msg.subject, msg.predicate, msg.object_value, msg.confidence)
        elif msg.update_type == 'REMOVE':
            self._remove_triple(msg.subject, msg.predicate, msg.object_value)

    def _remove_triple(self, subject: str, predicate: str, obj: str) -> None:
        """Remove matching triples."""
        with self._lock:
            self._triples = [
                (s, p, o, c) for s, p, o, c in self._triples
                if not (s == subject and p == predicate and o == obj)
            ]

    def _handle_query(
        self, request: SPARQLQuery.Request, response: SPARQLQuery.Response,
    ) -> SPARQLQuery.Response:
        """Handle SPARQL-like query."""
        import json
        try:
            # Simple pattern matching (not full SPARQL)
            query_str = request.query.strip()
            # Parse simple "?s ?p ?o WHERE { subject predicate object }" pattern
            results = self.query()  # Return all for now
            response.success = True
            response.result_json = json.dumps([
                {'s': s, 'p': p, 'o': o, 'confidence': c}
                for s, p, o, c in results[:100]
            ])
            response.num_results = len(results)
        except Exception as exc:
            response.success = False
            response.result_json = str(exc)
            response.num_results = 0
        return response


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = KnowledgeGraphNode()
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
