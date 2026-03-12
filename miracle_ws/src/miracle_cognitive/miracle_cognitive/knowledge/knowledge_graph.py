"""
Knowledge Graph Node.

Maintains a manufacturing knowledge graph using RDF triples.
Supports SPARQL queries, real-time updates from sensor data,
and JSONL-based persistence for surviving restarts.
"""

import json
import os
import tempfile
import time
from typing import Any, Dict, List, Optional, Tuple
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import KnowledgeUpdate
from miracle_msgs.srv import SPARQLQuery

_PERSIST_FORMAT_VERSION = 1


class KnowledgeGraphNode(MiracleLifecycleNode):
    """In-memory knowledge graph with SPARQL-like query support.

    Parameters:
        max_triples (int): Maximum stored triples.
        persist_path (str): Path for JSONL persistence file.
        autosave_interval_sec (float): Seconds between auto-saves (0 to disable).

    Subscribed Topics:
        ~/knowledge_updates (KnowledgeUpdate): Knowledge graph updates.

    Service Servers:
        ~/sparql_query (SPARQLQuery): Query the knowledge graph.

    Published Topics:
        ~/knowledge_events (KnowledgeUpdate): Graph change events.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__('knowledge_graph', criticality=self.CRITICALITY_HIGH, **kwargs)
        self._triples: List[Tuple[str, str, str, float, float]] = []  # (s, p, o, confidence, ts)
        self._index_spo: Dict[str, List[int]] = {}
        self._lock = threading.Lock()
        self._update_sub = None
        self._query_srv = None
        self._event_pub = None
        self._persist_path: str = ''
        self._autosave_timer = None

    def _do_configure(self) -> TransitionCallbackReturn:
        self.declare_and_validate_parameters({
            'max_triples': {'default': 100000, 'type': int, 'range': (1000, 10000000)},
            'persist_path': {'default': '/tmp/miracle_kg', 'type': str},
            'autosave_interval_sec': {'default': 60.0, 'type': float, 'range': (0.0, 86400.0)},
        })

        self._persist_path = self.get_parameter('persist_path').value
        autosave_sec = self.get_parameter('autosave_interval_sec').value

        # Load persisted triples from disk if file exists
        self._load_from_disk()

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

        # Set up auto-save timer
        if autosave_sec > 0:
            self._autosave_timer = self.create_timer(
                autosave_sec, self._autosave_callback,
            )

        self.get_logger().info("Knowledge graph configured")
        return TransitionCallbackReturn.SUCCESS

    def _autosave_callback(self) -> None:
        """Timer callback for periodic persistence."""
        self._save_to_disk()

    def _do_activate(self) -> TransitionCallbackReturn:
        self._load_base_ontology()
        self.get_logger().info("Knowledge graph activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        self._save_to_disk()
        self.get_logger().info("Knowledge graph saved on deactivate")
        return TransitionCallbackReturn.SUCCESS

    def _do_cleanup(self) -> TransitionCallbackReturn:
        self._save_to_disk()
        if self._autosave_timer is not None:
            self._autosave_timer.cancel()
            self._autosave_timer = None
        self.get_logger().info("Knowledge graph saved on cleanup")
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ #
    #  Persistence: save / load
    # ------------------------------------------------------------------ #

    def _save_to_disk(self) -> None:
        """Atomically write all triples to *persist_path* as JSONL.

        The file begins with a version header line, followed by one JSON
        object per triple.  Writing goes to a temporary file first and is
        then renamed into place so a crash mid-write never corrupts the
        persisted data.
        """
        path = self._persist_path
        if not path:
            return

        with self._lock:
            triples_snapshot = list(self._triples)

        try:
            parent_dir = os.path.dirname(path) or '.'
            os.makedirs(parent_dir, exist_ok=True)

            fd, tmp_path = tempfile.mkstemp(
                dir=parent_dir, prefix='.kg_', suffix='.tmp',
            )
            try:
                with os.fdopen(fd, 'w') as fh:
                    header = {
                        '_version': _PERSIST_FORMAT_VERSION,
                        '_saved_at': time.time(),
                        '_triple_count': len(triples_snapshot),
                    }
                    fh.write(json.dumps(header) + '\n')
                    for s, p, o, c, ts in triples_snapshot:
                        line = {
                            's': s, 'p': p, 'o': o,
                            'c': c, 'ts': ts,
                        }
                        fh.write(json.dumps(line) + '\n')
                os.replace(tmp_path, path)
            except BaseException:
                # Clean up temp file on any failure
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass
                raise
        except Exception as exc:
            self.get_logger().warning(f"Failed to persist knowledge graph: {exc}")

    def _load_from_disk(self) -> None:
        """Load triples from the JSONL persist file, if it exists.

        Corrupt lines are skipped with a warning.  Duplicate triples
        (same s, p, o) already present in-memory are not added again.
        """
        path = self._persist_path
        if not path or not os.path.isfile(path):
            return

        loaded = 0
        skipped = 0
        try:
            with open(path, 'r') as fh:
                for line_no, raw_line in enumerate(fh, start=1):
                    raw_line = raw_line.strip()
                    if not raw_line:
                        continue
                    try:
                        obj = json.loads(raw_line)
                    except json.JSONDecodeError as je:
                        self.get_logger().warning(
                            f"Skipping corrupt line {line_no} in {path}: {je}"
                        )
                        skipped += 1
                        continue

                    # Skip version header
                    if '_version' in obj:
                        continue

                    s = obj.get('s', '')
                    p = obj.get('p', '')
                    o = obj.get('o', '')
                    c = float(obj.get('c', 1.0))
                    ts = float(obj.get('ts', 0.0))

                    # Duplicate check
                    duplicate = False
                    for es, ep, eo, _ec, _ets in self._triples:
                        if es == s and ep == p and eo == o:
                            duplicate = True
                            break
                    if duplicate:
                        skipped += 1
                        continue

                    idx = len(self._triples)
                    self._triples.append((s, p, o, c, ts))
                    for key in (s, p, o):
                        if key not in self._index_spo:
                            self._index_spo[key] = []
                        self._index_spo[key].append(idx)
                    loaded += 1

        except Exception as exc:
            self.get_logger().warning(
                f"Error loading knowledge graph from {path}: {exc}"
            )

        if loaded or skipped:
            self.get_logger().info(
                f"Loaded {loaded} triples from disk ({skipped} skipped)"
            )

    # ------------------------------------------------------------------ #
    #  Import / Export
    # ------------------------------------------------------------------ #

    def export_triples(self, format: str = 'jsonl') -> str:
        """Export all triples as a JSONL string for external consumption.

        Args:
            format: Export format (currently only ``'jsonl'`` is supported).

        Returns:
            A string containing one JSON object per line.
        """
        if format != 'jsonl':
            raise ValueError(f"Unsupported export format: {format}")

        with self._lock:
            triples_snapshot = list(self._triples)

        lines: List[str] = []
        header = {
            '_version': _PERSIST_FORMAT_VERSION,
            '_saved_at': time.time(),
            '_triple_count': len(triples_snapshot),
        }
        lines.append(json.dumps(header))
        for s, p, o, c, ts in triples_snapshot:
            lines.append(json.dumps({'s': s, 'p': p, 'o': o, 'c': c, 'ts': ts}))
        return '\n'.join(lines) + '\n'

    def import_triples(self, data: str, format: str = 'jsonl') -> int:
        """Bulk-load triples from a JSONL string.

        Args:
            data: JSONL-formatted string (one JSON object per line).
            format: Import format (currently only ``'jsonl'`` is supported).

        Returns:
            Number of triples actually imported (duplicates are skipped).
        """
        if format != 'jsonl':
            raise ValueError(f"Unsupported import format: {format}")

        imported = 0
        for line_no, raw_line in enumerate(data.splitlines(), start=1):
            raw_line = raw_line.strip()
            if not raw_line:
                continue
            try:
                obj = json.loads(raw_line)
            except json.JSONDecodeError:
                self.get_logger().warning(
                    f"import_triples: skipping corrupt line {line_no}"
                )
                continue

            if '_version' in obj:
                continue

            s = obj.get('s', '')
            p = obj.get('p', '')
            o = obj.get('o', '')
            c = float(obj.get('c', 1.0))
            ts = float(obj.get('ts', time.time()))

            # Duplicate check
            with self._lock:
                duplicate = any(
                    es == s and ep == p and eo == o
                    for es, ep, eo, _ec, _ets in self._triples
                )
            if duplicate:
                continue

            self.add_triple(s, p, o, confidence=c, timestamp=ts)
            imported += 1

        return imported

    # ------------------------------------------------------------------ #
    #  Core triple operations
    # ------------------------------------------------------------------ #

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

    def add_triple(
        self,
        subject: str,
        predicate: str,
        obj: str,
        confidence: float = 1.0,
        timestamp: Optional[float] = None,
    ) -> None:
        """Add a triple to the knowledge graph."""
        if timestamp is None:
            timestamp = time.time()
        with self._lock:
            max_t = self.get_parameter('max_triples').value
            if len(self._triples) >= max_t:
                self._triples.pop(0)

            idx = len(self._triples)
            self._triples.append((subject, predicate, obj, confidence, timestamp))

            for key in (subject, predicate, obj):
                if key not in self._index_spo:
                    self._index_spo[key] = []
                self._index_spo[key].append(idx)

    def query(self, subject: str = '', predicate: str = '', obj: str = '') -> List[Tuple[str, str, str, float]]:
        """Query triples matching pattern (empty string = wildcard)."""
        with self._lock:
            results = []
            for s, p, o, c, _ts in self._triples:
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
                (s, p, o, c, ts) for s, p, o, c, ts in self._triples
                if not (s == subject and p == predicate and o == obj)
            ]

    def _handle_query(
        self, request: SPARQLQuery.Request, response: SPARQLQuery.Response,
    ) -> SPARQLQuery.Response:
        """Handle SPARQL-like query."""
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
