"""
Prometheus Metrics Exporter Node.

Subscribes to MIRACLE ROS2 topics and exports key metrics via
an HTTP endpoint on :9100/metrics for Prometheus scraping.
"""

from typing import Any, Dict
from collections import defaultdict
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_core.serialization import msg_to_dict
from miracle_msgs.msg import MachineState, SystemKPIs, AnomalyAlert


class _MetricsHandler(BaseHTTPRequestHandler):
    """HTTP handler that serves Prometheus text exposition format."""

    metrics_ref = None  # Set by PrometheusExporterNode

    def do_GET(self):
        if self.path != '/metrics':
            self.send_response(404)
            self.end_headers()
            return
        body = self.metrics_ref() if self.metrics_ref else ''
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain; version=0.0.4; charset=utf-8')
        self.end_headers()
        self.wfile.write(body.encode('utf-8'))

    def log_message(self, format, *args):
        pass  # Suppress default access logs


class PrometheusExporterNode(MiracleLifecycleNode):
    """Exports MIRACLE metrics to Prometheus.

    Parameters:
        http_port (int): HTTP server port for /metrics endpoint.
        machine_ids (str): Comma-separated machine IDs.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'prometheus_exporter',
            criticality=self.CRITICALITY_LOW,
            **kwargs,
        )
        self._state_subs = []
        self._kpi_sub = None
        self._anomaly_subs = []
        self._http_server = None
        self._http_thread = None

        self._machine_states: Dict[str, Dict] = {}
        self._kpis: Dict[str, float] = {}
        self._anomaly_counts: Dict[str, int] = defaultdict(int)
        self._lock = threading.Lock()

    def _do_configure(self) -> TransitionCallbackReturn:
        params = self.declare_and_validate_parameters({
            'http_port': {'default': 9100, 'type': int, 'range': (1024, 65535)},
            'machine_ids': {'default': 'cnc1,cnc2,cnc3', 'type': str},
        })
        machine_ids = self.get_machine_ids(params)

        self._state_subs = self.create_multi_machine_subscriptions(
            MachineState, 'state', self._on_state, QoSProfiles.state_data(), machine_ids)

        self._anomaly_subs = self.create_multi_machine_subscriptions(
            AnomalyAlert, 'anomaly', self._on_anomaly, QoSProfiles.alert(), machine_ids)

        self._kpi_sub = self.create_subscription(
            SystemKPIs, '/miracle/system_kpis', self._on_kpis, QoSProfiles.state_data())

        self._http_port = params['http_port']
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        _MetricsHandler.metrics_ref = self._render_metrics
        self._http_server = HTTPServer(('0.0.0.0', self._http_port), _MetricsHandler)
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever, daemon=True)
        self._http_thread.start()
        self.get_logger().info(f"Prometheus metrics at :{self._http_port}/metrics")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        if self._http_server:
            self._http_server.shutdown()
        return TransitionCallbackReturn.SUCCESS

    def _on_state(self, msg: MachineState) -> None:
        with self._lock:
            self._machine_states[msg.machine_id] = msg_to_dict(msg)

    def _on_kpis(self, msg: SystemKPIs) -> None:
        with self._lock:
            d = msg_to_dict(msg)
            self._kpis = d

    def _on_anomaly(self, msg: AnomalyAlert) -> None:
        with self._lock:
            mid = getattr(msg, 'machine_id', 'unknown')
            self._anomaly_counts[mid] += 1

    def _render_metrics(self) -> str:
        lines = []

        with self._lock:
            # OEE gauge
            oee = self._kpis.get('oee', 0)
            avail = self._kpis.get('availability', 0)
            lines.append('# HELP miracle_oee Overall equipment effectiveness')
            lines.append('# TYPE miracle_oee gauge')
            lines.append(f'miracle_oee {oee}')
            lines.append('# HELP miracle_availability Machine availability')
            lines.append('# TYPE miracle_availability gauge')
            lines.append(f'miracle_availability {avail}')

            # Per-machine gauges
            lines.append('# HELP miracle_spindle_rpm Current spindle RPM')
            lines.append('# TYPE miracle_spindle_rpm gauge')
            lines.append('# HELP miracle_feed_rate Current feed rate mm/min')
            lines.append('# TYPE miracle_feed_rate gauge')
            lines.append('# HELP miracle_spindle_load Spindle load percentage')
            lines.append('# TYPE miracle_spindle_load gauge')
            lines.append('# HELP miracle_machine_state Current machine state info')
            lines.append('# TYPE miracle_machine_state gauge')

            for mid, state in self._machine_states.items():
                rpm = state.get('spindle_speed', 0)
                feed = state.get('feed_rate', 0)
                load = state.get('spindle_load', 0)
                status = state.get('status', 'UNKNOWN')
                lines.append(f'miracle_spindle_rpm{{machine="{mid}"}} {rpm}')
                lines.append(f'miracle_feed_rate{{machine="{mid}"}} {feed}')
                lines.append(f'miracle_spindle_load{{machine="{mid}"}} {load}')
                lines.append(f'miracle_machine_state{{machine="{mid}",state="{status}"}} 1')

            # Anomaly counter
            lines.append('# HELP miracle_anomaly_count Total anomaly alerts')
            lines.append('# TYPE miracle_anomaly_count counter')
            for mid, count in self._anomaly_counts.items():
                lines.append(f'miracle_anomaly_count{{machine="{mid}"}} {count}')

        return '\n'.join(lines) + '\n'


def main(args=None):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    rclpy.init(args=args)
    node = PrometheusExporterNode()
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
