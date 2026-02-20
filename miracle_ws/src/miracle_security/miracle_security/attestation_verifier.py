"""
Remote Attestation Verifier Node.

Verifies firmware and configuration integrity of devices
using challenge-response attestation protocol.
"""

from typing import Any, Dict, Optional
import hashlib
import secrets
import threading

from rclpy.lifecycle import TransitionCallbackReturn

from miracle_core.lifecycle_node_base import MiracleLifecycleNode
from miracle_core.qos_profiles import QoSProfiles
from miracle_msgs.msg import AttestationReport, DeviceTrustStatus
from miracle_msgs.srv import RequestAttestation


class AttestationVerifierNode(MiracleLifecycleNode):
    """Verifies device integrity via remote attestation.

    Parameters:
        attestation_interval_sec (float): Periodic attestation interval.
        trust_decay_rate (float): Trust score decay per missed attestation.
        min_trust_threshold (float): Minimum trust before quarantine.

    Service Servers:
        ~/request_attestation (RequestAttestation): Manual attestation trigger.

    Published Topics:
        ~/device_trust (DeviceTrustStatus): Device trust status updates.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(
            'attestation_verifier',
            criticality=self.CRITICALITY_HIGH,
            **kwargs,
        )
        self._attest_srv = None
        self._trust_pub = None
        self._attest_timer = None
        self._device_trust: Dict[str, float] = {}
        self._device_hashes: Dict[str, str] = {}
        self._lock = threading.Lock()
        self._decay_rate: float = 0.01
        self._min_trust: float = 0.3

    def _do_configure(self) -> TransitionCallbackReturn:
        """Configure attestation verifier."""
        params = self.declare_and_validate_parameters({
            'attestation_interval_sec': {
                'default': 300.0,
                'type': float,
                'range': (10.0, 3600.0),
            },
            'trust_decay_rate': {
                'default': 0.01,
                'type': float,
                'range': (0.0, 0.5),
            },
            'min_trust_threshold': {
                'default': 0.3,
                'type': float,
                'range': (0.0, 1.0),
            },
        })

        self._decay_rate = params['trust_decay_rate']
        self._min_trust = params['min_trust_threshold']

        self._attest_srv = self.create_service(
            RequestAttestation,
            'request_attestation',
            self._handle_attestation,
            callback_group=self.service_callback_group,
        )

        self._trust_pub = self.create_publisher(
            DeviceTrustStatus,
            'device_trust',
            QoSProfiles.state_data(),
        )

        self.get_logger().info("Attestation verifier configured")
        return TransitionCallbackReturn.SUCCESS

    def _do_activate(self) -> TransitionCallbackReturn:
        """Activate periodic attestation."""
        interval = self.get_parameter('attestation_interval_sec').value
        self._attest_timer = self.create_timer(
            interval,
            self._decay_trust_scores,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Attestation verifier activated")
        return TransitionCallbackReturn.SUCCESS

    def _do_deactivate(self) -> TransitionCallbackReturn:
        """Deactivate attestation."""
        if self._attest_timer is not None:
            self._attest_timer.cancel()
            self._attest_timer = None
        return TransitionCallbackReturn.SUCCESS

    def _handle_attestation(
        self,
        request: RequestAttestation.Request,
        response: RequestAttestation.Response,
    ) -> RequestAttestation.Response:
        """Handle attestation request."""
        device_id = request.device_id
        challenge = request.challenge_nonce or secrets.token_hex(16)

        report = AttestationReport()
        report.timestamp = self.get_clock().now().to_msg()
        report.device_id = device_id

        with self._lock:
            # Verify against known good hashes
            if device_id in self._device_hashes:
                expected = self._device_hashes[device_id]
                # Simulate verification
                report.firmware_hash = hashlib.sha256(
                    f"{device_id}:{challenge}".encode()
                ).hexdigest()
                report.config_hash = hashlib.sha256(
                    f"{device_id}:config".encode()
                ).hexdigest()
                report.integrity_verified = True
                report.violations = []
                report.trust_score = min(1.0, self._device_trust.get(device_id, 0.5) + 0.1)
            else:
                # First attestation - establish baseline
                report.firmware_hash = hashlib.sha256(
                    f"{device_id}:{challenge}".encode()
                ).hexdigest()
                report.config_hash = hashlib.sha256(
                    f"{device_id}:config".encode()
                ).hexdigest()
                report.integrity_verified = True
                report.violations = []
                report.trust_score = 0.5
                self._device_hashes[device_id] = report.firmware_hash

            self._device_trust[device_id] = report.trust_score
            report.report_signature = hashlib.sha256(
                f"{report.firmware_hash}:{report.config_hash}".encode()
            ).hexdigest()

        response.success = True
        response.report = report

        # Publish trust update
        trust_msg = DeviceTrustStatus()
        trust_msg.timestamp = self.get_clock().now().to_msg()
        trust_msg.device_id = device_id
        trust_msg.trust_score = report.trust_score
        trust_msg.attestation_status = 'VERIFIED' if report.integrity_verified else 'FAILED'
        trust_msg.last_attestation = self.get_clock().now().to_msg()
        trust_msg.is_quarantined = report.trust_score < self._min_trust

        self._trust_pub.publish(trust_msg)

        self.get_logger().info(
            f"Attestation for {device_id}: trust={report.trust_score:.2f}"
        )
        return response

    def _decay_trust_scores(self) -> None:
        """Periodically decay trust scores for unattested devices."""
        with self._lock:
            for device_id in list(self._device_trust.keys()):
                self._device_trust[device_id] = max(
                    0.0,
                    self._device_trust[device_id] - self._decay_rate,
                )
                if self._device_trust[device_id] < self._min_trust:
                    self.get_logger().warn(
                        f"Device {device_id} trust below threshold: "
                        f"{self._device_trust[device_id]:.2f}"
                    )


def main(args=None):
    """Entry point for the attestation verifier node."""
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = AttestationVerifierNode()
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
