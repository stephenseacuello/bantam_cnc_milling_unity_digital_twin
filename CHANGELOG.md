# Changelog

All notable changes to the MIRACLE CNC Milling Digital Twin will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added
- GitHub Actions CI/CD pipeline (test, lint, Docker build)
- Root README with architecture overview and quick start
- Pinned Python dependencies (requirements.txt, requirements-dev.txt)
- Kubernetes RBAC, NetworkPolicy, PDB, HPA, Secrets
- Assembly definition splitting (Editor, Testing, Testing.Editor)
- Centralized logging with Grafana Loki and Promtail
- G-Code validator with machine limit checks
- ROS2 API reference documentation
- Performance benchmark suite
- Null safety validation (OnValidate) for event channels
- Apache 2.0 LICENSE file
- This CHANGELOG

## [0.3.0] - 2026-03-05

### Added
- Runtime G-code file browser for standalone builds
- Multi-machine switching with chart clearing and visualization toggling
- Digital twin replay mode (record/replay buttons, timeline scrubber)
- Prometheus metrics exporter node
- Grafana dashboards with auto-provisioning
- Configuration externalization (YAML defaults, env var overrides, ScriptableObject)
- 100+ new Python tests (Kafka, OPC-UA, IDS, e2e pipeline, HMI bridge)
- Unity dashboard integration tests (UXML element validation)
- ProfilerMarker instrumentation on hot paths
- Voxel metrics in PerformanceMonitor

### Fixed
- GPU compute shader fallback for unsupported hardware
- DashboardWiring no-machine warning
- HMI bridge RuntimeError during event loop shutdown
- Keyboard input guard for UI focus
- E-STOP toggle behavior

## [0.2.0] - 2026-03-04

### Added
- ROS2 lifecycle nodes with heartbeat mixin
- Multi-machine subscription support
- Kubernetes manifests (namespace, deployment, StatefulSet, ConfigMap)
- Circuit breaker pattern for resilient connections
- Lifecycle autostart utility
- Docker Compose orchestration (ROS2, micro-ROS, MQTT, Kafka, ZooKeeper)

## [0.1.0] - 2026-03-04

### Added
- Initial Unity digital twin for Bantam Tools Explorer CNC
- G-code execution pipeline with real-time simulation
- Voxel-based material removal with GPU compute shaders
- Marching cubes mesh rendering
- Dashboard UI with force, thermal, wear, and power charts
- ROS2 bridge (ROS-TCP-Connector) with ScriptableObject event system
- Coast Runner CR-1 CNC model support
- Robot arm controllers (Niryo Ned2, xArm6)
- Multi-agent task coordination
- Cutting force, thermal, tool wear, surface roughness models
- Audio feedback (cutting and spindle sounds)
- Visualization overlays (heat map, force arrows, surface roughness)
