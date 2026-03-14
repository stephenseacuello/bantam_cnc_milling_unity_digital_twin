# MIRACLE Testing Guide

## 1. Test Overview

The MIRACLE digital twin system includes 2739 Python tests spread across 13 packages, plus Unity EditMode and PlayMode tests and performance benchmarks.

Key facts:

- All Python tests run **without ROS2 installed**. ROS2 dependencies are mocked at the module level.
- Unity tests use the built-in Test Runner (EditMode and PlayMode).
- Benchmarks live in a separate `miracle_ws/benchmarks/` directory.

---

## 2. Running Tests

### Full Suite

```bash
python3 -m pytest miracle_ws/src/ --ignore=miracle_ws/src/miracle_unity_bridge -v --rootdir=miracle_ws
```

### Quick Run (quiet output)

```bash
python3 -m pytest miracle_ws/src/ --ignore=miracle_ws/src/miracle_unity_bridge -q --rootdir=miracle_ws
```

### Single Package

```bash
python3 -m pytest miracle_ws/src/miracle_twin/test/ -v --rootdir=miracle_ws
python3 -m pytest miracle_ws/src/miracle_scada/test/ -v --rootdir=miracle_ws
python3 -m pytest miracle_ws/src/miracle_security/test/ -v --rootdir=miracle_ws
# etc.
```

### Single Test File

```bash
python3 -m pytest miracle_ws/src/miracle_twin/test/test_cutting_sim_proxy.py -v --rootdir=miracle_ws
```

### Benchmarks

```bash
python3 -m pytest miracle_ws/benchmarks/ -v -s
```

### Unity Tests

Open Unity and navigate to **Window > General > Test Runner**, then select **EditMode** or **PlayMode** to run the corresponding test suites.

> **IMPORTANT:** Always use `--rootdir=miracle_ws`. Without it, `conftest.py` files will not be discovered and tests will fail with import errors.

---

## 3. Test Organization

| Package | Test Directory | Approx Tests | Key Test Files |
|---------|---------------|--------------|----------------|
| miracle_twin | miracle_twin/test/ | ~400 | test_cutting_sim_proxy, test_thermal_model, test_vibration_analysis, test_workholding, test_chip_load |
| miracle_scada | miracle_scada/test/ | ~350 | test_alarm_manager, test_alert_correlator, test_kpi_calculator, test_shift_report, test_escalation_engine |
| miracle_mes | miracle_mes/test/ | ~250 | test_digital_thread, test_job_scheduler, test_energy_tracker, test_queue_optimizer |
| miracle_cognitive | miracle_cognitive/test/ | ~200 | test_knowledge_graph, test_causal_inference, test_explanation_generator, test_root_cause_analyzer |
| miracle_security | miracle_security/test/ | ~150 | test_secure_storage, test_chain_verification, test_gcode_signer |
| miracle_cnc | miracle_cnc/test/ | ~150 | test_gcode_executor, test_macro_expansion, test_sensor_fusion |
| miracle_resiliency | miracle_resiliency/test/ | ~100 | test_recovery_orchestrator, test_partition_detector, test_chaos_injector |
| miracle_ai | miracle_ai/test/ | ~100 | test_anomaly_detector, test_phm_predictor |
| miracle_core | miracle_core/test/ | ~50 | test_exceptions, test_lifecycle_node_base |

---

## 4. Writing New Tests

### Mock Pattern (CRITICAL)

All tests mock ROS2 dependencies. Follow this pattern exactly:

```python
import sys
from unittest.mock import MagicMock

# Mock ROS2 (always use setdefault)
for mod in ('rclpy', 'rclpy.lifecycle', 'rclpy.node', 'rclpy.qos',
            'rclpy.parameter', 'rclpy.callback_groups', 'rclpy.executors',
            'std_msgs', 'std_msgs.msg'):
    sys.modules.setdefault(mod, MagicMock())

# Mock miracle_core SUBMODULES (NEVER mock top-level miracle_core!)
for sub in ('miracle_core.lifecycle_node_base', 'miracle_core.qos_profiles',
            'miracle_core.heartbeat_mixin'):
    sys.modules.setdefault(sub, MagicMock())

# Mock miracle_msgs submodules
sys.modules.setdefault('miracle_msgs', MagicMock())
sys.modules.setdefault('miracle_msgs.msg', MagicMock())
```

### Rules

1. **NEVER** use `sys.modules['miracle_core'] = MagicMock()` -- this replaces the real package and breaks miracle_core tests downstream.
2. **NEVER** use `sys.modules['miracle_security'] = MagicMock()` -- same issue.
3. **ALWAYS** use `sys.modules.setdefault(...)` for submodules.
4. If a module already exists and you need to add attributes, use:
   ```python
   mod = sys.modules['miracle_msgs.msg']
   if not hasattr(mod, 'SomeMessage'):
       mod.SomeMessage = MagicMock()
   ```
5. If testing a module that was polluted by other tests, reload it:
   ```python
   import importlib
   sys.modules.pop('my_package.my_module', None)
   from my_package.my_module import MyClass  # fresh import
   ```

### Creating a Node Test

When testing a ROS2 node (one whose `__init__` calls `super().__init__`), bypass the constructor:

```python
def _make_node():
    node = MyNode.__new__(MyNode)
    # Manually set all __init__ attributes
    node._some_field = default_value
    node.get_logger = lambda: MagicMock()
    node.get_clock = lambda: MagicMock(now=lambda: MagicMock(to_msg=lambda: MagicMock()))
    return node
```

When the real code adds new attributes in `__init__`, update `_make_node()` in every test file that uses it.

---

## 5. Conftest Files

Three `conftest.py` files handle test isolation:

- **`miracle_ws/conftest.py`** -- Adds all package directories to `sys.path` and eagerly imports `miracle_core`.
- **`miracle_ws/src/miracle_core/test/conftest.py`** -- Reloads `miracle_core` submodules to undo mock pollution from other packages.
- **`miracle_ws/src/miracle_security/test/conftest.py`** -- Reloads `miracle_security` submodules to undo mock pollution.

---

## 6. Common Test Failures

| Error | Cause | Fix |
|-------|-------|-----|
| `miracle_core is not a package` | A test replaced miracle_core with MagicMock | Remove `sys.modules['miracle_core'] = ...` |
| `No module named 'miracle_security.xxx'; 'miracle_security' is not a package` | A test replaced miracle_security with MagicMock | Use `setdefault` for submodules only |
| `AttributeError: node has no attribute '_xxx'` | `_make_node()` is missing a newly added field | Add the field to `_make_node()` |
| `reload() argument must be a module` | A mock replaced a real module so conftest cannot reload it | Fix the offending test's mock pattern |
| Tests pass alone but fail in suite | Mock pollution from test ordering | Add `importlib.reload()` or use conftest |

---

## 7. CI/CD

Tests run in GitHub Actions on every push and pull request. The pipeline:

1. Install Python dependencies.
2. Run pytest with coverage.
3. Lint with ruff.
4. Build Docker images.
