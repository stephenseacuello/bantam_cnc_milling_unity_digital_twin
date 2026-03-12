"""Workspace-level conftest that adds all package source directories to sys.path."""
import sys
from pathlib import Path

_src = Path(__file__).parent / "src"
for pkg_dir in sorted(_src.iterdir()):
    if pkg_dir.is_dir() and (pkg_dir / "setup.py").exists():
        if str(pkg_dir) not in sys.path:
            sys.path.insert(0, str(pkg_dir))

# Import miracle_core and its submodules early, so test mocks using
# sys.modules.setdefault can't replace them with bare/fake modules.
import miracle_core  # noqa: E402, F401
assert hasattr(miracle_core, '__path__'), "miracle_core must be a real package"

# Eagerly import submodules that don't need ROS2, so test setdefault is a no-op.
import miracle_core.exceptions  # noqa: E402, F401
# Most other submodules need ROS2 — tests mock them via setdefault.
