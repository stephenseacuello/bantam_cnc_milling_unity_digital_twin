"""Workspace-level conftest that adds all package source directories to sys.path."""
import sys
from pathlib import Path

_src = Path(__file__).parent / "src"
for pkg_dir in sorted(_src.iterdir()):
    if pkg_dir.is_dir() and (pkg_dir / "setup.py").exists():
        if str(pkg_dir) not in sys.path:
            sys.path.insert(0, str(pkg_dir))
