"""Restore real miracle_core submodules before collecting miracle_core tests.

Other test packages mock miracle_core submodules via sys.modules.setdefault at
import time. Since miracle_core/test/ is collected after those packages
(alphabetical order), the fakes are already installed. This conftest forces a
reimport of all miracle_core submodules from the real package.
"""
import importlib
import sys
import miracle_core

# Force-reload miracle_core and all its submodules that may have been replaced
# or had attributes overwritten by test mocks.
_submodules = [
    key for key in list(sys.modules)
    if key.startswith('miracle_core.') and key != 'miracle_core'
]
for key in _submodules:
    mod = sys.modules[key]
    if not hasattr(mod, '__file__') or mod.__file__ is None:
        # Fake module — remove entirely so reimport works.
        del sys.modules[key]
    elif hasattr(mod, '__file__'):
        # Real module but may have overwritten attributes — reload it.
        try:
            importlib.reload(mod)
        except Exception:
            # Some submodules need ROS2; skip those.
            pass

# Reload miracle_core itself to re-register submodules.
importlib.reload(miracle_core)
