"""Restore real miracle_security submodules before collecting miracle_security tests.

Other test packages mock miracle_security submodules via sys.modules.setdefault at
import time. This conftest forces a reimport of all miracle_security submodules from
the real package.
"""
import importlib
import sys

# Remove fake miracle_security submodules so reimport picks up the real ones.
_submodules = [
    key for key in list(sys.modules)
    if key.startswith('miracle_security.') and key != 'miracle_security'
]
for key in _submodules:
    mod = sys.modules[key]
    if not hasattr(mod, '__file__') or mod.__file__ is None:
        # Fake module — remove entirely so reimport works.
        del sys.modules[key]
    elif hasattr(mod, '__file__'):
        try:
            importlib.reload(mod)
        except Exception:
            pass

import miracle_security  # noqa: E402
importlib.reload(miracle_security)
