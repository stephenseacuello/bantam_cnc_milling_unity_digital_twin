"""
Centralized configuration loader for MIRACLE nodes.

Reads defaults from miracle_defaults.yaml, allows environment variable
overrides using the pattern: MIRACLE_<SECTION>_<KEY> (uppercase).

Example:
    MIRACLE_NETWORK_UNITY_TCP_PORT=10001  overrides network.unity_tcp_port
"""

import os
from pathlib import Path
from typing import Any, Dict, Optional

_cached_config: Optional[Dict[str, Any]] = None

# Search paths for the config file (first match wins)
_CONFIG_SEARCH_PATHS = [
    Path(__file__).resolve().parents[4] / "config" / "miracle_defaults.yaml",
    Path("/etc/miracle/miracle_defaults.yaml"),
    Path.home() / ".miracle" / "miracle_defaults.yaml",
]


def _load_yaml(path: Path) -> Dict[str, Any]:
    """Load a YAML file, returning an empty dict on failure."""
    try:
        import yaml
    except ImportError:
        return {}
    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f)
            return data if isinstance(data, dict) else {}
    except (OSError, yaml.YAMLError):
        return {}


def load_config(force_reload: bool = False) -> Dict[str, Any]:
    """Load and cache the MIRACLE configuration.

    Searches predefined paths for miracle_defaults.yaml.
    Results are cached after the first successful load.

    Args:
        force_reload: If True, ignore cache and reload from disk.

    Returns:
        Nested dictionary of configuration values.
    """
    global _cached_config
    if _cached_config is not None and not force_reload:
        return _cached_config

    for search_path in _CONFIG_SEARCH_PATHS:
        if search_path.exists():
            _cached_config = _load_yaml(search_path)
            if _cached_config:
                return _cached_config

    _cached_config = {}
    return _cached_config


def get_default(section: str, key: str, fallback: Any = None) -> Any:
    """Get a configuration value with environment variable override.

    Checks MIRACLE_<SECTION>_<KEY> environment variable first,
    then falls back to the YAML config, then to the provided fallback.

    Type coercion is automatic based on the fallback type:
    - int fallback → int(env_value)
    - float fallback → float(env_value)
    - bool fallback → env_value.lower() in ('true', '1', 'yes')
    - str fallback → raw string

    Args:
        section: Config section name (e.g. 'network', 'kafka').
        key: Config key within the section (e.g. 'unity_tcp_port').
        fallback: Default value if neither env nor YAML has the key.

    Returns:
        The resolved configuration value.
    """
    # 1. Check environment variable
    env_key = f"MIRACLE_{section.upper()}_{key.upper()}"
    env_val = os.environ.get(env_key)
    if env_val is not None:
        return _coerce(env_val, fallback)

    # 2. Check YAML config
    config = load_config()
    section_data = config.get(section, {})
    if isinstance(section_data, dict) and key in section_data:
        return section_data[key]

    # 3. Return fallback
    return fallback


def _coerce(value: str, reference: Any) -> Any:
    """Coerce a string value to match the type of the reference value."""
    if reference is None:
        return value
    if isinstance(reference, bool):
        return value.lower() in ("true", "1", "yes")
    if isinstance(reference, int):
        try:
            return int(value)
        except ValueError:
            return int(float(value))
    if isinstance(reference, float):
        return float(value)
    return value
