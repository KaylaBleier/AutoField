"""
config_loader.py

Reads config.yaml and exposes a single CFG dict used by all modules.

Usage:
    from config_loader import CFG

    port = CFG["serial"]["gnss_port"]
    baud = CFG["serial"]["gnss_baud"]
"""

import os
import yaml

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")


def _load() -> dict:
    try:
        with open(_CONFIG_PATH, "r") as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        raise RuntimeError(
            f"config.yaml not found at {_CONFIG_PATH}. "
            "Make sure it sits in the same directory as main.py."
        )
    except yaml.YAMLError as e:
        raise RuntimeError(f"Error parsing config.yaml: {e}")


CFG = _load()
