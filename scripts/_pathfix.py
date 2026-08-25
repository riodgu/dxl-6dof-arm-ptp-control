"""Adds src/ to sys.path so scripts can `import dxl_arm` without installation."""

import os
import sys

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
_SRC_DIR = os.path.join(_SCRIPTS_DIR, "..", "src")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)
