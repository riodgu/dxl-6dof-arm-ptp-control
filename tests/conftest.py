"""Test-only fallback for environments without the DYNAMIXEL SDK installed."""

import sys
import types


try:
    import dynamixel_sdk  # noqa: F401
except ImportError:
    sdk = types.ModuleType("dynamixel_sdk")
    sdk.COMM_SUCCESS = 0
    sdk.DXL_LOBYTE = lambda value: value & 0xFF
    sdk.DXL_HIBYTE = lambda value: (value >> 8) & 0xFF
    sdk.DXL_LOWORD = lambda value: value & 0xFFFF
    sdk.DXL_HIWORD = lambda value: (value >> 16) & 0xFFFF
    sys.modules["dynamixel_sdk"] = sdk
