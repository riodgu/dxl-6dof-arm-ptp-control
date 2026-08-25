"""DYNAMIXEL Protocol 2.0 control table addresses and shared constants.

Addresses below match the X-series (DXL430 / DXL330) control table used by
this project. Only the entries actually used by the driver are defined.
"""

# --- EEPROM ---
ADDR_OPERATING_MODE = 11

# --- RAM ---
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146

# --- Data lengths (bytes) ---
LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4

# --- Torque ---
TORQUE_OFF = 0
TORQUE_ON = 1

# --- Operating modes ---
POSITION_CONTROL_MODE = 3
EXTENDED_POSITION_CONTROL_MODE = 4

# --- Position/angle conversion ---
# X-series encoder resolution: 4096 pulses per revolution (0.088 deg/pulse).
PULSE_PER_REV = 4096
DEG_PER_PULSE = 360.0 / PULSE_PER_REV
PULSE_PER_DEG = PULSE_PER_REV / 360.0
