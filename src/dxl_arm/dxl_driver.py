"""DYNAMIXEL communication driver built on the DYNAMIXEL SDK.

Wraps PortHandler/PacketHandler for single-motor read/write and
GroupSyncRead/GroupSyncWrite for simultaneous multi-motor position control.
"""

from typing import List, Sequence

import dynamixel_sdk as dxl

from . import control_table as ct
from .kinematics import joint_deg_to_raw, joint_raw_to_deg


class DXLCommError(Exception):
    """Raised when a DYNAMIXEL communication or hardware error occurs."""


class DXLDriver:
    """Owns the serial port and issues single/group read-write DYNAMIXEL commands."""

    def __init__(
        self,
        device_name: str,
        baudrate: int,
        protocol_version: float,
        dxl_ids: Sequence[int],
        home_raw: Sequence[int],
        joint_sign: Sequence[int],
        operating_mode: int = ct.EXTENDED_POSITION_CONTROL_MODE,
        profile_acceleration: int = 20,
        profile_velocity: int = 80,
    ):
        self.device_name = device_name
        self.baudrate = baudrate
        self.protocol_version = protocol_version
        self.dxl_ids = list(dxl_ids)
        self.home_raw = list(home_raw)
        self.joint_sign = list(joint_sign)
        self.operating_mode = operating_mode
        self.profile_acceleration = profile_acceleration
        self.profile_velocity = profile_velocity

        if not self.dxl_ids:
            raise ValueError("dxl_ids must not be empty")
        if len(set(self.dxl_ids)) != len(self.dxl_ids):
            raise ValueError("dxl_ids must be unique")
        if len(self.home_raw) != len(self.dxl_ids):
            raise ValueError("home_raw length must match dxl_ids")
        if len(self.joint_sign) != len(self.dxl_ids):
            raise ValueError("joint_sign length must match dxl_ids")
        if any(sign not in (-1, 1) for sign in self.joint_sign):
            raise ValueError("joint_sign entries must be either -1 or 1")

        self.port_handler = dxl.PortHandler(self.device_name)
        self.packet_handler = dxl.PacketHandler(self.protocol_version)

        self._group_sync_read = None
        self._group_sync_write = None
        self._connected = False

    # ---------------------------------------------------------------- #
    # Connection lifecycle
    # ---------------------------------------------------------------- #

    def connect(self) -> None:
        """Open the serial port and set the configured baudrate."""
        try:
            if not self.port_handler.openPort():
                raise DXLCommError(f"failed to open port {self.device_name}")
            if not self.port_handler.setBaudRate(self.baudrate):
                raise DXLCommError(f"failed to set baudrate {self.baudrate}")

            self._group_sync_read = dxl.GroupSyncRead(
                self.port_handler, self.packet_handler,
                ct.ADDR_PRESENT_POSITION, ct.LEN_PRESENT_POSITION,
            )
            self._group_sync_write = dxl.GroupSyncWrite(
                self.port_handler, self.packet_handler,
                ct.ADDR_GOAL_POSITION, ct.LEN_GOAL_POSITION,
            )
            for dxl_id in self.dxl_ids:
                if not self._group_sync_read.addParam(dxl_id):
                    raise DXLCommError(f"GroupSyncRead.addParam failed for id {dxl_id}")

            self._connected = True
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        """Close the serial port."""
        if self.port_handler is not None:
            self.port_handler.closePort()
        self._connected = False

    def shutdown(self) -> None:
        """Disable torque (best-effort) and close the port. Safe to call multiple times."""
        try:
            if self._connected:
                self.disable_torque()
        except Exception as e:
            print(f"[DXLDriver] shutdown: failed to disable torque cleanly: {e}")
        finally:
            self.close()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()
        return False

    # ---------------------------------------------------------------- #
    # Low-level read/write
    # ---------------------------------------------------------------- #

    def write1(self, dxl_id: int, address: int, value: int) -> None:
        """Write a single byte to a motor's control table."""
        result, error = self.packet_handler.write1ByteTxRx(self.port_handler, dxl_id, address, value)
        self._check_comm(dxl_id, result, error, f"write1(addr={address}, value={value})")

    def write4(self, dxl_id: int, address: int, value: int) -> None:
        """Write 4 bytes to a motor's control table."""
        result, error = self.packet_handler.write4ByteTxRx(self.port_handler, dxl_id, address, value)
        self._check_comm(dxl_id, result, error, f"write4(addr={address}, value={value})")

    def read1(self, dxl_id: int, address: int) -> int:
        """Read a single byte from a motor's control table."""
        value, result, error = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, address)
        self._check_comm(dxl_id, result, error, f"read1(addr={address})")
        return value

    def read2(self, dxl_id: int, address: int) -> int:
        """Read 2 bytes from a motor's control table."""
        value, result, error = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, address)
        self._check_comm(dxl_id, result, error, f"read2(addr={address})")
        return value

    def read4(self, dxl_id: int, address: int) -> int:
        """Read 4 bytes from a motor's control table (returned as signed int32)."""
        value, result, error = self.packet_handler.read4ByteTxRx(self.port_handler, dxl_id, address)
        self._check_comm(dxl_id, result, error, f"read4(addr={address})")
        if value > 0x7FFFFFFF:
            value -= 0x100000000
        return value

    def ping(self, dxl_id: int) -> bool:
        """Ping a motor. Returns True if it responds, False otherwise."""
        model_number, result, error = self.packet_handler.ping(self.port_handler, dxl_id)
        return result == dxl.COMM_SUCCESS and error == 0

    def _check_comm(self, dxl_id: int, result: int, error: int, context: str) -> None:
        if result != dxl.COMM_SUCCESS:
            raise DXLCommError(
                f"id={dxl_id} {context}: {self.packet_handler.getTxRxResult(result)}"
            )
        if error != 0:
            raise DXLCommError(
                f"id={dxl_id} {context}: {self.packet_handler.getRxPacketError(error)}"
            )

    # ---------------------------------------------------------------- #
    # Motor setup
    # ---------------------------------------------------------------- #

    def setup_motors(self) -> None:
        """Set operating mode and motion profile on every configured motor, torque off first."""
        for dxl_id in self.dxl_ids:
            self.write1(dxl_id, ct.ADDR_TORQUE_ENABLE, ct.TORQUE_OFF)
            self.write1(dxl_id, ct.ADDR_OPERATING_MODE, self.operating_mode)
            self.write4(dxl_id, ct.ADDR_PROFILE_ACCELERATION, self.profile_acceleration)
            self.write4(dxl_id, ct.ADDR_PROFILE_VELOCITY, self.profile_velocity)
        self.enable_torque()

    def disable_torque(self) -> None:
        """Disable torque on every configured motor."""
        for dxl_id in self.dxl_ids:
            self.write1(dxl_id, ct.ADDR_TORQUE_ENABLE, ct.TORQUE_OFF)

    def enable_torque(self) -> None:
        """Enable torque on every configured motor."""
        for dxl_id in self.dxl_ids:
            self.write1(dxl_id, ct.ADDR_TORQUE_ENABLE, ct.TORQUE_ON)

    # ---------------------------------------------------------------- #
    # Group read/write: position
    # ---------------------------------------------------------------- #

    def read_joint_raw(self) -> List[int]:
        """Read present position (raw) of all configured motors via GroupSyncRead."""
        result = self._group_sync_read.txRxPacket()
        if result != dxl.COMM_SUCCESS:
            raise DXLCommError(f"GroupSyncRead failed: {self.packet_handler.getTxRxResult(result)}")

        raw_positions = []
        for dxl_id in self.dxl_ids:
            if not self._group_sync_read.isAvailable(dxl_id, ct.ADDR_PRESENT_POSITION, ct.LEN_PRESENT_POSITION):
                raise DXLCommError(f"GroupSyncRead: no data available for id {dxl_id}")
            raw = self._group_sync_read.getData(dxl_id, ct.ADDR_PRESENT_POSITION, ct.LEN_PRESENT_POSITION)
            if raw > 0x7FFFFFFF:
                raw -= 0x100000000
            raw_positions.append(raw)
        return raw_positions

    def read_joint_deg(self) -> List[float]:
        """Read present position of all configured motors, converted to degrees."""
        return self.joint_raw_values_to_deg(self.read_joint_raw())

    def joint_raw_values_to_deg(self, raw_positions: Sequence[int]) -> List[float]:
        """Convert one already-read raw position snapshot to joint degrees."""
        if len(raw_positions) != len(self.dxl_ids):
            raise ValueError("raw_positions length must match number of configured motors")
        return [
            joint_raw_to_deg(i, raw, self.home_raw, self.joint_sign, ct.DEG_PER_PULSE)
            for i, raw in enumerate(raw_positions)
        ]

    def sync_write_goal_raw(self, goal_raw: Sequence[int]) -> None:
        """Write goal position (raw) to all configured motors via GroupSyncWrite."""
        if len(goal_raw) != len(self.dxl_ids):
            raise ValueError("goal_raw length must match number of configured motors")

        self._group_sync_write.clearParam()
        for dxl_id, raw in zip(self.dxl_ids, goal_raw):
            raw_u32 = raw & 0xFFFFFFFF
            param = [
                dxl.DXL_LOBYTE(dxl.DXL_LOWORD(raw_u32)),
                dxl.DXL_HIBYTE(dxl.DXL_LOWORD(raw_u32)),
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(raw_u32)),
                dxl.DXL_HIBYTE(dxl.DXL_HIWORD(raw_u32)),
            ]
            if not self._group_sync_write.addParam(dxl_id, param):
                raise DXLCommError(f"GroupSyncWrite.addParam failed for id {dxl_id}")

        result = self._group_sync_write.txPacket()
        self._group_sync_write.clearParam()
        if result != dxl.COMM_SUCCESS:
            raise DXLCommError(f"GroupSyncWrite failed: {self.packet_handler.getTxRxResult(result)}")

    def sync_write_goal_deg(self, goal_deg: Sequence[float]) -> None:
        """Convert per-joint degree goals to raw and write via GroupSyncWrite."""
        if len(goal_deg) != len(self.dxl_ids):
            raise ValueError("goal_deg length must match number of configured motors")
        goal_raw = [
            joint_deg_to_raw(i, angle, self.home_raw, self.joint_sign, ct.PULSE_PER_DEG)
            for i, angle in enumerate(goal_deg)
        ]
        self.sync_write_goal_raw(goal_raw)

    # ---------------------------------------------------------------- #
    # Status reads
    # ---------------------------------------------------------------- #

    def read_temperature(self) -> List[int]:
        """Read present temperature (deg C) of every configured motor."""
        return [self.read1(dxl_id, ct.ADDR_PRESENT_TEMPERATURE) for dxl_id in self.dxl_ids]

    def read_voltage(self) -> List[float]:
        """Read present input voltage (V) of every configured motor."""
        return [self.read2(dxl_id, ct.ADDR_PRESENT_INPUT_VOLTAGE) / 10.0 for dxl_id in self.dxl_ids]

    def read_current(self) -> List[int]:
        """Read present current (mA-scale raw units) of every configured motor."""
        values = [self.read2(dxl_id, ct.ADDR_PRESENT_CURRENT) for dxl_id in self.dxl_ids]
        return [value - 0x10000 if value > 0x7FFF else value for value in values]

    def read_hardware_error(self) -> List[int]:
        """Read hardware error status byte of every configured motor."""
        return [self.read1(dxl_id, ct.ADDR_HARDWARE_ERROR_STATUS) for dxl_id in self.dxl_ids]
