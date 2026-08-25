"""Hardware-free tests for DXLDriver lifecycle and value conversion."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import dxl_arm.dxl_driver as driver_module
from dxl_arm.dxl_driver import DXLCommError, DXLDriver


class FakePort:
    def __init__(self, _device):
        self.closed = False

    def openPort(self):
        return True

    def setBaudRate(self, _baudrate):
        return False

    def closePort(self):
        self.closed = True


class FakePacket:
    pass


def build_driver(monkeypatch):
    port = FakePort("COM3")
    monkeypatch.setattr(driver_module.dxl, "PortHandler", lambda _device: port, raising=False)
    monkeypatch.setattr(
        driver_module.dxl, "PacketHandler", lambda _version: FakePacket(), raising=False
    )
    driver = DXLDriver("COM3", 1_000_000, 2.0, [1], [2048], [1])
    return driver, port


def test_connect_closes_port_when_baudrate_setup_fails(monkeypatch):
    driver, port = build_driver(monkeypatch)

    with pytest.raises(DXLCommError, match="baudrate"):
        driver.connect()

    assert port.closed
    assert not driver._connected


def test_goal_degree_count_is_validated_before_conversion(monkeypatch):
    driver, _port = build_driver(monkeypatch)

    with pytest.raises(ValueError, match="goal_deg length"):
        driver.sync_write_goal_deg([])


def test_present_current_is_converted_from_signed_16_bit(monkeypatch):
    driver, _port = build_driver(monkeypatch)
    driver.read2 = lambda _dxl_id, _address: 0xFFFF

    assert driver.read_current() == [-1]
