#!/usr/bin/env python3
"""Minimal ADXL355 SPI connectivity check for EVAL-ADXL355-PMDZ.

Usage:
  . .venv/bin/activate
  python check_adxl355_spi.py --bus 0 --device 0
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import spidev

REG_DEVID_AD = 0x00
REG_DEVID_MST = 0x01
REG_PARTID = 0x02
REG_REVID = 0x03
REG_XDATA3 = 0x08
REG_YDATA3 = 0x0B
REG_ZDATA3 = 0x0E
REG_POWER_CTL = 0x2D

EXPECTED_DEVID_AD = 0xAD
EXPECTED_DEVID_MST = 0x1D
EXPECTED_PARTID = 0xED


def read_reg(spi: spidev.SpiDev, addr: int) -> int:
    resp = spi.xfer2([(addr << 1) | 0x01, 0x00])
    return resp[1]


def write_reg(spi: spidev.SpiDev, addr: int, value: int) -> None:
    spi.xfer2([(addr << 1) & 0xFE, value & 0xFF])


def read_regs(spi: spidev.SpiDev, start_addr: int, length: int) -> list[int]:
    resp = spi.xfer2([((start_addr << 1) | 0x01)] + [0x00] * length)
    return resp[1:]


def decode_20bit(b0: int, b1: int, b2: int) -> int:
    val = ((b0 << 12) | (b1 << 4) | (b2 >> 4)) & 0xFFFFF
    if val & (1 << 19):
        val -= 1 << 20
    return val


def read_axis_raw(spi: spidev.SpiDev, base_addr: int) -> int:
    b0, b1, b2 = read_regs(spi, base_addr, 3)
    return decode_20bit(b0, b1, b2)


def ensure_spidev_node(bus: int, device: int) -> None:
    node = f"/dev/spidev{bus}.{device}"
    if not os.path.exists(node):
        print(f"ERROR: {node} が見つかりません。SPIを有効化して再起動してください。")
        sys.exit(2)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--speed", type=int, default=1_000_000)
    args = parser.parse_args()

    ensure_spidev_node(args.bus, args.device)

    spi = spidev.SpiDev()
    try:
        spi.open(args.bus, args.device)
        spi.max_speed_hz = args.speed
        spi.mode = 0b00

        devid_ad = read_reg(spi, REG_DEVID_AD)
        devid_mst = read_reg(spi, REG_DEVID_MST)
        partid = read_reg(spi, REG_PARTID)
        revid = read_reg(spi, REG_REVID)

        print(
            "ID:",
            f"DEVID_AD=0x{devid_ad:02X}",
            f"DEVID_MST=0x{devid_mst:02X}",
            f"PARTID=0x{partid:02X}",
            f"REVID=0x{revid:02X}",
        )

        if (
            devid_ad != EXPECTED_DEVID_AD
            or devid_mst != EXPECTED_DEVID_MST
            or partid != EXPECTED_PARTID
        ):
            print("ERROR: ADXL355のIDと一致しません。配線/CS/SPIモードを確認してください。")
            return 1

        # Measurement mode: clear standby bit (bit0) in POWER_CTL.
        pctl = read_reg(spi, REG_POWER_CTL)
        write_reg(spi, REG_POWER_CTL, pctl & 0xFE)
        time.sleep(0.02)

        x = read_axis_raw(spi, REG_XDATA3)
        y = read_axis_raw(spi, REG_YDATA3)
        z = read_axis_raw(spi, REG_ZDATA3)
        print(f"RAW XYZ: x={x} y={y} z={z}")
        print("OK: ADXL355 SPI応答を確認しました。")
        return 0
    finally:
        spi.close()


if __name__ == "__main__":
    raise SystemExit(main())
