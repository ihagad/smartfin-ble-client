"""Minimal .sfdat reader for fixture tests (matches src/pipeline/file_sink.hpp)."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

RIDE_FILE_MAGIC = 0x5346444154
RIDE_FILE_VERSION = 2
RECORD_TAG_QUAT_IMU = 0x02

QUAT_FMT = "<I3f3f3f4ffB"


@dataclass(frozen=True)
class SfdatHeader:
    magic: int
    version: int
    imu_size: int
    quat_size: int
    temp_size: int
    fwver_size: int


@dataclass(frozen=True)
class QuatImuRecord:
    elapsed_ms: int


def read_header(data: bytes) -> SfdatHeader:
    magic, version, imu_size, quat_size, temp_size, fwver_size = struct.unpack_from(
        "<QH4B", data, 0
    )
    return SfdatHeader(magic, version, imu_size, quat_size, temp_size, fwver_size)


def load_quat_imu_elapsed_ms(path: Path) -> list[int]:
    data = path.read_bytes()
    hdr = read_header(data)
    if hdr.magic != RIDE_FILE_MAGIC:
        raise ValueError(f"bad magic: {hex(hdr.magic)}")
    if hdr.version != RIDE_FILE_VERSION:
        raise ValueError(f"unsupported version: {hdr.version}")
    if hdr.quat_size != struct.calcsize(QUAT_FMT):
        raise ValueError(
            f"unexpected WireQuatImu size: {hdr.quat_size} != {struct.calcsize(QUAT_FMT)}"
        )

    elapsed: list[int] = []
    off = struct.calcsize("<QH4B")
    while off < len(data):
        tag = data[off]
        off += 1
        if tag != RECORD_TAG_QUAT_IMU:
            raise ValueError(f"unexpected record tag: {tag:#x}")
        (elapsed_ms,) = struct.unpack_from("<I", data, off)
        elapsed.append(elapsed_ms)
        off += hdr.quat_size
    return elapsed
