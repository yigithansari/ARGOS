import argparse
import json
import struct
import time
import zlib
from pathlib import Path


META_MAGIC = 0x4152474F
META_VALID = 0x56414C49
META_OFFSET = 0x4000
APP_OFFSET = 0x8000
FLASH_SIZE = 512 * 1024


def main():
    parser = argparse.ArgumentParser(description="Build ARGOS initial OTA bootstrap image")
    parser.add_argument("--bootloader", type=Path, required=True)
    parser.add_argument("--application", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--build-id", type=int, default=int(time.time()))
    args = parser.parse_args()

    bootloader = args.bootloader.read_bytes()
    application = args.application.read_bytes()
    if len(bootloader) > META_OFFSET:
        raise SystemExit(f"bootloader exceeds 16 KiB: {len(bootloader)} bytes")
    if len(application) > FLASH_SIZE - APP_OFFSET:
        raise SystemExit(f"application exceeds OTA region: {len(application)} bytes")
    if len(application) < 8:
        raise SystemExit("application image is too short to contain a vector table")

    initial_sp, reset_vector = struct.unpack_from("<II", application)
    if not 0x20000000 <= initial_sp <= 0x20020000:
        raise SystemExit(f"invalid initial stack pointer: 0x{initial_sp:08X}")
    reset_address = reset_vector & ~1
    app_start = 0x08000000 + APP_OFFSET
    app_end = 0x08000000 + FLASH_SIZE
    if not app_start <= reset_address < app_end:
        raise SystemExit(f"reset vector is outside OTA application region: 0x{reset_vector:08X}")

    image_crc = zlib.crc32(application) & 0xFFFFFFFF
    metadata_prefix = struct.pack(
        "<IIIII", META_MAGIC, META_VALID, len(application), image_crc, args.build_id
    )
    metadata = metadata_prefix + struct.pack("<I", zlib.crc32(metadata_prefix) & 0xFFFFFFFF)

    combined = bytearray(bootloader)
    combined.extend(b"\xFF" * (META_OFFSET - len(combined)))
    combined.extend(metadata)
    combined.extend(b"\xFF" * (APP_OFFSET - len(combined)))
    combined.extend(application)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(combined)
    manifest = {
        "format": "ARGOS_STM32_OTA_V1",
        "build_id": args.build_id,
        "bootloader_bytes": len(bootloader),
        "application_bytes": len(application),
        "application_crc32": f"{image_crc:08X}",
        "bootstrap_bytes": len(combined),
        "bootloader_address": "0x08000000",
        "metadata_address": "0x08004000",
        "application_address": "0x08008000",
        "initial_stack_pointer": f"0x{initial_sp:08X}",
        "reset_vector": f"0x{reset_vector:08X}",
    }
    args.manifest.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
