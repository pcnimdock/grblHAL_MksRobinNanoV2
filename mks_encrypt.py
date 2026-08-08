#
# mks_encrypt.py - produce the SD-card firmware image the stock MKS boot
#                  loader expects on MKS Robin boards.
#
# The boot loader shipped on the MKS Robin Nano V2.x reads a fixed filename
# from the microSD card and expects the payload to be lightly scrambled: bytes
# 320 through 31039 are XOR-ed with a repeating 32 byte key, everything else is
# copied verbatim.
#
# This is a direct port of encrypt_mks() from Marlin's
# buildroot/share/PlatformIO/scripts/marlin.py, which is what
# `board_build.encrypt_mks = Robin_nano35.bin` triggers in a Marlin build.
#
# Wire it up from platformio.ini with:
#
#     extra_scripts = post:mks_encrypt.py
#
# and optionally override the output name per environment with:
#
#     custom_mks_firmware_name = Robin_nano35.bin
#
# Unlike Marlin's version this keeps the plain firmware.bin around as well, so
# you can still flash over SWD/ST-Link to 0x08007000 if you prefer.
#

from pathlib import Path

Import("env")

# Repeating XOR key, applied over the scrambled window only.
KEY = [
    0xA3, 0xBD, 0xAD, 0x0D, 0x41, 0x11, 0xBB, 0x8D,
    0xDC, 0x80, 0x2D, 0xD0, 0xD2, 0xC4, 0x9B, 0x1E,
    0x26, 0xEB, 0xE3, 0x33, 0x4A, 0x15, 0xE4, 0x0A,
    0xB3, 0xB1, 0x3C, 0x93, 0xBB, 0xAF, 0xF7, 0x3E
]

SCRAMBLE_START = 320
SCRAMBLE_END = 31040  # exclusive

DEFAULT_NAME = "Robin_nano35.bin"


def _output_name():
    try:
        name = env.GetProjectOption("custom_mks_firmware_name")
    except Exception:
        name = None
    return name or DEFAULT_NAME


def encrypt(source, target, env):
    firmware = Path(target[0].path)

    if not firmware.exists():
        print("mks_encrypt: %s not found, nothing to do" % firmware)
        return

    data = bytearray(firmware.read_bytes())

    end = min(len(data), SCRAMBLE_END)
    for pos in range(SCRAMBLE_START, end):
        data[pos] ^= KEY[pos & 31]

    if len(data) < SCRAMBLE_END:
        print("mks_encrypt: warning - firmware is only %d bytes, shorter than the "
              "%d byte scrambled window" % (len(data), SCRAMBLE_END))

    out = firmware.with_name(_output_name())
    out.write_bytes(data)

    print("mks_encrypt: wrote %s (%d bytes)" % (out, len(data)))
    print("mks_encrypt: copy it to the root of a FAT32 microSD card, insert it, "
          "and power-cycle the board")


env.AddPostAction("$BUILD_DIR/firmware.bin", encrypt)
