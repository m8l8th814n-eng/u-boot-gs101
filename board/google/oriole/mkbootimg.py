#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
# Wrap u-boot.bin in an Android boot image (header v4) for `fastboot boot`.
import struct
import sys

src, dst = sys.argv[1], sys.argv[2]
kernel = bytearray(open(src, 'rb').read())
if kernel[56:60] != b'ARMd':
    sys.exit(f'{src}: no arm64 image header')
# arm64 image_size: room the loader keeps free past the load address
struct.pack_into('<Q', kernel, 16, 0x200000)

PAGE = 4096
hdr = b'ANDROID!' + struct.pack('<IIII4II', len(kernel), 0, 0, 1584,
                                0, 0, 0, 0, 4) + bytes(1536) + struct.pack('<I', 0)


def pad(b):
    return bytes(b) + bytes(-len(b) % PAGE)


open(dst, 'wb').write(pad(hdr) + pad(kernel))
