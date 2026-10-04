#!/bin/sh
# SPDX-License-Identifier: GPL-2.0+
# Build U-Boot for the Pixel 6 with LLVM and wrap it for `fastboot boot`.
# Run from the top of the U-Boot tree: board/google/oriole/build.sh
set -e

top=$(pwd)
shims="$top/.oriole-llvm"
mkdir -p "$shims"
printf '#!/bin/sh\nexec clang --target=aarch64-linux-gnu "$@" -fintegrated-as\n' \
	> "$shims/aarch64-linux-gnu-gcc"
for t in ar nm strip objcopy objdump ranlib readelf; do
	printf '#!/bin/sh\nexec llvm-%s "$@"\n' "$t" > "$shims/aarch64-linux-gnu-$t"
done
printf '#!/bin/sh\nexec ld.lld "$@"\n' > "$shims/aarch64-linux-gnu-ld"
printf '#!/bin/sh\nexec ld.lld "$@"\n' > "$shims/aarch64-linux-gnu-ld.bfd"
chmod +x "$shims"/aarch64-linux-gnu-*

export PATH="$shims:$PATH"
make oriole_defconfig
make CROSS_COMPILE=aarch64-linux-gnu- -j"$(nproc)"
python3 board/google/oriole/mkbootimg.py u-boot.bin oriole-uboot-boot.img
echo "fastboot boot $top/oriole-uboot-boot.img"
