#!/bin/bash
# Build Black Magic Probe for BlackPill F401CC with Pinout 0
# Pinout 0: SWDIO=PB9, SWCLK=PB8, TDI=PB6, TDO=PB7, nRST=PA5
#
# To flash with DFU (put BlackPill in DFU mode: hold BOOT0, press RESET,
# 		release BOOT0):
#   dfu-util -d 0483:df11 --alt 0 -s 0x08000000:leave -D
#   		build-f401cc-pinout0/blackmagic_blackpill_f401cc_bootloader.bin
#   (enter DFU mode again)
#   dfu-util -d 0483:df11 --alt 0 -s 0x08004000:leave -D
#   		build-f401cc-pinout0/blackmagic_blackpill_f401cc_firmware.bin

set -e

echo "Building Black Magic Probe for BlackPill F401CC - Pinout 0"
echo "============================================================"
echo "Pinout 0:"
echo "  SWDIO = PB9"
echo "  SWCLK = PB8"
echo "  TDI   = PB6"
echo "  TDO   = PB7"
echo "  nRST  = PA5"
echo "Serial:"
echo "  TxD   = PA2 - BlackPill output"
echo "  RxD   = PA3 - BlackPill input"
echo ""

BUILD_DIR="build-f401cc-pinout0"

# Clean old build
if [ -d "$BUILD_DIR" ]; then
    echo "Cleaning old build directory..."
    rm -rf "$BUILD_DIR"
fi

# Configure meson with pinout 0 (default)
echo "Configuring build..."
meson setup "$BUILD_DIR" \
    --cross-file=cross-file/blackpill-f401cc.ini \
    -Dbmd_bootloader=true \
    -Dalternative_pinout=0

# Build firmware and bootloader
echo ""
echo "Building firmware..."
ninja -C "$BUILD_DIR"

echo ""
echo "Building bootloader..."
ninja -C "$BUILD_DIR" boot-bin

# Show results
echo ""
echo "Build complete!"
echo "==============="
echo "Bootloader: $BUILD_DIR/blackmagic_blackpill_f401cc_bootloader.bin"
ls -lh "$BUILD_DIR/blackmagic_blackpill_f401cc_bootloader.bin"
echo ""
echo "Firmware:   $BUILD_DIR/blackmagic_blackpill_f401cc_firmware.bin"
ls -lh "$BUILD_DIR/blackmagic_blackpill_f401cc_firmware.bin"
echo ""
echo "Flash with DFU:"
echo "  1. Bootloader: dfu-util -d 0483:df11 --alt 0 -s 0x08000000:leave -D $BUILD_DIR/blackmagic_blackpill_f401cc_bootloader.bin"
echo "  2. Firmware:   dfu-util -d 0483:df11 --alt 0 -s 0x08004000:leave -D $BUILD_DIR/blackmagic_blackpill_f401cc_firmware.bin"
