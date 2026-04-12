#!/usr/bin/env bash
# flash.sh — Build and flash Merceves MotorDriver firmware
# Usage: ./flash.sh [build|flash|uf2|openocd|all]
#   build    — compile only
#   flash    — picotool load (USB, no debug probe needed)
#   uf2      — copy .uf2 to mounted Pico (hold BOOTSEL first)
#   openocd  — flash via CMSIS-DAP debug probe
#   all      — build + picotool flash  (default)

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
ELF="$BUILD_DIR/Merceves_Base.elf"
UF2="$BUILD_DIR/Merceves_Base.uf2"

# --- Resolve picotool (VSCode-installed SDK location, or PATH) ---
PICOTOOL="${PICOTOOL:-$(command -v picotool 2>/dev/null || echo "$HOME/.pico-sdk/tool/2.2.0/picotool/build/picotool")}"

# --- Resolve Pico SDK (env var or VSCode default) ---
export PICO_SDK_PATH="${PICO_SDK_PATH:-$HOME/.pico-sdk/sdk/2.2.0}"

do_build() {
    echo "==> Building firmware..."
    mkdir -p "$BUILD_DIR"
    cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -G Ninja
    ninja -C "$BUILD_DIR"
    echo "==> Build complete: $ELF"
}

do_flash_picotool() {
    if [ ! -f "$ELF" ]; then
        echo "Error: $ELF not found. Run './flash.sh build' first." >&2
        exit 1
    fi
    echo "==> Flashing via picotool (reboot into BOOTSEL if needed)..."
    "$PICOTOOL" load -fx "$ELF"
    echo "==> Done."
}

do_flash_uf2() {
    if [ ! -f "$UF2" ]; then
        echo "Error: $UF2 not found. Run './flash.sh build' first." >&2
        exit 1
    fi
    # Common mount points for the Pico in BOOTSEL mode
    for mount in /media/$USER/RPI-RP2 /Volumes/RPI-RP2 /run/media/$USER/RPI-RP2; do
        if [ -d "$mount" ]; then
            echo "==> Copying UF2 to $mount ..."
            cp "$UF2" "$mount/"
            echo "==> Done. Pico will reboot automatically."
            return
        fi
    done
    echo "Error: Pico not found in BOOTSEL mode. Hold BOOTSEL while plugging in USB." >&2
    exit 1
}

do_flash_openocd() {
    if [ ! -f "$ELF" ]; then
        echo "Error: $ELF not found. Run './flash.sh build' first." >&2
        exit 1
    fi
    echo "==> Flashing via OpenOCD (CMSIS-DAP)..."
    openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg \
        -c "adapter speed 5000" \
        -c "program $ELF verify reset exit"
    echo "==> Done."
}

# --- Main ---
CMD="${1:-all}"
case "$CMD" in
    build)   do_build ;;
    flash)   do_flash_picotool ;;
    uf2)     do_flash_uf2 ;;
    openocd) do_flash_openocd ;;
    all)     do_build && do_flash_picotool ;;
    *)
        echo "Usage: $0 [build|flash|uf2|openocd|all]"
        exit 1
        ;;
esac
