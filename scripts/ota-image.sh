#!/usr/bin/env bash
# Build the mesh node firmware as a raw image for `mise run ota-upload`.
#
# The image replaces the node's whole application, so build it the way
# the node was built: the same ADDRESS (compiled in) and LORA_PRESET in
# the environment, the same cargo features as arguments (for example
# --features board), and a FW_VERSION above the one the node runs. Give
# ota-upload that FW_VERSION as -v: the node refuses an offer that is not
# newer than the firmware it runs.
#
# The raw image starts at the app's link address, 0x08004000
# (memory-app.x), the active slot the bootloader moves a received image
# into. objcopy is the llvm-objcopy from the toolchain's llvm-tools
# component (rust-toolchain.toml), so nothing outside rustup is needed.
set -euo pipefail

cargo build --release "$@"

host=$(rustc -vV | sed -n 's/^host: //p')
objcopy="$(rustc --print sysroot)/lib/rustlib/$host/bin/llvm-objcopy"
out=target/firmware.bin
"$objcopy" -O binary target/thumbv7em-none-eabi/release/sx1262-mesh-rs "$out"
echo "$out: $(stat -c %s "$out") bytes, ADDRESS=${ADDRESS:-1} FW_VERSION=${FW_VERSION:-1}"
