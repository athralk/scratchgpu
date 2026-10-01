#!/usr/bin/env bash
# Copy synthesizable RTL from this WSL repo to the Windows Vivado workspace.
# Vivado scripts in D:/ScratchGPU_Vivado read only from its rtl/ folder.
set -euo pipefail

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
dest="${VIVADO_DIR:-/mnt/d/ScratchGPU_Vivado}"

mkdir -p "$dest/rtl/cpu"
rsync -a --delete --include='*/' --include='*.v' --include='*.vh' --include='*.sv' --exclude='*' \
    "$repo/cpu/rtl/" "$dest/rtl/cpu/"
if [ -d "$repo/gpu/rtl" ]; then
    mkdir -p "$dest/rtl/gpu"
    rsync -a --delete --include='*/' --include='*.v' --include='*.vh' --include='*.sv' --exclude='*' \
        "$repo/gpu/rtl/" "$dest/rtl/gpu/"
fi
# Board software: ARM bridge and the default Synapse image (the demo)
mkdir -p "$dest/sw"
[ -f "$repo/sw/arm_bridge/bridge.elf" ] && cp "$repo/sw/arm_bridge/bridge.elf" "$dest/sw/"
[ -f "$repo/sw/demo/demo.bin" ] && cp "$repo/sw/demo/demo.bin" "$dest/sw/"
echo "Synced RTL to $dest/rtl and software to $dest/sw"
