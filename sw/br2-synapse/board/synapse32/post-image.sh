#!/bin/sh
# Turn fw_payload.bin into the images the SoC consumes:
#   synapse.bin  raw image for 0x8000_0000 (the ARM copies it to DDR at DDR_BASE)
#   synapse.hex  32-bit word hex for the simulators (+hex=)
set -e
BIN="$BINARIES_DIR/fw_payload.bin"
cp "$BIN" "$BINARIES_DIR/synapse.bin"
python3 - "$BIN" "$BINARIES_DIR/synapse.hex" <<'PY'
import sys
data = open(sys.argv[1], "rb").read()
data += b"\0" * (-len(data) % 4)
with open(sys.argv[2], "w") as f:
    f.write("@00000000\n")
    for i in range(0, len(data), 4):
        f.write("%08x\n" % int.from_bytes(data[i:i+4], "little"))
PY
echo "synapse.bin / synapse.hex written to $BINARIES_DIR"
