#!/bin/bash
# Build + install the aipu (Zhouyi AIPU) kernel module for kernel 7.x via DKMS.
# Run on the board (aarch64). Assumes linux-headers-$(uname -r) is installed.
set -euo pipefail

VER="6.0.1"
SRC="$(cd "$(dirname "$0")" && pwd)/driver"

if [ ! -d "/lib/modules/$(uname -r)/build" ]; then
    echo "ERROR: kernel headers for $(uname -r) not installed" >&2
    exit 1
fi

echo "== removing any stale DKMS registration =="
sudo dkms remove -m aipu -v "$VER" --all 2>/dev/null || true
sudo rm -rf "/usr/src/aipu-$VER"

echo "== copying source to /usr/src =="
sudo cp -r "$SRC" "/usr/src/aipu-$VER"
sudo rm -f "/usr/src/aipu-$VER/"*.o "/usr/src/aipu-$VER/"*.ko "/usr/src/aipu-$VER/"*.mod* 2>/dev/null || true
sudo rm -rf "/usr/src/aipu-$VER/armchina-npu/"*.o "/usr/src/aipu-$VER/armchina-npu/"*.ko 2>/dev/null || true

echo "== dkms add/build/install =="
sudo dkms add -m aipu -v "$VER"
sudo dkms build -m aipu -v "$VER" --no-prepare-kernel
sudo dkms install -m aipu -v "$VER" --no-prepare-kernel

echo "== load =="
sudo modprobe aipu || true
sleep 1
if [ -e /dev/aipu ]; then
    echo "OK: /dev/aipu present"
    ls -la /dev/aipu
else
    echo "WARN: /dev/aipu not present; check 'dmesg | grep -i aipu'" >&2
    dmesg | grep -i "aipu\|armchina" | tail -10 || true
    exit 1
fi
