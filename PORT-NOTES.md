# aipu driver — kernel 7.0 port notes (branch `aipu-7.0-kernel-port`)

Upstream: `cixtech/cix_opensource__npu_driver` @ `cix_p1_k6.6_master` (25b55cc "CIX P1 2026Q2 RC4 RELEASE").

The driver originally targets the CIX vendor kernel (6.6-era, `CONFIG_ARCH_CIX`).
This branch ports it to build against **mainline/Ubuntu generic kernels 7.x**
(e.g. `7.0.0-29-generic`) so the Zhouyi AIPU can be used without switching
kernels. The port was verified by compiling the full module against a 7.1.5
kernel tree (only arm64 cache-maintenance inline asm fails to link on that
x86_64 host — it assembles fine on the aarch64 board).

## Changes

| File | 7.x API migration |
|---|---|
| `driver/armchina-npu/sky1/sky1.c` | SCMI perf-domain helpers removed in 7.x: `scmi_device_set_freq()` → `dev_pm_opp_set_rate()`, `scmi_device_get_freq()` → `dev_pm_opp_find_freq_ceil()`; `scmi_device_opp_table_parse()` dropped (OPP table is auto-populated by the perf domain's `attach_dev`); ACPI perf-domain attach via `fwnode_property_match_string()` + `genpd_dev_pm_attach_by_id()`; include `<linux/pm_opp.h>` + `<linux/property.h>` instead of `<linux/scmi_protocol.h>` |
| `driver/armchina-npu/sky1/sky1.c` | `pm_runtime_put()` returns `void` since 6.13 — drop return-value handling |
| `driver/armchina-npu/sky1/sky1.c` | `platform_driver::remove` returns `void` since 6.11 — version-guarded signature (`KERNEL_VERSION(6,11,0)`), keeps 6.6-era `int` form for older kernels |
| `driver/armchina-npu/aipu_mm.c` | `MAX_ORDER` renamed to `MAX_PAGE_ORDER` (≥6.8) — `AIPU_MAX_ORDER` compat macro |
| `driver/armchina-npu/aipu_mm.c` | `pgprot_dmacoherent()` moved out of the default include chain — `#ifndef` fallback to `pgprot_noncached()` |
| `driver/armchina-npu/aipu_dma_buf.c` | `MODULE_IMPORT_NS(DMA_BUF)` → `MODULE_IMPORT_NS("DMA_BUF")` (macro no longer stringifies; quoted form works on both old and new kernels) |
| `driver/Makefile` | `EXTRA_CFLAGS` removed from kbuild → `ccflags-y`; `-I$(PWD)/` → `-I$(M)/` (PWD resolves to the kernel dir under `make -C` and the flags were silently dropped, which is why headers were never found) |
| `driver/dkms.conf` | Dropped the `CONFIG_ARCH_CIX` `BUILD_EXCLUSIVE_KERNEL` gate so DKMS builds on generic kernels |

## Build (board, aarch64, Ubuntu 7.0.0-29-generic)

```sh
cd driver
make -C /lib/modules/$(uname -r)/build M=$(pwd) \
     BUILD_AIPU_VERSION_KMD=BUILD_ZHOUYI_V3 \
     BUILD_TARGET_PLATFORM_KMD=BUILD_PLATFORM_SKY1 \
     BUILD_NPU_DEVFREQ=y \
     COMPASS_DRV_BTENVAR_KMD_VERSION=6.0.1 modules
```

or via DKMS: `./board-install.sh` (builds, installs, modprobes, checks /dev/aipu).

## Still expected on first load

- `cix-noe-umd` (userspace runtime deb) has an unrelated python `>=3.10,<3.14`
  postinst incompatibility on the board's python 3.14 — it does not block the
  kernel module.
- Firmware blobs: the NPU needs its firmware (sky1-firmware); if `modprobe`
  reports missing firmware, install `sky1-firmware` from the sky1-linux apt repo.
