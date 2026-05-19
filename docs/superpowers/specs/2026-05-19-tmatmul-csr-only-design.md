# tmatmul smoke-test rewrite: CSR-only ioctl + userspace devdax buffer

**Status:** approved
**Date:** 2026-05-19
**Author:** vickieGPT (with Claude)
**Scope:** `drivers/cxl/cxl_type2_accel.c`, `include/uapi/linux/cxl_type2_accel.h`, `tools/testing/cxl/tmatmul_type2_run.c`

## Problem

The current tmatmul smoke-test ioctl (`CXL_TYPE2_TMATMUL_RUN`) hard-codes a host
physical address (HPA `0x4080000000`) and performs ~1 MiB of `memset`/`memcpy`
writes there from kernel context. On the IA-780I development system that HPA
is already inside `CXL Window 0`, region `region13` is in `mode=ram` (promoted
to System RAM via dax kmem), and the standard CXL region attach for this
device's endpoint failed (`cxl_port endpoint8: failed to attach decoder8.0 to
region0: -6`). The result:

- `memremap()` returns a kernel virtual address backed by the System-RAM direct
  map, not an MMIO mapping.
- `memset(addr, 0, 1 MiB)` writes into pages the page allocator may also be
  using.
- Writes ride a CXL.mem path the device does not fully service, so the CPU
  wedges on a transaction that never completes.
- The platform eventually resets the host without leaving any panic trace
  (also helped by `pci=noaer` on the boot cmdline).

Two earlier patches (PF1 enable in driver, HDM Global Decoder Enable in the
force-commit fallback) are necessary but do not solve this: even with both, the
hard-coded HPA still aliases System RAM and the kernel still corrupts its own
pool.

## Goal

Eliminate kernel-side CXL.mem writes from the smoke path. After this change,
the kernel only sequences CSRs; userspace owns all buffer prep and result
verification through a devdax mmap.

A broken device CXL.mem responder will then manifest as a userspace hang the
user can `kill -9`, not a kernel wedge.

## Non-goals

- **Not fixing** the underlying device-side CXL.mem responder. If the device is
  broken, the smoke still fails — it just fails safely.
- **Not preserving** the legacy `CXL_TYPE2_TMATMUL_RUN` ioctl. UAPI version
  bumps from 1 to 2; existing callers (the `tmatmul_type2_run` userspace tool
  in this tree) are updated atomically. There are no out-of-tree callers.
- **Not adding** a new CXL region or new HDM decoder. The driver's existing
  force-commit of HDM Decoder 0 stays as-is.

## Architecture

```
┌─ userspace ─────────────────────────────┐    ┌─ kernel ──────────────┐
│ tmatmul_type2_run                        │    │ cxl_type2_accel       │
│  ├─ auto-discover /dev/daxN.M            │    │                       │
│  ├─ open /dev/dax* ── mmap(MAP_SHARED) ──┼────│   /dev/daxN.M devdax  │
│  │   ↓ entire dax window size            │    │                       │
│  │  lay out matrix / input / output /    │    │                       │
│  │  program at fixed DPAs                │    │                       │
│  │   ↓                                   │    │                       │
│  ├─ msync(MS_SYNC)                       │    │                       │
│  ├─ open /dev/cxl_tmatmul*               │    │   /dev/cxl_tmatmul*   │
│  ├─ ioctl(RUN_CSR_ONLY, timeout_ms) ─────┼────┼─→ reset → start →     │
│  ├─ ← {dma_status, stall_status, ...}    │    │     poll CSRs only    │
│  └─ readback mmap → verify output zero   │    │   no CXL.mem touched  │
└──────────────────────────────────────────┘    └───────────────────────┘
```

## UAPI changes (`include/uapi/linux/cxl_type2_accel.h`)

Bump version constant:

```c
#define CXL_TYPE2_TMATMUL_UAPI_VERSION  2
```

**Delete:**

- `CXL_TYPE2_TMATMUL_RUN` ioctl number
- `struct cxl_type2_tmatmul_run`
- `CXL_TYPE2_TMATMUL_RUN_SMOKE` flag
- `CXL_TYPE2_TMATMUL_RESULT_OUTPUT_ZERO` flag
- `default_hpa_base` and `default_hpa_size` fields from `struct
  cxl_type2_tmatmul_info`. Replace the two `__u64` slots with `__u64
  reserved2[2]` so the struct length is preserved (the version field
  distinguishes layouts).

**Keep:**

- `struct cxl_type2_tmatmul_info` and `CXL_TYPE2_TMATMUL_GET_INFO` ioctl
- `CXL_TYPE2_TMATMUL_RESULT_STALLED`, `CXL_TYPE2_TMATMUL_RESULT_DMA_ERROR`

**Add:**

```c
struct cxl_type2_tmatmul_csr_run {
    /* in */
    __u32 timeout_ms;
    __u32 flags;           /* reserved, must be 0 */
    /* out */
    __u32 dma_status;
    __u32 stall_status;
    __u32 instr_count;
    __u32 dim_d;
    __u32 result_flags;    /* STALLED | DMA_ERROR */
    __u32 reserved0;
    __u64 reserved1[4];
};

#define CXL_TYPE2_TMATMUL_RUN_CSR_ONLY \
    _IOWR(CXL_TYPE2_TMATMUL_IOC_MAGIC, 0x02, struct cxl_type2_tmatmul_csr_run)
```

Ioctl number 0x01 is retired with no replacement (skipping it makes the change
visible to anyone diffing UAPI headers).

## Kernel driver changes (`drivers/cxl/cxl_type2_accel.c`)

### Remove

- `#include <asm/cacheflush.h>`
- Module parameters `tmatmul_hpa_base` and `tmatmul_hpa_size`
- `default_hpa_base` and `default_hpa_size` from `struct cxl_type2_tmatmul_dev`
- Helpers used only by the deleted path:
  - `tmatmul_range_ok`
  - `tmatmul_memremap`
  - `tmatmul_flush_mapping`
  - `tmatmul_zero_hpa`
  - `tmatmul_pattern_hpa`
  - `tmatmul_write_hpa`
  - `tmatmul_fill_input_vector`
  - `tmatmul_output_is_zero`
  - `tmatmul_encode_instr`
  - `tmatmul_build_smoke_program`
  - `tmatmul_upload_smoke_payload`
  - `tmatmul_launch_smoke`
- `TMATMUL_PROGRAM_BYTES`, `TMATMUL_MAX_MATRIX_BYTES`, `TMATMUL_DDR_*_ADDR`,
  `TMATMUL_UNCACHED_MEMREMAP_FLAGS`, `TMATMUL_WB_MEMREMAP_FLAGS` macros — all
  reference data layout that is now userspace's concern
- `default_hpa_base`/`default_hpa_size` from the `tmatmul ready: ...` info
  message in `cxl_type2_tmatmul_init`

### Add

```c
static int tmatmul_launch_csr_only(struct cxl_type2_tmatmul_dev *tmatmul,
                                   struct cxl_type2_tmatmul_csr_run *run)
{
    unsigned long deadline;
    u32 timeout_ms = run->timeout_ms ?: 5000;
    u32 num_instances;
    int rc = 0;

    if (run->flags)
        return -EINVAL;

    num_instances = tmatmul_rd32(tmatmul, TMATMUL_REG_NUM_INSTANCES);
    if (!num_instances)
        return -ENODEV;

    run->dim_d = tmatmul_rd32(tmatmul, TMATMUL_REG_DIM_D);

    mutex_lock(&tmatmul->lock);

    /* Reset the engine. Userspace has already written its instruction
     * program into the device-attached memory at the agreed-upon DPA.
     */
    tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_STALL_CLEAR), 1);
    tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_RST_TRIGGER), 1);
    msleep(50);

    /* Kick the instruction fetch from the well-known DPA. */
    tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_SRC_LO),
                 lower_32_bits(TMATMUL_DDR_INSTR_DPA));
    tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_SRC_HI),
                 upper_32_bits(TMATMUL_DDR_INSTR_DPA));
    tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_LEN),
                 TMATMUL_PROGRAM_BYTES);
    tmatmul_wr32(tmatmul, tmatmul_inst_off(0, TMATMUL_INST_INSTR_START), 1);

    deadline = jiffies + msecs_to_jiffies(timeout_ms);
    do {
        run->dma_status = tmatmul_rd32(tmatmul,
            tmatmul_inst_off(0, TMATMUL_INST_INSTR_START)) & 0xff;
        run->stall_status = tmatmul_rd32(tmatmul,
            tmatmul_inst_off(0, TMATMUL_INST_STALL_STATUS));
        run->instr_count = tmatmul_rd32(tmatmul,
            tmatmul_inst_off(0, TMATMUL_INST_DBG_INSTR_CNT));

        if (run->stall_status) {
            run->result_flags |= CXL_TYPE2_TMATMUL_RESULT_STALLED;
            goto out_unlock;
        }
        if (run->dma_status == TMATMUL_DMA_ERROR) {
            run->result_flags |= CXL_TYPE2_TMATMUL_RESULT_DMA_ERROR;
            rc = -EIO;
            goto out_unlock;
        }
        usleep_range(1000, 2000);
    } while (time_before(jiffies, deadline));

    rc = -ETIMEDOUT;
out_unlock:
    mutex_unlock(&tmatmul->lock);
    return rc;
}
```

Two new constants:

```c
#define TMATMUL_DDR_INSTR_DPA   0x00300000ULL  /* matches userspace layout */
#define TMATMUL_PROGRAM_BYTES   (6 * 16)
```

(Same numeric values as today. They define the **kernel/userspace contract**
for where userspace must place the program, so they stay in the driver.)

### Modify

`cxl_type2_tmatmul_ioctl` keeps the `GET_INFO` case unchanged; replaces the
`RUN` case with `RUN_CSR_ONLY` calling the new handler.

`cxl_type2_tmatmul_init` drops `hpa_base`/`hpa_size` from its info log line.

## Userspace tool changes (`tools/testing/cxl/tmatmul_type2_run.c`)

### Flags

- **Drop**: `--hpa-base`, `--hpa-size`, `--apply-setpci`, `--pf1`.
- **Keep**: `--dev PATH` (default `/dev/cxl_tmatmulad000`), `--timeout-ms`, `--help`.
- **Add**: `--dax PATH` (default: auto-discover).

### Auto-discovery algorithm

```
resolve dev_pci_path:
    real = realpath(/sys/class/misc/<basename(dev_path)>/device)
    # real is e.g. /sys/devices/pci0000:ac/0000:ac:08.0/0000:ad:00.0
    return real

find memdev:
    for entry in /sys/bus/cxl/devices/mem*:
        if realpath(entry/dev/<parent>) starts with dev_pci_path: yield entry

find dax candidates:
    matches = []
    for memdev in find memdev():
        # walk regions this memdev backs
        for region_link in glob(memdev/region*):
            region = realpath(region_link)
            # under each region, look for a dax_region child
            for dax_region in glob(region/dax_region*):
                for dax in glob(dax_region/dax*.*):
                    # subsystem link should resolve to .../bus/dax
                    if basename(realpath(dax/subsystem)) == "dax":
                        matches.append("/dev/" + basename(dax))
    return matches

main_resolve_dax:
    if --dax PATH provided: return PATH
    matches = find dax candidates()
    if len(matches) == 1: return matches[0]
    if len(matches) == 0: die with setup-instructions message (see below)
    die with "multiple candidates: ..., pass --dax"
```

**Zero-match error message** prints the exact commands to run:

```
no devdax device found for /dev/cxl_tmatmulad000

Required setup:
    sudo modprobe device_dax
    sudo daxctl reconfigure-device dax0.0 --mode=devdax --force
    ls /dev/dax*
```

### Buffer mmap

```
read dax_size from /sys/bus/dax/devices/<basename(dax_path)>/size
fd = open(dax_path, O_RDWR)
addr = mmap(NULL, dax_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0)
```

Full window. Virtual allocation only — physical pages fault lazily on first
access. The smoke uses ~3 MiB at the low end of the window for D=2048;
remainder of the mmap is never touched.

### Buffer layout (DPAs are device-physical offsets within the dax window)

| DPA            | Length            | Contents                                  |
|----------------|-------------------|-------------------------------------------|
| `0x00000000`   | `dim_d² / 4` B    | matrix region, zeroed                     |
| `0x00100000`   | `dim_d * 2` B     | input vector, `__le16` 0x0100 (Q8.8 1.0)  |
| `0x00200000`   | `dim_d * 2` B     | output region, byte pattern 0xa5          |
| `0x00300000`   | `96` B            | 6-instruction program                     |

For `dim_d = 2048`: matrix is exactly 1 MiB.

### Instruction encoding (ported verbatim from kernel)

`encode_smoke_program(uint8_t prog[96], uint32_t dim_d)` builds the same 6
instructions the kernel's `tmatmul_build_smoke_program` builds today. The bit
layout matches `tmatmul_encode_instr`:

```
word[0] = addr  (DPA, little-endian)
word[1] = (rms & 0x7)
       | ((tm & 0x3) << 3)
       | ((ls & 0x3) << 5)
       | ((va & 0x7) << 7)
       | ((vb & 0x7) << 10)
       | ((vy & 0x7) << 13)
       | ((op & 0xf) << 16)
       | ((fu & 0x7) << 20)   (little-endian)
```

The 6 instructions and their fields are unchanged from the kernel's current
`tmatmul_build_smoke_program`.

### Run flow

```c
1. dax_path = resolve_dax(argv);
2. open(/dev/cxl_tmatmul*) → fd_dev
3. ioctl(fd_dev, GET_INFO, &info);    /* dim_d, dev_id sanity */
4. dax_size = read_sysfs_size(dax_path);
5. fd_dax  = open(dax_path, O_RDWR);
6. base = mmap(NULL, dax_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd_dax, 0);
7. memset(base + 0x000000, 0,    matrix_len);
8. fill_input_vector(base + 0x100000, dim_d);
9. memset(base + 0x200000, 0xa5, vector_len);
10. encode_smoke_program(base + 0x300000, dim_d);
11. msync(base, 0x300000 + 96, MS_SYNC);
12. ioctl(fd_dev, RUN_CSR_ONLY, &run);
13. print_run(&run);
14. if (rc == 0 && !(run.result_flags & STALLED)):
        FAIL "tmatmul did not stall before timeout"
    if (run.result_flags & DMA_ERROR):
        FAIL "DMA error reported"
    /* Verify output buffer */
    msync(base + 0x200000, vector_len, MS_INVALIDATE);  /* discard CPU caches */
    for i in [0, vector_len):
        if (((uint8_t*)base)[0x200000 + i] != 0):
            FAIL "output non-zero at offset 0x%x: 0x%02x"
    PASS
```

## Pre-test setup (documented in the tool's header comment and in error output)

```bash
# One-time per boot, before running the tool:
sudo modprobe device_dax
sudo daxctl reconfigure-device dax0.0 --mode=devdax --force
ls -l /dev/dax0.0                          # confirm device exists
grep -E "System RAM|region" /proc/iomem    # confirm region is no longer System RAM
```

If `daxctl` is not installed: `sudo apt install daxctl-utils` (Debian/Ubuntu)
or distribution equivalent.

## Error matrix

| Symptom                                          | Reported by                  | Likely cause                                  |
|--------------------------------------------------|------------------------------|-----------------------------------------------|
| `no devdax device found …`                       | userspace, before any ioctl  | dax not configured; run setup steps           |
| `multiple candidates: … pass --dax`              | userspace, before any ioctl  | >1 CXL device; disambiguate manually          |
| `open(dax_path)` fails with EACCES               | userspace, before any ioctl  | not running as root or wrong perms            |
| `mmap` fails                                     | userspace, before any ioctl  | dax size larger than VA space; very rare      |
| `ioctl GET_INFO` returns `dev_id != "TMM1"`      | userspace                    | wrong miscdev / BAR not mapped                |
| `ioctl RUN_CSR_ONLY` returns `-ETIMEDOUT`        | userspace                    | device CSRs not progressing                   |
| `result_flags & DMA_ERROR`                       | userspace                    | DMA engine in device errored out              |
| `result_flags & STALLED` and output all zero     | userspace                    | **PASS**                                      |
| `result_flags & STALLED` and output non-zero     | userspace                    | device computed wrong result                  |
| userspace `msync` or output read hangs           | userspace (kill -9 ok)       | device CXL.mem responder broken; not kernel  |

## What this does and does not fix

**Fixes:**

- Kernel never writes to CXL.mem from the smoke path. The System-RAM-aliased
  HPA bug is structurally gone.
- A broken device CXL.mem responder cannot wedge the kernel via the smoke
  path. The userspace tool may hang, but the host stays up.
- UAPI is simpler; CSR-only ioctl is easy to reason about.

**Does not fix:**

- If the device's CXL.mem responder is broken end-to-end, `msync` from
  userspace will still hang (on the now-unblockable cache write-back). User
  can `kill -9` the tool.
- The standard CXL region attach failure (`failed to attach decoder8.0 to
  region0: -6`) is unchanged. That's a separate investigation tracked
  outside this design.
- The IA-780I-specific question of whether MEM_ACTIVE handshake works
  correctly is unchanged.

## Testing matrix

| Test                                                                    | Expected                              |
|-------------------------------------------------------------------------|---------------------------------------|
| Build kernel module: `make modules`                                     | clean, no warnings                    |
| Build userspace: `gcc … tmatmul_type2_run.c -o /tmp/tmatmul_type2_run`  | clean, no warnings                    |
| Boot, `insmod cxl_type2_accel.ko`                                       | dmesg PF1+HDM lines as today          |
| `tmatmul_type2_run` without devdax setup                                | clean "no devdax device" error, setup steps printed |
| Configure devdax, `tmatmul_type2_run` once                              | PASS or specific failure              |
| `tmatmul_type2_run` 100 iterations                                      | no kernel taint, no oops; host alive  |
| Run with `--dax` pointing at unrelated dax device                       | tool runs but DMA_ERROR/STALLED output non-zero (device sees garbage); kernel survives |

## File-by-file summary

```
include/uapi/linux/cxl_type2_accel.h    | ~40 lines changed
drivers/cxl/cxl_type2_accel.c           | ~250 lines removed, ~60 added
tools/testing/cxl/tmatmul_type2_run.c   | rewritten (~211 → ~350 lines)
docs/superpowers/specs/...md            | this file
```
