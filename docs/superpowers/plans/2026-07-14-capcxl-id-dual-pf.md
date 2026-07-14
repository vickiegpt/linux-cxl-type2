# CapCXL ID and Dual-PF Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an explicit CapCXL BAR identity, preserve CEU/SAT/HDM behavior, and bind PF0 as Type-2 and PF1 as Type-3.

**Architecture:** The CAFU router serves two read-only identity registers ahead of CEU routing. The driver reads the PF0 identity, validates the PF pair, registers a cachedev on PF0, and registers a memdev on PF1 using PF0's shared component-register locator.

**Tech Stack:** SystemVerilog, Verilator, Quartus Prime Pro 25.1, Linux 6.18 CXL core, Kbuild, C host tests.

## Global Constraints

- Identity magic is `0x43415043584c0001` at PF0 BAR0 `+0x1c0ff0`.
- Capability value is `0x0f` at PF0 BAR0 `+0x1c0ff8`.
- PCI consistency check remains PF0 `120000/rev02`, PF1 `050210/rev01`.
- Non-CapCXL `8086:0ddb` images retain their existing behavior.
- No region creation, DAX reconfiguration, DAX access, or memory online.

---

### Task 1: RTL Identity and Router Regression

**Files:**
- Create: `/root/CapCXL/rtl/tb/tb_capcxl_cafu_avmm_router.sv`
- Modify: `/root/CapCXL/rtl/capcxl_cafu_avmm_router.sv`

**Interfaces:**
- Produces read-only `CAPCXL_ID` and `CAPCXL_CAPS` registers.
- Preserves component capability-array, HDM decoder, CEU, and SAT routing.

- [ ] Write a router testbench that reads `0x1c0ff0` and `0x1c0ff8`, then checks component capability headers, HDM write/read/commit, CEU routing, and SAT routing.
- [ ] Compile and run it with:

```bash
/home/victoryang00/tools/verilator/bin/verilator --binary --timing -Wall -Wno-fatal --top-module tb_capcxl_cafu_avmm_router rtl/capcxl_cafu_avmm_router.sv rtl/tb/tb_capcxl_cafu_avmm_router.sv
./obj_dir/Vtb_capcxl_cafu_avmm_router
```

Expected RED: the first identity read returns zero.

- [ ] Add these router constants and selects:

```systemverilog
localparam logic [21:0] CAPCXL_ID_ADDR   = 22'h1C0FF0;
localparam logic [21:0] CAPCXL_CAPS_ADDR = 22'h1C0FF8;
localparam logic [63:0] CAPCXL_ID        = 64'h43415043584C0001;
localparam logic [63:0] CAPCXL_CAPS      = 64'h000000000000000F;
wire capcxl_id_sel = ip_address == CAPCXL_ID_ADDR;
wire capcxl_caps_sel = ip_address == CAPCXL_CAPS_ADDR;
```

Give identity reads priority over CEU reads and exclude both addresses from `ceu_read`/`ceu_write`.

- [ ] Rerun the test and require `tb_capcxl_cafu_avmm_router: PASS`.

---

### Task 2: CEU and SAT Functional Regressions

**Files:**
- Use: `/root/CapCXL/rtl/tb/tb_capability_extension_unit.sv`
- Create: `/root/CapCXL/rtl/tb/tb_capcxl_access_control.sv`

**Interfaces:**
- CEU test covers key provisioning, token generation, verification, shadow bounds, and revocation.
- SAT test covers entry programming, exact-match allow, permission denial, host mismatch, and range rejection.

- [ ] Run the existing CEU test with Verilator and record its current result.
- [ ] Write the SAT test before changing SAT RTL; drive the existing configuration and lookup interfaces directly.
- [ ] Run both tests. Fix only regressions exposed by the tests and keep the production interfaces unchanged.
- [ ] Require both tests to print PASS with no assertions or Verilator errors.

---

### Task 3: Driver Identity Test

**Files:**
- Create: `/home/victoryang00/cxl/drivers/cxl/capcxl_identity.h`
- Create: `/home/victoryang00/cxl/tools/testing/cxl/capcxl_identity_test.c`

**Interfaces:**
- Produces `capcxl_identity_matches()` and `capcxl_role_for_pair()` over primitive identity values.

- [ ] Write a host test covering exact magic/caps/PF match, bad magic, missing each capability bit, wrong class, wrong revision, wrong function, and wrong vendor/device.
- [ ] Compile it before the header exists and confirm the expected missing-header RED failure.
- [ ] Implement the dependency-free header with roles `NONE`, `TYPE2`, and `TYPE3`.
- [ ] Run:

```bash
gcc -std=c11 -Wall -Wextra -Werror tools/testing/cxl/capcxl_identity_test.c -o /tmp/capcxl_identity_test
/tmp/capcxl_identity_test
```

Expected GREEN: `capcxl_identity_test: PASS`.

---

### Task 4: PF0 Type-2 and PF1 Type-3 Driver Roles

**Files:**
- Modify: `/home/victoryang00/cxl/drivers/cxl/cxl_type2_accel.c`

**Interfaces:**
- PF0 maps BAR0 identity, validates both PFs, discovers component registers, and registers only a cachedev.
- PF1 validates through PF0, copies the physical component map with PF1 devres ownership, commits HDM, and registers only a memdev.

- [ ] Add a read-only PF0 BAR identity helper using `pci_iomap_range()` and `readq()`; unmap immediately.
- [ ] Add sibling lookup with balanced `pci_dev_put()` on every path.
- [ ] For CapCXL PF0, enable the device, set cache defaults, register cachedev, skip tmatmul and memdev setup, and log the Type-2 role.
- [ ] For CapCXL PF1, build `cxl_memdev_state`, set 4 GiB volatile DPA, discover component registers through PF0 but set `map.host = &pf1->dev`, set `skip_dvsec_range_decode`, require RCH topology, force-commit HDM, and register the memdev on PF1.
- [ ] Keep the current generic/QEMU/tmatmul path for identity mismatches.
- [ ] Build with:

```bash
make -j$(nproc) M=drivers/cxl cxl_type2_accel.ko
```

Expected: successful `LD [M] drivers/cxl/cxl_type2_accel.ko`, vermagic `6.18.0-rc5`, and no `type2_common` dependency.

---

### Task 5: Static Checks and Fresh SOF

**Files:**
- Modify: `/root/CapCXL/tools/verify_cxl_x8x8.py`
- Generate: `/root/CapCXL/output_files/ia780i_golden_top_bbrev1.sof`

**Interfaces:**
- Static verifier requires the ID, capability bits, CEU exclusion, and dual-PF driver contract.

- [ ] Add failing static checks before changing expected source markers.
- [ ] Run all RTL tests, driver identity test, driver module build, and `python3 tools/verify_cxl_x8x8.py`.
- [ ] Build synthesis, fitter, and assembler with:

```bash
export LM_LICENSE_FILE=/opt/altera_pro/25.1/lic_qsim_24_any.dat
/opt/altera_pro/25.1/quartus/bin/quartus_syn ia780i_golden_top_bbrev1 -c ia780i_golden_top_bbrev1
/opt/altera_pro/25.1/quartus/bin/quartus_fit ia780i_golden_top_bbrev1 -c ia780i_golden_top_bbrev1
/opt/altera_pro/25.1/quartus/bin/quartus_asm ia780i_golden_top_bbrev1 -c ia780i_golden_top_bbrev1
```

- [ ] Verify report success lines, SOF timestamp, size, and SHA256 before programming.

---

### Task 6: Reflash and Safe Live Validation

**Files:**
- Install: `/lib/modules/6.18.0-rc5/updates/cxl_type2_accel.ko`
- Create: `/etc/modprobe.d/capcxl.conf`

**Interfaces:**
- Consumes the fresh SOF and driver module.
- Produces PF0 Type-2 cachedev and PF1 Type-3 memdev/endpoint proof.

- [ ] Use `jtagconfig` to select only a currently reachable cable and program only the freshly hashed SOF.
- [ ] After reboot, read PF0 BAR identity and capabilities before loading the custom driver.
- [ ] Reload `cxl_acpi` with `rch_parent_uid=3`, then load the fresh driver.
- [ ] Verify both PCI bindings, cachedev, PF1-owned memdev, endpoint decoder, HDM readback, CEU ID, and absence of CXL/AER fatal errors.
- [ ] Install the proven module and persist `options cxl_acpi rch_parent_uid=3`.
- [ ] Stop without creating a CXL region, DAX device, or online memory.
