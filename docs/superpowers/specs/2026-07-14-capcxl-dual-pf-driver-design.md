# CapCXL dual-PF CXL driver design

Date: 2026-07-14

## Goal

Recognize the reflashed CapCXL image from its PCI function signature and
initialize PF0 as the Type-2 accelerator/control function and PF1 as the
Type-3 memory function.

The identifying signature is intentionally strict:

- vendor/device: `8086:0ddb` on both functions
- PF0: function 0, class `0x120000`, revision `0x02`
- PF1: function 1, class `0x050210`, revision `0x01`

All fields must match. Other `8086:0ddb` images retain their existing probe
behavior.

## Considered approaches

1. Use the PF class/revision pair. This works with the currently flashed image
   and distinguishes it from the prior two-accelerator ternary/NIC image. This
   is the selected approach.
2. Add an explicit `CAPCXL` BAR magic register. This is more extensible but
   requires another RTL build and reflash, so it is deferred.
3. Enable the existing PF0 memdev path globally. This is rejected because it
   misrepresents PF ownership and can affect non-CapCXL `0ddb` images.

## Function ownership

PF0 owns the Type-2 role:

- bind the CapCXL driver to the accelerator-class PCI function;
- discover and map the CXL component register block;
- register the CXL cache device for the Type-2 function;
- expose the CapCXL CEU control interface from BAR0;
- retain the discovered component register resource for its sibling PF1.

PF1 owns the Type-3 role:

- bind the CapCXL driver to the CXL memory-class PCI function after generic
  `cxl_pci` rejects the function for its missing Device DVSEC;
- initialize a class-memory `cxl_dev_state` and memory capacity;
- map the shared component/HDM register resource discovered through PF0;
- commit the Type-3 HDM decoder and register the CXL memdev on PF1;
- never require the legacy `TMM1` CSR.

The resulting kernel model is one Type-2 cache/control device rooted at PF0
and one Type-3 memory device rooted at PF1. The driver must not register the
Type-3 memdev under PF0.

## Probe coordination

Either PF may be presented to the driver first. Detection therefore looks up
the sibling function and validates the complete pair before selecting CapCXL
mode. PF1 initialization discovers PF0 with `pci_get_slot()`, enables both
functions as needed, and performs component register discovery through PF0.
Any borrowed physical register resource is mapped with PF1-managed lifetime;
the driver does not retain an uncounted PF0 pointer.

Repeated or deferred probes must be idempotent. PF1 returns `-EPROBE_DEFER`
when PF0 or the required root topology is not ready, rather than falling back
to a generic or tmatmul path.

## RCH topology and HDM safety

The IA-780I RCH path requires the existing `cxl_acpi` parent alias
`rch_parent_uid=3`. The driver verifies that a compatible root decoder covers
the configured HPA window before exposing the Type-3 memdev. A missing root
decoder is a probe failure with an actionable log message.

The HDM decoder must pass commit readback for global enable, committed state,
base, and size. The CapCXL path does not honor `allow_uncommitted_hdm`; failed
readback prevents memdev registration.

The driver and validation workflow do not create a region, reconfigure DAX,
or online memory automatically. Those remain explicit later experiments after
read/write routing has been proven without machine-check errors.

## Compatibility

Existing QEMU, tmatmul, ternary, and NIC behavior remains unchanged unless the
strict CapCXL signature matches. Existing module parameters remain available
for those paths. CapCXL role selection is automatic once the pair matches; it
does not depend on the global `enable_memdev` or `enable_cache` defaults.

## Verification

The implementation will be test-first:

1. Add a focused detector/role test covering exact match and every mismatch
   dimension before adding production role selection.
2. Build the CXL modules against the running `6.18.0-rc5` tree.
3. Install and reload only the rebuilt CapCXL/CXL modules with
   `cxl_acpi.rch_parent_uid=3`.
4. Verify PF0 and PF1 driver binding, Type-2 cache registration, PF1 memdev and
   endpoint creation, HDM commit readback, and absence of CXL/AER fatal errors.
5. Stop before region creation, DAX reconfiguration, or memory online.

Success means PF0 is represented as Type-2, PF1 owns the Type-3 memdev, and
the kernel log contains no legacy `TMM1` requirement or DVSEC range-decode
failure for the CapCXL path.
