# The driver as a loadable kernel module: what it needs

Date: 2026-10-21. Status: notes only. No code until the module design has been reviewed and its
spec exists. Today the driver links statically into each kernel cut.

## Where it sits in the design

- **Class `root`, placement `resident`.** The Gayle IDE can hold the root disk, so it is a root-class module.
  It runs in place in the module slot inside the kernel's `.data`, with its `.bss` in the slot. The memory
  semantics are today's static ones: kernel virtual == physical, and the PIO buffers are physical addresses
  from sd.
- **Loaded before sd `init`.** The root-module pass registers it with `sd_register(pn, queuefn, name,
  probefn, rankkey)`. After sd `init` the call fails (`EBUSY`).
- **Rank key: last.** The order is the Z3660, then the Zorro boards by address, then the IDE. That is
  today's `ide_sd_key` 0xfffffff0 from the 040/060 port's `ide_sd_glue040.s`, so card numbers do not change.
  Until the four-controller sd layout lands, a third controller is dropped with a console line. With that layout,
  the IDE beside a Z3660 and an A4092 gets a card of its own.

## What changes in the driver

| today (static) | module form |
|---|---|
| `driver.conf`: synthetic product number 0xFFFF1DE0, `idequeue`, `probe=idepresent`; the build hub generates the sd row | the same three values come from `.moddata` (`MODREV` 2) and go to `sd_register`; the row is filled only when the match gate passes |
| `ide_hooked()` searches `int2_tbl[]` for `ideintr` (`src/ide.c:875`) and runs polled when the hook is missing; on the 030 line `kernel-patches/kernel.c.patch` adds the hook | `intr_attach(2, ideintr)` at attach; polled when the call is absent or refuses. `int2_tbl` is never exported to modules, so the search and the kernel.c patch go. The Gayle is the first user of `intr_attach` |
| tunables are patched by symbol per cut: `ide_slave_enable`, `ide_intr_enable`, `ide_debug`, `ide_master_unit`, `ide_slave_unit`, `ide_spins` | typed module parameters with today's defaults, overridden locally and readable at run time. The per-cut byte patch (the 040/060 CD kernel's `ide_slave_enable` 0 -> 2) goes away |
| two build variants: 030 `ide.o`; 040/060 `ide_040.o` (`-DIDE_CACHE=1 -DIDE_NBPP=4096`) | two kmod recipes in this repo (one per CPU line) on the shared harness, plus a `kind: kmod` registry entry |
| no halt hook | a halt pass entry: disable the Gayle interrupt (`ide_irq_enable(bd, 0)`) and leave both drives idle before a reboot |
| no unload | pinned (default). It is a root module, so it never unloads |

## Imports and exports

- **Imports today** (`nm -u` on `ide_040.o` bdcd9072): `int2_tbl`, `printf`, `timeout`. In module form
  `int2_tbl` becomes `intr_attach`. `printf` and `timeout` must be on the module allowlist. The data-cache
  push uses inline `cpushl`/CACR reads and imports nothing.
- **Exports:** `idequeue`, `idepresent`, `ideintr` (through the registrations, not by name), the
  parameters, and the counters (`ide_resets`, `ide_atapi_*`, `ide_pagecross_n`,
  `ide_dup_submits`, `ideunits`). `ide_spins_hw`, and on k46 `ide_cache`, `ide_push_n` and
  `ide_push_bytes`, exist only in a diagnostic build (`-DIDE_DIAG`). Bench rows read the counters
  through `/dev/kmem` at map addresses. In a module those addresses move with the slot, so the bench tooling needs the module's load address from the
  manifest or from `getksym`.

## The probe under the module rules

The module rules require a motherboard probe that is `bprobe`-guarded, free of side effects, positively
identifying and gated by chipset id. Today's `idepresent()` -> `ide_detect()`:
- **Chipset gate: met.** It reads VPOSR first, and only an AGA Alice id (`(id & 0x0F) >= 2`) goes on, so an
  A3000 never reaches Gary space.
- **Positive identification: met for the A4000 row.** The A4000 row identifies the controller by a task-file
  write and readback. The A1200/A600 rows read the Gayle ID register at 0xDE1000, and only when selected.
- **Not met yet: `bprobe` guard.** The task-file accesses are not wrapped in `bprobe`. A bus error there would
  panic. This is the gap to close.
- **Not met yet: free of side effects.** The readback writes to the sector-count/LBA-low registers. That
  disturbs no drive state, but the module spec has to decide whether it counts as a side effect.

## Testing

- The host harness (`test/host`) keeps working: the module wrapper is a thin file around `ide.c`, and
  `ide.c` stays compilable against the mock.
- New harness cases: `intr_attach` present, absent and refused; parameter defaults and overrides; the halt
  pass; `sd_register` after `init` -> `EBUSY` handled.
- Emulator rows first (the slot boots, IDE root, ATAPI CD), then the metal rows: the A4000D on the 060
  (polled only), and the interrupt path on a 68030.
