# Porting the driver to a root-class kernel module: the plan

Date: 2026-10-27. Status: implemented the same day (`src/ide.c` under `-DIDE_MODULE`, `module/`,
`test/host/ide_modtest.c`; README "The loadable module"), with amix-dlm's answers to §7. It applies
amix-dlm's contracts to this driver:
C4-2 v3 (root-class driver), C4-1 v2 (`sd_register`, four cards), C4-5 v2 (DMA), C3-6 v5 (driver to
module), C3-5 v9 (package), and the DDI list (`amix-dlm/docs/abi/ddi.md`, `abi/<line>/ddi.list`).
The shape follows the first root-class disk module, amix-z3660scsi (`module/`, `src/z3660.c` under
`-DZ3660SCSI_MODULE`). Where this plan and [`module-form.md`](module-form.md) (written before the
contracts) differ, this plan wins: the rank is a class, not a sort key; interrupts attach late, not
at attach time; and `_halt` masks with nIEN.

## 1. The static integration today

| What | Where |
|---|---|
| sd row: synthetic product 0xFFFF1DE0, queue `idequeue`, `probe=idepresent` | `driver.conf:12`. On the 030 line the build hub's `sd.c` calls the probe when autocon has no entry. On the 040/060 line the port's `ide_sd_glue040.s` registers it with sort key 0xfffffff0 (C4-1:37-40) |
| Probe: VPOSR AGA gate, then a task-file write/readback at 0xDD2020 | `ide_detect()` `src/ide.c:2012-2041`, `ide_readback()` `:1979-1999`, `idepresent()` `:2047-2062` |
| Attach is lazy, on the first `idequeue()` (IDENTIFY, polled) | `src/ide.c:1727-1731`, `ide_attach()` `:887-909` |
| Interrupt: `ideintr` in `int2_tbl[]`, added by `kernel-patches/kernel.c.patch`. The driver searches the table and falls back to polled | `ide_hooked()` `src/ide.c:875-884`. On 040/060 the table is fixed, so that line is polled only (`:7-10`, README "The 68040/68060 kernel line") |
| Watchdog `timeout()`, armed only on the interrupt path | `ide_watch_arm()` `src/ide.c:1524-1530`, `ide_watchdog()` `:1653-1677` |
| Memory: static `.data`/`.bss` only. `ideunits[2]` (`:587`), the completion FIFO `ide_cq[32]` (`:1090-1091`), and the counters. No `kmem_alloc` | |
| Imports: `printf`, `timeout`, `int2_tbl` | `src/ide.c:70-72` |
| Per-cut tunables patched by symbol: `ide_slave_enable`, `ide_intr_enable`, `ide_board_force`, `ide_debug`, the units, the spin budgets | `src/ide.c:262-289` |

## 2. The module shape

One source. `-DIDE_MODULE` adds a section at the end of `ide.c`. Without the switch the static
object stays byte for byte as it is (z3660scsi's rule). A new `module/` directory holds `Makefile`
(on the kit's `mk/module.mk`), `Master` and `VERSION`.

**`.moddata`** (C4-2 §1, `C4-2:25-40`; §8 row `:141`): `name=gayle`, `$modtype misc`
(`MOD_HDRV_WRAPPER`, no switch slot), `class=root`, `match=mobo:gayle` (in `abi/registry/mobo`),
`rank=ide`, `unload=never`, no `majors`. The parameters replace the patched symbols:
`intr:int:1:0:1` (0 until 1.1.0-2, §7 question 1), `slave:int:0:0:2`, `debug:int:0:0:2`, `master:int:6:0:7`, `slaveunit:int:5:0:7`.

**`gayle_load(params)`** runs in `marc_root`: level 7, the clock stopped, no sleep (C4-2 §2,
`:42-68`). In order:
1. Parse the parameters. Absent names take the defaults above.
2. **Gate: VPOSR.** Read `0xDFF004` (a custom-chip register, decoded on every Amiga). Go on only
   when `((vposr >> 8) & 0x0F) >= 2`, i.e. AGA Alice (`IDE_VPOSR_IS_AGA`, `src/ide.c:131-133`;
   [`vposr-aga-gate.md`](vposr-aga-gate.md)). This is the same gate the 040/060 kernels' motherboard-SCSI
   probe uses. On an A3000 (ECS Fat Agnus, id 0x20/0x30) the module returns `ENXIO` after this single
   read and touches nothing else.
3. **Guard: `bprobe(0xDD303A, -1)`**, a read of the alternate status register (base + 0x101A). The
   read has no side effect: it does not clear INTRQ. A bus error returns `ENXIO`. This closes the
   gap `module-form.md` names: today the task-file accesses are unguarded.
4. **Identify: the task-file write/readback** (`ide_readback_regs()`, `src/ide.c:1960-1975`). It
   writes nIEN to the control register, writes SDH, and writes two patterns to NSECT/SECT. These
   are parameter registers, and no command is written, so the drive's state does not change. Open
   bus or no drive: `ENXIO`, silent.
5. **The Gayle ID register `0xDE1000` is never touched.** It lies in Gary space on the A3000 and
   A4000. Writing it is how `ide_gayle_id()` (`:554-565`) reads it, and today only the A1200/A600
   rows forced through `ide_board_force` reach it. The module drops those rows (question 2).
6. Attach in `_load`, not lazily: the polled IDENTIFY of both drives, bounded spins, no
   `lbolt`/`timeout`. Then `sd_register(idequeue, 0, (char *)0xDD2020, "A4000 IDE", 4)`. Its errno
   (`EBUSY`/`EINVAL`/`ENOSPC`) is printed and returned (C4-1 §3, `C4-1:121-145`).
7. If `intr` is set, call `root_defer(gayle_defer, 0)`.

Note on context. sd's first open (`rth1_apply`) runs while the root pass is still active, and
`idequeue()`'s `sdspl()` bracket (`:1718`) moves the level from 7 to 2 and back. The static driver
does exactly the same at that point today, and z3660scsi's `spl6` does the same.

**`gayle_defer(arg)`** runs at the first `sysinit`, process context, level 0 (C4-2 §3, `:70-94`):
1. `intr_attach(2, ide_intr2, 0, "gayle")`. `ide_intr2` wraps `ideintr()` (`:1583`) and returns
   nonzero only when the driver claimed the interrupt, as C3-6 §1.3 requires.
2. **Self-test.** A software raise of `INTREQ` cannot set the A4000's INTRQ mirror (`0xDD3020` bit 7,
   `:494`), so the test is a real command:
   - clear nIEN;
   - issue READ VERIFY SECTOR(S), 1 sector at LBA 0, on the master (a no-data command every ATA
     drive has);
   - wait in one-tick `timeout`/`sleep` rounds for `ide_intr_count` to move;
   - pass: set `ide_use_intr`, under `sdspl()` with the queue idle;
   - fail: set nIEN again, `intr_detach`, print `ide: interrupt self-test failed, polled I/O`, and
     return 0.
3. The watchdog's `timeout()` (`:1528`) is reached only after the test passes, so it never runs in
   the root pass.

**`gayle_halt()`** (C4-2 §5, `:103-115`): write `IDE_CTL_NIEN` to the device control register, and
call `ide_irq_enable(bd, 0)` too. On the A4000 row `irqena` is 0 (`:494`), so `ide_irq_enable` is a
no-op (`:541-551`), and nIEN is the only mask that row has. The halt does not wait for a command in
flight.

**`gayle_unload()`**: the core calls it only when a load fails after `_load` returned 0 (C3-6 §1.1).
It sets nIEN and forgets the probe state (`idereset()`, `:2069-2082`). It returns 0.

**Gone in the module form:** `ide_hooked()` and `kernel-patches/kernel.c.patch`;
`idepresent()`, `ide_board_force` and the A1200/A600 rows; the `driver.conf` row (it stays for
the static form).

**DDI names and their minors** (`abi/k30/ddi.list`, `abi/k46/ddi.list`):

| Name | Minor | Where it is used |
|---|---|---|
| `printf`, `bprobe`, `spl2`, `splx` | 0 | `_load` (`bprobe` is in C4-2 §2's root list; the root-context check for it must pass on both lines before the module ships) |
| `intr_attach`, `intr_detach`, `timeout`, `sleep`, `wakeup` | 0 | `gayle_defer` and the interrupt path, level 0 and up |
| `sd_register` | 1 | `_load` |
| `root_defer` | k46.1.1, **k30.1.2** | `_load` |

The module needs **kabi 1.2 on both lines**: it is built with the kits `k30.1.2` and `k46.1.2`
(amix-dlm `bbd20b6`), and runs on the kernels of those minors (`kern30m` -13 and `kern46m` -78 when
they are minted). The cache line operations are inline opcodes (`:348-350`) and import nothing.

**DMA: none (C4-5).** The Gayle IDE has no bus master: every byte moves by CPU PIO through the
task-file data register (`ide_pio_in`/`ide_pio_out`, `:712-745`). The board never holds an address
of memory, so C4-5's reasons (`C4-5:25-37`) do not apply:
- no coherency with a device writer;
- no reach problem: the CPU reaches the Z3660's DDR, Zorro DMA does not.

No `dma_*` call is needed. The 040/060 push and invalidate (`ide_cache_push`, `:389-411`) is
CPU-to-CPU alias coherency and stays as it is.

## 3. One Gayle, two drives, under C4-1

One controller means one `sd_register` with `c = 0`. The driver's state is single-instance and
`idequeue()` ignores `c` (`:1703-1708`). No array is indexed by `cp->card`, so C4-1 §2.3's resize
(`C4-1:92-119`) has nothing to change here.

The two drives are units within the card's minor bits 0-2 (C4-1 §2.1, `:50-64`): master unit 6,
slave unit 5 (`:284-285`). The card number comes from the class order z3660 < zorro < mobo < ide
(`C4-1:169-184`). So the IDE is always last:
- alone: card 0, `c6d0` / `c5d0` (today's names);
- behind one SCSI controller: card 1, `ced0` / `cdd0` (minor bit 3);
- behind two or three: card 2 or 3, `c16d0` / `c1ed0` (minor bit 7);
- a fifth controller is dropped by sd.

Today's shared 030 kernels put the IDE first, by base address. The class order reverses that for
Gayle IDE + Zorro SCSI on the 030 line, an accepted rename (`C4-1:242-251`). The "IDE only as the
sole controller" packaging rule (README "Sharing a kernel") ends with the module: the root is found by
identity.

## 4. k30 and k46

| | k30 (68030) | k46 (68040/68060) |
|---|---|---|
| Flags | as the static `ide.o` | `-DIDE_CACHE=1 -DIDE_NBPP=4096`; spin budgets ×4 (`:256-262`) |
| kabi | `k30.1.2` | `k46.1.2` |
| Interrupt path | proven on a real A4000/030 and in emulation (README "Status") | the static line was polled only; the module's `intr=1` first ran in emulation (§6.2 row 1), not yet on hardware |
| Register cache check | none | `ide_ci_check()` at attach (`:438-457`): the arena runs through DTT0 like the static drivers (C4-2 §6) |
| Root-pass level | 7 on both (C4-2 §2) | same |

## 5. Package rows (C3-5 §1.1, `C3-5:43-70`)

| Module | Short name | pkginst k30 / k46 | Catalog | Version |
|---|---|---|---|---|
| gayle | `gayle` | `gayle30` / `gayle46` | `gayle-k30` / `gayle-k46` | `module/VERSION`, `1.1.0-4` |

The package ships `/etc/conf/mod.d/gayle` and `/etc/conf/modreg.d/gayle`. It ships no device nodes,
because sd owns the disk majors. C3-5 v9 §1.1 carries the row (`gayle | gayle | gayle30 / gayle46`).
Up to 1.1.0-2 the package's `modreg.d/gayle` shipped with interrupts off on both lines. From 1.1.0-3
the module's own default is `intr=1` (§7 question 1), and the package sets no parameter. The polled
switch is the administrator's line `param intr=0` in `/etc/conf/modreg.local.d/gayle` (C3-5 §1's
`param <pname>=<value>`), followed by `kbootset build` and `commit`.

## 6. Test plan

1. **Host harness** (`test/host`): the module glue compiled against the mock. New cases:
   - the VPOSR gate on ECS and OCS: no task-file access, `ENXIO`;
   - a `bprobe` fault: `ENXIO`;
   - no access to `0xDE1000` on any path;
   - `sd_register` errors passed back;
   - `root_defer` with `intr` 0 and 1;
   - the self-test passing and failing, ending polled with nIEN set;
   - `_halt` sets nIEN on the A4000 row;
   - an ATAPI CD-ROM on the slave (`slave=2`, unit 5), polled and with interrupts: one PACKET per
     request with the caller's CDB, never an ATA command; a `dd.c`-shaped raw read (512-byte LBAs)
     of the 2048-byte drive returns the drive's own blocks `offset / 512`, the rest drained;
   - a page-crossing request: counted, said only with `debug=1`.

   Both binaries, 0 failures.
2. **Emulation, A4000 IDE model** (amiberry, `ide=a4000`), each on the 030 and the 060 kernel. Run
   with 1.1.0-1 on kern30m k30.1.2 -15 and kern46m k46.1.2 -80:
   1. IDE root alone: `c6d0`, polled, then `intr=1` — **passed** on both lines (on k46 the first
      run of the interrupt path: self-test passed, no spurious interrupts). On k30 every boot also
      printed `ide: raw request of 512 bytes crosses a page boundary` at root mount: `sdpart.c`'s
      static RDB buffer sits at page offset `0x6cc` in that kernel (`0xdbc` of a 4 KiB page on
      k46, no crossing). A kernel static is physically contiguous, so the request is safe; 1.1.0-2
      counts it (`ide_pagecross_n`) and prints it only with `debug=1`;
   2. IDE + Z3660 SCSI: IDE becomes card 1, `ced0`, root by identity — **passed** on both lines;
   3. the A3000 profile: one VPOSR read, `ENXIO`, the boot otherwise unchanged — **passed**;
   4. ATAPI on the slave (`slave=2`) — the drive attaches as unit 5 and takes every request for it
      as a PACKET command (`ide_atapi_cmds` +16 for 16 reads, both lines). The row's
      `dd if=/dev/rdsk/c5d0s0 bs=2048 count=16` is not a CD read: `dd.c` sends the byte offset / 512
      as the LBA and the length / 512 as the count, so the 2048-byte drive returned its blocks 0, 4,
      … 60, 8 KiB each, the first 2 KiB of each stored and 6 KiB drained (`ide_atapi_overrun`
      98304). `sum -r` 64611 is exactly the test ISO's blocks 0, 4, … 60, first 2 KiB each. No driver
      defect; the CD check for the row is `mount -F cdfs 0,5 /cdrom` and a file's checksum (or, as a
      quick look, `dd if=/dev/rdsk/c5d0s0 bs=512 skip=16 count=1`: the first 512 bytes of the
      volume descriptor, `\001CD001`);
   5. a halt and reboot with `intr=1` — **passed**;
   6. safe mode with an IDE root — **passed**.

   Re-run with 1.1.0-3 (interrupt I/O by default) on kern46m -82 and kern30m -17, installed with apkg:
   all six rows **passed** on both lines with no parameter set; row 1
   also with `param intr=0` (polled, no self-test) and back. Row 4 used the cdfs mount as its CD check.
3. **Real hardware, after emulation:** an A4000 with a 060 card (polled, then the first `intr=1`
   run on 040/060), and on a 68030 for the interrupt path.

## 7. Questions for amix-dlm, answered

amix-dlm accepted all seven recommendations (contracts C3-5 v9 and C3-6 v5), with these amendments,
which the code follows:
- **1:** the module default is 0. The switch is the package's `modreg.d/gayle` line `param intr=1`
  (not `gayle.intr`), and it ships off on both lines for now. **Changed 2026-10-27: interrupt I/O is
  the 1.1 default.** From 1.1.0-3 `intr` defaults to 1 on both lines,
  and polled I/O is the local line `param intr=0`. The A4000 hardware rows with interrupts are a 1.1
  release gate.
- **2:** the proof is that no module path accesses `0xDE1000`: a host test sums the mock's Gayle ID
  reads and writes over every case, and `make -C module` scans each linked module for the constant.
- **3:** the readback is the registered `mobo:gayle` identifying probe, allowed only after the VPOSR
  gate and the `bprobe` guard of ALTSTAT `0xDD303A`, in that order; parameter registers only.
- **4:** `bprobe` at level 7 with the clock stopped is amix-dlm's root-context row (C4-2 §2).
  The module is built and emulated meanwhile; shipping waits on that row.
- **5:** `ENXIO`, silent and unlinked, bounded by the static driver's own spin budget.
- **6:** `_halt` masks (nIEN) only; no flush in v1.
- **7:** the counters are global symbols, so `getksym` finds them (`ideunits` too in the module).

The questions as they were put:

1. **`intr` default.** C3-6 §1.3 keeps polled as the default, and the §8 row says INT2 through
   `root_defer`. *Recommend:* the module default is 0; the k30 package's `modreg.d` sets
   `param gayle.intr=1` once the emulation rows pass; k46 keeps 0 until a hardware row with
   interrupts passes.
2. **Forced A1200/A600 rows.** Their probe writes `0xDE1000` and is not gated by `mobo:gayle`.
   *Recommend:* drop them from the module and keep them only in the static source. Amix does not
   run on those machines.
3. **Is the readback a side effect?** It writes parameter registers but issues no command.
   *Recommend:* the `mobo:gayle` registry entry records it as an accepted identifying write, as
   `20-module-format-and-abi.md:218` already describes it.
4. **`bprobe` in the root pass.** *Recommend:* make the Gayle probe one of the root-class probe rows
   that check `bprobe` at level 7 on both lines; block shipping on it, as C4-2 §2 says.
5. **No drive answering.** The readback fails with no drive, so the module does not register.
   *Recommend:* `ENXIO`, so an empty A4000T IDE port takes no card. The A4000T has the same task
   file at `0xDD2020` and passes the VPOSR gate.
6. **FLUSH CACHE in `_halt`.** *Recommend:* not in v1; `_halt` stays mask-only, as C4-2 §5 says.
   A halt-time flush, with the fix for SYNCHRONIZE CACHE arriving mid-transfer, is a later driver change.
7. **Counters at load addresses.** Bench readers use `/dev/kmem` at link-map addresses.
   *Recommend:* resolve them through `getksym` and the module's load address. No contract change is
   needed.
