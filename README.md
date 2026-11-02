# amix-gayle-ide — native Amix IDE driver for the Amiga 4000's onboard Gayle port

> **Fork notice.** This is the amigaux.org project's working fork of
> [hmlio/amix-gayle-ide](https://github.com/hmlio/amix-gayle-ide) by Emre Bastuz, adopted 2026-09-29
> for the v1.1 installer line (`upstream` = his repository, `origin` = this fork). Changes here keep
> his layout and naming, one topic per commit, and are meant to go back upstream once the driver is
> tested and integrated on the current `amix-kerntools` hub; until then this fork is the working copy.
> The 2026-09-29 section of [`CHANGELOG.md`](CHANGELOG.md) lists what differs from upstream.

A native **Amiga UNIX (Amix 2.1, Commodore's SVR4.0 port to the 68030 Amiga)** driver for the
**Gayle IDE controller** built into the Amiga 4000 (the board table also carries untested A1200/A600
rows), so that Amix runs on an **unmodified A4000 with a 68030 CPU card** — no SCSI host adapter needed,
root on a CF card. Amix shipped with no IDE driver at all; this repo closes that gap. It follows
the framework, build environment and gotcha catalog of [`amix-a4091`](https://github.com/jusii/amix-a4091)
and [`amix-z3660scsi`](https://github.com/jusii/amix-z3660scsi).

The driver is a **pseudo SCSI host adapter**: the stock Amix SCSI disk layer (`dd.c`, block major 18)
hands it SCSI CDBs, and it executes them as ATA-2 PIO commands on the Gayle task file. Everything
above — RDB partitioning, `c<N>d<U>s<P>` slices, `mkfs`/`fsck`, the installer — is untouched.
Design record: [`docs/decision-hba-vs-block.md`](docs/decision-hba-vs-block.md).

## Scope / responsibility

This repo is **the Amix IDE driver for the Gayle port, and nothing else.** Its siblings in the
[amigaux.org](https://amigaux.org/) ecosystem:

- **Build harness** (kernel assembly, `driver.conf` discovery, clean-gate, host↔box bridge) →
  [`amix-kerntools`](https://github.com/jusii/amix-kerntools)
- **A4091 SCSI driver**, origin of the `support.c` phantom-A3000-SCSI patch →
  [`amix-a4091`](https://github.com/jusii/amix-a4091)
- **Documentation** of the Amix kernel and its quirks → [`grimoire-amix`](https://github.com/jusii/grimoire-amix)

Compilation and kernel integration go through `amix-kerntools`, which reads this repo's
[`driver.conf`](driver.conf) by relative path. This repo carries only the driver source, its
`driver.conf`, the kernel patches, a host-side test harness and the design docs.

## Status (2026-09)

**✅ Boots Amix with root on IDE on a real Amiga 4000/030** (2026-09-27): Commodore A4000/030 card
(68030 + MMU, 68882), Kickstart 3.1, SanDisk 1 GB CF on a 44-pin adapter. Kickstart autoboots the
`UNIX_Boot` partition, the kernel mounts root `c6d0s1` and swap `c6d0s2` through this driver, root
login works. To my knowledge the first Amix boot on an A4000 with root on IDE.

- ✅ **Probe** `idepresent()` — AGA chipset gate via VPOSR, then a task-file write/readback at the
  A4000 base; on an A3000 it reads VPOSR and nothing else. Automatic detection is A4000-only.
  A real PAL A4000 reads VPOSR id `0x23`.
- ✅ **Command execution** `idequeue()` — READ/WRITE(6)/(10), REQUEST SENSE, INQUIRY, TEST UNIT READY,
  READ CAPACITY(10), MODE SENSE(6), START/STOP UNIT, SYNCHRONIZE CACHE, VERIFY(10) → ATA READ/WRITE
  SECTOR(S), IDENTIFY DEVICE, FLUSH CACHE, READ VERIFY SECTOR(S), with synthesised sense data. PIO
  only, LBA28, LUN 0 only.
- 🟡 **ATAPI CD-ROM** — an ATAPI device (detected by its signature, then IDENTIFY PACKET DEVICE) gets
  the caller's CDBs as PACKET commands: INQUIRY, READ CAPACITY, READ(10), REQUEST SENSE are the
  drive's own answers, so a cdfs-capable kernel mounts it like a SCSI CD-ROM (see Hardware notes).
  Host harness, Amiberry (68030 and 68060), and a real drive on an A4000 (68060, polled): a 287 MB
  data CD reads intact, also with the IDE master busy. Interrupt-mode ATAPI is not yet run on
  hardware.
- ✅ **Interrupt-driven I/O** `ideintr()` — level-2 handler hooked into `int2_tbl[]` by
  `kernel.c.patch`, one interrupt per sector, `timeout()` watchdog; falls back to polled I/O on an
  unpatched kernel. Emulation: 107k interrupts, 2 spurious, 0 timeouts; 10 MB raw read 3.4 s → 0.7 s sys.
- ✅ **Native build** — compiles with the Amix K&R `cc` without diagnostics, links `checkunix`-clean
  through amix-kerntools, boots on an A3000 profile (probe silent) and the Amiberry A4000/AGA profile.
- ✅ **Host harness** — the unmodified driver source against a mock Gayle/ATA model with an ATAPI
  CD-ROM, built twice: the 68030 object (1236 checks) and the 68040/68060 one (1556 checks, cache
  and ordering included); the diagnostic build (`-DIDE_DIAG`) the same two ways (1253 / 1576
  checks); the loadable module form the same two ways (240 / 241 checks).
- 🟡 **Board table** — A4000 entry proven; A1200/A600 entries (task file `0xDA0000`, latched INTREQ,
  Gayle ID register) follow NetBSD/Amiberry and are untested. They are never auto-detected: select
  one with `ide_board_force` (2 = A1200, 3 = A600), which is the only time the Gayle ID register at
  `0xDE1000` is read. Amix itself has further A1200/A600 blockers (fast RAM expected at `0x07000000`).
- 🟡 **Open on real hardware** — stress and power-cycle `fsck` runs, spin-count calibration of the
  bounded polled waits, the installer route, and cache inhibition of the register ranges (assumed from the stock kernel's
  transparent-translation setup on the 68030; checked at attach on the 68040/68060 line, where a
  real 68060 passes). These are the `[verify]` markers left in `src/ide.c`. The **IDE
  slave** stays untouched unless `ide_slave_enable` is set: 1 attaches whatever answers there, 2
  attaches it only if it is an ATAPI device (the CD-ROM next to a CF master).

The IDE master is presented as **SCSI unit 6** (`ide_master_unit`) so that the stock root configuration
`c6d0s1`, `/etc/vfstab` and the installer's assumptions of an A3000-installed system apply unchanged.
`amix-a4091`'s `support.c.patch` (applied by the build hub to every kernel) removes the stock kernel's
phantom A3000 SCSI controller on AGA machines, which makes the IDE port **card 0**.

## Layout

```
driver.conf               0xFFFF1DE0 idequeue "A4000 IDE" ide.c probe=idepresent
src/ide.c                 the driver: register map, board table, probe, SCSI→ATA translation,
                          level-2 handler (K&R C, single file as amix-kerntools expects)
src/kernel-patches/       kernel.c.patch  -- ideintr into int2_tbl[] (master.d/kernel.c)
                          NOTICE          -- diffs only, not Amix source; the support.c patch
                                             (phantom A3000 SCSI removal) comes from amix-a4091
module/                   the loadable module form: Makefile (ABI kit), Master, VERSION
test/host/                host harness: mock Gayle/ATA, kernel stubs, tests (make check)
docs/                     decision record, VPOSR AGA-gate finding, the module port plan
CHANGELOG.md
```

The `sd.c` `scsicard[]` rows and the alien `Makefile` `OBJ` entry are **not** here — amix-kerntools
generates them from `driver.conf`.

## Requirements

- An Amiga 4000 or 4000T, whose Gayle IDE port sits at `0xDD2020` (the A1200/A600 rows are untested,
  see Status).
- 68030 **with a 68881/68882 FPU** (the stock Amix kernel has no software floating point; without an
  FPU it panics in the kernel at the first signal delivery, seconds after `no fpu detected`). For a
  68040/68060 see [The 68040/68060 kernel line](#the-6804068060-kernel-line).

## Building

This repo is **source only**. With [`amix-kerntools`](https://github.com/jusii/amix-kerntools)
checked out alongside it and an Amix 2.1 box reachable (real hardware or an
Amiberry/WinUAE profile), the single entry point is:

```sh
(cd ../amix-kerntools && ./amix-build gayle-ide --install)
```

The harness splices `src/ide.c` into `/usr/sys/amiga/alien/`, generates the controller rows from
`driver.conf`, applies `kernel.c.patch` together with `amix-a4091`'s `support.c.patch` (the hub
applies that one to every kernel; its AGA gate is the low-nibble test from `docs/vposr-aga-gate.md`),
relinks and clean-gates the kernel.

On an A4000 the resulting kernel boots straight from the CF card: Kickstart 3.1 autoboots the
`UNIX_Boot` partition of the IDE disk, and the kernel mounts root over this driver. No SCSI card,
no `unix_boot` patches (68030 = stock `unix_boot` 1.1c).

The host harness needs only a C compiler:

```sh
make -C test/host check      # every variant; exit 0 == every check passed
```

## The loadable module

The same source, built with `-DIDE_MODULE`, is `gayle`: a root-class loadable kernel module for
Amix kernels with amix-dlm's module core. The core loads it before the root is mounted, and it
registers the port with sd by itself, so no `driver.conf` row, kernel patch or relink is needed.
Without the switch `src/ide.c` is the static driver, byte for byte (`make -C module static` prints
the static objects' sums: 68030 `8f52b4f2…`, 68040/68060 `b4504a8f…`). The plan and the reasons are
in [`docs/MODULE-PORT.md`](docs/MODULE-PORT.md).

The investigation-only symbols — the spin high-water `ide_spins_hw`, and on the 68040/68060 line the
data-cache switch `ide_cache` and its counters `ide_push_n` / `ide_push_bytes` — exist only in a
diagnostic build, `-DIDE_DIAG` (`make -C module DIAG=1`, under `build/<line>-diag`). A default build,
static or module, has none of them, so the data-cache line operations cannot be switched off. The
static objects of a diagnostic build are those of 1.1.0-3 and earlier, byte for byte (68030
`af0ef514…`, 68040/68060 `bdcd9072…`).

```sh
make -C module          # build/k30/mod.d/gayle and build/k46/mod.d/gayle, checked by the kit
make -C module k30      # one line (or k46)
make -C module static   # the static objects compiled against the kits, and their sha256
make -C module clean
make -C module DIAG=1   # a diagnostic build (-DIDE_DIAG), never for shipping
```

It is built with amix-dlm's ABI kits, **kabi 1.2 on both lines** (`k30.1.2`, `k46.1.2`): `DLM=` names
an amix-dlm checkout with built kits (default `../amix-dlm`), or `KIT_k30=` / `KIT_k46=` name the kits.
The pinned cross tools (`m68k-cbm-sysv4-*`) and `python3` must be on `PATH`. Each build ends with the
kit's `modcheck` line (`ok (k30.1.2, root, …)`, `ok (k46.1.2, root, …)`) and a scan that the module
holds no `0xDE1000`.

| | |
|---|---|
| `.moddata` | `name=gayle`, `class=root`, `match=mobo:gayle`, `rank=ide`, `unload=never`, no `majors` (sd owns the disk majors); `cpu` and `kabi` from the kit |
| Parameters | `intr` (1: interrupts after a self-test, the default; 0: polled), `slave` (0/1/2 as `ide_slave_enable`), `debug` (0–2), `master` (6) and `slaveunit` (5), the SCSI unit numbers |
| Load | VPOSR AGA gate, then `bprobe` of the alternate status register `0xDD303A`, then the task-file write/readback (parameter registers only, no command); polled IDENTIFY; `sd_register(idequeue, 0, 0xDD2020, "A4000 IDE", 4)`. No drive, or no Gayle: `ENXIO`, silently, and no card is taken |
| Interrupts | by default (`intr=1`), at the first `sysinit`: `intr_attach` at level 2 and a self-test by one READ VERIFY SECTOR(S); polled if it fails or if no ATA drive is attached. Until then, and with `intr=0`, every command is polled |
| Halt | sets nIEN; no flush |
| Not in the module | the A1200/A600 rows (the Gayle ID register `0xDE1000` is never touched), the `int2_tbl` search, `idepresent()` |

Interrupt I/O is the default on both lines from 1.1.0-3, and the package sets no parameter. To run
polled, put the line `param intr=0` in `/etc/conf/modreg.local.d/gayle`, then `kbootset build` and
`kbootset commit`; removing the line goes back to interrupts. The counters (`ide_resets`,
`ide_intr_*`, `ideunits`, …; `ide_spins_hw` in a diagnostic build) are global symbols of the module,
so `getksym` finds them at the module's load address.

**Tested** in emulation (amiberry `ide=a4000`), on the 68060 module kernel kern46m and the 68030 one
kern30m; host harness `ide_modtest`, 239 checks per line. Module 1.1.0-3 (interrupt I/O by default) on
kern46m k46.1.2 -82 and kern30m k30.1.2 -17, installed with apkg:

| Row | What | Result |
|---|---|---|
| 1 | IDE root alone, `c6d0`, no parameters; then `param intr=0`; then the line removed again | passed on both lines: by default the self-test passes, then interrupt I/O (27 000–66 000 interrupts per boot, no timeouts, no resets); with `intr=0` no self-test and no interrupts, polled I/O; back to interrupts after the line is removed |
| 2 | IDE + Z3660 SCSI: the IDE port as card 1, root by identity (`ced0s1`) | passed on both lines, interrupt I/O |
| 3 | the A3000 profile: one VPOSR read, `ENXIO`, the boot otherwise unchanged | passed on both lines: the module is not loaded |
| 4 | ATAPI CD-ROM on the slave (`param slave=2`, unit 5) with the cdfs module | passed on both lines: the ATA master's self-test passes; `mount -F cdfs 0,5 /cdrom`, and every file's `sum -r` matches the test ISO; 1716 PACKET commands, 0 resets |
| 5 | halt and reboot (every row boots 2–4 times; the halt pass runs at each shutdown) | passed on both lines |
| 6 | safe mode with an IDE root | passed on both lines: the safe boot loads only gayle, with interrupt I/O |

Each copy test (`cp` of the kernel twice with `sync`, `cmp`) matched. In 5 of 24 counter samples, one
interrupt arrived while no command was waiting; it was counted (`ide_intr_spurious`) and ignored. The
`intr=1` runs of 1.1.0-1 showed the same (1 of 26 samples). Row 4's raw read, before the cdfs check: the
row first used `dd if=/dev/rdsk/c5d0s0 bs=2048 count=16`, which is not a CD read: `dd.c` addresses in
512-byte blocks, so the 2048-byte drive answers its own blocks 0, 4, … 60 (Hardware notes). The first
module rows (1.1.0-1, `intr` 0 by default) passed the same six rows polled and with
`intr=1`. Row 1 on the 68030 line also showed the boot-time page-crossing note (Hardware notes,
`ide_pagecross_n`); from 1.1.0-2 the module counts it and prints it only with `param debug=1`.

**Not tested yet:**
- the module on real hardware: the A4000 rows, with interrupts by default, are a 1.1 release gate;
- the interrupt path on a 68040/68060 on real hardware (the static 040/060 line was polled only).

## Hardware notes

- Gayle task file at `0xDD2020` (A4000/A4000T), byte registers spaced by 4, control block at `+0x101A`;
  IRQ mirror at `0xDD3020` bit 7. INT2 is shared with CIA-A, so the handler checks that bit and
  returns "not mine" otherwise. No explicit interrupt acknowledge on the A4000 (reading STATUS drops it).
- `0xDD2020` lies in the identity-mapped low 1 GB (`phystokv(p) == p`), so no `sptalloc()`; the 68030
  cache inhibit comes from the transparent-translation register, not from a per-page bit.
- A 16-bit DATA read returns the drive's little-endian ATA word byte-swapped relative to the 68k's
  view, which is the drive's byte stream in natural order: sector data is copied as is, and only
  IDENTIFY's words (numbers and the two-character string cells) are swapped once. Confirmed on the real
  machine (2026-09-27: root mounts, model string and sector count decode).
- The clock is a level-2 interrupt and `sdspl == spl2`, so `lbolt` is frozen inside the queue routine:
  polled waits are bounded spin counts, the interrupt path uses a `timeout()` watchdog.
- A command that has not completed within `ide_timeout_secs` (interrupt path) or its spin budget
  (polled path) is failed with a hardware-error sense after a software reset of the cable (SRST,
  `ide_resets` counts them), so a drive still executing the abandoned command cannot complete it into
  the next request's buffer. A drive that stays busy through the reset fails each request without
  hanging the kernel.
- `ide_pagecross_n` counts raw requests (one page or less) whose buffer straddles a page boundary; the
  static driver logs the first, the module only with `param debug=1`. Where it fires, it fires once per
  boot on root mount: the partition layer (`sdpart.c`) reads the RDB and partition blocks 512 bytes at
  a time into a static kernel buffer (`block`, 512 bytes in the kernel's bss), and whether that buffer
  sits across a page boundary depends on the kernel's link. In the 68030 module kernel kern30m -15 it
  is at page offset `0x6cc` and crosses the 2 KB page; in the 68060 one, kern46m -80, it sits at
  `0xdbc` of a 4 KB page and does not. A kernel static is physically contiguous, so nothing needs
  splitting. Raw I/O from user pages reaches the driver already split at page boundaries (`dd.c`'s
  `breakup` through `amiga_dma_pageio`), so the counter is a diagnostic, not a hazard seen so far.
- **ATAPI CD-ROM.** On the slave with `ide_slave_enable` 2 (or 1) it is SCSI unit 5 of the IDE
  card; with a cdfs-capable kernel, `mount -F cdfs <card>,5 /cdrom` mounts it (card 0 where the IDE
  port is the only controller, card 1 below the Z3660 on the 68040/68060 line). Every CDB
  goes to the drive as a 12- (or 16-) byte packet; only READ(6)/WRITE(6), which MMC drives lack,
  become READ(10)/WRITE(10). A CHECK CONDITION leaves the drive's own sense for the next REQUEST
  SENSE, which goes to the drive too, unless the driver itself raised the condition (no such LUN,
  timeout). The data moves in the DRQ phases the drive chooses; bytes beyond the caller's `nbyte`
  are drained, not stored (`ide_atapi_overrun` counts them), so a 512-byte `dd.c` read of a
  2048-byte CD block cannot overrun its buffer. `/dev/dsk` is not the way to a CD: `dd.c` expects
  512-byte blocks and an RDB. A raw read of `/dev/rdsk/c5d0s0` does reach the drive, but `dd.c` puts
  the byte offset / 512 in the LBA field and the length / 512 in the count, so the drive returns its
  block `offset / 512` (2048 bytes per block requested) and only the first `nbyte` are kept: the bytes
  are the CD's, at the wrong addresses. Polled ATAPI waits allow `ide_spins_atapi` iterations per phase (a
  disc spinning up); interrupt-driven ones are bounded by the `ide_timeout_secs` watchdog. A polled
  command ends only in the status phase (BSY and DRQ clear with IREASON = CoD|IO): Amiberry's model
  shows BSY and DRQ clear for a moment before the data phase, and a device that never sets CoD|IO at
  the end is accepted after `ide_spins_short` quiet reads (`ide_atapi_irlate` counts those).
- Stock Amix `init` behaviour, not the driver, but every IDE box will meet it: a run-level change requested
  while **nobody is logged in on the console** (the console `getty` sits at `login:`, and `shutdown -i6`
  comes over telnet) stops at `INIT: New run level: 6` and never spawns `/etc/rc6`. The disk keeps
  working, `who -r` reports level 6, telnet still answers, and only a power cycle gets the box back.
  The same happens with the stock kernel on an A3000 without this driver. Request reboots and halts
  from a root shell logged in on the console (or leave one logged in); then `rc6` runs and the machine
  reboots normally.
- The stock disk driver `dd.c` completes a buffer (`iodone`) before it starts the next one, and the
  completion callback can resubmit the disk's single `sdcom` while the first submission is still on
  the cable or queued. `idequeue()` ignores such a duplicate (`ide_dup_submits` counts them); the
  request completes once, from its first submission. The amigaux.org build hub additionally applies
  `amix-z3660scsi`'s `dd.c` ordering patch (start the next buffer, then `iodone`) to its IDE kernels;
  on a stock `dd.c` the guard alone keeps the queue sane.

## The 68040/68060 kernel line

The same `src/ide.c` builds for the 68040/68060 Amix line (amix-040-060-port), whose relink pass
compiles it with `-m68040 -DIDE_NBPP=4096 -DIDE_CACHE=1` and registers the port itself. Without
`IDE_CACHE` none of the following is in the object.

- **Interrupts: polled only.** The line's `int2_tbl[]` is a fixed table that the relink pass cannot
  grow, so `ideintr` is never in it. Attach finds no hook, keeps the drive's nIEN set and runs every
  command polled inside the queue routine (`ide: A4000 IDE, polled I/O` on the console). A transfer
  holds IPL 2 while it runs (an 8 KB block is 16 sectors of PIO), so the clock tick waits behind it,
  and a drive slow to come ready holds IPL 2 for up to the spin budget.
- **Data cache.** The CPU copies the sectors through the cache-inhibited identity alias, while the
  same physical page may sit in the copyback data cache under a user mapping. Every buffer is pushed
  and invalidated (`CPUSHL` + `CINVL` per 16-byte line, by physical address) before the transfer, and a
  READ's buffer again after it. Only in a diagnostic build (`-DIDE_DIAG`), `ide_cache` (kmem,
  default 1) set to 0 skips the line operations for an A/B run, and `ide_push_n` / `ide_push_bytes`
  count them; a default build always does them, since without them data can be silently corrupted.
- **Register cache inhibition** is checked at attach from DTT0/DTT1: `ide_ci_ok` is 1 when the task
  file and IRQ mirror are reached cache-inhibited, 2 (with a console line) when they are not.
- **Ordering.** The registers are cache-inhibited *nonserialized*; a `nop` before every task-file read
  keeps it behind earlier writes once the 68060 store buffer is enabled.
- **Page size** 4096 for the page-crossing counter (`ide_pagecross_n`).
- **Spin budgets** ×4 (`IDE_SPIN_SCALE`): the polled loop is bound by the motherboard bus cycle, not the
  CPU, so the same counts would run out sooner only by the loop overhead. In a diagnostic build
  `ide_spins_hw` records the longest successful wait, for calibration on the machine.

Where the port sits in card order beside the Z3660's SCSI is the relink pass's decision, not the
driver's.

**On hardware** (2026-10-10): an A4000 with a Z3660 in real 68060 mode, an SD-to-CF adapter as
master and an ATAPI DVD-RAM as slave. DTT0 reads `0x003FC060` (`ide_ci_ok` 1); 64 MB raw reads repeat
with equal checksums; writes read back intact; `ide_spins_hw` 2765; Amix boots with root and swap on
the IDE disk. With CD and disk I/O back to back, logins can stall for a minute or two: the IPL 2 hold
described above.

## Sharing a kernel with a SCSI host adapter

The Amix SCSI layer numbers its controllers by ascending base address, and `0xDD2020` sorts below
every Zorro card. In a kernel that carries this driver together with a SCSI host adapter driver
(A4091, Z3660), an A4000 with an IDE disk attached therefore gets the IDE port as **card 0** and its
master at `c6d0` — the very slot the stock root configuration names — while the SCSI adapter moves
to card 1 and its disks change names; with two SCSI adapters already registered the IDE port is
refused with `sd: too many controllers`. This driver does not arbitrate that: the amigaux.org build
hub ships the IDE driver in an IDE-rooted kernel of its own and registers the port only when it is
the sole controller. A hand-assembled shared kernel must decide which controller roots, or leave the
IDE disk unplugged. The slave, when enabled, appears as unit 5 (`ide_slave_unit`).

The loadable module (above) changes this: sd orders controllers by class, and the IDE port comes
last, so beside one SCSI adapter it is card 1 (`ced0`), and the root is found by identity.

## Troubleshooting

- `PANIC: KERNEL FAULT … pc=<kernel base>+0x12A, vector=0xB (Line-F Emulator)` in pid 1 right after
  the copyright block, with `no fpu detected` above → the CPU card has no FPU. Not the IDE driver, not
  Kickstart, not RAM. Fit an FPU (or `fpu_model=68882` in the emulator). FPU-less 030 support is
  planned for a later release.

## License

The original work in this repo is released under the **MIT license** (see [`LICENSE`](LICENSE)):
[`src/ide.c`](src/ide.c), [`driver.conf`](driver.conf), the test harness and this repository's
documentation.

The patch in [`src/kernel-patches/`](src/kernel-patches/) is a unified diff against a stock Amix
file: the added lines are MIT, the few quoted stock context lines remain under the original
Commodore SVR4 copyright. See [`src/kernel-patches/NOTICE`](src/kernel-patches/NOTICE).

Amix itself (SVR4.0), its disk images, Kickstart ROMs and manuals are proprietary Commodore software
and are **not** included — you need your own Amix 2.1 system. No Commodore/AT&T or GPL code is
reproduced in the driver, the harness or the docs; the kernel patch quotes a few unchanged stock lines
as diff context, as noted above. The ATA behaviour follows the ATA-2 standard, and NetBSD's
`wdc`/`gayle` sources (BSD) and Linux `gayle.c` (GPL) were read as documentation only.

## Credits

By Emre Bastuz (hmlio). AI tooling — **Claude Code** — was used throughout to investigate, build and verify this
work. Built on the framework, build harness and kernel documentation of the amigaux.org community
(Jussi Alanärä, asokero, isoriano1968) and the NetBSD `wdc` driver as the ATA reference.
