# Changelog

## 2026-11-13 — `make -C module` builds buddha beside gayle (219dba3, 68d2bf8, e6f2d39)

`buddha` is in the default target on both lines and needs kabi 1.5 kits (`k30.1.5` / `k46.1.5`), whose module core takes the match key `zorro:1212:00`; `gayle` still needs only kabi 1.2. `buddha` builds in its own `build/buddha/<line>`, so the two modules build in either order. The default `DLM` is `../amix/kernel/dlm`, with a standalone `../amix-dlm` still used where only that exists. buddha k30 13fa7fa4 (15580 bytes), k46 1a71a253 (16292 bytes); the static objects and `gayle` are unchanged.

## 2026-11-13 — the buddha module: Buddha and Catweasel Z-II IDE (2292d21..cab07c3)

A build flavour of the driver (`-DIDE_BUDDHA`, Buddha code in marked sections of src/ide.c) gives a second module `buddha` for the Individual Computers Buddha (two ports) and Catweasel Z-II (three ports). Boards are found with autocon and each port probed; one sd_register per board at its AutoConfig base, rank ide. Units are fixed per port and drive: 6/5, 3/2, 1/0. Disks and ATAPI drives, master and slave, one command on the board at a time; INT2 through root_defer; nIEN on every drive at halt. Host harness: per-port cables in the mock and ide_budtest, 174 checks per line. The static objects and `gayle` are byte-identical to 1.1.0-5. The module stays out of the default target until the kit accepts a product-00 match key. Tested on the host only.

## 2026-10-31 — docs: the same facts without internal ids or run numbers (13ef41e)

README, CHANGELOG, docs/MODULE-PORT.md and docs/module-form.md reworded; text only, no build input changed.

## 2026-10-27 — gayle module 1.1.0-3: interrupt I/O is the default on both lines (a17f353, b2935fc)

`param intr=0` in modreg.local.d selects polled. Rows 1–6 re-run in emulation on kern46m -82 / kern30m -17, all passed, including
polled via intr=0 and a cdfs mount of the ATAPI CD. Module objects k30 e465bb12, k46 5c8d7d64; static objects unchanged.

## 2026-10-27 — gayle module 1.1.0-2: a page-crossing request is counted, not printed; the ATAPI raw-read finding explained (a8414f9, 1961515, 07c63ae)

The k30 console line `ide: raw request of 512 bytes crosses a page boundary` on every boot was sdpart.c's static RDB buffer at
page offset 0x6cc of a 2 KiB page (no crossing on k46's 4 KiB pages) — a physically contiguous kernel static, harmless; the module
now counts it (ide_pagecross_n) and prints only with param debug=1. Row 4's raw-read finding was the test itself: stock dd.c
hard-codes 512-byte blocks into READ(10), so `dd bs=2048` of an ATAPI unit asks LBA 4N and keeps the first 2 KiB of each 8 KiB
answer — the bytes ARE the CD's (sum 64611 = the ISO's blocks 0,4,8,…), ide_atapi_cmds went 0 → 16 (the before-sample was read);
the real CD check is `mount -F cdfs`. Host: ATAPI-slave cases + the quiet crossing, ide_modtest 157 → 230; static objects unchanged.

## 2026-10-27 — the `gayle` root-class loadable module (2252428, 03465f8, ac1075e, ef60811)

-DIDE_MODULE turns src/ide.c into module `gayle` (module/: Makefile on amix-dlm's ABI kit, Master, VERSION 1.1.0-1). Load:
VPOSR AGA gate → bprobe ALTSTAT 0xDD303A → task-file readback → polled IDENTIFY → sd_register rank ide; silent ENXIO with no
Gayle or no drive; intr=1 (default 0) attaches INT2 via root_defer with a READ VERIFY self-test; _halt sets nIEN; no A1200/A600
rows, 0xDE1000 never touched; counters global. modcheck ok k30.1.2 (11964 B, 5092db6f) and k46.1.2 (12740 B, d2699c2d);
static objects unchanged (k30 af0ef514, k46 bdcd9072). New host tests ide_modtest 157 checks per build, mutation-checked;
harness 1253/1576 unchanged. Shipping waits on amix-dlm's root-context bprobe row. Package rows gayle30/gayle46 proposed.

## 2026-10-27 — docs: the plan for the root-class kernel module port (1820e85, 4918016)

docs/module-form.md (2026-10-21, what the driver needs as a loadable module) and docs/MODULE-PORT.md (the plan under the
amix-dlm contracts C4-2 v3, C4-1 v2, C4-5 v2, C3-5 v8, C3-6 v4; it wins where the two differ). Probe: the VPOSR AGA gate,
a bprobe-guarded ALTSTAT read at 0xDD303A, the task-file readback; 0xDE1000 never touched. sd_register rank ide; INT2 via
root_defer with a READ VERIFY self-test (an INTREQ software raise cannot pass on the A4000); _halt sets nIEN (the A4000 row has
no Gayle INTENA); no DMA calls. Packages gayle-k30/-k46 (gayle30/gayle46). The A4000T has the same task file at 0xDD2020 and
passes the gate: with no drive the module returns ENXIO. Seven open questions for dlm in §7. Docs only; harness 1253/1576 pass.

## 2026-10-14 — README: requirements and troubleshooting (3f81e8e)

68030 needs a 68881/68882: the stock kernel panics at the first signal without one (`sendsig` → `fpu_setup` FRESTORE,
pc = load base + 0x12A, vector 0xB in pid 1), as a tester's FPU-less A1200 card showed. Triage line added.

## 2026-10-11 — real-hardware docs: VPOSR id, the 040/060 line, ATAPI CD read (880ec5c..815e182)

VPOSR id 0x23 confirmed on a real PAL A4000; the 68040/68060 line on a real A4000 with a Z3660 060; an ATAPI CD-ROM
read on real hardware. PR branches pr/1..7 carry the upstream series with the results from a real A4000.

## 2026-10-03 — ATAPI polled commands end only in the status phase (e2c7122..32c0420)

A polled PACKET command is complete only when the status phase is reached; carried into the kernels the build hub assembles.

## 2026-10-02 — 68040/68060 build variant; ATAPI CD-ROM (d19812b, 70e768e)

- **68040/68060 build** (`-DIDE_CACHE=1 -DIDE_NBPP=4096`): data-cache push/invalidate around every PIO buffer, a
  nop ordering barrier before task-file reads, an attach-time DTT0/DTT1 cache-inhibit check, spin budgets x4,
  the `ide_spins_hw` high-water counter; polled-only interrupt shape on the KD line documented; the banner no
  longer divides by 2048. Host harness runs both builds (811 + 1046 checks).
- **ATAPI CD-ROM:** detection by signature and IDENTIFY PACKET DEVICE, CDB passthrough as PACKET commands
  (polled and interrupt-driven), the drive's own sense for REQUEST SENSE, READ(6)/WRITE(6) -> (10), overrun
  drain; `ide_slave_enable` 2 attaches an ATAPI slave only (default 0 unchanged). Mock ATAPI drive in the
  harness (1197 + 1511 checks). Emulator: a 68030 kernel mounts an ATAPI CD on the Gayle slave via cdfs, 22/22
  files byte-identical over 5 rounds, including under CF write load.

## 2026-09-29 (amigaux.org fork)

- **README:** a run-level change requested over telnet while the console sits at `login:` stalls in stock
  init (reproduced without the driver); request it from a console root shell (f80390d).

- **README:** the boot-time page-cross line is sdpart's static RDB buffer, not a driver defect (f130a69).

- **Kernel patches:** `support.c.patch` dropped — the A3000 SCSI auto-detection patch is
  `amix-a4091`'s (with the Agnus-id low-nibble gate) and the build hub applies it to every kernel.
  `kernel.c.patch` regenerated against stock `master.d/kernel.c` so it applies at zero fuzz.
- **Probe:** automatic detection is A4000-only (VPOSR gate, then the A4000 task-file readback); the
  Gayle ID register at `0xDE1000` is read only when an A1200/A600 row is selected with
  `ide_board_force`, never on an A3000 or a driveless A4000. The readback runs before the BSY wait, so
  an empty port no longer spins `ide_spins_reset`.
- **Queue:** `idequeue()` ignores a request re-submitted while it is active or queued (`dd.c`'s
  iodone-before-startio order can do that; `ide_dup_submits` counts). SYNCHRONIZE CACHE and VERIFY(10)
  take the request queue in interrupt mode instead of running inline over a transfer in flight.
- **Recovery:** after a timeout on either path the cable gets a software reset (SRST, bounded !BSY
  wait, STATUS read to drop a pending INTRQ, nIEN restored; `ide_resets` counts) before the next request
  starts; the watchdog counts ticks per request generation.
- **Correctness:** VERIFY(10) executes ATA READ VERIFY SECTOR(S) instead of answering GOOD (BYTCHK
  refused); DF without ERR is a hardware error; the LBA + count range check no longer wraps at 2^32;
  a LUN other than 0 gets LUN NOT SUPPORTED (INQUIRY: qualifier 011b); a full completion FIFO
  delivers nested instead of dropping; interrupts taken while BSY and raw requests crossing a page
  boundary are counted (`ide_intr_busy`, `ide_pagecross_n`, page size `IDE_NBPP`).
- **Docs:** README, design record and source notes aligned (unit 6 / `c6d0`, one byte-order
  formulation, remaining `[verify]` items listed, a section on sharing a kernel with a SCSI adapter,
  licensing note vs the patch's quoted context lines, A1200/A600 marked untested).
- **Host harness:** 439 → 787 checks; the mock gained a command-order log, a hang-until-SRST drive,
  an SRST counter and a device-fault sector.

## 2026-09-28

- **First release.** License: MIT (`LICENSE`). Repository shaped after the
  amix-kerntools driver-author contract (`driver.conf`, `src/`, `src/kernel-patches/` + `NOTICE`,
  `docs/`, `test/host/`).

## 2026-09-27

- **Real hardware: Amix boots from IDE on an Amiga 4000/030** (Commodore A4000/030 card, 68882,
  Kickstart 3.1, SanDisk 1 GB CF on a 44-pin adapter). Kickstart autoboots the `UNIX_Boot` partition,
  the kernel mounts root `c6d0s1` and swap `c6d0s2` through this driver, root login works. IDENTIFY
  model string and sector count decode correctly, confirming the DATA-register byte-order model.
- **Phase 2 — interrupts and board table.** `ideintr()` registered in `int2_tbl[]` via
  `kernel.c.patch`; one interrupt per sector, `timeout()` watchdog, polled fallback when the hook is
  absent. `struct ideboard` confines every board-specific address and interrupt convention (A4000
  entry active; A1200/A600 entries prepared). In the Amiberry A4000/AGA profile: 107k interrupts,
  2 spurious, 0 timeouts; sys time for a 10 MB raw read 3.4 s → 0.7 s. Host harness 439 checks.
- **Phase 1 — polled PIO pseudo SCSI host adapter.** `idepresent()` probe (AGA gate via VPOSR, task-file
  write/readback), `idequeue()` translating READ/WRITE(6)/(10), REQUEST SENSE, INQUIRY, TEST UNIT READY,
  READ CAPACITY(10), MODE SENSE(6), START/STOP, SYNCHRONIZE CACHE to ATA-2 commands with synthesised
  sense data. Compiled by the native Amix `cc` without diagnostics on the first attempt; linked through
  amix-kerntools; boots on an A3000 profile with the probe correctly silent; root on IDE boots in the
  Amiberry A4000/AGA profile. IDE master presented as SCSI unit 6 so a stock A3000-installed image
  boots unchanged.
- `support.c.patch` carried with the AGA gate corrected to the low nibble of the VPOSR id
  (`docs/vposr-aga-gate.md`): `id >= 0x22` misclassifies NTSC ECS machines (0x30) as AGA.
- Architecture decided: pseudo SCSI host adapter (Option A), `docs/decision-hba-vs-block.md`.
