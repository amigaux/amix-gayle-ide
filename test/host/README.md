# Host test harness for `src/ide.c`

Compiles the **real** driver source with the host C compiler against a mock of the
A4000 Gayle IDE port and two ATA-2 drives, and checks the SCSI→ATA translation byte
for byte. No Amiga, no emulator, sub-second turnaround.

```sh
make -C test/host check     # build + run; exit 0 == every assertion passed
make -C test/host clean
```

## How the driver is compiled here

`ide.c` is K&R C and is built with `-std=gnu89` in a kernel-only include world:
`-Istubs` shadows `sys/types.h`, `sys/errno.h`, `sys/inline.h`, `rico.h` and `sd.h`
with minimal shims (the `struct sdcom` layout is the kernel ABI, copied field for
field). `-DHOST_TEST -include mock_regs.h` routes the driver's register seam
(`IDE_RB/IDE_WB/IDE_RW/IDE_WW/IDE_IRQSTAT/IDE_VPOSR`) into `mock_gayle.c`. Without
`HOST_TEST` the driver source is unchanged and the macros are plain volatile
dereferences of the identity-mapped addresses.

`kstubs.c` supplies `spl2()/splx()` (counted, so the harness can assert the
bracket is balanced) and a `printf()` that records the driver's console output.

## What the mock models

- Register map as defined at the top of `ide.c` (offsets relative to `0xDD2020`, control block at
  `0x101A`); unknown offsets abort the test.
- Two drives with an in-memory disk image, IDENTIFY DEVICE (model string, LBA
  capability, LBA28 sector count, FLUSH CACHE support), READ/WRITE SECTOR(S) with
  the DRQ-per-sector state machine, FLUSH CACHE, ABRT for anything else, IDNF for
  out-of-range LBAs, an injectable UNC at a chosen LBA, a "stuck BSY" knob, an
  absent master (open bus, `0xFF`) and an absent slave.
- **Byte order:** a 16-bit DATA read returns `(stream[0] << 8) | stream[1]` where
  `stream` is the byte sequence the drive presents in little-endian word order, the
  same model WinUAE/Amiberry use for Gayle. On the 68k that puts sector bytes in
  natural order and IDENTIFY words byte-swapped (which the driver undoes once).
  On a little-endian host the driver's integer word stores land the two bytes
  swapped, so `disk_equals()` in `ide_test.c` compares against a pair-swapped
  image there. The driver source is identical in both worlds.
- `VPOSR` (chipset id) so the probe's AGA gate can be tested on an "A3000".

## Gating assertions (`ide_test.c`)

- Probe: on an ECS chipset no IDE register is touched; on AGA without a drive the
  probe reports absent without issuing a command; with a drive it reports the base,
  disables drive interrupts, is cached, never selects the slave.
- Attach: one IDENTIFY per unit, model string in order, sector count decoded.
- INQUIRY / READ CAPACITY(10) / MODE SENSE(6) shapes, `nbyte` respected.
- READ(10) single sector, 16 sectors (a UFS block), short `nbyte` (drive drained),
  300 sectors (split into two ATA commands), READ(6); WRITE(10) and read-back.
- Errors: LBA out of range (rejected before the drive), UNC, unknown opcode,
  stuck BSY (bounded spin, no hang) → CHECK CONDITION and the expected
  REQUEST SENSE key/ASC, consumed on read; `okay == TRUE` for all of them.
- Slave: never selected while `ide_slave_enable == 0`; works when enabled; an
  enabled but absent slave does not disturb the master.
- Completion: delivered exactly once per request, iteratively (a request issued from
  inside the completion does not nest), spl bracket balanced, FIFO never overflows.
- CHS-only drives are refused with a console message.
- ATAPI CD-ROM (`mock_add_cdrom`: signature, IDENTIFY PACKET, PACKET with CoD/IO phases of the
  host's byte count or a forced chunk, sense held for REQUEST SENSE, no DRDY): attach per slave mode
  0/1/2, the cdfs mount sequence, multi-phase and odd-length transfers, the overrun drain, the
  drive's sense vs the driver's own, READ(6) → READ(10), CDB tail zeroing, data out, 16-byte
  packets, a hung drive (polled budget and watchdog, SRST), CD and CF requests queued together,
  the polled status phase (a drive that shows BSY/DRQ clear before its data phase, as Amiberry's
  does, and one that never sets IREASON CoD|IO at the end).
- The 68040/68060 build (`-DIDE_CACHE=1 -DIDE_NBPP=4096`, `ide_test_040`): every buffer line pushed
  and invalidated before the transfer and a read's again after it, nothing outside the buffer, the
  TTR cache-inhibit check on five DTT shapes, no task-file read ahead of an unsynchronised write
  (asserted after every request), the ×4 spin budgets.
- The diagnostic build (`-DIDE_DIAG`, `ide_test_diag` and `ide_test_040_diag`): the spin
  high-water `ide_spins_hw`, and on the 68040/68060 line the push counters and the `ide_cache = 0`
  A/B switch. The default builds declare those symbols weak and check that they are absent, and that
  a 68040/68060 READ still pushes and invalidates its buffer before and after the transfer.
- The root-class module form (`-DIDE_MODULE`, `ide_modtest` and `ide_modtest_040`, from
  `ide_modtest.c`, which mocks `bprobe`, `sd_register`, `root_defer`, `intr_attach`/`intr_detach`
  and `sleep`/`wakeup`): the probe order (one VPOSR read, the Kickstart ROM walk, then `bprobe` of
  the row's ALTSTAT, `0xDD303A` or `0xDA101A`, then the task-file readback, no command before
  IDENTIFY); a silent, unregistered `ENXIO` on OCS/ECS (the ROM not read), an A3000, an A600, a
  `bprobe` bus error and an empty port; the board choice against a mock ROM window (an A4000-style
  ROM with the string `card.resource` and a tag-shaped decoy, an A1200 ROM with it resident, a ROM
  with no tags, an A1200 ROM on an A4000) and the `board` parameter (-1 touches nothing, 1/2 skip the
  ROM); the A1200 path never touching `0xDD0000`–`0xDDFFFF`, the A4000 path never
  `0xDA0000`–`0xDAFFFF`; the registration (`idequeue`, card 0, `0xDD2020`, `"A4000 IDE"` or
  `0xDA0000`, `"A1200 IDE"`, rank 4) and polled requests through it; the A1200 self-test (a stale
  latch acknowledged, the interrupt enable set, the latch clear after the acknowledge, interrupt
  I/O; a latch that does not clear and a mute drive give polled I/O with the enable cleared; the
  PCMCIA enable bits preserved; `_halt` and `_unload` clear the enable); no spl,
  timeout or sleep in the load entry; the parameters (defaults, overrides, bad values); `intr=1`:
  `root_defer`, `intr_attach` at level 2, one READ VERIFY, interrupt I/O on a pass, polled with nIEN
  set and the handler detached on a mute drive, a refused attach or an ATAPI-only cable;
  `sd_register` and `root_defer` errors; `_halt` (nIEN, no command) and `_unload`; and **no access
  to the Gayle ID register `0xDE1000` on any module path** (`mock_id_reads` + `mock_id_writes`
  summed over every case), and none of the diagnostic-build symbols.
- The Buddha / Catweasel Z-II module (`-DIDE_MODULE -DIDE_BUDDHA`, `ide_budtest` and
  `ide_budtest_040`, from `ide_budtest.c`, which mocks `autocon` beside the module form's kernel
  calls) against the mock's Buddha boards (`MOCK_BOARD_BUDDHA`: one cable of two drives per port,
  the Gayle being cable 0; any access a board does not allow is counted in `mock_bud_forbidden`):
  discovery (`autocon` for 0x12120000, then 0x1212002A; no board, no answering port or no drive is a
  silent `ENXIO`; no chipset or ROM gate); `bprobe` of each port's alternate status (board + 0x91A,
  0xB1A, and 0xD1A on a Catweasel Z-II), a faulting port left alone; units 6/5, 3/2, 1/0 reaching
  their port and drive for reads and writes, 4 and 7 unused; one `sd_register` per board with its
  index and AutoConfig base, two boards each reached through its own card, polled and interrupt
  driven; ATAPI on a port's slave and alone on a port; the self-test (the enable written once per
  board, every port with a drive tested, nIEN cleared on all on a pass, kept on all with the handler
  detached when a port stays silent or `intr_attach` refuses); a stray INTRQ on an idle port dropped;
  parameters and `sd_register`/`root_defer` errors; `_halt` and `_unload`; and no access outside the
  allowed registers in any case (the mock's own counting checked first).
