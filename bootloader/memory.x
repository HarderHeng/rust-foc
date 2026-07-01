/* Bootloader segment: first 16 KB of flash, then 2 KB OTA-flag page,
 * app fills the rest (128 KB total chip).
 *
 * This file is INCLUDEd by cortex-m-rt's `link.x` (line 23), which
 * looks for it relative to cortex-m-rt's own OUT_DIR — `build.rs`
 * copies it there.  If embassy-stm32 also emitted a `memory.x` for
 * this build, ours wins because it sits next to link.x (link-search
 * resolves ties to first match).
 *
 * `FLASH : LENGTH = 16K` overrides embassy's default 128 KB.  The
 * produced binary lands inside the 16 KB segment; physical loading
 * at 0x08000000 (via `probe-rs --base-address`) puts it on chip.
 *
 * `__etext` (provided by cortex-m-rt's link.x) is the absolute
 * end-of-text address.  The ASSERT below is the only link-time
 * guard against bootloader code spilling into the OTA flag page
 * (0x08003800-0x08004000) — that range is page-erased by
 * `FlashOtaFlag::set_pending` and any code sitting on it would be
 * silently destroyed at first OTA use.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 16K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

ASSERT(__etext <= 0x08003800,
       "bootloader code overflowed OTA flag page (must end at or before 0x08003800)")
