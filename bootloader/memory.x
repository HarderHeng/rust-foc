/* Bootloader segment: 0x0800_0000 - 0x0800_4000 (16 KB).
 * App starts at 0x0800_4000 (defined in app's memory.x).
 * The first 2KB page within the bootloader segment is reserved for
 * config (e.g. OTA_FLAG at 0x0800_3F00); linker fills it with
 * padding if the code is shorter than 16KB.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 16K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
