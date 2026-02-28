# Changelog

## [v0.3.0] - 2026-02-26

### Added
- STM32WB55 support, tested on WeAct STM32WB55CGU6 using P-NUCLEO-WB55 / NUCLEO WB55 USB Dongle board definition
- Nucleo-64 board definition support — most F1/F4/G4 Nucleo-64 variants should work as a side effect
- `dfu_boot_stm32wb.c` — WB55-specific `Reset_Handler` override for reliable DFU bootloader entry
- WeAct STM32G474 confirmed working
- WeAct STM32F405 confirmed working

### Changed
- DFU bootloader entry completely overhauled — previous implementation was not reliable
  - F1/F4/G4: now uses direct jump to ST ROM bootloader after disabling interrupts and resetting clocks
  - WB55: uses magic value in `BKP0R` + `NVIC_SystemReset()`, intercepted by `Reset_Handler` override before `SystemInit()` runs
- `boards.txt` entries now require four lines per board (two additional lines for `use_1200bps_touch` and `wait_for_upload_port` to enable Arduino IDE auto-upload)
- `boards_txt_additions.txt` updated to reflect new four-line format and added Nucleo_64 entry

### Known Limitations
- Generic WB55 board definitions lack USB clock configuration and are not supported — use P-NUCLEO-WB55 or NUCLEO WB55 USB Dongle board definition instead. Proper generic support requires adding a clock configuration (PLLSAI1Q 48 MHz USB clock) to the generic WB55 variant in the STM32duino core.
- STM32F1 DFU entry requires further validation on boards with factory bootloader.

---

## [v0.2.0] - 2026-02-21

### Added
- STM32F1 support, tested on STM32F103 BluePill
- STM32F4 support, tested on WeAct STM32F411 BlackPill
- STM32G4 support, tested on WeAct STM32G431
- Automatic USB initialisation via `initVariant()` — no sketch boilerplate required
- Automatic task polling via `HAL_IncTick()` override and `serialEventRun()` hook
- `Serial` automatically aliased to `SerialTinyUSB`
- DFU bootloader entry via "touch 1200" auto-reset
- `TinyUSB_Port_GetSerialNumber()` using factory UID registers for all families


## [v0.1.1] - 2026-01-25
### Fixed
- Added missing redirect of `Serial` to `TinyUSBSerial`

## [v0.1.0] - 2026-01-25

### Added
- Initial version with STM32F4 support, tested on WeAct STM32F411 BlackPill
