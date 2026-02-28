# TinyUSB for STM32 (Arduino)

> **Note:** An earlier version of this port with partial support for the F4 family is integrated into the official Adafruit TinyUSB library. I will submit the updated version once it is ready and hopefully get the changes made in the stm32 core files so that manual editing of boards.txt is not required.

This port adds STM32F1, STM32F4, STM32G4, and STM32WB55 support to the [Adafruit TinyUSB Arduino Library](https://github.com/adafruit/Adafruit_TinyUSB_Arduino), enabling advanced USB device functionality (MIDI, HID, MSC, CDC, etc.) on STM32 microcontrollers.

## Overview

The Adafruit TinyUSB library includes an initial verion of this STM32 port, currently listed as experimental. This repository extends that work — adding support for additional STM32 families, fully automatic USB initialisation and task polling, and reliable DFU bootloader entry, with no boilerplate required in your sketch. `Serial` is automatically aliased to `SerialTinyUSB`. The goal is that sketches written for officially supported cores like RP2040 and SAMD should work on STM32 without modification. 

Support has been tested across four STM32 families: **F1**, **F4**, **G4**, and **WB55**. See the [Changelog](CHANGELOG.md) for the full history of what's been added and changed.

## Compatibility

### Tested and Working
| Board | MCU | Family | Board Definition |
|---|---|---|---|
| WeAct STM32F411 BlackPill | STM32F411 | F4 | Generic F4 |
| WeAct STM32F405 | STM32F405 | F4 | Generic F4 |
| STM32F103 BluePill | STM32F103 | F1 | Generic F1 |
| WeAct STM32G431 | STM32G431 | G4 | Generic G4 |
| WeAct STM32G474 | STM32G474 | G4 | Generic G4 |
| WeAct STM32WB55CGU6 | STM32WB55 | WB55 | Nucleo WB55 USB Dongle / P-NUCLEO-WB55 |

### Should Work (untested)
- **STM32F4 series:** F401, F407, F446 and other F4xx variants — same OTG_FS peripheral
- **STM32F1 series:** Other F103 variants — same USB peripheral
- **STM32G4 series:** G491, G4A1 — same USB peripheral
- **STM32WB55 series:** Other WB55 variants with 32MHz HSE crystal using Nucleo board definition
- **Nucleo-64 boards:** Most F1/F4/G4 Nucleo-64 variants should work via the Nucleo_64 board definition

### Not Supported
- STM32F0, F2, F3 — different USB peripheral architecture
- STM32L, STM32H7 — would need clock configuration changes
- Generic WB55 board definitions — see Known Limitations

**If you test on other boards, please report your results!**

## Features
- ✅ USB MIDI
- ✅ USB CDC (Virtual Serial Port)
- ✅ USB HID (Keyboard / Mouse / Gamepad)
- ✅ USB MSC (Mass Storage)
- ✅ Multiple USB classes simultaneously
- ✅ No sketch boilerplate — USB initialises and polls automatically
- ✅ `Serial` works without any `#define` in your sketch
- ✅ Arduino IDE "touch 1200" DFU bootloader entry

## How It Works

The port implements the hooks required by the STM32 Arduino core to integrate TinyUSB as a first-class citizen. `initVariant()` is called automatically before `setup()` and handles all USB hardware and stack initialisation. `HAL_IncTick()` is overridden to signal every 1ms SysTick tick, which `yield()` and `serialEventRun()` use to service the TinyUSB task in thread context — ensuring USB events are processed reliably regardless of what the sketch is doing. DFU bootloader entry is triggered by the Arduino IDE's "touch 1200" sequence: on F1/F4/G4 the port disables interrupts, resets clocks, and jumps directly to the ST ROM bootloader; on WB55 a magic value is written to a backup register and the chip resets, with a `Reset_Handler` override intercepting the reboot before `SystemInit()` runs to jump to the bootloader from a near-clean state.

## Prerequisites
1. [Arduino IDE](https://www.arduino.cc/en/software)
2. [STM32duino Core](https://github.com/stm32duino/Arduino_Core_STM32) (official STMicroelectronics core)
3. [Adafruit TinyUSB Library](https://github.com/adafruit/Adafruit_TinyUSB_Arduino) (v3.7.4 or higher - install via Library Manager)

## Installation

### Step 1: Download this repository
Download and extract the `stm32` folder from this repository.

### Step 2: Copy port files
Place the `stm32` folder into:
```
libraries/Adafruit_TinyUSB_Library/src/arduino/ports/
```

### Step 3: Edit boards.txt
Open your STM32 core's `boards.txt`, typically at:
```
Arduino15/packages/STMicroelectronics/hardware/stm32/[version]/boards.txt
```

Refer to `boards_txt_additions.txt` (included in this repository) for the exact lines to add and instructions on finding the right prefix for your board. Each board entry now requires four lines — two to enable TinyUSB and two to enable "touch 1200" DFU upload.

### Step 4: Restart Arduino IDE
Close and reopen Arduino IDE for changes to take effect.

### Step 5: Select TinyUSB
In Arduino IDE, go to **Tools > USB** and select **"Adafruit TinyUSB"**.

## Usage

Sketches require no USB initialisation boilerplate. `Serial` works normally. 

## File Descriptions

- **`Adafruit_TinyUSB_stm32.cpp`** — Hardware initialisation, IRQ handlers, automatic task polling, and DFU bootloader entry for F1, F4, G4, and WB55 families
- **`tusb_config_stm32.h`** — TinyUSB configuration and class enable flags for STM32
- **`dfu_boot_stm32wb.c`** — WB55-specific Reset_Handler override for reliable DFU bootloader entry
- **`boards_txt_additions.txt`** — Reference file with the lines to add to the STM32duino core's `boards.txt`

## Known Limitations
- Generic WB55 board definitions (e.g. Generic WB55CGUx) are not supported — these variants lack a `SystemClock_Config` implementation in the STM32duino core, so the USB peripheral never receives a valid 48MHz clock. Use the **P-NUCLEO-WB55** or **NUCLEO WB55 USB Dongle** board definition instead. Custom WB55 boards with a 32MHz HSE crystal should also work with these definitions. A proper fix would require a `generic_clock.c` to be added to the generic WB55 variant in the STM32duino core upstream — this may be raised as a separate issue or PR.
- The "Adafruit TinyUSB" option will appear in the IDE USB menu for unsupported Nucleo-64 variants — this is a side effect of the WB55 workaround.
- Requires the official STM32duino core — other STM32 cores are untested.
- DFU bootloader entry requires that your board's upload method is configured to use STM32CubeProgrammer in DFU mode.

## Troubleshooting

**Device not enumerating / "USB device malfunctioned":**
- Verify you selected "Adafruit TinyUSB" from Tools > USB
- Check that D− and D+ pins on your board are not used for other purposes
- Try a different USB cable (some cables are charge-only)
- On WB55, ensure you are using the P-NUCLEO-WB55 or NUCLEO WB55 USB Dongle board definition, not a generic WB55 variant

**Compilation errors:**
- Confirm the latest version of Adafruit TinyUSB library is installed via Library Manager
- Make sure USB support in board setting is set to `Adafruit TinyUSB`
- Restart Arduino IDE after making changes to `tusb_config.h` or `boards.txt`
- Verify the `stm32` folder is in the correct location under `ports/`
- Verify `U(S)ART Support` in board options is set to `Enabled (generic 'serial')`

**"Touch 1200" / DFU upload not working:**
- Confirm all four lines are present in `boards.txt` for your board (see `boards_txt_additions.txt`)
- Confirm your board's upload method is set to STM32CubeProgrammer (DFU)
- Check that the correct DFU drivers are installed on your system (use [Zadig](https://zadig.akeo.ie/) on Windows if needed)
- One sketch must be uploaded using manual DFU (or other method) once the port is installed in order to enable the automated DFU push

## Contributing
Issues and pull requests welcome! If you test this on other STM32 boards or families, please report your results.

## Hardware Support Requests

Want support for a board or STM32 family not listed here? I'm happy to add it, but I can't afford to buy every STM32 dev board out there. If you send me the board (or cover the cost of it), I'll add support and test it properly.

Get in touch via Ko-fi — use the Commissions feature to describe what you need: **[ko-fi.com/codefiasco](https://ko-fi.com/codefiasco)**

**☕ General donations to keep the project going are also very welcome!**

## Credits
Based on the [Adafruit TinyUSB Arduino Library](https://github.com/adafruit/Adafruit_TinyUSB_Arduino).
