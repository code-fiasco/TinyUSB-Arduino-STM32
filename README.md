# TinyUSB for STM32 (Arduino)

> **Note:** An earlier version of this port is integrated into the official Adafruit TinyUSB library. I will submit the updated version once it is ready and hopefully get the changes made in the stm32 core files so that manual editing of boards.txt is not required.

This port adds STM32F1, STM32F4, and STM32G4 support to the [Adafruit TinyUSB Arduino Library](https://github.com/adafruit/Adafruit_TinyUSB_Arduino), enabling advanced USB device functionality (MIDI, HID, MSC, CDC, etc.) on STM32 microcontrollers.

## What's New

Whilst this port should still be considered experimental, it is now a lot more fleshed out than previous versions:

```cpp
// Previously required in setup():
if (!TinyUSBDevice.isInitialized()) { TinyUSBDevice.begin(0); }
if (TinyUSBDevice.mounted()) { TinyUSBDevice.detach(); delay(10); TinyUSBDevice.attach(); }

// Previously required in loop():
#ifdef TINYUSB_NEED_POLLING_TASK
TinyUSBDevice.task();
#endif
```

This version eliminates all of that. USB initialisation, task polling, CDC flushing, and DFU bootloader entry are all handled automatically — sketches work the same way as on officially supported cores like RP2040 and SAMD. `Serial` is now also automatically aliased to `SerialTinyUSB`, so no `#define` is needed in your sketch, `Serial` commands work like they should. And most importantly no need to press the BOOT and RESET buttons when uploading anymore!

Support has been added and tested for three STM32 families: **F1**, **F4**, and **G4**.

## Compatibility

### Tested and Working
| Board | MCU | Family |
|---|---|---|
| WeAct STM32F411 BlackPill | STM32F411 | F4 |
| STM32F103 BluePill | STM32F103 | F1 |
| WeAct STM32G431 | STM32G431 | G4 |

### Should Work (untested)
- **STM32F4 series:** F401, F405, F407, F446 and other F4xx variants — same OTG_FS peripheral
- **STM32F1 series:** Other F103 variants — same USB peripheral
- **STM32G4 series:** G474, G491, G4A1 — same USB peripheral

### Not Supported (yet)
- STM32F0, F2, F3 — different USB peripheral architecture
- STM32L, STM32H7 — would need clock configuration changes

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

The port implements the three hooks required by the STM32 Arduino core to integrate TinyUSB as a first-class citizen. `initVariant()` is called automatically before `setup()` and handles all USB hardware and stack initialisation. `HAL_IncTick()` is overridden to signal every 1ms SysTick tick, which `yield()` and `serialEventRun()` use to service the TinyUSB task in thread context — ensuring USB events are processed reliably regardless of what the sketch is doing. `TinyUSB_Port_EnterDFU()` handles the Arduino IDE's "touch 1200" auto-reset by disconnecting from USB cleanly, writing the STM32duino bootloader magic value to a backup register, and issuing a system reset.

## Prerequisites
1. [Arduino IDE](https://www.arduino.cc/en/software)
2. [STM32duino Core](https://github.com/stm32duino/Arduino_Core_STM32) (official STMicroelectronics core)
3. [Adafruit TinyUSB Library](https://github.com/adafruit/Adafruit_TinyUSB_Arduino) (install via Library Manager)

## Installation

### Step 1: Download this repository
Download and extract the `stm32` folder from this repository.

### Step 2: Copy port files
Place the `stm32` folder into:
```
libraries/Adafruit_TinyUSB_Library/src/arduino/ports/
```

### Step 3: Edit tusb_config.h
Open `Arduino/libraries/Adafruit_TinyUSB_Library/src/tusb_config.h` and find the platform detection block (around line 30–40):

```cpp
#if defined(ARDUINO_ARCH_SAMD)
  #include "arduino/ports/samd/tusb_config_samd.h"
#elif defined(ARDUINO_NRF52_ADAFRUIT)
  #include "arduino/ports/nrf/tusb_config_nrf.h"
#elif defined(ARDUINO_ARCH_RP2040)
  #include "arduino/ports/rp2040/tusb_config_rp2040.h"
#elif defined(ARDUINO_ARCH_ESP32)
  // do nothing since we force include "arduino/ports/esp32/tusb_config_esp32.h" in tusb_option.h
#elif defined(ARDUINO_ARCH_CH32) || defined(CH32V20x) || defined(CH32V30x)
  #include "arduino/ports/ch32/tusb_config_ch32.h"
#else
  #error TinyUSB Arduino Library does not support your core yet
#endif
```

Add the following lines **before** the `#else`:

```cpp
#elif defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_ARDUINO_CORE_STM32)
  #include "arduino/ports/stm32/tusb_config_stm32.h"
```

### Step 4: Edit boards.txt
Open your STM32 core's `boards.txt`, typically at:
```
Arduino15/packages/STMicroelectronics/hardware/stm32/[version]/boards.txt
```

Find your board's section and add a TinyUSB USB menu entry. The board identifier prefix (e.g. `GenF4`, `GenF1`, `GenG4`) must match the one already used in that board's section. I have included the lines you need to add in `boards_txt_additions.txt` for ease of use. Examples for each family:

**STM32F4 (e.g. Generic F4):**
```
GenF4.menu.usb.TinyUSB=Adafruit TinyUSB
GenF4.menu.usb.TinyUSB.build.usb_flags={build.extra_flags} -DARDUINO_ARCH_TINYUSB
```

**STM32F1 (e.g. Generic F1):**
```
GenF1.menu.usb.TinyUSB=Adafruit TinyUSB
GenF1.menu.usb.TinyUSB.build.usb_flags={build.extra_flags} -DARDUINO_ARCH_TINYUSB
```

**STM32G4 (e.g. Generic G4):**
```
GenG4.menu.usb.TinyUSB=Adafruit TinyUSB
GenG4.menu.usb.TinyUSB.build.usb_flags={build.extra_flags} -DARDUINO_ARCH_TINYUSB
```

### Step 5: Restart Arduino IDE
Close and reopen Arduino IDE for changes to take effect.

### Step 6: Select TinyUSB
In Arduino IDE, go to **Tools > USB** and select **"Adafruit TinyUSB"**.

## Usage

Sketches require no USB initialisation boilerplate. `Serial` works normally. Here is a minimal CDC echo example — the entire sketch is just application code:

```cpp
#include <Adafruit_TinyUSB.h>

void setup() {
  Serial.begin(115200);
  // optional delay
  while (!TinyUSBDevice.mounted()) delay(1);
  Serial.println("Hello from STM32!");
}

void loop() {
  if (Serial.available()) {
    Serial.write(Serial.read());
  }
}
```

> **Note:** `while (!TinyUSBDevice.mounted()) delay(1)` is entirely optional but useful if your sketch needs to send data immediately on startup. It works correctly because `tud_task()` is serviced inside `delay()`.

## File Descriptions

- **`Adafruit_TinyUSB_stm32.cpp`** — Hardware initialisation, IRQ handlers, automatic task polling, and DFU bootloader entry for F1, F4, and G4 families
- **`tusb_config_stm32.h`** — TinyUSB configuration and class enable flags for STM32

## Known Limitations
- Only the F1, F4, and G4 families are currently supported — see compatibility table above
- Requires the official STM32duino core — other STM32 cores are untested
- DFU bootloader entry requires that your board's upload method in Arduino board settings is configured to use STM32CubeProgrammer in DFU mode

## Troubleshooting

**Device not enumerating / "USB device malfunctioned":**
- Verify you selected "Adafruit TinyUSB" from Tools > USB
- Check that USB D+ and D- pins on your board are not used for other purposes
- Try a different USB cable (or plugging directly into the PC if you are using a hub)
- On G4 boards, ensure no other USB stack (e.g. the STM32duino built-in CDC) is also enabled

**Compilation errors:**
- Confirm the Adafruit TinyUSB library is installed via Library Manager
- Make sure USB support in board setting is set to `Adafruit TinyUSB`
- Restart Arduino IDE after making changes to `tusb_config.h` or `boards.txt`
- Verify the `stm32` folder is in the correct location under `ports/`

**"Touch 1200" / DFU upload not working:**
- Confirm your board's upload method is set to STM32CubeProgrammer (DFU)
- Check that the correct DFU drivers are installed on your system (use [Zadig](https://zadig.akeo.ie/) on Windows if needed)

## Contributing
Issues and pull requests welcome! If you test this on other STM32 boards or families, please report your results.

## Hardware Support Requests

Want support for a board or STM32 family not listed here? I'm happy to add it, but I can't afford to buy every STM32 dev board out there. If you send me the board (or cover the cost of it), I'll add support and test it properly.

Get in touch via Ko-fi — use the Commissions feature to describe what you need: 
**[ko-fi.com/yourname](https://ko-fi.com/yourname)**

> ☕ General donations to keep the project going are also very welcome!

## Credits
Based on the [Adafruit TinyUSB Arduino Library](https://github.com/adafruit/Adafruit_TinyUSB_Arduino).