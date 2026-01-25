# TinyUSB for STM32 (Arduino)

This port adds STM32F4 support to the [Adafruit TinyUSB Arduino Library](https://github.com/adafruit/Adafruit_TinyUSB_Arduino), enabling advanced USB device functionality (MIDI, HID, MSC, etc.) on STM32 microcontrollers.

## Compatibility

### Tested and Working
- STM32F411 BlackPill (WeAct Studio)

### Should Work (but untested)
- Other STM32F4 series (F401, F405, F407, F446, etc.) - uses same USB peripheral
- STM32F7 series - similar OTG_FS peripheral

### Won't Work Without Modification
- STM32F0, F1, F3 series - different USB peripheral architecture
- STM32L series - may need different configuration
- STM32H7 - may need clock configuration changes

**If you test on other boards, please report your results!**

## Features
- ✅ USB MIDI
- ✅ USB CDC (Virtual Serial Port)
- ✅ USB HID (Keyboard/Mouse/Gamepad)
- ✅ USB MSC (Mass Storage)
- ✅ Multiple USB classes simultaneously

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
Open `libraries/Adafruit_TinyUSB_Library/src/tusb_config.h`

Find the platform detection section (around line 30-40). You'll see a series of `#elif` statements like this:
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

**Add the following lines BEFORE the `#else` statement** (not after the `#endif`!):
```cpp
#elif defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_ARDUINO_CORE_STM32)
  #include "arduino/ports/stm32/tusb_config_stm32.h"
```

Your final code should look like this:
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
#elif defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_ARDUINO_CORE_STM32)
  #include "arduino/ports/stm32/tusb_config_stm32.h"
#else
  #error TinyUSB Arduino Library does not support your core yet
#endif
```


### Step 4: Edit boards.txt
Open your STM32 core's `boards.txt` file, typically located at:
```
Arduino15/packages/STMicroelectronics/hardware/stm32/[version]/boards.txt
```

Add these lines to your board configuration (example shown for Generic F4):
```
GenF4.menu.usb.TinyUSBMIDI=Adafruit TinyUSB
GenF4.menu.usb.TinyUSBMIDI.build.usb_flags={build.extra_flags} -DARDUINO_ARCH_TINYUSB
```

**Note:** Adjust `GenF4` to match your specific board identifier if different.

### Step 5: Restart Arduino IDE
Close and reopen Arduino IDE for changes to take effect.

### Step 6: Select TinyUSB
In Arduino IDE, go to **Tools > USB** and select **"Adafruit TinyUSB"**

## Usage Example

Here's a simple USB MIDI example:
```cpp
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>

Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

void setup() {
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
  
  usb_midi.setStringDescriptor("My STM32 MIDI Device");
  MIDI.begin(MIDI_CHANNEL_OMNI);
}

void loop() {
  MIDI.read();
  
  // Send a MIDI note every second
  static uint32_t last_note = 0;
  if (millis() - last_note > 1000) {
    MIDI.sendNoteOn(60, 127, 1);
    delay(100);
    MIDI.sendNoteOff(60, 0, 1);
    last_note = millis();
  }
}
```

## Technical Details

### Key Implementation Notes
- **VBUS Sensing:** Disabled to allow proper enumeration on bus-powered devices
- **USB Peripheral:** Uses STM32F4's OTG_FS controller
- **IRQ Handler:** Forwards USB interrupts to TinyUSB stack
- **Clock Configuration:** Assumes STM32duino core provides correct 48MHz USB clock

### File Descriptions
- **Adafruit_TinyUSB_stm32.cpp** - Platform-specific USB initialization and IRQ handling
- **tusb_config_stm32.h** - TinyUSB configuration for STM32F4

## Known Limitations
- Only tested on STM32F411 (other F4 variants should work but are untested)
- Requires the official STM32duino core (other cores may not work)
- DFU bootloader entry not yet implemented

## Troubleshooting

**Device not enumerating:**
- Verify you selected "Adafruit TinyUSB" from Tools > USB menu
- Check that USB D+ (PA12) and D- (PA11) pins are not being used for other purposes
- Try a different USB cable (some cables are charge-only)

**Compilation errors:**
- Make sure you installed the Adafruit TinyUSB library via Library Manager
- Restart Arduino IDE after making the modifications
- Verify file paths are correct

## Contributing
Issues and pull requests welcome! If you test this on other STM32F4 boards, please report your results.

## Credits
- Based on the [Adafruit TinyUSB Arduino Library](https://github.com/adafruit/Adafruit_TinyUSB_Arduino)
- Port structure inspired by existing CH32, SAMD, and RP2040 implementations

## License
MIT License (same as TinyUSB and Adafruit TinyUSB Library)
