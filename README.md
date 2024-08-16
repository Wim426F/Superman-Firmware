# Superman controller
This project aims to make the heatpump system used is modern Tesla vehicles easy to use in any EV conversion or else.
The controller plugs directly into the existing wiring harness of the supermanifold and can be controlled by simple inputs, or via CAN.

### Heatpump features:
- 3 coolant loops
  - Motor
  - Battery
  - Radiator
- 3 refrigerant loops
  - Cabin left heat
  - Cabin right heat
  - Cabin cooling
- Heat energy can be moved between any loop for ultra efficiency

### Controller features:
- Digital inputs
  - Heat battery
  - Cool battery
  - Heat cabin left
  - Heat cabin right
  - Cool cabin
  - Enable compressor input
  - Key-on / Wakeup
- Analog inputs
  - 4x temperature sensors for cabin/motor/battery etc.
- ESP32 with webserver onboard for configuration and diagnostics
- CAN-bus for complete control and diagnostics
- PWM outputs
  - 12V pwm for radiator fan
  - 5V pwm for radiator shutter servo
 

# OTA (over the air upgrade)
The firmware is linked to leave the 4 kb of flash unused. Those 4 kb are reserved for the bootloader
that you can find here: https://github.com/jsphuebner/tumanako-inverter-fw-bootloader
When flashing your device for the first time you must first flash that bootloader. After that you can
use the ESP8266 module and its web interface to upload your actual application firmware.
The web interface is here: https://github.com/jsphuebner/esp8266-web-interface

# Compiling
You will need the arm-none-eabi toolchain: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
On Ubuntu type

`sudo apt-get install git gcc-arm-none-eabi`

The only external depedencies are libopencm3 and libopeninv. You can download and build these dependencies by typing

`make get-deps`

Now you can compile stm32-<yourname> by typing

`make`

And upload it to your board using a JTAG/SWD adapter, the updater.py script or the esp8266 web interface.