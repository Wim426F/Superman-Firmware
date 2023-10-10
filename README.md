# Superman controller
This project aims to make the heatpump system used is modern Tesla vehicles easy to use in any EV conversion or else.
The controller plugs directly into the existing wiring harness of the supermanifold and can be controlled by simple inputs, or via CAN.

### Heatpump in short:
- 3 coolant loops
  - Motor
  - Battery
  - Radiator
- 3 refrigerant loops
  - Cabin left heat
  - Cabin right heat
  - Cabin cooling
- Heat energy can be moved back and forth in any loop

### Controller in short:
- Digital inputs
  - Heat battery
  - Cool battery
  - Heat cabin left
  - Heat cabin right
  - Cool cabin
  - Enable input
  - General purpose input
- Analog inputs
  - 4x temperature sensors for cabin/motor/battery etc.
- RS232 port for openinverter esp8266 (not onboard)
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

# Editing
The repository provides a project file for Code::Blocks, a rather leightweight IDE for cpp code editing.
For building though, it just executes the above command. Its build system is not actually used.
Consequently you can use your favority IDE or editor for editing files.

# Adding classes or modules
As your firmware grows you probably want to add classes. To do so, put the header file in include/ and the 
source file in src/ . Then add your module to the object list in Makefile that starts in line 43 with .o
extension. So if your files are called "mymodule.cpp" and "mymodule.h" you add "mymodule.o" to the list.

When changing a header file the build system doesn't always detect this, so you have to "make clean" and
then make. This is especially important when editing the "*_prj.h" files.
