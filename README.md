# Wireless Space Mouse

This is a wireless variant of [Teaching Tech's space mouse](https://www.printables.com/model/864950-open-source-spacemouse-space-mushroom-remix). It uses an ESP32 C3 (Super Mini) to send 
movement data over bluetooth. The device emulates a 3DConnexion wireless mouse, so it works with 3DConnexion software. 

## BOM

TODO

## Changes from the original design

### Electronics changes

- ESP32 C3 (Super Mini) instead of Arduino Pro Micro. Chosen because it has bluetooth, and is also quite cheap.
- 3.7V LiPo battery + charger board (in work)
- CD4051 multiplexer to read all axes, as the ESP32 C3 has fewer ADC pins

### Hardware Changes

- TODO: modify the 3D printed case 
  - reduce clearance between joystick balls and knob to reduce play
    - print with smaller layer height for smoother surfaces
    - https://www.printables.com/model/893084-tighter-tolerance-balljoint
  - add bosses for mounting the ESP32 C3 and extra circuit board
  - add a hole for the USB-C charging port
- TODO: move to hall effect joysticks?

### Software changes

- Using BleCompositeHID library
- Reworked axis reading logic
  - motion on each axis is now determined by first checking conditions are met (dependent on directions of motion of multiple axes), and then calculating the motion based on a single joystick reading. 
  - As different axes had different deadzones and produced different readings for similar inputs, I was getting some annoying coupling (e.g. a Z translation move was registering first as RX and RY). This change fixed that.
- Added averaged axis sampling to reduce noise

### Possible future changes

- implement constants for sensitivity of each axis
- add smoothing
- battery level reporting
- add a power switch
- sleep mode
- add a charging indicator/low battery LED
- add macro buttons

## Schematic

- TODO

## Setup guide

- remove caps from joysticks
- place joystick balls on joystick stems
- drill ... TODO
- open the project in PlatformIO in VSCode
- dependencies should install based on the `platformio.ini` file
- upload the code to the ESP32 C3
- pair the device with your computer
- open 3DConnexion software and check that the device is detected
- use debug settings and serial monitor to check mouse outputs
- adjust sensitivity and deadzones as needed



- TODO

## Related projects

- https://github.com/AndunHH/spacemouse

