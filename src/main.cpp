/*
 * Based off Teaching Tech's arduino sketch for spacemouse and the BLE Gamepad
 * example from the ESP32 BLE Arduino library
 */

#include <Arduino.h>
//#include <BleGamepad.h>
#include <BleCompositeHID.h>
#include <GamepadDevice.h>

// Debugging
// 0: Debugging off. Set to this once everything is working.
// 1: Output raw joystick values. 0-4096 raw ADC 12-bit values
// 2: Output centered joystick values. Values should be approx -500 to +500,
// jitter around 0 at idle. 
// 3: Output centered joystick values, filtered for deadzone. Approx -500 to +500, locked to zero at idle. 
// 4: Output translation
// and rotation values. Approx -800 to 800 depending on the parameter. 
// 5: Output debug 4 and 5 side by side for direct cause and effect reference.
int debug = 5;

// Direction
// Modify the direction of translation/rotation depending on preference. This
// can also be done per application in the 3DConnexion software. Switch between
// true/false as desired.
bool invX = false;   // pan left/right
bool invY = false;   // pan up/down
bool invZ = true;    // zoom in/out
bool invRX = true;   // Rotate around X axis (tilt front/back)
bool invRY = false;  // Rotate around Y axis (tilt left/right)
bool invRZ = true;   // Rotate around Z axis (twist left/right)

// const int potPin = 0;                // Potentiometer is connected to GPIO 0
// // old : 34 (Analog ADC1_CH6) const int numberOfPotSamples = 5;     // Number
// of pot samples to take (to smooth the values) const int delayBetweenSamples =
// 4;    // Delay in milliseconds between pot samples const int
// delayBetweenHIDReports = 5; // Additional delay in milliseconds between HID
// reports

// Default Assembly when looking from above:
//    C           Y+
//    |           .
// B--+--D   X-...Z+...X+
//    |           .
//    A           Y-
//

// number of potentiometer axes that are read
int NUM_AXES = 4;

// Wiring
int PINLIST[] = {
    // The positions of the reads
    A1,  // X-axis A
    A0,  // Y-axis A
    A3,  // X-axis B
    A2,  // Y-axis B
         //   A7, // X-axis C
         //   A6, // Y-axis C
         //   A9, // X-axis D
         //   A8  // Y-axis D
};

// Deadzone to filter out unintended movements. Increase if the mouse has small
// movements when it should be idle or the mouse is too senstive to subtle
// movements.
int DEADZONE = 12;  // Recommended to have this as small as possible for V2 to
                   // allow smaller knob range of motion.

// sensitivity values for each axis
int TX_SENSITIVITY = 1;
int TY_SENSITIVITY = TX_SENSITIVITY;
int TZ_SENSITIVITY = 1;
int RX_SENSITIVITY = 1;
int RY_SENSITIVITY = RX_SENSITIVITY;
int RZ_SENSITIVITY = 1;

// Centerpoint variable to be populated during setup routine - stores idle
// position of each axis.
int centerPoints[8];

// Function to read and store analogue voltages for each joystick axis.
// TODO: averaging
void readAllFromJoystick(int *rawReads) {
  for (int i = 0; i < NUM_AXES; i++) {
    rawReads[i] = analogRead(PINLIST[i]);
  }
}

BleCompositeHID compositeHID;

// Gamepad with Report ID 1 (Axes: X, Y, Z)
GamepadDevice *gamepadTranslation;
GamepadConfiguration gamepadTranslationConfig;

// Gamepad with Report ID 2 (Axes: RX, RY, RZ)
GamepadDevice *gamepadRotation;
GamepadConfiguration gamepadRotationConfig;

// Gamepad with Report ID 3 (Buttons: 24 buttons)
GamepadDevice *gamepadButtons;
GamepadConfiguration gamepadButtonsConfig;

// Axes are matched to pin order.
#define AX 0
#define AY 1
#define BX 2
#define BY 3
#define CX 4
#define CY 5
#define DX 6
#define DY 7

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEHostConfiguration hostConfig;
  // Set the vendor ID (VID) and product ID (PID)
  hostConfig.setVid(0x256f); 
  hostConfig.setPid(0xc631); 

  // Set the controller type to multi-axis
  gamepadTranslationConfig.setControllerType(CONTROLLER_TYPE_MULTI_AXIS);
  gamepadTranslationConfig.setWhichAxes(true, true, true, false, false, false, false, false); // No buttons in this report
  gamepadTranslationConfig.setAxesMin(-32768);
  gamepadTranslationConfig.setAxesMax(32767);
  gamepadTranslationConfig.setButtonCount(0); // No buttons in this report

  gamepadRotationConfig.setControllerType(CONTROLLER_TYPE_MULTI_AXIS);
  gamepadRotationConfig.setWhichAxes(false, false, false, true, true, true, false, false); // No buttons in this report
  gamepadRotationConfig.setAxesMin(-32768);
  gamepadRotationConfig.setAxesMax(32767);
  gamepadRotationConfig.setButtonCount(0); // No buttons in this report

  gamepadButtonsConfig.setControllerType(CONTROLLER_TYPE_MULTI_AXIS);
  gamepadButtonsConfig.setButtonCount(24); // 24 buttons
  gamepadButtonsConfig.setWhichAxes(false, false, false, false, false, false, false, false); // No axes in this report
  gamepadButtonsConfig.setAxesMin(0); // No axes in this report
  gamepadButtonsConfig.setAxesMax(1);

  gamepadTranslation = new GamepadDevice(gamepadTranslationConfig);
  gamepadRotation = new GamepadDevice(gamepadRotationConfig);
  gamepadButtons = new GamepadDevice(gamepadButtonsConfig);

  // 
  compositeHID.addDevice(gamepadTranslation);
  compositeHID.addDevice(gamepadRotation);
  compositeHID.addDevice(gamepadButtons);

  compositeHID.begin(hostConfig);

  readAllFromJoystick(centerPoints);

  // This portion sets up the communication with the 3DConnexion software. The
  // communication protocol is created here. hidReportDescriptor webpage can be
  // found here:
  // https://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/
  // format: report id prefix, value
  // static const uint8_t _hidReportDescriptor[] PROGMEM = {
  //   0x05, 0x01,           //  Usage Page (Generic Desktop)
  //   Done: 0x09, 0x08,           //  0x08: Usage (Multi-Axis)

  //   0xa1, 0x01,           //  Collection (Application)

  //   0xa1, 0x00,           // Collection (Physical)
  //   Done: 0x85, 0x01,           //  Report ID
  //   0x16, 0x00, 0x80,     //logical minimum (-500)
  //   0x26, 0xff, 0x7f,     //logical maximum (500)
  //   0x36, 0x00, 0x80,     //Physical Minimum (-32768)
  //   0x46, 0xff, 0x7f,     //Physical Maximum (32767)
  //   Done: 0x09, 0x30,           //    Usage (X)
  //   Done: 0x09, 0x31,           //    Usage (Y)
  //   Done: 0x09, 0x32,           //    Usage (Z)
  //   0x75, 0x10,           //    Report Size (16)
  //   0x95, 0x03,           //    Report Count (3)
  //   0x81, 0x02,           //    Input (variable,absolute)
  //   0xC0,                 //  End Collection

  //   0xa1, 0x00,           // Collection (Physical)
  //   0x85, 0x02,           //  Report ID
  //   Done: 0x16, 0x00, 0x80,     //logical minimum (-500)
  //   Done: 0x26, 0xff, 0x7f,     //logical maximum (500)
  //   Done: 0x36, 0x00, 0x80,     //Physical Minimum (-32768)
  //   Done: 0x46, 0xff, 0x7f,     //Physical Maximum (32767)
  //   Done: 0x09, 0x33,           //    Usage (RX)
  //   Done: 0x09, 0x34,           //    Usage (RY)
  //   Done: 0x09, 0x35,           //    Usage (RZ)
  //   0x75, 0x10,           //    Report Size (16)
  //   0x95, 0x03,           //    Report Count (3)
  //   0x81, 0x02,           //    Input (variable,absolute)
  //   0xC0,                 //  End Collection

  //   0xa1, 0x00,           // Collection (Physical)
  //   0x85, 0x03,           //  Report ID
  //   0x15, 0x00,           //   Logical Minimum (0)
  //   0x25, 0x01,           //    Logical Maximum (1)
  //   0x75, 0x01,           //    Report Size (1)
  //   0x95, 32,             //    Report Count (24)
  //   0x05, 0x09,           //    Usage Page (Button)
  //   0x19, 1,              //    Usage Minimum (Button #1)
  //   0x29, 32,             //    Usage Maximum (Button #24)
  //   0x81, 0x02,           //    Input (variable,absolute)
  //   0xC0,

//   0xC0
// };

}

void loop() {
  int rawReads[NUM_AXES], centered[NUM_AXES];
  // Joystick values are read. 0-1023
  if (compositeHID.isConnected()) {
    readAllFromJoystick(rawReads);
    // Report back 0-1023 raw ADC 10-bit values if enabled
    // Refactor debug reports
    if (debug == 1) {
      Serial.print("Raw - ");
      Serial.print("AX:");
      Serial.print(rawReads[0]);
      Serial.print(",");
      Serial.print("AY:");
      Serial.print(rawReads[1]);
      Serial.print(",");
      Serial.print("BX:");
      Serial.print(rawReads[2]);
      Serial.print(",");
      Serial.print("BY:");
      Serial.println(rawReads[3]);
    }

    // Determine current position relative to idle position
    for (int i = 0; i < NUM_AXES; i++) {
      centered[i] = rawReads[i] - centerPoints[i];
    }

    // Report centered joystick values if enabled. Values should be approx -500 to
    // +500, jitter around 0 at idle.
    if (debug == 2) {
      Serial.print("Centred - ");
      Serial.print("AX:");
      Serial.print(centered[0]);
      Serial.print(",");
      Serial.print("AY:");
      Serial.print(centered[1]);
      Serial.print(",");
      Serial.print("BX:");
      Serial.print(centered[2]);
      Serial.print(",");
      Serial.print("BY:");
      Serial.println(centered[3]);
    }

    // Filter movement values. Set to zero if movement is below deadzone
    // threshold.
    for (int i = 0; i < NUM_AXES; i++) {
      if (centered[i] < DEADZONE && centered[i] > -DEADZONE) {
        centered[i] = 0;
      }
    }

    // Report centered joystick values. Filtered for deadzone. Approx -500 to
    // +500, locked to zero at idle
    if (debug == 3) {
      Serial.print("Centred/Filtered - ");
      Serial.print("AX:");
      Serial.print(centered[0]);
      Serial.print(",");
      Serial.print("AY:");
      Serial.print(centered[1]);
      Serial.print(",");
      Serial.print("BX:");
      Serial.print(centered[2]);
      Serial.print(",");
      Serial.print("BY:");
      Serial.println(centered[3]);
    }

    // TODO: refactor as a vector
    // Declare movement variables at 16 bit int16_t to match HID protocol expectation
    int16_t transX, transY, transZ, rotX, rotY, rotZ;  

    // compute translations
    transX = -centered[AY] / TX_SENSITIVITY;
    transY = -centered[BY] / TY_SENSITIVITY;
    if ((abs(centered[AX]) > DEADZONE) && (abs(centered[BX]) > DEADZONE)) {
      transZ = (-centered[AX] - centered[BX]) / TZ_SENSITIVITY;
      transX = 0;
      transY = 0;
    } else {
      transZ = 0;
    }
    
    // compute rotations
    rotZ = 0;
    rotX = (-centered[AX]) / RX_SENSITIVITY;
    rotY = (+centered[BX]) / RY_SENSITIVITY;

    // check for Z axis rotation - necessary to differentiate from coupled +X-Y translation
    if ((abs(centered[AY]) > DEADZONE) && (abs(centered[BY]) > DEADZONE)) {
      int diff = abs(centered[AY] - centered[BY]);
      // if AY and BY values are similar (within threshold), it's a rotation rather than coupled translation
      // if AY and BY values are substantially different, it's a coupled translation
      if (diff < DEADZONE) {
        rotZ = (-centered[AX] + centered[AY]) / RZ_SENSITIVITY;
        transX = 0;
        transY = 0;
        rotX = 0;
        rotY = 0;
      }
    } 

    // TODO: refactor as a vector
    // Invert directions if needed
    if (invX == true) {
      transX = transX * -1;
    };
    if (invY == true) {
      transY = transY * -1;
    };
    if (invZ == true) {
      transZ = transZ * -1;
    };
    if (invRX == true) {
      rotX = rotX * -1;
    };
    if (invRY == true) {
      rotY = rotY * -1;
    };
    if (invRZ == true) {
      rotZ = rotZ * -1;
    };

    // Report translation and rotation values if enabled. Approx -800 to 800
    // depending on the parameter.
    if (debug == 4) {
      Serial.print("Transformed - ");
      Serial.print("TX:");
      Serial.print(transX);
      Serial.print(",");
      Serial.print("TY:");
      Serial.print(transY);
      Serial.print(",");
      Serial.print("TZ:");
      Serial.print(transZ);
      Serial.print(",");
      Serial.print("RX:");
      Serial.print(rotX);
      Serial.print(",");
      Serial.print("RY:");
      Serial.print(rotY);
      Serial.print(",");
      Serial.print("RZ:");
      Serial.println(rotZ);
    }
    // Report debug 4 and 5 info side by side for direct reference if enabled.
    // Very useful if you need to alter which inputs are used in the arithmatic
    // above.
    if (debug == 5) {
      Serial.print("AX:");
      Serial.print(centered[0]);
      Serial.print(",");
      Serial.print("AY:");
      Serial.print(centered[1]);
      Serial.print(",");
      Serial.print("BX:");
      Serial.print(centered[2]);
      Serial.print(",");
      Serial.print("BY:");
      Serial.print(centered[3]);
      Serial.print("||");
      Serial.print("Transformed - ");
      Serial.print("TX:");
      Serial.print(transX);
      Serial.print(",");
      Serial.print("TY:");
      Serial.print(transY);
      Serial.print(",");
      Serial.print("TZ:");
      Serial.print(transZ);
      Serial.print(",");
      Serial.print("RX:");
      Serial.print(rotX);
      Serial.print(",");
      Serial.print("RY:");
      Serial.print(rotY);
      Serial.print(",");
      Serial.print("RZ:");
      Serial.println(rotZ);
    }

    // set data for the BLE gamepad
    (*gamepadTranslation).setAxes(transX, transY, transZ);
    (*gamepadRotation).setRX(rotX);
    (*gamepadRotation).setRY(rotY);
    (*gamepadRotation).setRZ(rotZ);

  } else {
    Serial.println("BLE not connected");
  }
  delay(100);
}

/*void loop()
{
    if (bleGamepad.isConnected())
    {
        int potValues[numberOfPotSamples]; // Array to store pot readings
        int potValue = 0;                  // Variable to store calculated pot
reading average

        // Populate readings
        for (int i = 0; i < numberOfPotSamples; i++)
        {
            potValues[i] = analogRead(potPin);
            potValue += potValues[i];
            delay(delayBetweenSamples);
        }

        // Calculate the average
        potValue = potValue / numberOfPotSamples;

        // Map analog reading from 0 ~ 4095 to 32737 ~ 0 for use as an axis
reading int adjustedValue = map(potValue, 0, 4095, 32737, 0);

        // Update X axis and auto-send report
        bleGamepad.setX(adjustedValue);
        delay(delayBetweenHIDReports);

        // The code below (apart from the 2 closing braces) is for pot value
degugging, and can be removed
        // Print readings to serial port
        Serial.print("Sent: ");
        Serial.print(adjustedValue);
        Serial.print("\tRaw Avg: ");
        Serial.print(potValue);
        Serial.print("\tRaw: {");

        // Iterate through raw pot values, printing them to the serial port
        for (int i = 0; i < numberOfPotSamples; i++)
        {
            Serial.print(potValues[i]);

            // Format the values into a comma seperated list
            if (i == numberOfPotSamples - 1)
            {
                Serial.println("}");
            }
            else
            {
                Serial.print(", ");
            }
        }
    }
}*/