/*
 * Based off Teaching Tech's arduino sketch for spacemouse 
 */

#include <Arduino.h>
#include <BleCompositeHID.h>
#include <GamepadDevice.h>

// TODO: probably revert back to BleGamepad.h
// #include <BleGamepad.h>

// Debugging
// 0: Debugging off. Set to this once everything is working.
// 1: Output raw joystick values. 0-4096 raw ADC 12-bit values
// 2: Output centered joystick values. Values should be approx -500 to +500,
// jitter around 0 at idle.
// 3: Output centered joystick values, filtered for deadzone. Approx -500 to
// +500, locked to zero at idle. 4: Output translation and rotation values.
// Approx -800 to 800 depending on the parameter. 5: Output debug 4 and 5 side
// by side for direct cause and effect reference.
int debug = 5;

// Direction
// Modify the direction of translation/rotation depending on preference. This
// can also be done per application in the 3DConnexion software. Switch between
// true/false as desired.
bool invX = true;   // pan left/right
bool invY = true;   // pan up/down
bool invZ = true;    // zoom in/out
bool invRX = true;   // Rotate around X axis (tilt front/back)
bool invRY = false;  // Rotate around Y axis (tilt left/right)
bool invRZ = false;   // Rotate around Z axis (twist left/right)

// Default Assembly when looking from above:
//    C           Y+
//    |           .
// B--+--D   X-...Z+...X+
//    |           .
//    A           Y-
//

// Multiplexer setup - using CD4051BE
// channels
int MULTIPLEXER_CONTROL_A = 1;
int MULTIPLEXER_CONTROL_B = 2;
int MULTIPLEXER_CONTROL_C = 3;

// Analog read pin for joystick sensing
int MULTIPLEXER_SIG = A0;

// number of potentiometer axes that are read
int NUM_AXES = 8;

// Deadzone to filter out unintended movements. Increase if the mouse has small
// movements when it should be idle or the mouse is too senstive to subtle
// movements.
// Recommended to have this as small as possible for V2 to
// allow smaller knob range of motion.
int DEADZONE = 25;

// todo: use these values to set the sensitivity of the joystick
// sensitivity values for each axis
int TX_SENSITIVITY = 1;
int TY_SENSITIVITY = TX_SENSITIVITY;
int TZ_SENSITIVITY = 1;
int RX_SENSITIVITY = 1;
int RY_SENSITIVITY = RX_SENSITIVITY;
int RZ_SENSITIVITY = 1;

// TODO - implement smoothing
float SMOOTHING_FACTOR = 0.9;
/*
    float smoothedAX = SMOOTHING_FACTOR * previousAX + (1 - SMOOTHING_FACTOR) * centered[AX];
    float smoothedBX = SMOOTHING_FACTOR * previousBX + (1 - SMOOTHING_FACTOR) * centered[BX];
    float smoothedCX = SMOOTHING_FACTOR * previousCX + (1 - SMOOTHING_FACTOR) * centered[CX];
    float smoothedDX = SMOOTHING_FACTOR * previousDX + (1 - SMOOTHING_FACTOR) * centered[DX];

    // Update previous values
    previousAX = smoothedAX;
    previousBX = smoothedBX;
    previousCX = smoothedCX;
    previousDX = smoothedDX;
*/

int NUM_SAMPLES = 3;  // Number of pot samples to take (to smooth the values)
// int SAMPLE_DELAY = 2;  // Delay in milliseconds between pot samples
int MAIN_LOOP_DELAY = 50;  // Delay in milliseconds between HID reports

// Centerpoint variable to be populated during setup routine - stores idle
// position of each axis.
int centerPoints[8];


// Function to read and store analogue voltages for each joystick axis.
// Takes average of NUM_SAMPLES samples from each axis and stores in rawReads
void readAllFromJoystick(int* rawReads) {
  int potValues[NUM_AXES] = {0};  // Array to store pot readings multiplexer

  // Populate readings
  for (int i = 0; i < NUM_AXES; i++) {
    // set channel
    digitalWrite(MULTIPLEXER_CONTROL_A, bitRead(i, 0));
    digitalWrite(MULTIPLEXER_CONTROL_B, bitRead(i, 1));
    digitalWrite(MULTIPLEXER_CONTROL_C, bitRead(i, 2));
    for (int j = 0; j < NUM_SAMPLES; j++) {
      // leave time for the multiplexer to stabilise
      delay(1);
      // take sample from the axis
      potValues[i] += analogRead(MULTIPLEXER_SIG);
    }

  }

  for (int i = 0; i < NUM_AXES; i++) {
    // compute average and store in array
    rawReads[i] = potValues[i] / NUM_SAMPLES;
    // reset potValues
    potValues[i] = 0;
  }
}

#define numOfButtons 0
#define numOfHatSwitches 0

BleCompositeHID compositeHID;
GamepadDevice* gamepad;
GamepadConfiguration bleGamepadConfig;

// Axes are matched to multiplexer channel
#define AX 1
#define AY 0
#define BX 3
#define BY 2
#define CX 5
#define CY 4
#define DX 7
#define DY 6

void setup() {
  // Set up multiplexer pins
  pinMode(MULTIPLEXER_CONTROL_A, OUTPUT);
  pinMode(MULTIPLEXER_CONTROL_B, OUTPUT);
  pinMode(MULTIPLEXER_CONTROL_C, OUTPUT);
  pinMode(MULTIPLEXER_SIG, INPUT);

  // Set up serial
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  // set up the HID report descriptor
  BLEHostConfiguration hostConfig;
  hostConfig.setVid(0x256f);
  hostConfig.setPid(0xc631);

  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType( CONTROLLER_TYPE_MULTI_AXIS);  
  bleGamepadConfig.setButtonCount(numOfButtons);
  bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);
  bleGamepadConfig.setAxesMin(0x8001);  // -32767 --> int16_t - 16 bit signed integer - Can be in  decimal or hexadecimal
  bleGamepadConfig.setAxesMax(0x7FFF);  // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal

  gamepad = new GamepadDevice(bleGamepadConfig);  
  compositeHID.addDevice(gamepad);
  compositeHID.begin(hostConfig);

  // give a little time to stabilise
  delay(1000);
  // determine centre points
  readAllFromJoystick(centerPoints);
}

void loop() {
  int rawReads[NUM_AXES], centered[NUM_AXES];
  unsigned long startTime = millis();
  unsigned long remainingTime;

  if (compositeHID.isConnected()) {
    // Serial.println("\nn--- Axes Decimal ---");
    // Serial.print("Axes Min: ");
    // Serial.println(bleGamepadConfig.getAxesMin());
    // Serial.print("Axes Max: ");
    // Serial.println(bleGamepadConfig.getAxesMax());

    readAllFromJoystick(rawReads);
    // Report back 0-1023 raw ADC 10-bit values if enabled
    if (debug == 1) {
      char buffer[200];
      sprintf(buffer,
              "Raw - AX:%4d, AY:%4d, BX:%4d, BY:%4d, CX:%4d, CY:%4d, DX:%4d, "
              "DY:%4d",
              rawReads[AX], rawReads[AY], rawReads[BX], rawReads[BY],
              rawReads[CX], rawReads[CY], rawReads[DX], rawReads[DY]);
      Serial.println(buffer);
    }

    // Determine current position relative to idle position
    for (int i = 0; i < NUM_AXES; i++) {
      centered[i] = rawReads[i] - centerPoints[i];
    }

    // Report centered joystick values if enabled. Values should be approx -500
    // to +500, jitter around 0 at idle.
    if (debug == 2) {
      char buffer[200];
      sprintf(buffer,
              "Centred - AX:%4d, AY:%4d, BX:%4d, BY:%4d, CX:%4d, CY:%4d, "
              "DX:%4d, DY:%4d",
              centered[AX], centered[AY], centered[BX], centered[BY],
              centered[CX], centered[CY], centered[DX], centered[DY]);
      Serial.println(buffer);
    }

    // Filter movement values. Set to zero if movement is below deadzone
    // threshold.
    for (int i = 0; i < NUM_AXES; i++) {
      if (centered[i] < DEADZONE && centered[i] > -DEADZONE) {
        centered[i] = 0;
      } else {
        // offset values to start from deadzone
        // if (centered[i] < 0) {
        //   centered[i] += DEADZONE;
        // } else {
        //   centered[i] -= DEADZONE;
        // }
      }
    }

    // Report centered joystick values. Filtered for deadzone. Approx -500 to
    // +500, locked to zero at idle
    if (debug == 3) {
      char buffer[200];
      sprintf(buffer,
              "Centred/Filtered - AX:%4d, AY:%4d, BX:%4d, BY:%4d, CX:%4d, "
              "CY:%4d, DX:%4d, DY:%4d",
              centered[AX], centered[AY], centered[BX], centered[BY],
              centered[CX], centered[CY], centered[DX], centered[DY]);
      Serial.println(buffer);
    }

    // TODO: refactor as a vector
    // TODO: improve TX/TY coupled motion detection
    // Declare movement variables at 16 bit int16_t to match HID protocol
    // expectation
    int16_t transX, transY, transZ, rotX, rotY, rotZ;
    // For each axis, we want to check the conditions are met for the axis to be active. 
    // Once met, we compute the translation or rotation value for the axis using a single sensor
    // due to differences in output values between sensors under similar deflection.
    
    // Compute translations
    // X translation requires CY and AY to be active, and CY to be in opposite directions
    if ((abs(centered[CY]) > DEADZONE) && (abs(centered[AY]) > DEADZONE) &&
        (centered[CY] * centered[AY] < 0)) {
        transX = -centered[CY];  // Use CY as the primary source for translation
    } else {
        transX = 0;
    }

    if ((abs(centered[BY]) > DEADZONE) && (abs(centered[DY]) > DEADZONE) &&
        (centered[BY] * centered[DY] < 0)) {
        transY = centered[BY];  // Use BY as the primary source for translation
    } else {
        transY = 0;
    }

    // Z translation requires all axes to be active and in the same direction
    if ((abs(centered[AX]) > DEADZONE) && (abs(centered[BX]) > DEADZONE) &&
        (abs(centered[CX]) > DEADZONE) && (abs(centered[DX]) > DEADZONE) &&
        (centered[AX] < 0 && centered[BX] < 0 && centered[CX] < 0 && centered[DX] < 0) ||
        (centered[AX] > 0 && centered[BX] > 0 && centered[CX] > 0 && centered[DX] > 0)) {
        transZ = -centered[AX];  // Use AX as the primary source for translation
    } else {
        transZ = 0;
    }

    // Compute rotations
    // RX requires CX and AX to be active, and CX to be rotating in the opposite direction
    if ((abs(centered[AX]) > DEADZONE) && (abs(centered[CX]) > DEADZONE) &&
        (centered[AX] * centered[CX] < 0)) {
        rotX = -centered[AX];  // Use AX as the primary source for rotation
    } else {
        rotX = 0;
    }

    if ((abs(centered[BX]) > DEADZONE) && (abs(centered[DX]) > DEADZONE) &&
        (centered[BX] * centered[DX] < 0) ) {
        rotY = centered[BX];  // Use BX as the primary source for rotation
    } else {
        rotY = 0;
    }

    // RZ requires all axes to be active and in the same direction
    if ((abs(centered[AY]) > DEADZONE) && (abs(centered[BY]) > DEADZONE) &&
        (abs(centered[CY]) > DEADZONE) && (abs(centered[DY]) > DEADZONE) &&
        ((centered[AY] > 0 && centered[CY] > 0 && centered[BY] > 0 && centered[DY] > 0) ||
        (centered[AY] < 0 && centered[CY] < 0 && centered[BY] < 0 && centered[DY] < 0))) {
        rotZ = centered[AY];  // Use AY as the primary source for rotation
    } else {
        rotZ = 0;
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
      char buffer[200];
      sprintf(buffer,
              "Transformed - TX:%4d, TY:%4d, TZ:%4d, RX:%4d, RY:%4d, RZ:%4d",
              transX, transY, transZ, rotX, rotY, rotZ);
      Serial.println(buffer);
    }
    // Report debug 4 and 5 info side by side for direct reference if
    // enabled. Very useful if you need to alter which inputs are used in
    // the arithmatic above.
    if (debug == 5) {
      char buffer[400];
      sprintf(buffer,
              "AX:%4d, AY:%4d, BX:%4d, BY:%4d, CX:%4d, CY:%4d, DX:%4d, DY:%4d "
              "|| Transformed - TX:%4d, TY:%4d, TZ:%4d, RX:%4d, RY:%4d, RZ:%4d",
              centered[AX], centered[AY], centered[BX], centered[BY],
              centered[CX], centered[CY], centered[DX], centered[DY], transX,
              transY, transZ, rotX, rotY, rotZ);
      Serial.println(buffer);
    }

    gamepad->setAxes(transX, transY, transZ, rotX, rotY, rotZ);  
    gamepad->sendGamepadReport();
  } else {
    if (debug > 0) {
      Serial.println("BLE not connected");
    }
  }

  remainingTime = MAIN_LOOP_DELAY - (millis() - startTime);
  if (remainingTime > 0) {
    // Serial.print("Main loop delay:"); Serial.println(remainingTime);
    delay(remainingTime);
  }
}
