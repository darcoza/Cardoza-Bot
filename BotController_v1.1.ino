#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

int leftWheelDirectionPin = 9;
int leftWheelSpeedPin = 10;
int rightWheelDirectionPin = 20;
int rightWheelSpeedPin = 21;
int connectedPin = 4;

#define forward 255
#define reverse 0


// Arduino setup function. Runs in CPU 1
void setup() 
{
    //my stuff
    pinMode(leftWheelDirectionPin, OUTPUT);
    pinMode(leftWheelSpeedPin, OUTPUT);
    pinMode(connectedPin, OUTPUT);
    
    digitalWrite(connectedPin, LOW);

    analogWrite(leftWheelDirectionPin, 0);
    analogWrite(leftWheelSpeedPin, 0);


    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();

    // Enables mouse / touchpad support for gamepads that support them.
    // When enabled controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "vritual device", is a mouse
    // By default it is disabled.
    BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        ControllerPtr myController = myControllers[i];

        if (myController && myController->isConnected()) 
        {
            digitalWrite(connectedPin, HIGH);
            if (myController->isGamepad()) {
                processGamepad(myController);
            } 
            else 
            {
                Serial.printf("Data not available yet\n");
                digitalWrite(connectedPin, LOW);
                continue;
            }
            // See ArduinoController.h for all the available functions.
        }
        else
        {
          digitalWrite(connectedPin, LOW);
        }
    }
    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

     vTaskDelay(10);
    //delay(150);
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    
    }
}
int axisX = 0;



void dumpGamepad(ControllerPtr ctl) 
{
    //new gpt code
    int axisX = ctl->axisRX();
    int axisY = ctl->axisY();
    int throttle = ctl->throttle();
    int brake = ctl->brake();

    // Apply a deadzone to the X-axis
    if (abs(axisX) < 30) {
        axisX = 0;
    }
    if (abs(axisY) < 30) {
        axisY = 0;
    }
    
    //trim brake/throttle down
    brake = map(brake, 0, 1023, 0, 255);
    throttle = map(throttle, 0, 1023, 0, 255);

    // Calculate motor speeds based on joystick and trigger values
    int leftSpeed = axisY - axisX;
    int rightSpeed = axisY + axisX;

        // Ensure motor speeds are within the valid range (0-255)
    //leftSpeed = constrain(leftSpeed, -255, 255);
    //leftSpeed = map(leftSpeed, -512, 512, -255, 255);//regular
    leftSpeed = map(leftSpeed, -512, 512, -100, 100);//slow
    //rightSpeed = constrain(rightSpeed, -255, 255);
    //rightSpeed = map(rightSpeed, -512, 512, -255, 255);//regular
    rightSpeed = map(rightSpeed, -512, 512, -100, 100);//slow


    // Print debug information
    Serial.print("axisX: ");
    Serial.print(axisX);
    Serial.print(", axisY: ");
    Serial.print(axisY);
    //Serial.print(", brake: ");
    //Serial.print(brake);
    Serial.print(", leftSpeed: ");
    Serial.print(leftSpeed);
    Serial.print(", rightSpeed: ");
    Serial.println(rightSpeed);


    //Special if throttle > 0 lunge forward fullspeed
    if(throttle > 20)//small dead zone
    {
      analogWrite(leftWheelDirectionPin, 0);
      analogWrite(leftWheelSpeedPin, 255);
      analogWrite(rightWheelDirectionPin, 0);
      analogWrite(rightWheelSpeedPin, 255);
    }
    else if(brake > 20)//if brake go full speed backwards
    {
      analogWrite(leftWheelDirectionPin, 255);
      analogWrite(leftWheelSpeedPin, 0);
      analogWrite(rightWheelDirectionPin, 255);
      analogWrite(rightWheelSpeedPin, 0);
    }
    else// otherwise steer normally
    {
        // Set motor directions and speeds
      if (leftSpeed >= 0) 
      {
        analogWrite(leftWheelDirectionPin, abs(leftSpeed));
        analogWrite(leftWheelSpeedPin, 0);
      } 
      else 
      {
        analogWrite(leftWheelDirectionPin, 0);
        analogWrite(leftWheelSpeedPin, abs(leftSpeed));
      }

      if (rightSpeed <= 0) 
      {
        analogWrite(rightWheelDirectionPin, 0);
        analogWrite(rightWheelSpeedPin, abs(rightSpeed));
      } 
      else 
      {
        analogWrite(rightWheelDirectionPin, abs(rightSpeed));
        analogWrite(rightWheelSpeedPin, 0);
      }

    }
    
    
    
    
    
    /* //old code I wrote
    axisX = ctl->axisX();
    if(abs(axisX) < 20)//creates a small deadzone
    {
      axisX = 0;
    }
    if(axisX < 0)
    {
      int temp = map(abs(axisX), 0, 512, 0, 255);
      Serial.print("axis < 0 = ");
      Serial.println(temp);
      analogWrite(leftWheelDirectionPin, 0);
      analogWrite(leftWheelSpeedPin, temp);
    }
    else if(axisX > 0)
    {
      int temp = map(abs(axisX), 0, 511, 0, 255);
      Serial.print("axis > 0 = ");
      Serial.println(temp);
      analogWrite(leftWheelDirectionPin, temp);
      analogWrite(leftWheelSpeedPin, 0);//do the inverse
    }
    else //in dead zone don't move
    {
      analogWrite(leftWheelDirectionPin, 0);
      analogWrite(leftWheelSpeedPin, 0);
    }
    */

    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // DPAD
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmak of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

void dumpMouse(ControllerPtr ctl) {
    Serial.printf("idx=%d, buttons: 0x%04x, scrollWheel=0x%04x, delta X: %4d, delta Y: %4d\n",
                   ctl->index(),        // Controller Index
                   ctl->buttons(),      // bitmask of pressed buttons
                   ctl->scrollWheel(),  // Scroll Wheel
                   ctl->deltaX(),       // (-511 - 512) left X Axis
                   ctl->deltaY()        // (-511 - 512) left Y axis
    );
}

void dumpKeyboard(ControllerPtr ctl) {
    // TODO: Print pressed keys
    Serial.printf("idx=%d\n", ctl->index());
}

void dumpBalanceBoard(ControllerPtr ctl) {
    Serial.printf("idx=%d,  TL=%u, TR=%u, BL=%u, BR=%u, temperature=%d\n",
                   ctl->index(),        // Controller Index
                   ctl->topLeft(),      // top-left scale
                   ctl->topRight(),     // top-right scale
                   ctl->bottomLeft(),   // bottom-left scale
                   ctl->bottomRight(),  // bottom-right scale
                   ctl->temperature()   // temperature: used to adjust the scale value's precision
    );
}

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
        static int colorIdx = 0;
        // Some gamepads like DS4 and DualSense support changing the color LED.
        // It is possible to change it by calling:
        switch (colorIdx % 3) {
            case 0:
                // Red
                ctl->setColorLED(255, 0, 0);
                break;
            case 1:
                // Green
                ctl->setColorLED(0, 255, 0);
                break;
            case 2:
                // Blue
                ctl->setColorLED(0, 0, 255);
                break;
        }
        colorIdx++;
    }

    if (ctl->b()) {
        // Turn on the 4 LED. Each bit represents one LED.
        static int led = 0;
        led++;
        // Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
        // support changing the "Player LEDs": those 4 LEDs that usually indicate
        // the "gamepad seat".
        // It is possible to change them by calling:
        ctl->setPlayerLEDs(led & 0x0f);
    }

    if (ctl->x()) {
        // Duration: 255 is ~2 seconds
        // force: intensity
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
        // rumble.
        // It is possible to set it by calling:
        ctl->setRumble(0xc0 /* force */, 0xc0 /* duration */);
    }

    // Another way to query controller data is by getting the buttons() function.
    // See how the different "dump*" functions dump the Controller info.
    dumpGamepad(ctl);
}

void processMouse(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->scrollWheel() > 0) {
        // Do Something
    } else if (ctl->scrollWheel() < 0) {
        // Do something else
    }

    // See "dumpMouse" for possible things to query.
    dumpMouse(ctl);
}

void processKeyboard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->isKeyPressed(Keyboard_A)) {
        // Do Something
        Serial.println("Key 'A' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftShift)) {
        // Do something else
        Serial.println("Key 'LEFT SHIFT' pressed");
    }

    // Don't do "else" here.
    // Multiple keys can be pressed at the same time.
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        // Do something else
        Serial.println("Key 'Left Arrow' pressed");
    }

    // See "dumpKeyboard" for possible things to query.
    dumpKeyboard(ctl);
}

void processBalanceBoard(ControllerPtr ctl) {
    // This is just an example.
    if (ctl->topLeft() > 10000) {
        // Do Something
    }

    // See "dumpBalanceBoard" for possible things to query.
    dumpBalanceBoard(ctl);
}

