#include <Bluepad32.h>

#ifndef BOTCONTROL_V2
#define BOTCONTROL_V1
#define L_PWM_PIN_A 5
#define L_PWM_PIN_B 6
#define R_PWM_PIN_A 7
#define R_PWM_PIN_B 8
#else
#define L_PWM_PIN_A 10
#define L_PWM_PIN_B 11
#define R_PWM_PIN_A 12
#define R_PWM_PIN_B 13
#endif

#define LED_BT_CONNECTION 4
#define CYCLOTRON_MAX_CONTROLLERS 1

ControllerPtr myControllers[CYCLOTRON_MAX_CONTROLLERS];

// Arduino setup function. Runs in CPU 1
void setup() 
{
    // Setup PWM pins for motor control
    pinMode(L_PWM_PIN_A, OUTPUT);
    pinMode(L_PWM_PIN_B, OUTPUT);
    pinMode(R_PWM_PIN_A, OUTPUT);
    pinMode(R_PWM_PIN_B, OUTPUT);

    analogWrite(L_PWM_PIN_A, 255);
    analogWrite(L_PWM_PIN_B, 255);
    analogWrite(R_PWM_PIN_A, 255);
    analogWrite(R_PWM_PIN_B, 255);

    // Setup LED pins
    pinMode(LED_BT_CONNECTION, OUTPUT);
    digitalWrite(LED_BT_CONNECTION, LOW);

    // Setup SERIAL comms
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
    for (int i = 0; i < CYCLOTRON_MAX_CONTROLLERS; i++) {
        ControllerPtr myController = myControllers[i];

        if (myController && myController->isConnected()) 
        {
            digitalWrite(LED_BT_CONNECTION, HIGH);
            if (myController->isGamepad()) {
                // continue;
                // processGamepad(myController);
                runRobot(myController);
            } 
            else 
            {
                // Serial.printf("Data not available yet\n");
                // digitalWrite(LED_BT_CONNECTION, LOW);
                continue;
            }
            // See ArduinoController.h for all the available functions.
        }
        else if (myController)
        {
          digitalWrite(LED_BT_CONNECTION, LOW);
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
// Up to 4 gamepads can be connected at the same time. // ONLY CYCLOTRON_MAX_CONTROLLERS will be available for use
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

void runRobot(ControllerPtr ctl) 
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
      analogWrite(L_PWM_PIN_A, 0);
      analogWrite(L_PWM_PIN_B, 255);
      analogWrite(R_PWM_PIN_A, 0);
      analogWrite(R_PWM_PIN_B, 255);
    }
    else if(brake > 20)//if brake go full speed backwards
    {
      analogWrite(L_PWM_PIN_A, 255);
      analogWrite(L_PWM_PIN_B, 0);
      analogWrite(R_PWM_PIN_A, 255);
      analogWrite(R_PWM_PIN_B, 0);
    }
    else// otherwise steer normally
    {
        // Set motor directions and speeds
      if (leftSpeed >= 0) 
      {
        analogWrite(L_PWM_PIN_A, abs(leftSpeed));
        analogWrite(L_PWM_PIN_B, 0);
      } 
      else 
      {
        analogWrite(L_PWM_PIN_A, 0);
        analogWrite(L_PWM_PIN_B, abs(leftSpeed));
      }

      if (rightSpeed <= 0) 
      {
        analogWrite(R_PWM_PIN_A, 0);
        analogWrite(R_PWM_PIN_B, abs(rightSpeed));
      } 
      else 
      {
        analogWrite(R_PWM_PIN_A, abs(rightSpeed));
        analogWrite(R_PWM_PIN_B, 0);
      }

    }
}