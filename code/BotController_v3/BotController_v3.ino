#include <Bluepad32.h>

#define PCB_V2
#ifdef PCB_V2
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
#define L_DEADZONE 30
#define R_DEADZONE 30
#define TRIGGER_DEADZONE 10

#define MAX_SPEED_SLOW 140
#define MAX_SPEED_BOOST 255

ControllerPtr myController;
int last_cmd[3] = {0, 0, 0};

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

  // LEFT_POWER {-100-100}, RIGHT_POWER {-100-100}, BOOST {0, 1}
  int cmd[3] = {0, 0, 0};

  processControllerInput(myController, cmd);
  processCmd(cmd);

  vTaskDelay(10);
}

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time. // ONLY CYCLOTRON_MAX_CONTROLLERS will be available for use
void onConnectedController(ControllerPtr ctl) {
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
    myController = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
    myController = nullptr;
}

void processControllerInput(ControllerPtr ctl, int cmd[3]) {

  // No controller detected
  if (!ctl || !ctl->isConnected()){
    digitalWrite(LED_BT_CONNECTION, LOW);
    return;
  }

  // Gamepad is valid and connected
  digitalWrite(LED_BT_CONNECTION, HIGH);

  // Read from controller
  int l_axisY = ctl->axisY();
  int r_axisY = ctl->axisRY();
  int r_trigger = ctl->throttle();

  // Apply a deadzone to the axes
  l_axisY = abs(l_axisY) < L_DEADZONE ? 0 : l_axisY;
  r_axisY = abs(r_axisY) < R_DEADZONE ? 0 : r_axisY;

  // Determine the COMMAND from controller
  cmd[0] = map(l_axisY, -512, 512, 100, -100);
  cmd[1] = map(r_axisY, -512, 512, 100, -100);
  cmd[2] = r_trigger > 100 ? 1 : 0;
}

void processCmd(int cmd[3]){
  
  // No change since last time
  if (last_cmd[0] == cmd[0] && last_cmd[1] == cmd[1] && last_cmd[2] == cmd[2]) 
    return;

  // Store latest
  last_cmd[0] = cmd[0];
  last_cmd[1] = cmd[1];
  last_cmd[2] = cmd[2];

  Serial.printf("CMD: %d %d %d\n", cmd[0], cmd[1], cmd[2]);
  int l_percent = cmd[0];
  int r_percent = cmd[1];
  int boost = cmd[2];

  // Motor speeds (PWM) are 0-255
  // Forward/reverse depends on which pin is 0 vs PWM'd
  int l_pwm = 0;
  int r_pwm = 0;
  if (boost) {
    l_pwm = map(l_percent, -100, 100, -1*MAX_SPEED_BOOST, MAX_SPEED_BOOST);
    r_pwm = map(r_percent, -100, 100, -1*MAX_SPEED_BOOST, MAX_SPEED_BOOST);
  } else {
    l_pwm = map(l_percent, -100, 100, -1*MAX_SPEED_SLOW, MAX_SPEED_SLOW);
    r_pwm = map(r_percent, -100, 100, -1*MAX_SPEED_SLOW, MAX_SPEED_SLOW);
  }

  Serial.printf("PWM: %d %d\n", l_pwm, r_pwm);
  analogWrite(L_PWM_PIN_A, l_pwm < 0 ? abs(l_pwm) : 0);
  analogWrite(L_PWM_PIN_B, l_pwm >= 0 ? abs(l_pwm) : 0);

  analogWrite(R_PWM_PIN_A, r_pwm < 0 ? abs(r_pwm) : 0);
  analogWrite(R_PWM_PIN_B, r_pwm >= 0 ? abs(r_pwm) : 0);

}
