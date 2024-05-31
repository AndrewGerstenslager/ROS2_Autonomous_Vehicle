#include <FastLED.h>
#include <TimerOne.h>

#define rc_pin_left_right       3         //BLUE WIRE Horizontal right stick
#define rc_pin_up_down          4         //GREEN WIRE Vertical right stick
#define rc_pin_trainer_toggle   6         //ORANGE WIRE 
#define rc_pin_right_left       5         //YELLOW WIRE Horizontal left stick. Not used
#define rc_pin_throttle         2         //PURPLE WIRE Vertical left stick. Not used
#define rc_pin_power           13         //Plugs into RC pwr
#define LED_PIN                12
#define NUM_LEDS               16
#define KANGAROO_POWER         11
#define RESET                  10

CRGB leds[NUM_LEDS];

long int turn_calculated;
long int drive_calculated;
int trainer_toggle;
int prev_trainer_toggle;
int controller_tolerance = 25;
int led_counter = 0;

int turn_value;
int turn_min;
int turn_max;
int turn_range;
int turn_direct;
int turn_deadzone = 300;

int drive_value;
int drive_min;
int drive_max;
int drive_range;
int drive_direct;
int drive_deadzone = 300;

long int speed_control_value;
int speed_control_direct;
int speed_control_min;
int speed_control_max;
int speed_control_range;

volatile bool shouldBlink = false;
bool self_drive_flag = false;
bool calibrated = false;
bool kangaroo_on = true;

const CRGB RED = CRGB(255, 0, 0);
const CRGB BLUE = CRGB(0, 0, 255);
const CRGB YELLOW = CRGB(255, 255, 0);
const CRGB ORANGE = CRGB(255, 80, 0);
const CRGB GREEN = CRGB(0, 255, 0);
const CRGB PURPLE = CRGB(255, 0, 255);
const CRGB TEAL = CRGB(0, 255, 255);
const CRGB BLACK = CRGB(0, 0, 0);

// Reset Function
void (*resetFunc)(void) = 0;

String currentStatus = "STATUS: SETUP";
String previousStatus = "";

//____________________LEDS_________________________________

void randColor() {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB(random(0, 255), random(0, 255), random(0, 255));
  }
}

void setColor(CRGB color) {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

//__________________INTERRUPT METHODS______________________

void handleInterrupt() {
  static unsigned long lastStatusTime = 0;
  static unsigned long lastLedToggleTime = 0;
  unsigned long currentTime = millis();

  // Send status every 1000ms
  if (currentTime - lastStatusTime >= 1000) {
    Serial2.println(currentStatus);
    lastStatusTime = currentTime;
  }

  // Toggle LED every 500ms
  if (shouldBlink && currentTime - lastLedToggleTime >= 500) {
    static bool LEDState = HIGH;
    LEDState = !LEDState;
    if (LEDState) {
      setColor(GREEN);
    } else {
      setColor(BLACK);
    }
    lastLedToggleTime = currentTime;
  }
}

//__________________MATH FUNCTIONS__________________________

void find_turn_range() {
  if (turn_value < turn_min) {
    turn_min = turn_value;
  }
  if (turn_value > turn_max) {
    turn_max = turn_value;
  }
}

void find_drive_range() {
  if (drive_value < drive_min) {
    drive_min = drive_value;
  }
  if (drive_value > drive_max) {
    drive_max = drive_value;
  }
}

void find_speed_control_range() {
  if (speed_control_value < speed_control_min) {
    speed_control_min = speed_control_value;
  }
  if (speed_control_value > speed_control_max) {
    speed_control_max = speed_control_value;
  }
}

//________________SERIAL COMMUNICATION______________________

String write_read(String data, bool send_back = 0) {
  /*
   * This method is designed to send data to the kangaroo 
   * and return the appropriate response to the sender
   */
  Serial1.println(data);

  //UNCOMMENT LINE BELOW TO SEE THE SENT MESSAGE
  //Serial2.println(s);

  //get response
  String response = Serial1.readString();

  //Send response back to sender
  if (send_back) {
    Serial2.println(response);
  }

  return response;
}

//__________________DRIVE MODES_________________________________

void self_drive_control() {
  /*
   * This method is used to receive data over the serial port from the NUC
   * 
   * We determine what command the NUC is sending, and then we execute that
   * command then send the NUC back an appropriate response
   * 
   */
  String com_usb;

  setColor(GREEN);

  do {
    com_usb = Serial2.readString();
    trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
    if (trainer_toggle < 1400 || trainer_toggle > 1600) {
      break;
    }
  } while (com_usb == "");

  // Check to start or stop autonomous mode to listen to commands from serial port

  // b is begin self drive enable
  if (com_usb.charAt(0) == 'b') {
    self_drive_flag = true;
    shouldBlink = true;
    Serial2.println("DEBUG: BEGIN SELF DRIVE ENABLE");
  }

  // e is end self drive enable
  else if (com_usb.charAt(0) == 'e') {
    self_drive_flag = false;
    shouldBlink = false;
    Serial1.println("d,s0");
    Serial1.println("t,s0");
    Serial2.println("DEBUG: END SELF DRIVE ENABLE");
  }

  // Only execute commands if self drive is started
  else if (self_drive_flag) {
    // Read and execute drive command
    if (com_usb.charAt(0) == 'd') {
      Serial1.println(com_usb);
      Serial2.println("DEBUG: MOTOR COMMAND SENT WAS " + com_usb);
    }

    // Read and execute turn command
    else if (com_usb.charAt(0) == 't') {
      Serial1.println(com_usb);
      Serial2.println("DEBUG: MOTOR COMMAND SENT WAS " + com_usb);
    }

    // Get position
    else if (com_usb.charAt(0) == 'p') {
      String response_d_getp = write_read("d,getp", false);
      String response_t_getp = write_read("t,getp", false);
      String response_d_gets = write_read("d,gets", false);
      String response_t_gets = write_read("t,gets", false);

      // Trim responses
      response_d_getp.trim();
      response_t_getp.trim();
      response_d_gets.trim();
      response_t_gets.trim();

      // Combine the responses into a single semicolon-separated string
      String combined_response = response_d_getp + ";" + response_t_getp + ";" + response_d_gets + ";" + response_t_gets;

      // Send the combined response back
      Serial2.println(combined_response);
    }

    // Stop command without ending self drive enable
    else if (com_usb.charAt(0) == 's') {
      Serial1.println("d,s0");
      Serial1.println("t,s0");
      Serial2.println("DEBUG: STOPPED");
    }

    // Unrecognized command
    else {
      if (com_usb.length() > 0) {
        Serial2.println("DEBUG: COMMAND NOT RECOGNIZED: " + com_usb);
      }
    }
  }

  com_usb = ""; // Clear this variable
}

void rc_control() {
  /*
   * This method updates the drive_calculated and turn_calculated
   * values for us to access and write to the motors
   * 
   */
  String turn_setup = "t,s";
  String drive_setup = "d,s";

  setColor(BLUE);
  turn_direct = pulseIn(rc_pin_left_right, HIGH);
  drive_direct = pulseIn(rc_pin_up_down, HIGH);
  speed_control_direct = pulseIn(rc_pin_throttle, HIGH);

  // Calculate the speed control value as a percentage (0 to 100)
  speed_control_value = map(speed_control_direct, speed_control_min, speed_control_max, 0, 100);

  // Calculate the turn and drive values based on the throttle percentage and desired ranges
  turn_calculated = map(turn_direct, turn_min, turn_max, 50, -50) * speed_control_value / 100;
  drive_calculated = map(drive_direct, drive_min, drive_max, 150, -150) * speed_control_value / 100;

  // Format and send turn command
  if (abs(turn_calculated) >= 3000) {
    // Ignore values out of range
  } else {
    turn_setup.concat(turn_calculated);
    Serial1.println(turn_setup);
  }

  // Format and send drive command
  if (abs(drive_calculated) >= 3000) {
    // Ignore values out of range
  } else {
    drive_setup.concat(drive_calculated);
    Serial1.println(drive_setup);
  }
}

//______________________SETUP HARDWARE CODE______________________
void wait_for_rc() {
  //Stop motors just in case
  Serial1.println("t,s0");
  Serial1.println("d,s0");

  setColor(YELLOW);
  //Turn on rc receiver
  digitalWrite(rc_pin_power, LOW);
  delay(500);
  digitalWrite(rc_pin_power, HIGH);
  trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
  //wait for rc receiver to bind to the controller
  while (trainer_toggle == 0) {
    trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);

    //additional check if kangaroo turns off in this loop
    kangaroo_on = digitalRead(KANGAROO_POWER);
    if (kangaroo_on == false) {
      setup();
    }
  }
}

void initialize_kangaroo() {
  /*
   * This method is designed to establish a connection
   * to the kangaroo motor controller.
   * 
   * We keep sending start for drive and turn channels
   * until we get an appropriate response
   */

  setColor(ORANGE);
  shouldBlink = false;
  //Setup Variables
  String d = "";
  String t = "";

  //Check Power To Kangaroo Motion Controller
  Serial2.println("DEBUG: WAITING FOR KANGAROO TO POWER ON");
  kangaroo_on = digitalRead(KANGAROO_POWER);
  while (kangaroo_on == false) {
    kangaroo_on = digitalRead(KANGAROO_POWER);
  }

  //Initialize Drive Channel
  Serial2.println("DEBUG: WAITING FOR SABERTOOTH DRIVE COMMAND RESPONSE");
  while (d.equals("")) {
    d = write_read("d,start\nd,getp");
    delay(100);
  }
  Serial.println("DEBUG: RESPONSE FROM KANGAROO WAS " + d);
  Serial2.println("DEBUG: DRIVE INITIALIZED");

  //Initialize Turn Channel
  Serial2.println("DEBUG: WAITING FOR SABERTOOTH TURN COMMAND RESPONSE");
  while (t.equals("")) {
    t = write_read("t,start\nt,getp");
    delay(100);
  }
  Serial.println("DEBUG: RESPONSE FROM KANGAROO WAS " + t);
  Serial2.println("DEBUG: TURN INITIALIZED");

  //Set units to 100cm to 2048 lines
  //Wheel diameter is 100cm (1m) and encoder has a resolution of 2048/rev
  Serial2.println("d,units 100cm = 2048 lines");
  Serial2.println("d,units 100cm = 2048 lines");

}

void calibrate_controller() {
  setColor(PURPLE);
  do {
    trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
  } while (trainer_toggle < 1600);

  setColor(TEAL);
  turn_value = pulseIn(rc_pin_left_right, HIGH);
  drive_value = pulseIn(rc_pin_up_down, HIGH);
  speed_control_value = pulseIn(rc_pin_throttle, HIGH);

  turn_min = turn_value;
  turn_max = turn_value;
  drive_min = drive_value;
  drive_max = drive_value;
  speed_control_min = speed_control_value;
  speed_control_max = speed_control_value;

  while (trainer_toggle > 1400) {
    turn_value = pulseIn(rc_pin_left_right, HIGH);
    drive_value = pulseIn(rc_pin_up_down, HIGH);
    speed_control_value = pulseIn(rc_pin_throttle, HIGH);
    find_turn_range();
    find_drive_range();
    find_speed_control_range();
    trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
  }
  turn_range = turn_max - turn_min;
  drive_range = drive_max - drive_min;
  speed_control_range = speed_control_max - speed_control_min;

  Serial2.println("DEBUG: Turn Range: " + String(turn_min) + " - " + String(turn_max));
  Serial2.println("DEBUG: Range = " + String(turn_range));
  Serial2.println("DEBUG: Drive Range: " + String(drive_min) + " - " + String(drive_max));
  Serial2.println("DEBUG: Range = " + String(drive_range));
  Serial2.println("DEBUG: Speed Control Range: " + String(speed_control_min) + " - " + String(speed_control_max));
  Serial2.println("DEBUG: Range = " + String(speed_control_range));
}

//_____________________SETUP________________________________________

void setup() {
  currentStatus = "STATUS: SETUP";
  previousStatus = "";

  Timer1.initialize(1000000); // 1 second
  Timer1.attachInterrupt(handleInterrupt);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  //Setup controller inputs
  pinMode(rc_pin_left_right, INPUT);
  pinMode(rc_pin_up_down, INPUT);
  pinMode(rc_pin_trainer_toggle, INPUT);
  pinMode(rc_pin_power, OUTPUT);
  pinMode(KANGAROO_POWER, INPUT);
  digitalWrite(rc_pin_power, HIGH);
  trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);

  //Begin Serial ports
  Serial2.begin(9600);
  Serial1.begin(9600);
  Serial.begin(9600);

  //Stop motors just in case
  Serial1.println("t,s0");
  Serial1.println("d,s0");

  //Set timeout - important because readString() will wait until 10ms of data is read in
  //default value is 1000ms (1 sec) and is really slow otherwise
  Serial2.setTimeout(10);
  Serial1.setTimeout(10);

  Serial2.println("DEBUG: INITIALIZE KANGAROO");
  initialize_kangaroo();

  Serial2.println("DEBUG: INITIALIZE RC RECEIVER AND CONTROLLER");
  wait_for_rc();

  Serial2.println("DEBUG: CONTROLLER CALIBRATION STARTED");
  calibrate_controller();

  Serial2.println("DEBUG: READY");
}

//________________________LOOP_____________________________________
void loop() {
  trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);

  kangaroo_on = digitalRead(KANGAROO_POWER);
  if (kangaroo_on == false) {
    resetFunc();
  }

  if (trainer_toggle - prev_trainer_toggle > 1600) {
    self_drive_flag = false;
    shouldBlink = false;
    if (currentStatus != "STATUS: RC") {
      currentStatus = "STATUS: RC";
      Serial2.println(currentStatus);
    }
    rc_control();
  } 
  else if (trainer_toggle - prev_trainer_toggle > 1400) {
    if (currentStatus != "STATUS: SELF DRIVE") {
      currentStatus = "STATUS: SELF DRIVE";
      Serial2.println(currentStatus);
    }
    self_drive_control();
  } 
  else if (trainer_toggle == 0) {
    setup();
  } 
  else {
    self_drive_flag = false;
    shouldBlink = false;
    if (currentStatus != "STATUS: STOPPED") {
      currentStatus = "STATUS: STOPPED";
      Serial2.println(currentStatus);
    }
    setColor(RED);
    Serial1.println("d,s0");
    Serial1.println("t,s0");
  }
}
