//TODO: 3. report encoder data back to the Serial2 port for odom messages


#include <FastLED.h>
#include <TimerOne.h>
#define rc_pin_left_right       3         //BLUE WIRE Horizontal right stick
#define rc_pin_up_down          4            //GREEN WIRE Vertical right stick
#define rc_pin_trainer_toggle   6     //ORANGE WIRE 
#define rc_pin_right_left       5         //YELLOW WIRE Horizontal left stick. Not used
#define rc_pin_throttle         2           //PURPLE WIRE Vertical left stick. Not used
#define rc_pin_power           13             //Plugs into RC pwr
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

const CRGB RED= CRGB (255, 0, 0);
const CRGB BLUE= CRGB ( 0, 0, 255);
const CRGB YELLOW= CRGB ( 255, 255, 0);
const CRGB ORANGE= CRGB (255, 80, 0);
const CRGB GREEN= CRGB ( 0, 255, 0);
const CRGB PURPLE= CRGB ( 255, 0, 255);
const CRGB TEAL= CRGB ( 0, 255, 255);
const CRGB BLACK= CRGB ( 0, 0, 0);

//Reset Function
void(*resetFunc)(void) = 0;

//____________________LEDS_________________________________

void randColor(){
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB(random(0, 255), random(0, 255), random(0, 255));
  }
}

void setColor(CRGB color){
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = color;
  }
  FastLED.show();
}

void toggleLED(){
  static bool LEDState = HIGH;
  if (shouldBlink){
    LEDState = !LEDState;
    if (LEDState){
      setColor(GREEN);
    }
    else{
      setColor(BLACK);
    }
  }
}

//__________________MATH FUNCTIONS__________________________

void find_turn_range(){
  if (turn_value < turn_min){
    turn_min = turn_value;}
  if (turn_value > turn_max){
    turn_max = turn_value;}
}

void find_drive_range(){
  if (drive_value < drive_min){
    drive_min = drive_value;}
  if (drive_value > drive_max){
    drive_max = drive_value;}
}

void find_speed_control_range(){
  if (speed_control_value < speed_control_min){
    speed_control_min = speed_control_value;}
  if (speed_control_value > speed_control_max){
    speed_control_max = speed_control_value;}
}


//________________SERIAL COMMUNICATION______________________

String write_read(String data, bool send_back = 0){
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
  if(send_back){
    Serial2.println(response);
  }
  
  return response;
}

//__________________DRIVE MODES_________________________________

void self_drive_control(){
  /*
   * This method is used to receive data over the serial port from the NUC
   * 
   * We determine what command the NUC is sending, and then we execute that
   * command then send the NUC back an appropriate response
   * 
   */
  String com_usb;

    setColor(GREEN);
    
  do{ com_usb = Serial2.readString();
      trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
      if (trainer_toggle < 1400 || trainer_toggle > 1600){
        break;
      }
  }while(com_usb == "");
  Serial2.println(com_usb);
  Serial.println(com_usb);
  
  //if (com_usb != ""){
    //Check to start or stop autonomous mode to 
    //listen to commands from serial port
    if(com_usb.charAt(0) == 'B'){
      self_drive_flag = true;
      shouldBlink = true;
      Serial2.println("Self-Drive Started");
    }
    else if(com_usb.charAt(0) == 'E'){
      self_drive_flag = false;
      shouldBlink = false;
      Serial1.println("d,s0");
      Serial2.println("d,s0");
      Serial1.println("t,s0");
      Serial.println("t,s0");
      Serial2.println("STOPPED");
      Serial2.println("Self-Drive Ended");
    }

    if(self_drive_flag){
      
      //read and execute command
      if(com_usb.charAt(0) == 'd'){
        //Drive Command
        Serial1.println(com_usb);
        Serial2.println(com_usb);
        //write_read("d,gets", true);
        //write_read("d,getp", true);
      }
      else if(com_usb.charAt(0) == 't' ){
        //Turn Command
        Serial1.println(com_usb);
        Serial2.println(com_usb);
        //send speed back back to NUC
        //write_read("t,gets", true);
        //write_read("t,getp", true);
      }
      else if(com_usb.charAt(0) == 'p'){
        write_read("d,getp", true);
        write_read("t,getp", true);
        write_read("d,gets", true);
        write_read("t,gets", true);
      }
      else if(com_usb.charAt(0) == 's'){
        Serial1.println("d,s0");
        Serial2.println("d,s0");
        Serial1.println("t,s0");
        Serial.println("t,s0");
        Serial2.println("STOPPED");
      }
      else{
        //Serial2.println("COMMAND NOT RECOGNIZED");
      }
    //}
    
    com_usb = "";//clear this variable
  }
}


void rc_control(){
  /*
   * This method updates the drive_calculated and turn_calculated
   * values for us to access and write to the motors
   * 
   */
  String turn_setup = "t,s";
  String drive_setup = "d,s";
  //int left_right_stick;
  //int up_down_stick;

  setColor(BLUE);
  turn_direct = pulseIn(rc_pin_left_right, HIGH);
  drive_direct = pulseIn(rc_pin_up_down, HIGH);
  speed_control_direct = pulseIn(rc_pin_throttle, HIGH);
  speed_control_value = (10000 / speed_control_range) * (speed_control_direct - speed_control_min) / 100;
  turn_calculated = (((10000 / turn_range) * (turn_direct - turn_min) * speed_control_value / 100) - (50 * speed_control_value)) / (-4);
  drive_calculated = (((10000 / drive_range) * (drive_direct - drive_min) * speed_control_value / 100) - (50 * speed_control_value)) / (-2); 
  
  //Serial2.println(left_right_stick);
  //Serial2.println(up_down_stick);

  //Turn Setup and Commands
  if (turn_calculated < 300 && turn_calculated > -300){
    turn_setup.concat(0);
    Serial1.println(turn_setup);
  }
  else if (turn_calculated > 3000 || turn_calculated < -3000){
  }
  else{
    turn_setup.concat(turn_calculated);
    Serial1.println(turn_setup);
  }

  //Drive Setup and Commands
  if ( drive_calculated < 300 && drive_calculated > -300){
    drive_setup.concat(0);
    Serial1.println(drive_setup);
  }
  else if (drive_calculated > 3000 || drive_calculated < -3000){
  }
  else{
  drive_setup.concat(drive_calculated);
  Serial1.println(drive_setup);
  }
  
  //Serial1.println(turn_setup);
  //Serial1.println(drive_setup);

  Serial2.print("Speed Control Direct: ");
  Serial2.println(speed_control_direct);
  Serial2.print("Speed Control Calculated: ");
  Serial2.println(speed_control_value);

  Serial2.print("Turn Direct: ");
  Serial2.println(turn_direct);
  Serial2.print("Turn Calculated: ");
  Serial2.println(turn_setup);

  Serial2.print("Drive Direct: ");
  Serial2.println(turn_direct);
  Serial2.print("Drive Calculated: ");
  Serial2.println(drive_setup);


}


//______________________SETUP HARDWARE CODE______________________
void wait_for_rc(){
  Serial1.println("t,s0");
  Serial1.println("d,s0");
    setColor(YELLOW);
    digitalWrite(rc_pin_power, LOW);
    delay(500);
    digitalWrite(rc_pin_power, HIGH);
    trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
    while (trainer_toggle < 100){
      trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
      Serial2.println(trainer_toggle);
    }
}




void initialize_kangaroo(){
  /*
   * This method is designed to establish a connection
   * to the kangaroo motor controller.
   * 
   * We keep sending start for drive and turn channels
   * until we get an appropriate response
   */
  
  setColor(ORANGE);
  //Setup Variables
  String d = "";
  String t = "";

  //Check Power To Kangaroo Motion Controller
  kangaroo_on = digitalRead(KANGAROO_POWER);
  while(kangaroo_on == false){
    kangaroo_on = digitalRead(KANGAROO_POWER); 
    Serial2.println(kangaroo_on); 
  }
  
  //Initialize Drive Channel
  while(d.equals("")){
    d = write_read("d,start\nd,getp");
    delay(1000);
    Serial2.println("Waiting for Turn");
  }
  Serial.println(d);
  Serial2.println("Drive Initialized");

  //Initialize Turn Channel
  while(t.equals("")){
    Serial2.println("Waiting for Drive");
    t = write_read("t,start\nt,getp");
    delay(1000);
  }
  Serial2.println(t);
  Serial2.println("Turn Initialized");

  //Send These values to "wake up" motors
  //NOTE: idk why we need this but it won't take drive and turn
  //      commands until we send these
  Serial1.println("t,s100");
  Serial1.println("d,s100");
  Serial1.println("t,s0");
  Serial1.println("d,s0");
  
}

void calibrate_controller(){
  setColor(PURPLE);
  do{
    trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);  
  }while(trainer_toggle < 1600);

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
  
  while(trainer_toggle > 1400){
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
  
  Serial2.print("Turn Range: ");
  Serial2.print(turn_min);
  Serial2.print(" - ");
  Serial2.println(turn_max);
  Serial2.print("Range = ");
  Serial2.println(turn_range);

  Serial2.print("Drive Range: ");
  Serial2.print(drive_min);
  Serial2.print(" - ");
  Serial2.println(drive_max);
  Serial2.print("Range = ");
  Serial2.println(drive_range);

  Serial2.print("Speed Control Range: ");
  Serial2.print(speed_control_min);
  Serial2.print(" - ");
  Serial2.println(speed_control_max);
  Serial2.print("Range = ");
  Serial2.println(speed_control_range);

  Serial.print("Turn Range: ");
  Serial.print(turn_min);
  Serial.print(" - ");
  Serial.println(turn_max);
  Serial.print("Range = ");
  Serial.println(turn_range);

  Serial.print("Drive Range: ");
  Serial.print(drive_min);
  Serial.print(" - ");
  Serial.println(drive_max);
  Serial.print("Range = ");
  Serial.println(drive_range);

  Serial.print("Speed Control Range: ");
  Serial.print(speed_control_min);
  Serial.print(" - ");
  Serial.println(speed_control_max);
  Serial.print("Range = ");
  Serial.println(speed_control_range);
}


//_____________________SETUP________________________________________
void setup() {
  
  //Reset Pin
  //pinMode(RESET, OUTPUT);
  //digitalWrite(RESET, HIGH);
  
  Timer1.initialize(500000);
  Timer1.attachInterrupt(toggleLED);
  
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

  //Set timeout - important because readString() will wait until 10ms of data is read in
  //default value is 1000ms (1 sec) and is really slow otherwise
  Serial2.setTimeout(10);
  Serial1.setTimeout(10);

  Serial2.println("Initializing Kangaroo");
  initialize_kangaroo();

  Serial2.println("Waiting For RC");
  wait_for_rc();

  Serial2.println("Begin Controller Calibration");
  calibrate_controller();
  
  //KEEP THIS LINE IN
  Serial2.println("Ready");
}


//________________________LOOP_____________________________________
void loop() {
  trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
  
  kangaroo_on = digitalRead(KANGAROO_POWER);
  if (kangaroo_on == false){
    resetFunc();
  }
  
  if(trainer_toggle - prev_trainer_toggle > 1600){
    self_drive_flag = false;
    shouldBlink = false;
    rc_control();
    //Serial2.println("RC MODE");
  }
  else if(trainer_toggle - prev_trainer_toggle > 1400){
    self_drive_control();
    //Serial2.println("SELF DRIVING MODE");
  }
  else if(trainer_toggle == 0){
    wait_for_rc();
  }
  else{
    self_drive_flag = false;
    shouldBlink = false;
    setColor(RED);
    Serial1.println("d,s0");
    Serial1.println("t,s0");
    //Serial2.println("STOPPED");
  }  
  //write_read("d,getp", true);
  //delay(20);
  //write_read("t,getp", true);
  //delay(20);
  
}
