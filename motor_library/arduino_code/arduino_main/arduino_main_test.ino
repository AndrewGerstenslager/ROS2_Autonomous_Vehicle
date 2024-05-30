//TODO: 1. Find the RC receiver pins that give us the left-right & up-down for the left stick
// Rename variables to be more clear on the stick being used now that there are two
//TODO 2: add a calibration mode, where the controller is used to set max and min values for the controller

//TODO: 3. make the left stick modify the speed by its up-down value. bottom 
//TODO: 4. report encoder data back to the Serial2 port for odom messages


#include <FastLED.h>
#define rc_pin_left_right 3         //BLUE WIRE Horizontal right stick
#define rc_pin_up_down 4            //GREEN WIRE Vertical right stick
#define rc_pin_trainer_toggle 6     //ORANGE WIRE 
#define rc_pin_right_left 5         //YELLOW WIRE Horizontal left stick. Not used
#define rc_pin_throttle 2           //PURPLE WIRE Vertical left stick. Not used
#define rc_pin_power 13             //Plugs into RC pwr
#define LED_PIN     12
#define NUM_LEDS   16

CRGB leds[NUM_LEDS];

int turn_calculated;
int drive_calculated;
int trainer_toggle;
int prev_trainer_toggle;
int controller_tolerance = 25;
int led_counter = 0;
bool led_on = true;
bool self_drive_flag = false;


const CRGB RED= CRGB (255, 0, 0);
const CRGB BLUE= CRGB ( 0, 0, 255);
const CRGB YELLOW= CRGB ( 255, 255, 0);
const CRGB ORANGE= CRGB (255, 80, 0);
const CRGB GREEN= CRGB ( 0, 255, 0);
const CRGB BLACK= CRGB ( 0, 0, 0);

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
  String com_usb = Serial2.readString();
  if(self_drive_flag){//if in autonomous mode
    //led control code for flashing
    led_counter++;
    if(led_counter == 30){
      if(led_on){
        led_on = false;
      }
      else{
        led_on = true;
      }
      led_counter = 0;
    }
    if(led_on){
      setColor(RED);
    }
    else{
      setColor(BLACK);
    }
  }
  else{
    setColor(GREEN);
  }
  
  if (com_usb != ""){
    //Check to start or stop autonomous mode to 
    //listen to commands from serial port
    if(com_usb == "START"){
      self_drive_flag = true;
    }
    else if(com_usb == "STOP"){
      self_drive_flag = false;
    }

    if(self_drive_flag){
      
      //read and execute command
      if(com_usb.charAt(0) == 'd'){
        //Drive Command
        Serial1.println(com_usb);
        write_read("d,gets", true);
      }
      else if(com_usb.charAt(0) == 't' ){
        //Turn Command
        Serial1.println(com_usb);
        //send speed back back to NUC
        write_read("t,gets", true);
      }
      else if(com_usb.charAt(0) == 's'){
        Serial1.println("d,s0");
        Serial1.println("t,s0");
        Serial2.println("STOPPED");
      }
      else{
        //Serial2.println("COMMAND NOT RECOGNIZED");
      }
    }
    
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
  int left_right_stick;
  int up_down_stick;
   
  trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);

  //If toggle switch is on setting "0"
  //or the lowest setting
 // if(trainer_toggle > 1600){
    left_right_stick = 1487 - pulseIn(rc_pin_left_right, HIGH);
    //filter out inputs that are close to 0
    if (left_right_stick > (-1 * controller_tolerance) && 
                            left_right_stick < controller_tolerance){
      turn_calculated = 0;
    }
    else{
      turn_calculated = left_right_stick * 2;
    }
  
    //filter out inputs that are close to 0
    up_down_stick = 1574 - pulseIn(rc_pin_up_down, HIGH);
    if (up_down_stick > (-1 * controller_tolerance) && 
                            up_down_stick < controller_tolerance){
      drive_calculated = 0;
    }
    else{
      drive_calculated = up_down_stick * 5;
    }
  Serial2.println(left_right_stick);
  Serial2.println(up_down_stick);
  turn_setup.concat(turn_calculated);
  drive_setup.concat(drive_calculated);
  Serial1.println(turn_setup);
  Serial1.println(drive_setup);

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

  //Initialize Drive Channel
  while(d.equals("")){
    d = write_read("d,start\nd,getp");
    delay(1000);
    Serial2.println("Waiting for Turn");
  }
  Serial2.println("Drive Initialized");

  //Initialize Turn Channel
  while(t.equals("")){
    Serial2.println("Waiting for Drive");
    t = write_read("t,start\nt,getp");
    delay(1000);
  }
  Serial2.println("Turn Initialized");

  //Send These values to "wake up" motors
  //NOTE: idk why we need this but it won't take drive and turn
  //      commands until we send these
  Serial1.println("t,s100");
  Serial1.println("d,s100");
  Serial1.println("t,s0");
  Serial1.println("d,s0");
  
}


//_____________________SETUP________________________________________
void setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  
  //Setup controller inputs
  pinMode(rc_pin_left_right, INPUT);
  pinMode(rc_pin_up_down, INPUT);
  pinMode(rc_pin_trainer_toggle, INPUT);
  pinMode(rc_pin_power, OUTPUT);
  digitalWrite(rc_pin_power, HIGH);
  trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
  
  //Begin Serial ports
  Serial2.begin(9600);
  Serial1.begin(9600);

  //Set timeout - important because readString() will wait until 10ms of data is read in
  //default value is 1000ms (1 sec) and is really slow otherwise
  Serial2.setTimeout(10);
  Serial1.setTimeout(10);

  initialize_kangaroo();

  wait_for_rc();
 

  
  //KEEP THIS LINE IN
  Serial2.println("Ready");
}


//________________________LOOP_____________________________________
void loop() {
  trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
  
  
  if(trainer_toggle - psrev_trainer_toggle > 1600){
    setColor(BLUE);
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
    setColor(RED);
    Serial1.println("d,s0");
    Serial1.println("t,s0");
    //Serial2.println("STOPPED");
  }  
  
  
}