#include <FastLED.h>
#include <SoftwareSerial.h>
#define rc_pin_left_right 4
#define rc_pin_up_down 5
#define rc_pin_trainer_toggle 7
#define rc_pin_power 13
#define LED_PIN     12
#define NUM_LEDS    16
#define RX_PIN 2
#define TX_PIN 3

SoftwareSerial softSerial(RX_PIN, TX_PIN);

CRGB leds[NUM_LEDS];

int turn_calculated;
int drive_calculated;
int trainer_toggle;
String driving_mode;
int controller_tolerance = 25;

String write_read(String data, bool send_back = 0){
  /*
   * This method is designed to send data to the kangaroo 
   * and return the appropriate response to the sender
   */
  softSerial.println(data);
  
  //UNCOMMENT LINE BELOW TO SEE THE SENT MESSAGE
  //Serial.println(s);

  //get response
  String response = softSerial.readString();
  
  //Send response back to sender
  if(send_back){
    Serial.println(response);
  }
  
  return response;
}



void self_drive_control(){
  /*
   * This method is used to receive data over the serial port from the NUC
   * 
   * We determine what command the NUC is sending, and then we execute that
   * command then send the NUC back an appropriate response
   * 
   */
  String com_usb = Serial.readString();
  
  if (com_usb != ""){
    if(com_usb.charAt(0) == 'd'){
      //Drive Command
      softSerial.println(com_usb);
      write_read("d,gets", true);
    }
    else if(com_usb.charAt(0) == 't' ){
      //Turn Command
      softSerial.println(com_usb);
      //send speed back back to NUC
      write_read("t,gets", true);
    }
    else if(com_usb.charAt(0) == 's'){
      softSerial.println("d,s0");
      softSerial.println("t,s0");
      Serial.println("STOPPED");
    }
    else{
      //Serial.println("COMMAND NOT RECOGNIZED");
    }
    
    com_usb = "";
  
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
  if(trainer_toggle > 1600){
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
      drive_calculated = up_down_stick * 2;
    }
  }

  turn_setup.concat(turn_calculated);
  drive_setup.concat(drive_calculated);
  softSerial.println(turn_setup);
  softSerial.println(drive_setup);

}


// ____________________LED CODE__________________________
void setred () {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ( 255, 0, 0);
  }
  FastLED.show();
}
void setblue () {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ( 0, 0, 255);
  }
  FastLED.show();
}
void setyellow (){
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ( 255, 255, 0);
}
FastLED.show();
}
void setorange () {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB (255, 80, 0);
  
  }
  FastLED.show();
}
void setgreen (){
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ( 0, 255, 0);
}
FastLED.show();
}

void setblack () {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ( 0, 0, 0);
}
FastLED.show();
}


//______________________SETUP HARDWARE CODE______________________
void wait_for_rc(){
  softSerial.println("t,s0");
  softSerial.println("d,s0");
    setyellow();
    digitalWrite(rc_pin_power, LOW);
    delay(2000);
    digitalWrite(rc_pin_power, HIGH);
    trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
    while (trainer_toggle < 100){
      trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
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
  setorange();
  //Setup Variables
  String d = "";
  String t = "";

  //Initialize Drive Channel
  while(d.equals("")){
    d = write_read("d,start\nd,getp");
    delay(1000);
    Serial.println("Waiting for Turn");
  }
  Serial.println("Drive Initialized");

  //Initialize Turn Channel
  while(t.equals("")){
    Serial.println("Waiting for Drive");
    t = write_read("t,start\nt,getp");
    delay(1000);
  }
  Serial.println("Turn Initialized");

  //Send These values to "wake up" motors
  //NOTE: idk why we need this but it won't take drive and turn
  //      commands until we send these
  softSerial.println("t,s100");
  softSerial.println("d,s100");
  softSerial.println("t,s0");
  softSerial.println("d,s0");
  
}


//_____________________SETUP________________________________________
void setup() {
  Serial.begin(9600);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  
  //Setup controller inputs
  pinMode(rc_pin_left_right, INPUT);
  pinMode(rc_pin_up_down, INPUT);
  pinMode(rc_pin_trainer_toggle, INPUT);
  pinMode(rc_pin_power, OUTPUT);
  digitalWrite(rc_pin_power, HIGH);
  trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
  
  //Begin Serial ports
  Serial.begin(9600);
  softSerial.begin(9600);

  //Set timeout - important because readString() will wait until 10ms of data is read in
  //default value is 1000ms (1 sec) and is really slow otherwise
  Serial.setTimeout(10);
  softSerial.setTimeout(10);

  initialize_kangaroo();

  wait_for_rc();
 

  
  //KEEP THIS LINE IN
  Serial.println("Ready");
}


//________________________LOOP_____________________________________
void loop() {
  trainer_toggle = pulseIn(rc_pin_trainer_toggle, HIGH);
  
  if(trainer_toggle > 1600){
    setblue();
    rc_control();
    driving_mode = "RC";
    //setgreen();
    Serial.println("RC MODE");
  }
  else if(trainer_toggle > 1400){
    setgreen();
    if(driving_mode == "RC"){
      softSerial.println("d,s0");
      softSerial.println("t,s0");
    }
    self_drive_control();
    driving_mode = "SD";
    //setblue();
    Serial.println("SELF DRIVING MODE");
  }
  else if(trainer_toggle == 0){
    softSerial.println("d,s0");
    softSerial.println("t,s0");
    Serial.println("NO CONTROLLER DETECTED");
    wait_for_rc();
  }
  else{
    setred();
    softSerial.println("d,s0");
    softSerial.println("t,s0");
    driving_mode = "ST";
    Serial.println("STOPPED");
  }  
  
}
