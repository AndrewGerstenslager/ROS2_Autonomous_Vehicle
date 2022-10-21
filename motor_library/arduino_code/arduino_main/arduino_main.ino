#define rc_pin_left_right 4
#define rc_pin_up_down 5
#define rc_pin_trainer_toggle 7

int turn_calculated;
int drive_calcualted;
int left_right_stick;
int up_down_stick;
int trainer_toggle;
int controller_tolerance = 20;
String temp = "";

void calculate_speeds(){
  /*
   * This method updates the drive_calculated and turn_calculated
   * values for us to access and write to the motors
   * 
   */
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
      drive_calcualted = 0;
    }
    else{
      drive_calcualted = up_down_stick * 2;
    }
  }
  else{
    turn_calculated = 0;
    drive_calcualted = 0;
  }
}

String write_read(String data, bool send_back = 0){
  /*
   * This method is designed to send data to the kangaroo 
   * and return the appropriate response to the sender
   */
  Serial1.println(data);
  
  //UNCOMMENT LINE BELOW TO SEE THE SENT MESSAGE
  //Serial.println(s);

  //get response
  String response = Serial1.readString();
  
  //Send response back to sender
  if(send_back){
    Serial.println(response);
  }
  
  
  return response;
}

void initialize_kangaroo(){
  /*
   * This method is designed to establish a connection
   * to the kangaroo motor controller.
   * 
   * We keep sending start for drive and turn channels
   * until we get an appropriate response
   */
  //Setup Variables
  String d = "";
  String t = "";

  //Initialize Drive Channel
  while(d.equals("")){
    d = write_read("d,start\nd,getp");
    delay(1000);
  }
  Serial.println("Drive Initialized");

  //Initialize Turn Channel
  while(t.equals("")){
    t = write_read("t,start\nt,getp");
    delay(1000);
  }
  Serial.println("Turn Initialized");

  //Send These values to "wake up" motors
  //NOTE: idk why we need this but it won't take drive and turn
  //      commands until we send these
  Serial1.println("t,s100");
  Serial1.println("d,s100");
  Serial1.println("t,s0");
  Serial1.println("d,s0");
  
}

void setup() {
  //Setup controller inputs
  pinMode(rc_pin_left_right, INPUT);
  pinMode(rc_pin_up_down, INPUT);
  pinMode(rc_pin_trainer_toggle, INPUT);
  
  //Begin Serial ports
  Serial.begin(9600);
  Serial1.begin(9600);

  //Set timeout - important because readString() will wait until 10ms of data is read in
  //default value is 1000ms (1 sec) and is really slow otherwise
  Serial.setTimeout(10);
  Serial1.setTimeout(10);

  initialize_kangaroo();
  
  
  Serial.println("Ready");
  
  //Serial1.print("d,s1");
  //Serial1.print("t,s1");
  //Serial1.print("d,s0");
  //Serial1.print("t,s0");

  
}

void loop() {
  //wait if nothing is sent
  //while (!Serial.available());
  
  //String received_data = Serial.readString();
  //write_read(received_data, 1);

  temp = Serial.readString();

  if (temp != ""){
    Serial.println(temp);
    temp = "";
  }

  
  //calculate_speeds();
  
  //Serial1.println("t,s" + String(turn_calculated));
  //Serial1.println("d,s" + String(drive_calcualted));
}
