#define rc_pin_left_right 3         //BLUE WIRE Horizontal right stick
#define rc_pin_up_down 4            //GREEN WIRE Vertical right stick
#define rc_pin_trainer_toggle 6     //ORANGE WIRE 
#define rc_pin_right_left 5         //YELLOW WIRE Horizontal left stick. Not used
#define rc_pin_throttle 2           //PURPLE WIRE Vertical left stick.
#define rc_pin_power 13             //Plugs into RC pwr

long int xvalue;
int xmin;
int xmax;
int xrange;
int xdirect;
int xdeadzone;

long int yvalue;
int ymin;
int ymax;
int yrange;
int ydirect;
int ydeadzone;

long int wvalue;
int wmin;
int wmax;
int wrange;

void FindXRange(){
  if (xvalue < xmin){
    xmin = xvalue;}
  if (xvalue > xmax){
    xmax = xvalue;}
}

void FindYRange(){
  if (yvalue < ymin){
    ymin = yvalue;}
  if (yvalue > ymax){
    ymax = yvalue;}
}

void FindWRange(){
  if (wvalue < wmin){
    wmin = wvalue;}
  if (wvalue > wmax){
    wmax = wvalue;}
}


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
  //setColor(ORANGE);
  //Setup Variables
  String d = "";
  String t = "";

  //Check Power To Kangaroo Motion Controller
  /*kangaroo_on = digitalRead(KANGAROO_POWER);
  while(kangaroo_on == false){
    kangaroo_on = digitalRead(KANGAROO_POWER); 
    Serial.println(kangaroo_on); 
  }*/
  
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
  Serial1.println("t,s100");
  Serial1.println("d,s100");
  Serial1.println("t,s0");
  Serial1.println("d,s0");
  
}

void setup(){
  int i = 0;
  Serial.begin(9600);
  Serial1.begin(9600);
  pinMode(rc_pin_left_right, INPUT);
  pinMode(rc_pin_up_down, INPUT);
  pinMode(rc_pin_throttle, INPUT);
  pinMode(rc_pin_power, OUTPUT);
  digitalWrite(rc_pin_power, HIGH);

  initialize_kangaroo();
  
  do{
    xvalue = pulseIn(rc_pin_left_right, HIGH);
    yvalue = pulseIn(rc_pin_up_down, HIGH);
    wvalue = pulseIn(rc_pin_throttle, HIGH);
  }while(xvalue == 0);
  
  xmin = xvalue;
  xmax = xvalue;
  ymin = yvalue;
  ymax = yvalue;
  wmin = wvalue;
  wmax = wvalue;  

  while(i < 100){
    xvalue = pulseIn(rc_pin_left_right, HIGH);
    yvalue = pulseIn(rc_pin_up_down, HIGH);
    wvalue = pulseIn(rc_pin_throttle, HIGH); 
    FindXRange();
    FindYRange();
    FindWRange();
    i++;
    delay(100);
    Serial.println(xmin);
    Serial.println(ymin);
  }
  xrange = xmax - xmin;
  yrange = ymax - ymin;
  wrange = wmax - wmin;
  
  Serial.print("X range: ");
  Serial.print(xmin);
  Serial.print(" - ");
  Serial.println(xmax);
  Serial.print("X range = ");
  Serial.println(xrange);
  
  Serial.print("Y range: ");
  Serial.print(ymin);
  Serial.print(" - ");
  Serial.println(ymax);
  Serial.print("Y range = ");
  Serial.println(yrange); 
  
  Serial.print("W range: ");
  Serial.print(wmin);
  Serial.print(" - ");
  Serial.println(wmax);
  Serial.print("W range = ");
  Serial.println(wrange);
  
  Serial.println("Calibration Complete");
}

void loop(){

  String turn_setup = "t,s";
  String drive_setup = "d,s";
  
  xdirect = pulseIn(rc_pin_left_right, HIGH);
  ydirect = pulseIn(rc_pin_up_down, HIGH);
  wvalue = (10000 / wrange) * ( pulseIn(rc_pin_throttle, HIGH) - wmin) / 100;
  xvalue = (((10000 / xrange) * (xdirect - xmin) * wvalue / 100) - (50 * wvalue)) / (-4);
  yvalue = (((10000 / yrange) * (ydirect - ymin) * wvalue / 100) - (50 * wvalue)) / (-2);

  Serial.print("wvalue = ");
  Serial.println(wvalue);
  if (xvalue < 300 && xvalue > -300){
    Serial.print("x = ");
    Serial.println(0);
    Serial.println(xdirect);

    turn_setup.concat(0);
    Serial1.println(turn_setup);
    Serial.println(turn_setup);
}
  else{
    Serial.print("x = ");
    Serial.println(xvalue);
    Serial.println(xdirect);

    turn_setup.concat(xvalue);
    Serial1.println(turn_setup);
    Serial.println(turn_setup);
;
  }
  if (yvalue < 300 && yvalue > -300){
    Serial.print("y = ");
    Serial.println(0);
    Serial.println(ydirect);
    
    drive_setup.concat(0);
    Serial1.println(drive_setup);
    Serial.println(drive_setup);
}
  else{
    Serial.print("y = ");
    Serial.println(yvalue);
    Serial.println(ydirect);

    drive_setup.concat(yvalue);
    Serial1.println(drive_setup);
    Serial.println(drive_setup);
  }
  delay(500);
}
