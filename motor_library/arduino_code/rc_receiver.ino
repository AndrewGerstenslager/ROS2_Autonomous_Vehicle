#define rc_pin_left_right 4
#define rc_pin_up_down 5
#define rc_pin_trainer_toggle 7

int turn_calculated;
int drive_calcualted;
int left_right_stick;
int up_down_stick;
int trainer_toggle;
int controller_tolerance = 20;

void calculate_speeds(){
  
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
}


void setup() {
  Serial.begin(9600);
  pinMode(rc_pin_left_right, INPUT);
  pinMode(rc_pin_up_down, INPUT);
  pinMode(rc_pin_trainer_toggle, INPUT);
}




void loop() {
 calculate_speeds();
 Serial.println("d,s" +  String(turn_calculated));
 Serial.println("t,s" + String(drive_calcualted));
 delay(500);
}

//in4 left right - all left = {1184} all right = {1791}
//in5 up down    - all up =   {1874} all down = {1274}
//in7 trainer controller - 0 = {1878}, 1 = {1579}, 2 = {1085}
