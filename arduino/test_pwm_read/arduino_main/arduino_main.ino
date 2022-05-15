float throtle_sensitivity = 1;
int throtle_deadzone = 180;
//float turn_sensitivity = 1;
int turn_deadzone = 80;

int negate(int num, bool flag){
  return (num ^ -flag) + flag;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int T_pwm_value = pulseIn(6, HIGH);
  int A_pwm_value = pulseIn(9, HIGH);
  int E_pwm_value = pulseIn(10, HIGH);
  int G_pwm_value = pulseIn(11, HIGH);

  //the gear output only outputs values 1090 and 1890
  //thus the gear switch can be used as the boolean for
  //whether or not to use the RC inputs
  bool use_RC = (G_pwm_value < 1500);

  if(use_RC){
    //this should give the throtle a range 700 (1200-1900)
    //with a small deadzone near 0
    int motor_power = max(0,(T_pwm_value-1030-throtle_deadzone)/throtle_sensitivity);

    int turn_ratio;
    if(abs(A_pwm_value-1490)-turn_deadzone){
      turn_ratio = 0;
    }
    else{
      turn_ratio = (A_pwm_value-1490)/400;
    }

    bool reverse = (E_pwm_value > 1700);

    int left_motor_power = (1-turn_ratio)*negate(motor_power,reverse);
    int right_motor_power = (1+turn_ratio)*negate(motor_power,reverse);
  }
}
