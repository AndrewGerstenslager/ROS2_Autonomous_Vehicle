//#include <String>
void setup() {
  // put your setup code here, to run once:
 pinMode(3, INPUT);
 pinMode(5, INPUT);
 pinMode(6, INPUT);
 Serial.begin(115200);
 delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int T_pwm_value = pulseIn(6, HIGH);
  int A_pwm_value = pulseIn(9, HIGH);
  int E_pwm_value = pulseIn(10, HIGH);
  int G_pwm_value = pulseIn(11, HIGH);
  String output;
  output += (String(G_pwm_value));
  output += ("\t");
  output += (String(E_pwm_value));
  output += ("\t");
  output += (String(A_pwm_value));
  output += ("\t");
  output += (String(T_pwm_value));
  Serial.println(output);
} 
