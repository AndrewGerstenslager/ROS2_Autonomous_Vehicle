String temp = "";

void setup() {
 Serial.begin(9600);
 Serial.println("Ready");
}

void loop() {
 temp = Serial.readString();

  if (temp != ""){
    Serial.println(temp);
    temp = "";
  }
}