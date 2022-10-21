String com_usb = "";

void setup() {
 Serial.begin(9600);
 Serial.println("Ready");
}
void loop() {
 com_usb = Serial.readString();

  if (com_usb != ""){
    if(com_usb.charAt(0) == 't' || com_usb.charAt(0) == 'd'){
      Serial.println("Kangaroo Command");
    }
    else if(com_usb.charAt(0) == 'r'){
      Serial.println("RC receiver Command");
    }
    else{
      Serial.println("Command not recognized");
    }
    
    com_usb = "";
  }
}
