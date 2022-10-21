String com_usb = "";

void setup() {
 Serial.begin(9600);
 Serial.setTimeout(10);
 Serial.println("Ready");
}
void loop() {
  com_usb = Serial.readString();
  
  if (com_usb != ""){
    
    if(com_usb.charAt(0) == 't' || com_usb.charAt(0) == 'd'){
      Serial.println("Kangaroo Command");
    }
    else if(com_usb.charAt(0) == 'r' && com_usb.charAt(1) == 'c'){
      Serial.println("RC receiver Command");
    }
    else{
      Serial.println("Command not recognized");
    }
    Serial.println(com_usb);
    
    com_usb = "";
  }
}
