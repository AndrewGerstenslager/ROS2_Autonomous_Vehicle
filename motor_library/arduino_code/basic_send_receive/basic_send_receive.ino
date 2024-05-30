String com_usb = "";

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


void setup() {
 Serial1.begin(9600);
 Serial.begin(9600);

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
  Serial1.println("t,s100");
  Serial1.println("d,s100");
  Serial1.println("t,s0");
  Serial1.println("d,s0");

  Serial.println("SETUP COMPLETE");
}



void loop() {
  com_usb = Serial.readString();
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
        Serial.println("STOPPED");
      }
  Serial.println(com_usb);
}
