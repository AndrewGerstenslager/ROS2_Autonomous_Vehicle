#include <FastLED.h>

#define LED_PIN     12
#define NUM_LEDS    16

CRGB leds[NUM_LEDS];
String RecievedInput;
String setcolor;

String WaitForInput(String setcolor) {
  Serial.println(setcolor);
 
  while(!Serial.available()) {
    // wait for input
  }
  return Serial.readStringUntil(10);
}

void setred () {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ( 125, 0, 10);
    delay(50);
  
  }
  FastLED.show();
}

void setyellow (){
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ( 255, 255, 0);
    delay(20);
}
FastLED.show();
}

void setgreen (){
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ( 0, 128, 0);
    delay(20);
}
FastLED.show();
}

void setblack () {
  for (int i = 0; i <= NUM_LEDS; i++) {
    leds[i] = CRGB ( 0, 0, 0);
    delay(20);
}
FastLED.show();
}

void setblue () {
  for (int i = 19; i >= 0; i--) {
    leds[i] = CRGB ( 10, 0, 125);
    FastLED.show();
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("I'M ALIVE!");

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

}

void loop() {
  RecievedInput = WaitForInput("Set color: ");
  Serial.println("The string you entered was: ");
  Serial.println(RecievedInput);
  
  if (RecievedInput.equals("setred")){
    setred();
  }
  else if (RecievedInput.equals("setblue")){
      setblue();
  }
  else if (RecievedInput.equals("setgreen")){ 
      setgreen();
  }
  else if (RecievedInput.equals("setyellow")){
    setyellow();
  }
  else {
    setblack();
  }
}
