#include <ServoTimer2.h>

ServoTimer2 servo;

#define PSD_F A5
#define PSD_R A4
#define PSD_L A3

// constants won't change. They're used here to
// set pin numbers:
const int buttonPin = 12;     // the number of the pushbutton pin
long int count = 544;
boolean turn = 0;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  Serial.begin(9600);
  servo.attach(2);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(A5,INPUT);
  pinMode(A4,INPUT);
  pinMode(A3,INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    
    //if(turn == 0){
      //count += 10;
    //}
    //else if(turn == 1){
      //count -= 10;
    //}
    //if(count == 2400){
      //turn = 1;
    //}
    //if(count == 0){
      //turn = 0;
    //}
  } else {
    //if(turn == 0){
      //servo.write(2400);
    //}
    //else if(turn == 1){
      //servo.write(544);
    //}

    //if(turn == 0){
      //turn = 1;
    //}
    //else if(turn == 1){
      //turn = 0;
    //}
    servo.write(1240);
    
  }
  Serial.print(" M:");
  Serial.print(analogRead(A5));
  Serial.print(" R:");
  Serial.print(analogRead(A4));
  Serial.print(" L:");
  Serial.println(analogRead(A3));
  
  delay(20);
}
