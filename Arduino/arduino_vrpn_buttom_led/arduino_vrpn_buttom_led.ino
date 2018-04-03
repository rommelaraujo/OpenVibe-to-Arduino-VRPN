#include <Servo.h>

int inByte = 0; //Signal from OV
int button = 0; //Button from OV

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {

   //test if there's new data from C++ client
  if (Serial.available())  
  {
     // get the first byte of incoming serial data available
       inByte = Serial.read();   
        
       if (inByte == 0){
        digitalWrite(13,LOW);
       }

       if (inByte == 1){
        digitalWrite(13,HIGH);
       }
  }
}


     
         

     
  

