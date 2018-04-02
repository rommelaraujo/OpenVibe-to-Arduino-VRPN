#include <Servo.h>

int inByte = 0; //Signal from OV
int button = 0; //Button from OV
Servo mg946r;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  mg946r.attach(9); //Define o pino 9 como saída PWM
  mg946r.write(0); //Zera a referência do servomotor
  digitalWrite(13, LOW);
}

void loop() {

   //test if there's new data from C++ client
  if (Serial.available())  
  //if (1)  
  {
      // get the first byte of incoming serial data available
     //this will be the EEG command, whether to turn pumps on or off
      inByte = Serial.read();   
      //inByte = -1;
      //button = Serial.read();
      
     // delay(1);
     // if (button == HIGH){
        //mg946r.write(90);
       // digitalWrite(13, HIGH);
     // }
     // else{
     //   digitalWrite(13, LOW);
     // }
//      
      if (inByte == 2){
      //digitalWrite(13, HIGH);
      mg946r.write(45);
      }
      if (inByte == 3){
        digitalWrite(13,LOW);
        mg946r.write(15);
      }
      //if (inByte == 2){
      //mg946r.write(15);  
      //}
 //else {
//      digitalWrite(13, LOW);
   //   mg946r.write(0);
     // }

     
         

     
  
  }
}
