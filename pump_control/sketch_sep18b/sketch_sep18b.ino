
//char DATA_LENGTH = 16; //number of bytes to send over to C++ client

int inByte = 0; //Signal from OV
//char dataOut[16];  //all data to be sent over to C++ client


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13, OUTPUT);
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
      if (inByte == 1){
      digitalWrite(13, HIGH);
      }
      else {
      digitalWrite(13, LOW);
      }

     
         

     
  }
}
