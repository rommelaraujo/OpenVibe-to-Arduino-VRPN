
#include <PID_v1.h>
#include <SPI.h>

/*
SOPHIA code for interfacing and controlling the 4 flexibe fingers
*/

//*********************************************************************
//Pinout configuration for pumps, valves, sensors
//*********************************************************************
//pumps and valves
const int pumpA1 = 11; const int valveA1 = 7;
const int pumpA2 = 10; const int valveA2 = 6;
const int pumpA3 = 9; const int valveA3 = 5;
const int pumpA4 = 8; const int valveA4 = 4;

//flex Sensors
const int flexSensorPin1 = A0;
const int flexSensorPin2 = A1;
const int flexSensorPin3 = A2;
const int flexSensorPin4 = A3;

//Multiplexer pins
const int multiplexerPin1 = 36;
const int multiplexerPin2 = 38;

// SPI pins used for the connection with the pressure sensor
const int chipSelectPin = 53;

//SAFETY BUTTON: when pressed, open valves and turn-off pumps
const int safetyButton = 22;
//*********************************************************************

//*********************************************************************
//Constants
//*********************************************************************
char DATA_LENGTH = 16; //number of bytes to send over to C++ client
double kp=10; double ki=5; double kd=0; //PID gains

//*********************************************************************

//*********************************************************************
//Variables
//*********************************************************************
int flexSensor1; //flex sensor reading
int flexSensor2; //flex sensor reading
int flexSensor3; //flex sensor reading
int flexSensor4; //flex sensor reading

char normFlexSensor1;
char normFlexSensor2;
char normFlexSensor3;
char normFlexSensor4;

unsigned int pressureSensor1;  //pressure sensor reading
unsigned int pressureSensor2;  //pressure sensor reading
unsigned int pressureSensor3;  //pressure sensor reading
unsigned int pressureSensor4;  //pressure sensor reading

double flexSensor1_PIDsetpoint, flexSensor1_PIDinput, flexSensor1_PIDoutput; //PID variables
double flexSensor2_PIDsetpoint, flexSensor2_PIDinput, flexSensor2_PIDoutput; //PID variables
double flexSensor3_PIDsetpoint, flexSensor3_PIDinput, flexSensor3_PIDoutput; //PID variables
double flexSensor4_PIDsetpoint, flexSensor4_PIDinput, flexSensor4_PIDoutput; //PID variables

int inByte = 0; //Signal from OV
char dataOut[16];  //all data to be sent over to C++ client
char PUMP_STATE;  //1:pump is on; 0:pump is off
unsigned long start_time; //to keep track of time

char pressureByte1[1]; //can only transmit chars to C++ client, but pressure data is int
char pressureByte2[1]; //remember to add the number of sensors between brackets

//Specify the links and initial tuning parameters
//Input: The variable we're trying to control (flex sensor)
//Output: The variable that will be adjusted by the pid (pumps)
//Setpoint: The value we want to Input to maintain (flex sensor reading corresponding to full extension)
PID flexSensor1PID(&flexSensor1_PIDinput, &flexSensor1_PIDoutput, &flexSensor1_PIDsetpoint,kp,ki,kd, DIRECT);
PID flexSensor2PID(&flexSensor2_PIDinput, &flexSensor2_PIDoutput, &flexSensor2_PIDsetpoint,kp,ki,kd, DIRECT);
PID flexSensor3PID(&flexSensor3_PIDinput, &flexSensor3_PIDoutput, &flexSensor3_PIDsetpoint,kp,ki,kd, DIRECT);
PID flexSensor4PID(&flexSensor4_PIDinput, &flexSensor4_PIDoutput, &flexSensor4_PIDsetpoint,kp,ki,kd, DIRECT);
//*********************************************************************

//Runs only once at the beginning of the program
void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(9600);

  //start the SPI library  
  SPI.begin();
  SPI.beginTransaction(SPISettings(400000, MSBFIRST, SPI_MODE1));  
  pinMode(chipSelectPin, OUTPUT); // initialise the  data ready and chip select pins
    
  //Setup Pump/Valve IO States
  pinMode(pumpA1, OUTPUT); pinMode(valveA1, OUTPUT);
  pinMode(pumpA2, OUTPUT); pinMode(valveA2, OUTPUT);
  pinMode(pumpA3, OUTPUT); pinMode(valveA3, OUTPUT);
  pinMode(pumpA4, OUTPUT); pinMode(valveA4, OUTPUT);

  //Multiplexer pins
  pinMode(multiplexerPin1, OUTPUT);
  pinMode(multiplexerPin2, OUTPUT);

  //Safety button
  pinMode(safetyButton, INPUT);

  // PID variables
  flexSensor1_PIDsetpoint = 60;  //value when fingers are fully extended
  flexSensor2_PIDsetpoint = 50;  //value when fingers are fully extended
  flexSensor3_PIDsetpoint = 60;  //value when fingers are fully extended
  flexSensor4_PIDsetpoint = 40;  //value when fingers are fully extended
  
  flexSensor1PID.SetMode(AUTOMATIC);
  flexSensor2PID.SetMode(AUTOMATIC);
  flexSensor3PID.SetMode(AUTOMATIC);
  flexSensor4PID.SetMode(AUTOMATIC);
  
  PUMP_STATE=0; //pumps start turned-off
  pinMode(13, OUTPUT);

  start_time=millis();  
}

// The loop() method runs over and over again, for as long as the Arduino has power
void loop()
{
  unsigned long current_time = millis();

  //******************
  //flex sensor reading
  flexSensor1 = readFlexSensor(flexSensorPin1);
  normFlexSensor1 = normaliseFlexSensor(flexSensor1,435,720);

  flexSensor2 = readFlexSensor(flexSensorPin2);
  normFlexSensor2 = normaliseFlexSensor(flexSensor2,505,720);

  flexSensor3 = readFlexSensor(flexSensorPin3);
  normFlexSensor3 = normaliseFlexSensor(flexSensor3,422,720);

  flexSensor4 = readFlexSensor(flexSensorPin4);
  normFlexSensor4 = normaliseFlexSensor(flexSensor4,335,650);
  //******************
  
  //pressure sensor reading
  pressureSensor1 = readPressureSensor(chipSelectPin,1,0,0);
  pressureSensor2 = readPressureSensor(chipSelectPin,2,0,1);
  pressureSensor3 = readPressureSensor(chipSelectPin,3,1,0);
  pressureSensor4 = readPressureSensor(chipSelectPin,4,1,1);  
  //******************
  
  //PID calculation  
  flexSensor1_PIDinput = (double)normFlexSensor1;
  flexSensor1PID.Compute();

  flexSensor2_PIDinput = (double)normFlexSensor2;
  flexSensor2PID.Compute();

  flexSensor3_PIDinput = (double)normFlexSensor3;
  flexSensor3PID.Compute();

  flexSensor4_PIDinput = (double)normFlexSensor4;
  flexSensor4PID.Compute();
  //******************

  //Display hardware status on screen every 1000 ms
  displayMessagesOnScreen(current_time, 1000);  
  
  //test if there's new data from C++ client
  //if (Serial.available())  
  if (1)  
  {
      // get the first byte of incoming serial data available
     //this will be the EEG command, whether to turn pumps on or off
      inByte = Serial.read();   
      //inByte = 1;

      //Turn pumps and valves on/off depending on the PID output for a given flex sensor reading
      commandPumpsValves(pumpA1,valveA1,flexSensor1_PIDoutput,inByte);
      commandPumpsValves(pumpA2,valveA2,flexSensor2_PIDoutput,inByte);
      //commandPumpsValves(pumpA3,valveA3,flexSensor3_PIDoutput,inByte);
      //commandPumpsValves(pumpA4,valveA4,flexSensor4_PIDoutput,inByte);

      //append data to the array to be sent to C++ client
      appendDataOut(normFlexSensor1,PUMP_STATE,pressureByte1[1],pressureByte2[1],0);    
      appendDataOut(normFlexSensor2,PUMP_STATE,pressureByte1[2],pressureByte2[2],4);
      appendDataOut(normFlexSensor3,PUMP_STATE,pressureByte1[3],pressureByte2[3],8);
      appendDataOut(normFlexSensor4,PUMP_STATE,pressureByte1[4],pressureByte2[4],12);    

      //write to C++ code
      //writeToCpp(dataOut);
  }

  if(digitalRead(safetyButton))
  {
      digitalWrite(pumpA1, LOW);
      digitalWrite(valveA1, LOW);
      digitalWrite(pumpA2, LOW);
      digitalWrite(valveA2, LOW);
      digitalWrite(pumpA3, LOW);
      digitalWrite(valveA3, LOW);
      digitalWrite(pumpA4, LOW);
      digitalWrite(valveA4, LOW);
  }
  
  delay(500);
}

void appendDataOut(char flex,char p_state,char p_byte1, char p_byte2,char start_index)
{
      dataOut[start_index]=flex;  //flex sensor data    
      dataOut[++start_index]=p_state;  //pump on or off
      dataOut[++start_index]=p_byte1;  //pressure sensor data byte 1
      dataOut[++start_index]=p_byte2;  //pressure sensor data byte 2
}

void writeToCpp(char data[])
{
    for(int i=0;i<DATA_LENGTH;i++)
    {
      Serial.write(data[i]);
    } 
}

int readFlexSensor(int flexSensorID)
{
  return(analogRead(flexSensorID));
}

char normaliseFlexSensor(int flexSensorReading, int minValue, int maxValue)
{
  //maps flex sensor readings from 0 to 100
  unsigned int sensorValue = map(flexSensorReading, minValue, maxValue, 0, 100);
  return(floor(sensorValue/10)*10); //readings vary only in the second digit, makes pump PID commands less flickerys    
}

unsigned int readPressureSensor(int pressureSensorID, char pressureSensorNumber, int multiplexerSelector1, int multiplexerSelector2)
{
  //Choose which sensor to read from multiplexer
  digitalWrite(multiplexerPin1,multiplexerSelector1);
  digitalWrite(multiplexerPin2,multiplexerSelector2);
            
  digitalWrite(pressureSensorID, LOW);  //SS line down activates the sensor for reading  
  //Can receive 4 bytes of data, but only 2 MSB matter (pressure)
  unsigned int  pressure_data = SPI.transfer(0x00);  //read first byte
  unsigned int pressure_data_ok = pressure_data & 0xC0; //isolate status bit from first byte   
  unsigned int pressure_data_B1 = pressure_data & 0x3F;; //isolate 6 data bits from first byte (see pressure sensor datasheet)
  
  unsigned int pressure_data_2 = SPI.transfer(0x00);  //read second byte
  unsigned int pressure_data_B2 = pressure_data_2; //isolate status bit from first byte
  digitalWrite(pressureSensorID, HIGH); //SS line up deactivate sensor for reading

  pressureByte1[pressureSensorNumber]= pressure_data_B1;  
  pressureByte2[pressureSensorNumber]= pressure_data_B2;  
   
  return ((pressure_data_B1 << 8) | pressure_data_B2);  
}

void commandPumpsValves(int pumpID, int valveID, double sensorPIDOutput, int controlFromEEG)
{
  if (controlFromEEG==1)
  { 
    digitalWrite(13, HIGH);
   
    if (sensorPIDOutput > 20) //arbitrary threshold value for PID output
    {   
      analogWrite(pumpID, sensorPIDOutput);
      digitalWrite(valveID, HIGH);      
      PUMP_STATE = 1;      
    }
    else
    {
      digitalWrite(pumpID, LOW);
      digitalWrite(valveID, HIGH);
      PUMP_STATE = 0;      
    }
  }
  else
  {
    digitalWrite(pumpID, LOW);
    digitalWrite(valveID, LOW);
    PUMP_STATE = 0;
    digitalWrite(13, LOW);    
  }
}

void displayMessagesOnScreen(int current_time, int time_interval)
{
  if(current_time-start_time>time_interval)  //arbitrary 1s interval for screen plot
  {
    Serial.print("Flex Sensor 1: ");
    Serial.println((int)normFlexSensor1); //435-720:669
    Serial.print("Flex Sensor 1 PID output: ");
    Serial.println(flexSensor1_PIDoutput);    
    Serial.print("Pressure Sensor 1 = ");
    Serial.println(pressureSensor1);
//
    Serial.print("Flex Sensor 2: ");
    Serial.println((int)normFlexSensor2); //505-720:675
    Serial.print("Flex Sensor 2 PID output: ");
    Serial.println(flexSensor2_PIDoutput);    
    Serial.print("Pressure Sensor 2 = ");
    Serial.println(pressureSensor2);
//
    Serial.print("Flex Sensor 3: ");  //422-720:676
    Serial.println((int)normFlexSensor3);
    Serial.print("Flex Sensor 3 PID output: ");
    Serial.println(flexSensor3_PIDoutput);    
    Serial.print("Pressure Sensor 3 = ");
    Serial.println(pressureSensor3);
//
    Serial.print("Flex Sensor 4: ");  //335-650:528
    Serial.println((int)normFlexSensor4);
    Serial.print("Flex Sensor 4 PID output: ");
    Serial.println(flexSensor4_PIDoutput);
    Serial.print("Pressure Sensor 4 = ");
    Serial.println(pressureSensor4);
    Serial.println("");
    start_time=current_time;    
  }  
}



