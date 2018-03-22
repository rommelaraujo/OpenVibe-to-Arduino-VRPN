#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <vrpn_Analog.h>
#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	
#include <string>
#include <conio.h>

using namespace std;

//*********************************************************************
//VARIABLES
//*********************************************************************
double time_stamp = 0;	//sync clock coming from OV
double rawClassifier = 0;	//raw classifier values
double previousClassifierData=0;	//to make sure a new classifier data has arrived
char dataFromOV = 0;	//function requirement
char dataFromOVProcessed = 0;	//1: open hands; 0: close hands
char incomingDataLength = 5;	//number of bytes coming from Arduino (check Arduino code for details)
char outgoingDataLength = 1;	//number of bytes going to Arduino

char incomingData[4] = "";			//data from Arduino - don't forget to pre-allocate memory
char outgoingData[1] = "";			//data (commands) to be sent to Arduino - 1 mild speed; 2 high speed; 3 close hand

int keyboardInput;	//to test 'q' key has been pressed to terminate the program
int readResult = 0;	//when reading/writing to arduino, a confirmation int is returned to indicate success
int writeResult = 0;

Serial* SP = new Serial("\\\\.\\COM5");    //communication port with Arduino

//data file creation
ofstream myfile;
//*********************************************************************


void printMessageOnScreen(char messageCode)
{
	if (messageCode == 1)
	{
		printf("OV to Arduino: Pump! \n");
	}	
	else if (messageCode == 0)
	{
		printf("OV to Arduino: Stop! \n");
	}
}

void VRPN_CALLBACK vrpn_analog_callback(void* user_data, vrpn_ANALOGCB analog)
{		
	//almost impossible to have two exactly equal values over time from OV, 
	//without this line the data file would have an entry for every time 
	//stamp produced by the synchronisation clock in OV
	if (!(analog.channel[0]==previousClassifierData))	
	{
		printf("Analog data sent from OV to Arduino: %f \n", analog.channel[0]);
		if (analog.channel[0] > 0) //user wants to open hand
		{			
			*(char*)user_data = 1;	//mild speed
			dataFromOVProcessed = 1;
		}
		else
		{
			*(char*)user_data = 0;	//close hand
			dataFromOVProcessed = 0;
		}

		rawClassifier = (double) analog.channel[0];
		time_stamp=(double) analog.channel[1];
		previousClassifierData=analog.channel[0];
				
		outgoingData[0] = dataFromOVProcessed;
		
		//data file writing
		myfile << time_stamp;	//simulation time from OV, to be used later for synchronising with raw EEG data
		myfile << '\t';
		myfile << rawClassifier;	//raw classifier value from OV
		myfile << '\t';
		myfile << (int)outgoingData[0];	//classifier data
		myfile << '\t';

		writeResult = SP->WriteData(outgoingData, outgoingDataLength);	//write to Arduino

		printMessageOnScreen(outgoingData[0]);	//show on screen if hand is opening or closing
		
		//dataFromOV = 0;	//clear flag		
		readResult = SP->ReadData(incomingData, incomingDataLength);	//Read from Arduino, returns -1 if there's nothing to read

		if (readResult>0)	//message received from Arduino
		{
			printf("Flex sensor reading: %d \n", incomingData[0]);
			printf("Pump state: %d \n", incomingData[1]);
			unsigned short int pressureSensor = ((incomingData[2] << 8) | incomingData[3]);
			printf("Pressure Sensor: %u \n\n", pressureSensor);

			myfile << (int)incomingData[0] << '\t' << pressureSensor << '\t' << (int)incomingData[1] << '\t';	//data file writing
		}
		else
		{
			printf("No Arduino data");
			myfile << -1 << '\t' << -1 << '\t' << -1;	//no message received from Arduino			
		}
		myfile << '\n';			
	}
	else
	{
		printf("No new data from OV\n");
	}
}

int main(int argc, char** argv)
{	
	/* VRPN Analog object */
	vrpn_Analog_Remote* VRPNAnalog;

	/* Binding of the VRPN Analog to a callback */
	VRPNAnalog = new vrpn_Analog_Remote("openvibe_vrpn_analog@localhost");
	VRPNAnalog->register_change_handler(&dataFromOV, vrpn_analog_callback);

	//this is the data file containing the raw classifier value, the BMI command (open/close hand), the flex sensors, pressure sensors, and pump state (on/off)
	myfile.open("data", ios::out | ios::binary);
	myfile << "Time \t Raw Classifier \t BMI command \t Flex sensor \t Pressure sensor \t Pump state \n";
	
	//*************************************
	//Arduino communication
		
	//attempt connection with Arduino
	printf("Attempting communication with Arduino\n\n");
	if (SP->IsConnected())
	{
		printf("We're connected\n");		
	}
	else
		printf("Connection failed");
	
	//While Arduino connection is active
	while (SP->IsConnected())
	{
		//Check for new incoming OV data. All data will be synched by the EEG classifier, every time there's an output, we also update sensors and actuators			
		VRPNAnalog->mainloop();	

		//To finish the code, user must type in "q"
		if (kbhit())
		{	
			keyboardInput = getch();		
			if (keyboardInput=='q')
			{
				break;
			}
		}	
	}

	myfile.close();	//close data file writing
	//************************************************
	return 0;
}