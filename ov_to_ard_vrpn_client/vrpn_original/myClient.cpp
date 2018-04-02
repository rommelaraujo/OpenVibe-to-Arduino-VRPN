#include "stdafx.h"

#include <iostream>
//using namespace std;

#include <vrpn_Analog.h>
#include <vrpn_Button.h>

#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>

int num_button = 0;

void VRPN_CALLBACK vrpn_button_callback(void* user_data, vrpn_BUTTONCB button)
{
	//std::cout << "Button ID : " << button.button << " / Button State : " << button.state << std::endl;

	
	num_button = button.button;

	//if (button.button == 1)
	//{
		//*(bool*)user_data = false;
	//}
}




/*
void VRPN_CALLBACK vrpn_analog_callback(void* user_data, vrpn_ANALOGCB analog)
{
	//int nbChannels = analog.num_channel;
	if (analog.channel[0] > 0)
	{
		//cout << "Analógico: ";

		//for (int i = 0; i < analog.num_channel; i++){
			//cout << analog.channel[i] << " ";
		//}

		*(char*)user_data = analog.channel[0];
	//	*(char*)user_data = 1;	//open hand

		//cout << endl;

	}
	else
	{
		*(char*)user_data = -1;	//close hand
	}	
}
*/

int main(int argc, char** argv)
{

	bool running = true;
	//char dataFromOV = 0;
	

	// VRPN Analog object 
	//vrpn_Analog_Remote* VRPNAnalog;


	/* VRPN Button object */
	vrpn_Button_Remote* VRPNButton;

	/* Binding of the VRPN Analog to a callback */
	//VRPNAnalog = new vrpn_Analog_Remote("openvibe_vrpn_analog@localhost");
	//VRPNAnalog->register_change_handler(&dataFromOV, vrpn_analog_callback);

	
	/* Binding of the VRPN Button to a callback */
	VRPNButton = new vrpn_Button_Remote("openvibe_vrpn_button@localhost");
	VRPNButton->register_change_handler(&running, vrpn_button_callback);

	//*************************************
	//Arduino communication
	
	Serial* SP = new Serial("\\\\.\\COM10");    // adjust as needed
	
	//int incomingDataLength = 1;
	int outgoingDataLength = 1;
	
	//char incomingData[1] = "";			// don't forget to pre-allocate memory
	char outgoingData[1] = "";				//data to be sent

	//bool in = "";
	//bool out = "";

	//bool readResult_B = false;
	//bool writeResult_B = false;
	

	//int readResult = 0;
	int writeResult = 0;
	

	

	printf("Attempting communication with Arduino\n\n");

	if (SP->IsConnected())
	{
		printf("We're connected\n");
		
	}
	else
		printf("Connection failed");
	

	
	while (SP->IsConnected())
	{
		//VRPNAnalog->mainloop();	//check for new incoming OV data

		VRPNButton->mainloop(); 

		if (running != false)	//data received from OV
		{
			outgoingData[0] = num_button;
			//in = running;


			writeResult = SP->WriteData(outgoingData, outgoingDataLength);	//write to Arduino


			printf("Data sent from OV to Arduino: %d \n", outgoingData[0]);

			//printf("%d \n", dataFromOV);

			//running = false;
			//running = false;
		}

		//readResult = SP->ReadData(incomingData, incomingDataLength);	//returns -1 if there's nothing to read

		//if (readResult > 0)	//a message has been sent from Arduino
			//printf("Data received from Arduino: %d \n\n", incomingData[0]);

		//Sleep(500);
		//*/
		
		//while (running){
		//VRPNButton->mainloop();
		//}
	}
		//************************************************
	
	return 0;
}