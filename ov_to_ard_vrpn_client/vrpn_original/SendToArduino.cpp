#include "stdafx.h"

#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>

// application reads from the specified serial port and reports the collected data
int _tmain(int argc, _TCHAR* argv[])
{
	printf("Welcome to the serial test app!\n\n");

	Serial* SP = new Serial("\\\\.\\COM5");    // adjust as needed

	if (SP->IsConnected())
		printf("We're connected");

	char incomingData[256] = "";			// don't forget to pre-allocate memory
	//printf("%s\n",incomingData);


	int dataLength = 256;
	int readResult = 0;

	while (SP->IsConnected())
	{
		//readResult = SP->ReadData(incomingData, dataLength);
		incomingData[1] = 11;
		incomingData[2] = 17;
		readResult = SP->WriteData(incomingData, dataLength);

		printf("Bytes read: (-1 means no data available) %i\n", readResult);

		

		Sleep(500);
	}
	return 0;
}