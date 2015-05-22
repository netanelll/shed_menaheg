#include "CXBOXController.h"
#include <iostream>
#include <windows.h>

using namespace std;

CXBOXController* Player1;
int main(int argc, char* argv[])
{
	HANDLE serialHandle;
	serialHandle = CreateFile("\\\\.\\COM15", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	// get serial parameters
	DCB dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof (dcbSerialParams);
	if (!GetCommState(serialHandle, &dcbSerialParams)) {
		cout << "error getting state\n";
		exit(0);
	}

	// set serial params
	dcbSerialParams.BaudRate = CBR_9600;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	if (!SetCommState(serialHandle, &dcbSerialParams)) {
		cout << "error setting parameters\n";
		exit(0);
	}

	COMMTIMEOUTS timeout = { 0 };
	timeout.ReadIntervalTimeout = 50;
	timeout.ReadTotalTimeoutConstant = 50;
	timeout.ReadTotalTimeoutMultiplier = 50;
	timeout.WriteTotalTimeoutConstant = 50;
	timeout.WriteTotalTimeoutMultiplier = 10;

	SetCommTimeouts(serialHandle, &timeout);
	
	Player1 = new CXBOXController(1);

	cout << "Instructions:\n";
	cout << "[A] Vibrate Left Only\n";
	cout << "[B] Vibrate Right Only\n";
	cout << "[X] Vibrate Both\n";
	cout << "[Y] Vibrate Neither\n";
	cout << "[BACK] Exit\n";

	while (true)
	{
		if (Player1->IsConnected())
		{
			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP)
			{
				Player1->Vibrate(65535, 0);
			}

			//if (Player1->GetState().Gamepad.sThumbLX)
			//{
			//	int tmp = Player1->GetState().Gamepad.sThumbLX;
			//	if (tmp > 0)
			//		Player1->Vibrate(0, tmp);
			//	else if (tmp < 0)
			//		Player1->Vibrate(tmp, 0);
			//	else
			//		Player1->Vibrate();
			//}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_Y)
			{
				Player1->Vibrate(65535, 65535);
				char send = 'a';
				DWORD dwBytesWritten = 0;
				WriteFile(serialHandle, &send, 1, &dwBytesWritten, NULL);
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER)
			{
				Player1->Vibrate();
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_BACK)
			{
				break;
			}
		}
		else
		{
			cout << "\n\tERROR! PLAYER 1 - XBOX 360 Controller Not Found!\n";
			cout << "Press Any Key To Exit.";
			cin.get();
			break;
		}
		unsigned char temp = 0;
		DWORD dwBytesRead = 0;
		ReadFile(serialHandle, &temp, 1, &dwBytesRead, NULL);
		if (1 == dwBytesRead) cout << temp;

	}
	delete(Player1);
	


	CloseHandle(serialHandle);


	return(0);
}