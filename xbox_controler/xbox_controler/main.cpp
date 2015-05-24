#include "CXBOXController.h"
#include <iostream>
#include <windows.h>

using namespace std;

#define ENABLE_COM_PORT 1

CXBOXController* Player1;
int main(int argc, char* argv[])
{
#if ENABLE_COM_PORT == 1
	HANDLE serialHandle;
	serialHandle = CreateFile("\\\\.\\COM17", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
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
#endif
	Player1 = new CXBOXController(1);
	int current_pitch, previous_pitch = 0, current_yaw, previous_yaw = 0;
	UINT8 pitch_to_send, yaw_to_send;
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
			current_pitch = ((Player1->GetState().Gamepad.sThumbLY * 15) / 32768) + 15;
			current_yaw = ((Player1->GetState().Gamepad.sThumbLX * 15) / 32768) + 15;

			if (current_pitch != previous_pitch)
			{
				pitch_to_send = (current_pitch & 0x1F) + (1 << 6);
			#if ENABLE_COM_PORT == 1
				DWORD dwBytesWritten = 0;
				WriteFile(serialHandle, &pitch_to_send, 1, &dwBytesWritten, NULL);
			#endif
				previous_pitch = current_pitch;
			}
			if (current_yaw != previous_yaw)
			{
				pitch_to_send = (current_yaw & 0x1F);
			#if ENABLE_COM_PORT == 1
				DWORD dwBytesWritten = 0;
				WriteFile(serialHandle, &pitch_to_send, 1, &dwBytesWritten, NULL);
			#endif
				previous_yaw = current_yaw;
			}
			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP)
			{
				Player1->Vibrate(65535, 0);
			}



			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_Y)
			{
				Player1->Vibrate(65535, 65535);
			#if ENABLE_COM_PORT == 1
				char send = 254;
				DWORD dwBytesWritten = 0;
				WriteFile(serialHandle, &send, 1, &dwBytesWritten, NULL);
			#endif
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
		#if ENABLE_COM_PORT == 1
		unsigned char temp = 0;
		DWORD dwBytesRead = 0;
		ReadFile(serialHandle, &temp, 1, &dwBytesRead, NULL);
		if (1 == dwBytesRead) cout << temp;
#		endif

	}
	delete(Player1);


	#if ENABLE_COM_PORT == 1
	CloseHandle(serialHandle);
	#endif

	return(0);
}