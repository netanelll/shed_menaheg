#include "CXBOXController.h"
#include <iostream>
#include <windows.h>

CXBOXController* Player1;
int main(int argc, char* argv[])
{
	HANDLE hSerial;
	hSerial = CreateFile("COM1", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
	Player1 = new CXBOXController(1);

	std::cout << "Instructions:\n";
	std::cout << "[A] Vibrate Left Only\n";
	std::cout << "[B] Vibrate Right Only\n";
	std::cout << "[X] Vibrate Both\n";
	std::cout << "[Y] Vibrate Neither\n";
	std::cout << "[BACK] Exit\n";

	while (true)
	{
		if (Player1->IsConnected())
		{
			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP)
			{
				Player1->Vibrate(65535, 0);
			}

			if (Player1->GetState().Gamepad.sThumbLX)
			{
				int tmp = Player1->GetState().Gamepad.sThumbLX;
				if (tmp > 0)
					Player1->Vibrate(0, tmp);
				else if (tmp < 0)
					Player1->Vibrate(tmp, 0);
				else
					Player1->Vibrate();
			}

			if (Player1->GetState().Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER)
			{
				Player1->Vibrate(65535, 65535);
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
			std::cout << "\n\tERROR! PLAYER 1 - XBOX 360 Controller Not Found!\n";
			std::cout << "Press Any Key To Exit.";
			std::cin.get();
			break;
		}
	}

	delete(Player1);

	return(0);
}