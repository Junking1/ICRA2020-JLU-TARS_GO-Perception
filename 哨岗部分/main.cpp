#include"functions_2.h"

using namespace watchdog;



int main()
{
	//std::cout << "1" << endl;
	//int a1,a2;
	
	const char * ip = "192.168.1.100";
	cam_initialize();
	//DWORD exitCode1;
	//DWORD exitCode2;
	thread camera_1(watchdog_1);
	thread camera_2(watchdog_2);
	//a1 = GetExitCodeThread(camera_1.native_handle(), &exitCode1);
	//a2 = GetExitCodeThread(camera_2.native_handle(), &exitCode2);
	send_param(ip);
	camera_1.join();
	camera_2.join();

	return 0;
}

