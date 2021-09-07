#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <unistd.h>
#include <fstream>
#include <time.h>

using namespace std;
int main(int, char**) {
	ofstream output;
	srand(time(NULL));
	output.open("/sys/module/houston/parameters/fps_boost");
	while (1) {
		if (!output.is_open())
			return 0;
		output.seekp(0);
		output << 1 << "," << 5 << "," << 21;
		output.flush();

		int nanosec = 10000 + rand() % 20000;
		cout << "sleep " << nanosec << endl;
		usleep(nanosec);
	}
	output.close();
	return 0;
}
