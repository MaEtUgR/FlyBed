/*   X- Configuration
        m2   m3                      --           >
          \ /                      /    \       /
          / \                            V     |
        m1   m0                                 \
                                    ROLL       PITCH */
#include "mbed.h"
#include "LED.h"        // LEDs framework for blinking ;)
#include "PC.h"         // Serial Port via USB by Roland Elmiger for debugging with Terminal (driver needed: https://mbed.org/media/downloads/drivers/mbedWinSerial_16466.exe)

#include "IMU_10DOF.h"  // Complete IMU class for 10DOF-Board (L3G4200D, ADXL345, HMC5883, BMP085)
#include "RC_Channel.h" // RemoteControl Channels with PPM
#include "PID.h"        // PID Library (slim, self written)
#include "Servo.h"      // Motor PPM using any DigitalOut Pin

#include "ParameterSystem.h"

PC          pc(USBTX, USBRX, 115200);   // USB
ParameterSystem params;

extern "C" void mbed_reset();

void wait_for_reset() {
	pc.printf("end");
	while(1) {
		char c = pc.getc();
		if(c == 'X')
			mbed_reset();
	}
}

int main() {
	pc.printf("\r\nup and running!\r\n");
	//params.writeParametersToFile();
	params.readParametersFromFile();

	params.setParameter(1, 5.555);

	for(int i = 0; i < 3; i++)
		printf("%f\r\n", params[i]);

	wait_for_reset();


	// old test

	LocalFileSystem local("local");

	pc.cls();

    Timer FileTimer;
    char mytext[100] = "Hello!\n";
    int testint = -1;

    FILE *fp = fopen("/local/out.txt", "w");
    pc.printf("%d",fp);
    wait(1);
    FileTimer.start();
    fwrite(mytext, 1, 7, fp);	// writing bytes
    fprintf(fp, "Hello!\r\n");		// writing a formated string
    fwrite(&testint, sizeof(testint), 1, fp);
    int time = FileTimer.read_us();
    fclose(fp);

    pc.printf("writing the file took: %dus", time);


}
