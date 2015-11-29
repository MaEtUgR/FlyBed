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

PC          pc(USBTX, USBRX, 115200);   // USB

extern "C" void mbed_reset();

LocalFileSystem local("local");

int main() {

	pc.cls();
	pc.printf("\r\nup and running!\r\n");
    Timer FileTimer;
    char mytext[100] = "Hello!\n";

    FILE *fp = fopen("/local/out.txt", "w");
    FileTimer.start();
    fwrite(mytext, 1, 7, fp);	// writing bytes
    fprintf(fp, "Hello!");		// writing a formated string
    int time = FileTimer.read_us();
    fclose(fp);

    pc.printf("writing the file took: %dus", time);

    while(1) {
    	char c = pc.getc();
    	if(c == 'X')
    		mbed_reset();
    }
}
