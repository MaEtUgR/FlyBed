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
#include "RemoteControl.h"       // RemoteControl Channels with PPM
#include "PID.h"        // PID Library (slim, self written)
#include "Servo.h"      // Motor PPM using any DigitalOut Pin

#define PPM_FREQU       495     // Hz Frequency of PPM Signal for ESCs (maximum <500Hz)
#define INTEGRAL_MAX    300     // maximal output offset that can result from integrating errors
#define ROLL            0       // Axes
#define PITCH           1
#define YAW             2

#define SQRT2           0.7071067811865

bool  debug = true;                     // shows if we want output for the computer
bool  level = false;                     // switches between self leveling and acro mode
float P_R = 2.6, I_R = 0.3, D_R = 0;      // PID values for the rate controller
float P_A = 1.9, I_A = 0.2, D_A = 0;        // PID values for the angle controller      P_A = 1.865, I_A = 1.765, D_A = 0
float PY = 2.3, IY = 0, DY = 0;         // PID values for Yaw
float Motor_speed[4] = {0,0,0,0};       // Mixed Motorspeeds, ready to send
bool  toRCCalibrate = false;

Timer LoopTimer;
float Times[10] = {0,0,0,0,0,0,0,0,0,0};
float control_frequency = 800;//PPM_FREQU;         // frequency for the main loop in Hz
int counter = 0;
int divider = 20;

LED				LEDs;
PC				pc(USBTX, USBRX, 115200);   // USB
//PC				pc(p9, p10, 115200);       // Bluetooth PIN: 1234
IMU_10DOF		IMU(p5, p6, p7, p19, p28, p27);
RemoteControl	RC; // no p19/p20 !
PID				Controller_Rate[] = {PID(P_R, I_R, D_R, INTEGRAL_MAX), PID(P_R, I_R, D_R, INTEGRAL_MAX), PID(PY, IY, DY, INTEGRAL_MAX)}; // 0:X:Roll 1:Y:Pitch 2:Z:Yaw
PID				Controller_Angle[] = {PID(P_A, I_A, D_A, INTEGRAL_MAX), PID(P_A, I_A, D_A, INTEGRAL_MAX), PID(0, 0, 0, INTEGRAL_MAX)};
Servo			ESC[] = {Servo(p21,PPM_FREQU), Servo(p22,PPM_FREQU), Servo(p23,PPM_FREQU), Servo(p24,PPM_FREQU)};   // use any DigitalOit Pin

extern "C" void mbed_reset();

void loop() {			// TODO: Times!!
	LoopTimer.start();				// a timer to monitor how long the steps take
	RC.run(IMU.angle[YAW]);			// remote control tasks (like arming!)
	IMU.readAngles();				// reading sensor data
	Times[1] = LoopTimer.read(); 	// 197us

	// Setting PID Values from auxiliary RC channels
	for(int i=0;i<3;i++)
		Controller_Angle[i].setPID(P_A,I_A,D_A);
	for(int i=0;i<2;i++)
		Controller_Rate[i].setPID(P_R,I_R,D_R); // give the new PID values to roll and pitch controller
	Controller_Rate[YAW].setPID(PY,IY,DY);
	Times[2] = LoopTimer.read(); // 7us

	Times[3] = LoopTimer.read(); // 6us

	// Controlling
	if (level) {
		for(int i=0;i<2;i++) { // LEVEL
			Controller_Angle[i].setIntegrate(RC.armed()); // only integrate in controller when armed, so the value is not totally odd from not flying
			if (counter % 16 == 0)
				Controller_Angle[i].compute(RC._angle[i], IMU.angle[i]); // give the controller the actual angles and get his advice to correct
			Controller_Rate[i].setIntegrate(RC.armed()); // only integrate in controller when armed, so the value is not totally odd from not flying
			Controller_Rate[i].compute(-Controller_Angle[i].Value, /*IMU.mpu2.data_gyro[i]*/IMU.mpu.Gyro[i]); // give the controller the actual gyro values and get his advice to correct
			//Controller_Rate[i].compute(-Controller_Angle[i].Value, (IMU.mpu2.data_gyro[i] + IMU.mpu.Gyro[i])/2 );
		}
	} else {
		for(int i=0;i<2;i++) { // ACRO
			Controller_Rate[i].setIntegrate(RC.armed()); // only integrate in controller when armed, so the value is not totally odd from not flying
			Controller_Rate[i].compute((RC[i]-500.0)*100.0/500.0, /*IMU.mpu2.data_gyro[i]*/IMU.mpu.Gyro[i]); // give the controller the actual gyro values and get his advice to correct
			//Controller_Rate[i].compute((RC[i].read()-500.0)*100.0/500.0, (IMU.mpu2.data_gyro[i] + IMU.mpu.Gyro[i])/2 );
		}
	}

	Controller_Rate[2].setIntegrate(RC.armed()); // only integrate in controller when armed, so the value is not totally odd from not flying
	if (RC[THROTTLE] > 20)
		Controller_Rate[2].compute(-(RC[RUDDER]-500.0)*100.0/500.0, IMU.mpu.Gyro[2]); // give the controller the actual gyro values and get his advice to correct
	else
		Controller_Rate[2].compute(0, IMU.mpu.Gyro[2]); // give the controller the actual gyro values and get his advice to correct

	float throttle = 100 + (RC[THROTTLE] * 500 / 1000);	// power limitation to 60% TODO: better throttle control, so we don't need this
	Times[4] = LoopTimer.read(); // 53us

	// Mixing
	Motor_speed[0] = throttle   +SQRT2*Controller_Rate[ROLL].Value  -SQRT2*Controller_Rate[PITCH].Value		-Controller_Rate[YAW].Value;  // X Configuration
	Motor_speed[1] = throttle   -SQRT2*Controller_Rate[ROLL].Value  -SQRT2*Controller_Rate[PITCH].Value		+Controller_Rate[YAW].Value;
	Motor_speed[2] = throttle   -SQRT2*Controller_Rate[ROLL].Value  +SQRT2*Controller_Rate[PITCH].Value		-Controller_Rate[YAW].Value;
	Motor_speed[3] = throttle   +SQRT2*Controller_Rate[ROLL].Value  +SQRT2*Controller_Rate[PITCH].Value		+Controller_Rate[YAW].Value;
	Times[5] = LoopTimer.read(); // 17us

	if(0) { // for SECURITY!
		debug = false;
		// PITCH
		//ESC[0] = (int)Motor_speed[0]>50 ? (int)Motor_speed[0] : 50;
		//ESC[2] = (int)Motor_speed[2]>50 ? (int)Motor_speed[2] : 50;
		// ROLL
		//ESC[1] = (int)Motor_speed[1]>50 ? (int)Motor_speed[1] : 50;
		//ESC[3] = (int)Motor_speed[3]>50 ? (int)Motor_speed[3] : 50;
		for(int i=0;i<4;i++)   // Set new motorspeeds
			ESC[i] = (int)Motor_speed[i]>100 ? (int)Motor_speed[i] : 100;
	} else {
		for(int i=0;i<4;i++) // for security reason, set every motor to zero speed
			ESC[i] = 0;
		debug = true;
	}
	Times[6] = LoopTimer.read(); // 6us

	LEDs.rollnext();

	counter++;

	Times[7] = LoopTimer.read(); // 7us TOTAL 297us
	while(LoopTimer.read() < 1/control_frequency); // Kill the rest of the time TODO: make a better solution so we can do misc things with these cycles
	Times[8] = LoopTimer.read();
	LoopTimer.stop();
	LoopTimer.reset();

	if(toRCCalibrate) {
		toRCCalibrate = false;
		LEDs.shownumber(0xF);
		RC.calibrate(10);
		LEDs.rollreset();
	}

	if (debug) {
		pc.printf("$STATE,%d,%d,%.f,%.3f,%.3f\r\n", RC.armed(), level, control_frequency, IMU.dt*1e3, IMU.dt_sensors*1e6);
		pc.printf("$RC, %d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\r\n", RC.present(), RC[AILERON], RC[ELEVATOR], RC[RUDDER], RC[THROTTLE], RC[CHANNEL6], RC[CHANNEL7], RC[CHANNEL8]);
		//pc.printf("$GYRO,%.3f,%.3f,%.3f\r\n", IMU.mpu.Gyro[ROLL], IMU.mpu.Gyro[PITCH], IMU.mpu.Gyro[YAW]);
		//pc.printf("$GYRO2,%.3f,%.3f,%.3f\r\n", IMU.mpu2.data_gyro[ROLL], IMU.mpu2.data_gyro[PITCH], IMU.mpu2.data_gyro[YAW]);
		//pc.printf("$ACC,%.3f,%.3f,%.3f\r\n", IMU.mpu.Acc[ROLL], IMU.mpu.Acc[PITCH], IMU.mpu.Acc[YAW]);
		//pc.printf("$ANG,%.3f,%.3f,%.3f\r\n", IMU.angle[ROLL], IMU.angle[PITCH], IMU.angle[YAW]);
		pc.printf("$RCANG,%.3f,%.3f,%.3f\r\n", RC._angle[ROLL], RC._angle[PITCH], RC._angle[YAW]);
		//pc.printf("$CONTR,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", Controller_Rate[ROLL].Value, Controller_Rate[PITCH].Value, Controller_Rate[YAW].Value, P_R, I_R, D_R, PY);
		//pc.printf("$CONTA,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", Controller_Angle[ROLL].Value, Controller_Angle[PITCH].Value, Controller_Angle[YAW].Value, P_A, I_A, D_A);
		//pc.printf("$MOT,%d,%d,%d,%d\r\n", (int)Motor_speed[0], (int)Motor_speed[1], (int)Motor_speed[2], (int)Motor_speed[3]);
		/*pc.printf("$TIMES");
        for(int i = 1; i < 10; i++)
            pc.printf(",%.3f", (Times[i]-Times[i-1])*1e6);
        pc.printf("\r\n");*/
		wait(0.1);
	}
}

void executer() {
	char command = pc.getc();
	switch(command) {
		case 'X': mbed_reset(); break;
		case '-': debug = !debug; break;
		case ':': RC._armed = true; break;
		case ' ': RC._armed = false; break;
		case 'q': level = true; break;
		case 'a': level = false; break;
		case 'w': P_R += 0.1; break;
		case 's': P_R -= 0.1; break;
		case 'e': I_R += 0.1; break;
		case 'd': I_R -= 0.1; break;
		case 'x': D_R += 0.001; break;
		case 'c': D_R -= 0.001; break;
		case 'r': P_A += 0.1; break;
		case 'f': P_A -= 0.1; break;
		case 't': I_A += 0.1; break;
		case 'g': I_A -= 0.1; break;
		case 'z': PY += 0.1; break;
		case 'h': PY -= 0.1; break;
		case 'o': control_frequency += 100; break;
		case 'l': control_frequency -= 100; break;
		case '?': toRCCalibrate = true; break;
		case '\'': RC.enableStickCentering(); break;
	}
	pc.putc(command);
	LEDs.tilt(2);
}

int main() {
	pc.attach(&executer);
	while(1) {
		loop();
	}
}
