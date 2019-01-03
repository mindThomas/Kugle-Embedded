#include "MainTask.h"
#include "cmsis_os.h"

/* Include Periphiral drivers */
#include "ADC.h"
#include "Encoder.h"
#include "I2C.h"
#include "InputCapture.h"
#include "IO.h"
#include "PWM.h"
#include "SMBus.h"
#include "SPI.h"
#include "Timer.h"
#include "UART.h"
#include "USBCDC.h"
#include "Watchdog.h"

/* Include Device drivers */
#include "Battery.h"
#include "ESCON.h"
#include "IMU.h"
#include "LCD.h"
#include "LSPC.hpp"
#include "MPU9250.h"
#include "MTI200.h"
#include "QuadratureKnob.h"

/* Include Module libraries */
#include "LQR.h"
#include "SlidingMode.h"
#include "Debug.h"
#include "COMEKF.h"
#include "MadgwickAHRS.h"
#include "QEKF.h"
#include "VelocityEKF.h"
#include "Parameters.h"
#include "PowerManagement.h"
#include "FrontPanel.h"
#include "Joystick.h"

/* Include Application-layer libraries */
#include "AttitudeController.h"
#include "Communication.h"
#include "HealthMonitor.h"
#include "PathFollowingController.h"

/* Miscellaneous includes */
#include "MATLABCoderInit.h"
#include <stdlib.h>
#include <vector>

void handl(const std::vector<uint8_t>& payload);

void MainTask(void const * argument)
{
	/* Use this task to:
	 * - Create objects for each module
	 *     (OBS! It is very important that objects created with "new"
	 *      only happens within a thread due to the usage of the FreeRTOS managed heap)
	 * - Link any modules together if necessary
	 * - Create message exchange queues and/or semaphore
	 * - (Create) and start threads related to modules
	 *
	 * Basically anything related to starting the system should happen in this thread and NOT in the main() function !!!
	 */

	/* Initialize global parameters and MATLAB coder globals */
	Parameters& params = Parameters::Get();
	MATLABCoder_initialize();

	/* Initialize communication */
	USBCDC * usb = new USBCDC(3);
	LSPC * lspcUSB = new LSPC(usb, osPriorityNormal, osPriorityNormal); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!

	/* Initialize and configure IMU */
	SPI * spi = new SPI(SPI::PORT_SPI6, MPU9250_Bus::SPI_LOW_FREQUENCY, GPIOG, GPIO_PIN_8);
	MPU9250 * imu = new MPU9250(spi);
	imu->Configure(MPU9250::ACCEL_RANGE_2G, MPU9250::GYRO_RANGE_250DPS);
	if (params.estimator.EnableSensorLPFfilters) {
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_92HZ, MPU9250::DLPF_BANDWIDTH_92HZ); // sensor bandwidths should be lower than half the sample rate to avoid aliasing problems
		imu->setFilt(MPU9250::DLPF_BANDWIDTH_92HZ, MPU9250::DLPF_BANDWIDTH_250HZ); // however best results are seen with 250 Hz gyro bandwidth, even though we sample at 200 Hz
	} else {
		imu->setFilt(MPU9250::DLPF_BANDWIDTH_OFF, MPU9250::DLPF_BANDWIDTH_OFF);
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_184HZ, MPU9250::DLPF_BANDWIDTH_184HZ);
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_41HZ, MPU9250::DLPF_BANDWIDTH_41HZ);
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_20HZ, MPU9250::DLPF_BANDWIDTH_20HZ);
		//imu->setFilt(MPU9250::DLPF_BANDWIDTH_OFF, MPU9250::DLPF_BANDWIDTH_184HZ);
	}
	imu->ConfigureInterrupt(GPIOE, GPIO_PIN_3);

	/* Initialize microseconds timer */
	Timer * microsTimer = new Timer(Timer::TIMER6, 1000000);

	/* Initialize motors */
	ESCON * motor1 = new ESCON(1);
	ESCON * motor2 = new ESCON(2);
	ESCON * motor3 = new ESCON(3);

	/******* APPLICATION LAYERS *******/
	AttitudeController * attitudeController = new AttitudeController(params, *imu, *motor1, *motor2, *motor3, *lspcUSB, *microsTimer);
	if (!attitudeController) ERROR("Could not initialize attitude controller");

	while (1)
	{
		vTaskSuspend(NULL); // suspend this task
	}
}

void handl(const std::vector<uint8_t>& payload)
{
	uint8_t * buffer = const_cast<uint8_t *>(payload.data());
	uint32_t length = payload.size();
}
