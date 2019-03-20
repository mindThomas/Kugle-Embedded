/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "Priorities.h"
#include "stm32h7xx_hal.h"

#include <string>
#include <stdlib.h>

#include "UART.h"
#include "SPI.h"
#include "I2C.h"
#include "PWM.h"
#include "Encoder.h"
#include "Timer.h"
#include "IO.h"
#include "QuadratureKnob.h"
#include "ADC.h"
#include "MPU9250.h"
#include "MTI200.h"
#include "ESCON.h"
#include "PowerManagement.h"
#include "Parameters.h"
#include "FrontPanel.h"
#include "VelocityEKF.h"


void TestBench(void * pvParameters);
TaskHandle_t testBenchTaskHandle;
UART * uart;
I2C * i2c;
PWM * pwm;
Encoder * encoder;
Timer * timer;
Timer * timer2;
IO * pin;
QuadratureKnob * knob;
ADC * adc;
MTI200 * mti200;

void TestBench_Init()
{
	/* Create Test bench thread */
	xTaskCreate(TestBench, "testBenchTask", 256, (void*) NULL, TEST_BENCH_PRIORITY, &testBenchTaskHandle);
}

int32_t encoderValue;
uint32_t timerValue1, timerValue2, timerValue3;
bool oldState = false;
uint8_t value;

bool level1, level2, level3;

float ax, ay, az, gx, gy, gz, mx, my, mz;

float angle = 0;

#if 1
void TestBench(void * pvParameters)
{

		/* Initialize communication */
		USBCDC * usb = new USBCDC(USBCDC_TRANSMITTER_PRIORITY);
		LSPC * lspcUSB = new LSPC(usb, LSPC_RECEIVER_PRIORITY, LSPC_TRANSMITTER_PRIORITY); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!
		Debug * dbg = new Debug(lspcUSB); // pair debug module with configured LSPC module to enable "Debug::print" functionality
		//params.AttachLSPC(lspcUSB); // attach USB object to allow modification of parameters over USB

		/* Initialize microseconds timer */
		Timer * microsTimer = new Timer(Timer::TIMER6, 1000000); // create a 1 MHz counting timer used for micros() timing

		/* Initialize power management */
		IO * enable19V = new IO(GPIOE, GPIO_PIN_4); // configure as output
		IO * enable5V = new IO(GPIOC, GPIO_PIN_3); // configure as output
		PWM * powerLED = new PWM(PWM::TIMER17, PWM::CH1, POWER_LED_PWM_FREQUENCY, POWER_LED_PWM_RANGE);
		PowerManagement * pm = new PowerManagement(*enable19V, *enable5V, *powerLED, POWER_MANAGEMENT_PRIORITY, *lspcUSB, *microsTimer);
		pm->SetPowerMode(pm->PowerMode_t::POWERMODE_ALL_ON);

/*
		SMBUS_HandleTypeDef Device;

		const uint8_t SMBUS_MASTER_ADDRESS = 0x08; // smbus.h defined to 38

		Device.Init.OwnAddress1 = SMBUS_MASTER_ADDRESS;
		Device.Init.AnalogFilter = SMBUS_ANALOGFILTER_ENABLE;
		Device.Init.AddressingMode = SMBUS_ADDRESSINGMODE_7BIT;
		Device.Init.DualAddressMode = SMBUS_DUALADDRESS_DISABLE;
		Device.Init.OwnAddress2 = 0;
		Device.Init.OwnAddress2Masks = SMBUS_OA2_NOMASK;
		Device.Init.GeneralCallMode = SMBUS_GENERALCALL_DISABLE;
		Device.Init.NoStretchMode = SMBUS_NOSTRETCH_DISABLE;
		Device.Init.PacketErrorCheckMode = SMBUS_PEC_ENABLE;//SMBUS_PEC_DISABLE;
		Device.Init.PeripheralMode = SMBUS_PERIPHERAL_MODE_SMBUS_HOST;
		Device.Init.SMBusTimeout = 0x000084C4;

		uint32_t ErrCode = HAL_SMBUS_Init(&Device);
		STACK_SMBUS_GetBuffer(&context);
		STACK_SMBUS_HostCommand(&context,&cmd,addr,WRITE);
*/

		// poll stack context from when ready stat

		/* Send CPU load every second */
			char * pcWriteBuffer = (char *)pvPortMalloc(1024);
			while (1)
			{
				vTaskGetRunTimeStats(pcWriteBuffer);
				char * endPtr = &pcWriteBuffer[strlen(pcWriteBuffer)];
				*endPtr++ = '\n'; *endPtr++ = '\n'; *endPtr++ = 0;

				// Split into multiple packages and send
				uint16_t txIdx = 0;
				uint16_t remainingLength = strlen(pcWriteBuffer);
				uint16_t txLength;

				while (remainingLength > 0) {
					txLength = remainingLength;
					if (txLength > LSPC_MAXIMUM_PACKAGE_LENGTH) {
						txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
						while (pcWriteBuffer[txIdx+txLength] != '\n' && txLength > 0) txLength--; // find and include line-break (if possible)
						if (txLength == 0) txLength = LSPC_MAXIMUM_PACKAGE_LENGTH;
						else txLength++;
					}
					lspcUSB->TransmitAsync(lspc::MessageTypesToPC::CPUload, (uint8_t *)&pcWriteBuffer[txIdx], txLength);

					txIdx += txLength;
					remainingLength -= txLength;
				}
				osDelay(1000);
			}
}
#endif


#if 0
void TestBench(void * pvParameters)
{
	PowerManagement * pm = new PowerManagement(osPriorityNormal);
	ESCON * motor1 = new ESCON(1);
	ESCON * motor2 = new ESCON(2);
	ESCON * motor3 = new ESCON(3);

	pm->Enable(true, true);

	motor1->Enable();
	motor2->Enable();
	motor3->Enable();

	motor1->SetTorque(0.05);
	motor2->SetTorque(0.05);
	motor3->SetTorque(0.05);

	osDelay(1000);

	while (1)
	{
		/*angle = motor1->GetAngle();
		encoderValue = motor1->GetEncoderRaw();*/

		motor1->SetTorque(-0.05);
		motor2->SetTorque(-0.05);
		motor3->SetTorque(-0.05);
		osDelay(1000);
		motor1->SetTorque(0.0);
		motor2->SetTorque(0.0);
		motor3->SetTorque(0.0);
		osDelay(1000);
		motor1->SetTorque(0.05);
		motor2->SetTorque(0.05);
		motor3->SetTorque(0.05);
		osDelay(1000);
		motor1->SetTorque(0.0);
		motor2->SetTorque(0.0);
		motor3->SetTorque(0.0);
		osDelay(1000);
	}
}
#endif

#if 0
void TestBench(void * pvParameters)
{
	SPI * spi = new SPI(SPI::PORT_SPI6, MPU9250_Bus::SPI_LOW_FREQUENCY, GPIOG, GPIO_PIN_8);
	MPU9250 * imu = new MPU9250(spi);

	imu->Configure(MPU9250::ACCEL_RANGE_2G, MPU9250::GYRO_RANGE_250DPS);
	imu->setFilt(MPU9250::DLPF_BANDWIDTH_250HZ, MPU9250::DLPF_BANDWIDTH_184HZ, 8);
	imu->ConfigureInterrupt(GPIOE, GPIO_PIN_3);

	while (1) {
		imu->WaitForNewData();
		imu->WaitForNewData();
		imu->WaitForNewData();
		imu->WaitForNewData();
	    imu->getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	    osDelay(100);
	}
}
#endif

#if 0
uint8_t count = 0;
void MessageCallback(const std::vector<uint8_t>& payload)
{
	uint8_t * buffer = const_cast<uint8_t *>(payload.data());
	uint32_t length = payload.size();
	count++;
}

void TestBench(void * pvParameters)
{
	USBCDC * usb = new USBCDC(3);
	LSPC * lspcUSB = new LSPC(usb, 11, 10); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!
	lspcUSB->registerCallback(lspc::MessageTypesIn::Test, MessageCallback);

	int step = 0;
	int count = 0;
	while (1)
	{
		const uint8_t package[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, count};
		lspcUSB->TransmitAsync(lspc::MessageTypesOut::Test, package, sizeof(package));
		osDelay(5);

		if (count++ > 200) {
			count = 0;
			//Debug::print("Test\n");
			Debug::printf("Test: %d\n", step++);
		}
	}
}
#endif

#if 0
void UART_Callback(void * param, uint8_t * buffer, uint32_t bufLen)
{
	uart->Write(buffer, bufLen);
}

void TestBench(void * pvParameters)
{
	uart = new UART(UART::PORT_UART3, 115200, 100);
	spi = new SPI(SPI::PORT_SPI6, 500000);
	i2c = new I2C(I2C::PORT_I2C1, 0x68);
	imu = new MPU9250<I2C,I2C::port_t>(i2c);

	pwm = new PWM(PWM::TIMER1, PWM::CH1, 1, 50000);
	encoder = new Encoder(Encoder::TIMER2);
	timer = new Timer(Timer::TIMER6, 10000);
	timer2 = new Timer(Timer::TIMER7, 10000);
	pin = new IO(GPIOA, GPIO_PIN_4, true);
	//knob = new QuadratureKnob(GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1);
	adc = new ADC(ADC::ADC_1, ADC_CHANNEL_8);

	pwm->Set(5000);

	uint8_t buffer[10];

	std::string testString("This is a test");
	uart->RegisterRXcallback(UART_Callback, 10);

	SemaphoreHandle_t semaphore;
	//semaphore = xSemaphoreCreateCounting(100, 0);
	semaphore = xSemaphoreCreateBinary();
	vQueueAddToRegistry(semaphore, "Test semaphore");
	//timer->RegisterInterrupt(10, semaphore);
	//pin->RegisterInterrupt(IO::TRIGGER_BOTH, semaphore);

	uint16_t adcValue;

	while (1) {
		/*uart->TransmitBlocking(reinterpret_cast<uint8_t *>(const_cast<char *>(testString.c_str())), testString.length());
		osDelay(500);
		spi->Write(0x19, 0xAB);
		osDelay(500);
		spi->Read(0x73 | 0x80, buffer, 4);
		//spi->Read(0x75 | 0x80);
		osDelay(500);*/
		/*i2c->Write(0x19, 0xAB);
		pwm->Set(250);
		osDelay(500);
		i2c->Read(0x73, buffer, 4);
		pwm->Set(750);
		osDelay(500);*/
		encoderValue = encoder->Get();
		adcValue = adc->Read();


		/*timer->Reset();
		timerValue1 = timer2->Get();
		level1 = pin->Read();
		xSemaphoreTake( semaphore, ( TickType_t ) portMAX_DELAY );
		timerValue2 = timer2->Get();
		level2 = pin->Read();
		//timerValue1 = xTaskGetTickCount();
		//timerValue1 = timer2->Get();
		//osDelay(1000);
		xSemaphoreTake( semaphore, ( TickType_t ) portMAX_DELAY );
		timerValue3 = timer2->Get();
		level3 = pin->Read();
		//timerValue2 = xTaskGetTickCount();
		//timerValue2 = timer2->Get();*/

		//osDelay(100);

		/*if (pin->Read() != oldState) {
			oldState = pin->Read();

			if (oldState == false)
				timerValue1 = (uint32_t)TIM1->CNT;

			osDelay(1);
		}*/

		osDelay(100);
	}
}
#endif

#if 0
void TestBench(void * pvParameters)
{
	Parameters& params = Parameters::Get();
	HealthMonitor * hm = new HealthMonitor();
	VelocityEKF * vEKF = new VelocityEKF(params);

	while (1) {
		params.test.var2 += 2;
		osDelay(100);
	}
}
#endif

#if 0
void UART_Callback(void * param, uint8_t * buffer, uint32_t bufLen)
{
	Debug::printf("Received %d bytes from UART3\n", bufLen);
	uart->Write(buffer, bufLen);
}

void TestBench(void * pvParameters)
{
	uart = new UART(UART::PORT_UART3, 460800, 100);
	uart->RegisterRXcallback(UART_Callback);

	std::string testString("This is a test\n");
	while (1) {
		uart->Write(reinterpret_cast<uint8_t *>(const_cast<char *>(testString.c_str())), testString.length());
		osDelay(1000);
	}
}
#endif

#if 0
void TestBench(void * pvParameters)
{
	osDelay(4000);
	uart = new UART(UART::PORT_UART3, 460800, 100);
	mti200 = new MTI200(uart);

	osDelay(5000);

	IMU::Measurement_t meas;
	while (1) {
		mti200->Get(meas);
		MTI200::LastMeasurement_t meas2 = mti200->GetLastMeasurement();
		Debug::printf("Time = [%.2f  (dt = %.2f ms)\n", meas2.Time, 1000*meas2.dt);
		Debug::printf("Accelerometer = [%.3f, %.3f, %.3f]\n", meas.Accelerometer[0], meas.Accelerometer[1], meas.Accelerometer[2]);
		Debug::printf("Gyroscope = [%.3f, %.3f, %.3f]\n", meas.Gyroscope[0], meas.Gyroscope[1], meas.Gyroscope[2]);
		Debug::printf("Magnetometer = [%.3f, %.3f, %.3f]\n\n", meas.Magnetometer[0], meas.Magnetometer[1], meas.Magnetometer[2]);
		Debug::printf("q = [%.3f, %.3f, %.3f, %.3f]\n\n", meas2.Quaternion[0], meas2.Quaternion[1], meas2.Quaternion[2], meas2.Quaternion[3]);
		Debug::printf("dq = [%.3f, %.3f, %.3f, %.3f]\n\n", meas2.QuaternionDerivative[0], meas2.QuaternionDerivative[1], meas2.QuaternionDerivative[2], meas2.QuaternionDerivative[3]);
		osDelay(100);
	}
}
#endif

