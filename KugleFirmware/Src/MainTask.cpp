#include "main.h"
#include "USBCDC.h"
#include "cmsis_os.h"

#include "UART.h"
#include "SPI.h"
#include "I2C.h"
#include "PWM.h"
#include "Encoder.h"
#include "Timer.h"
#include "IO.h"
#include "QuadratureKnob.h"

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

	USBCDC * usb = new USBCDC(3);

	/*UART * uart = new UART(UART::PORT_UART3, 115200, 100);
	SPI * spi = new SPI(SPI::PORT_SPI6, 500000);
	I2C * i2c = new I2C(I2C::PORT_I2C1, 0x68);

	PWM * pwm = new PWM(PWM::TIMER1, PWM::CH1, 1, 50000);
	Encoder * encoder = new Encoder(Encoder::TIMER2);
	Timer * timer = new Timer(Timer::TIMER6, 10000);
	Timer * timer2 = new Timer(Timer::TIMER7, 10000);
	IO * pin = new IO(GPIOA, GPIO_PIN_4, true);*/

	TestBench_Init();

	/* Infinite loop */
	for(;;)
	{
		osDelay(1000);
	}
}
