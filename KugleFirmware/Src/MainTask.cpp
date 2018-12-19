#include <LSPC.hpp>
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
#include "LSPC.hpp"

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

	/*USBCDC * usb = new USBCDC(3);
	lspc::Socket<USBCDC> * mySocket = new lspc::Socket<USBCDC>(usb, osPriorityNormal); // very important to use "new", otherwise the object gets placed on the stack which does not have enough memory!
	mySocket->registerCallback(1, handl);*/

	TestBench_Init();

	while (1)
	{
		osDelay(100);
	}
}

void handl(const std::vector<uint8_t>& payload)
{
	uint8_t * buffer = const_cast<uint8_t *>(payload.data());
	uint32_t length = payload.size();
}
