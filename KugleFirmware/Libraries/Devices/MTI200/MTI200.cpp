/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details. 
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#include "MTI200.h"
#include "UART.h"
#include "Debug.h"
#include "cmsis_os.h"

MTI200::MTI200(UART * uart) : _uart(uart)
{
	_interruptSemaphore = xSemaphoreCreateBinary();
	if (_interruptSemaphore == NULL) {
		ERROR("Could not create MTI200 interrupt semaphore");
		return;
	}
	vQueueAddToRegistry(_interruptSemaphore, "MTI200 Interrupt");

	_resourceSemaphore = xSemaphoreCreateBinary();
	if (_resourceSemaphore == NULL) {
		ERROR("Could not create MTI200 resource semaphore");
		return;
	}
	vQueueAddToRegistry(_resourceSemaphore, "MTI200 Resource");
	xSemaphoreGive( _resourceSemaphore ); // give the semaphore the first time

	_messageQueue = xQueueCreate( MESSAGE_QUEUE_LENGTH, sizeof(XbusMessage *) );
	if (_messageQueue == NULL) {
		ERROR("Could not create MTI200 message queue");
		return;
	}
	vQueueAddToRegistry(_messageQueue, "MTI200 Messages");

	_responseQueue = xQueueCreate( RESPONSE_QUEUE_LENGTH, sizeof(XbusMessage *) );
	if (_responseQueue == NULL) {
		ERROR("Could not create MTI200 response queue");
		return;
	}
	vQueueAddToRegistry(_responseQueue, "MTI200 Response");

	XbusParserCallback xbusCallback = {};
    xbusCallback.allocateBuffer = &allocateMessageData;
    xbusCallback.deallocateBuffer = &deallocateMessageData;
    xbusCallback.handleMessage = &mtMessageHandler;
    xbusCallback.parameter = (void *)this;

    _xbusParser = XbusParser_create(&xbusCallback);

	if (_uart) {
		_uart->RegisterRXcallback(&UART_Callback, (void *)this);
	}

	osDelay(500);
	Configure();
}

MTI200::~MTI200()
{
	if (_interruptSemaphore) {
		vQueueUnregisterQueue(_interruptSemaphore);
		vSemaphoreDelete(_interruptSemaphore);
	}
	if (_resourceSemaphore) {
		vQueueUnregisterQueue(_resourceSemaphore);
		vSemaphoreDelete(_resourceSemaphore);
	}
	if (_messageQueue) {
		vQueueUnregisterQueue(_messageQueue);
		vQueueDelete(_messageQueue);
	}
	if (_responseQueue) {
		vQueueUnregisterQueue(_responseQueue);
		vQueueDelete(_responseQueue);
	}

	if (_uart) {
		_uart->DeregisterCallback();
	}
}

void MTI200::Configure()
{
	sendCommand(XMID_GotoConfig);
	configureMotionTracker();
	sendCommand(XMID_GotoMeasurement);
}

uint32_t MTI200::WaitForNewData(uint32_t xTicksToWait) // blocking call
{
	if (!_interruptSemaphore)
		return pdFALSE;

	return xSemaphoreTake( _interruptSemaphore, ( TickType_t ) xTicksToWait );
}

void MTI200::Get(Measurement_t& measurement)
{
	memcpy(measurement.Accelerometer, LastMeasurement.Accelerometer, sizeof(LastMeasurement.Accelerometer));
	memcpy(measurement.Gyroscope, LastMeasurement.Gyroscope, sizeof(LastMeasurement.Gyroscope));
	memcpy(measurement.Magnetometer, LastMeasurement.Magnetometer, sizeof(LastMeasurement.Magnetometer));
}

void MTI200::UART_Callback(void * param, uint8_t * buffer, uint32_t bufLen)
{
	MTI200 * mti200 = (MTI200 *)param;
	if (!mti200) return;

	XbusParser_parseBuffer(mti200->_xbusParser, buffer, bufLen);
}

/*!
 * \brief Allocate message data buffer from the message data pool.
 */
void* MTI200::allocateMessageData(size_t bufSize)
{
	return (void *)pvPortMalloc(bufSize);
}

/*!
 * \brief Deallocate message data previously allocated from the message
 * data pool.
 */
void MTI200::deallocateMessageData(void const* buffer)
{
	vPortFree((void *)buffer);
}

/*!
 * \brief XbusParser callback function to handle received messages.
 * \param message Pointer to the last received message.
 *
 * In this example received messages are copied into one of two message
 * queues for later handling by the main thread. Data messages are put
 * in one queue, while all other responses are placed in the second queue.
 * This is done so that data and other messages can be handled separately
 * by the application code.
 */
void MTI200::mtMessageHandler(void * param, struct XbusMessage const* message)
{
	MTI200 * mti200 = (MTI200 *)param;
	if (!mti200) return;

	if (message->mid == XMID_MtData2) { // Message queue disabled - parse data immediately
		mti200->parseMTData2Message(message);
		deallocateMessageData(message->data);
	} else { // other type of message, put into response queue
		XbusMessage * msgPtr = (XbusMessage *)pvPortMalloc(sizeof(XbusMessage));
	    if (msgPtr) {
	    	memcpy(msgPtr, message, sizeof(XbusMessage));
	    	if (xQueueSend(mti200->_responseQueue, (void *)&msgPtr, (TickType_t) 0) != pdPASS ) {
				deallocateMessageData(message->data); // failed putting message into queue - deallocate message data memory
			}
		} else {
			deallocateMessageData(message->data); // could not allocate memory for received Xbus message
		}
	}

#if 0 // USE MESSAGE QUEUE BUFFER
	XbusMessage * msgPtr = (XbusMessage *)pvPortMalloc(sizeof(XbusMessage));
    if (msgPtr) {
    	memcpy(msgPtr, message, sizeof(XbusMessage));
		if (message->mid == XMID_MtData2)
		{
			if (xQueueSend(mti200->_messageQueue, (void *)&msgPtr, (TickType_t) 0) != pdPASS ) {
				deallocateMessageData(message->data); // failed putting message into queue - deallocate message data memory
			}
		}
		else
		{
			if (xQueueSend(mti200->_responseQueue, (void *)&msgPtr, (TickType_t) 0) != pdPASS ) {
				deallocateMessageData(message->data); // failed putting message into queue - deallocate message data memory
			}
		}
    } else {
    	deallocateMessageData(message->data); // could not allocate memory for received Xbus message
    }
#endif
}

/*!
 * \brief Send a message to the MT
 *
 * This function formats the message data and writes this to the MT serial
 * port. It does not wait for any response.
 */
void MTI200::sendMessage(XbusMessage const* m)
{
    uint8_t buf[64];
    size_t rawLength = XbusMessage_format(buf, m, XLLF_Uart);
    if (_uart) {
    	_uart->Write(buf, rawLength);
    }
}

/*!
 * \brief Send a message to the MT and wait for a response.
 * \returns Response message from the MT, or NULL is no response received
 * within 500ms.
 *
 * Blocking behaviour is implemented by waiting for a response to be written
 * to the response queue by the XbusParser.
 */
XbusMessage const * MTI200::doTransaction(XbusMessage const* m)
{
	xSemaphoreTake( _resourceSemaphore, ( TickType_t ) portMAX_DELAY ); // take hardware resource

    sendMessage(m);

	// Wait for the transmission to finish
    XbusMessage * msgPtr = NULL;
    xQueueReceive( _responseQueue, &msgPtr, ( TickType_t ) 500 ); // check USB connection every 100 ms

    xSemaphoreGive( _resourceSemaphore ); // give hardware resource back

    return msgPtr;
}

/*!
 * \brief RAII object to manage message memory deallocation.
 *
 * Will automatically free the memory used by an XbusMessage when going out
 * of scope.
 */
class XbusMessageMemoryManager
{
    public:
        XbusMessageMemoryManager(XbusMessage const* message)
            : m_message(message)
        {
        }

        ~XbusMessageMemoryManager()
        {
            if (m_message)
            {
                if (m_message->data)
                	vPortFree((void *)m_message->data);
                vPortFree((void *)m_message);
            }
        }

    private:
        XbusMessage const* m_message;
};

/*!
 * \brief Dump information from a message to the PC serial port.
 */
void MTI200::dumpResponse(XbusMessage const* response)
{
    switch (response->mid)
    {
        case XMID_GotoConfigAck:
        	Debug::print("Device went to config mode.\r\n");
            break;

        case XMID_Error:
        	Debug::print("Device error!\r\n");
            break;

        default:
            Debug::printf("Received response MID=%X, length=%d\r\n", response->mid, response->length);
            break;
    }
}

/*!
 * \brief Send a command to the MT and wait for a response.
 * \param cmdId The XsMessageId of the command to send.
 *
 * Commands are simple messages without and payload data.
 */
bool MTI200::sendCommand(XsMessageId cmdId)
{
    XbusMessage m = {cmdId};
    XbusMessage const * response = doTransaction(&m);
    XbusMessageMemoryManager janitor(response);

    if (response)
    {
        dumpResponse(response);
        return true;
    }
    else
    {
    	Debug::print("Timeout waiting for response.\r\n");
    	return false;
    }
}

/*!
 * \brief Read the device ID of the motion tracker.
 */
uint32_t MTI200::readDeviceId(void)
{
    XbusMessage reqDid = {XMID_ReqDid};
    XbusMessage const * didRsp = doTransaction(&reqDid);
    XbusMessageMemoryManager janitor(didRsp);
    uint32_t deviceId = 0;
    if (didRsp)
    {
        if (didRsp->mid == XMID_DeviceId)
        {
            deviceId = *(uint32_t*)didRsp->data;
        }
    }
    return deviceId;
}

/*!
 * \brief Sets MT output configuration.
 * \param conf Pointer to an array of OutputConfiguration elements.
 * \param elements The number of elements in the configuration array.
 *
 * The response from the device indicates the actual values that will
 * be used by the motion tracker. These may differ from the requested
 * parameters as the motion tracker validates the requested parameters
 * before applying them.
 */
bool MTI200::setOutputConfiguration(OutputConfiguration const* conf, uint8_t elements)
{
    XbusMessage outputConfMsg = {XMID_SetOutputConfig, elements, (void*)conf};
    XbusMessage const* outputConfRsp = doTransaction(&outputConfMsg);
    XbusMessageMemoryManager janitor(outputConfRsp);
    if (outputConfRsp)
    {
        if (outputConfRsp->mid == XMID_OutputConfig)
        {
            Debug::print("Output configuration set to:\r\n");
            OutputConfiguration* conf = (OutputConfiguration*)outputConfRsp->data;
            for (int i = 0; i < outputConfRsp->length; ++i)
            {
            	Debug::printf("\t%s: %d Hz\r\n", XbusMessage_dataDescription(conf->dtype), conf->freq);
                ++conf;
            }
            return true;
        }
        else
        {
            dumpResponse(outputConfRsp);
        }
    }
    else
    {
    	Debug::print("Failed to set output configuration.\r\n");
    }
    return false;
}

/*!
 * \brief Sets the motion tracker output configuration based on the function
 * of the attached device.
 *
 * The output configuration depends on the type of MTi-1 device connected.
 * An MTI-1 (IMU) device does not have an onboard orientation filter so
 * cannot output quaternion data, only inertial and magnetic measurement
 * data.
 * MTi-2 and MTi-3 devices have an onboard filter so can send quaternions.
 */
bool MTI200::configureMotionTracker(void)
{
    uint32_t deviceId = readDeviceId();

    if (deviceId)
    {
    	Debug::printf("Found device with ID: %08X.\r\n", deviceId);

        DeviceFunction function = XsDeviceId_getFunction(deviceId);
        Debug::printf("Device is an MTi-%d: %s.\r\n", function, XsDeviceId_functionDescription(function));

        if (function == DF_IMU)
        {
            OutputConfiguration conf[] = {
                {XDI_PacketCounter, 65535},
                {XDI_SampleTimeFine, 65535},
                {XDI_Acceleration, 100},
                {XDI_RateOfTurn, 100},
                {XDI_MagneticField, 100}
            };
            return setOutputConfiguration(conf,
                    sizeof(conf) / sizeof(OutputConfiguration));
        }
        else
        {
            OutputConfiguration conf[] = {
                {XDI_PacketCounter, 65535},
                {XDI_SampleTimeFine, 65535},
                {XDI_Quaternion, 400},
				{XDI_DeltaQ, 400},
				{XDI_Acceleration, 400},
				{XDI_RateOfTurn, 400},
				{XDI_MagneticField, 400},
                {XDI_StatusWord, 65535}
            };
            return setOutputConfiguration(conf,
                    sizeof(conf) / sizeof(OutputConfiguration));
        }
    }

    return false;
}


/*!
 * \brief Output the contents of a data message to the PC serial port.
 */
void MTI200::printMessageData(XbusMessage const* message)
{
    if (!message)
        return;

    Debug::print("MTData2:");
    uint16_t counter;
    if (XbusMessage_getDataItem(&counter, XDI_PacketCounter, message))
    {
    	Debug::printf(" Packet counter: %5d", counter);
    }
    uint32_t sampleTimeFine;
    float time;
    if (XbusMessage_getDataItem(&sampleTimeFine, XDI_SampleTimeFine, message))
    {
    	time = (float)sampleTimeFine / 10000.0f;
    	Debug::printf(" Time: %.2f", time);
    }
    float ori[4];
    if (XbusMessage_getDataItem(ori, XDI_Quaternion, message))
    {
    	Debug::printf(" Orientation: (% .3f, % .3f, % .3f, % .3f)", ori[0], ori[1],
                ori[2], ori[3]);
    }
    float acc[3];
    if (XbusMessage_getDataItem(acc, XDI_Acceleration, message))
    {
    	Debug::printf(" Acceleration: (% .3f, % .3f, % .3f)", acc[0], acc[1], acc[2]);
    }
    float gyr[3];
    if (XbusMessage_getDataItem(gyr, XDI_RateOfTurn, message))
    {
    	Debug::printf(" Rate Of Turn: (% .3f, % .3f, % .3f)", gyr[0], gyr[1], gyr[2]);
    }
    float mag[3];
    if (XbusMessage_getDataItem(mag, XDI_MagneticField, message))
    {
    	Debug::printf(" Magnetic Field: (% .3f, % .3f, % .3f)", mag[0], mag[1], mag[2]);
    }
    uint32_t status;
    if (XbusMessage_getDataItem(&status, XDI_StatusWord, message))
    {
    	Debug::printf(" Status:%X", status);
    }
    Debug::print("\r\n");
}

XbusMessage MTI200::GetMessage(uint32_t timeout)
{
	XbusMessage msg;
	msg.data = 0;
	msg.length = 0;
	msg.mid = XMID_Undefined;

    XbusMessage * msgPtr = NULL;
    xQueueReceive( _messageQueue, &msgPtr, ( TickType_t ) timeout ); // check USB connection every 100 ms
    if (msgPtr) {
    	memcpy(&msg, msgPtr, sizeof(XbusMessage));
    	vPortFree((void *)msgPtr);
    }
    return msg;
}

void MTI200::parseMTData2Message(XbusMessage const* message)
{
    if (!message)
        return;

    LastMeasurement.PackageCounter = 0;
    XbusMessage_getDataItem(&LastMeasurement.PackageCounter, XDI_PacketCounter, message);

    uint32_t sampleTimeFine;
    LastMeasurement.Time = 0;
    if (XbusMessage_getDataItem(&sampleTimeFine, XDI_SampleTimeFine, message))
    {
    	LastMeasurement.Time = (float)sampleTimeFine / 10000.0f;
    }

    memset(LastMeasurement.Quaternion, 0, sizeof(LastMeasurement.Quaternion));
    XbusMessage_getDataItem(LastMeasurement.Quaternion, XDI_Quaternion, message);

    memset(LastMeasurement.QuaternionDerivative, 0, sizeof(LastMeasurement.QuaternionDerivative));
    XbusMessage_getDataItem(LastMeasurement.QuaternionDerivative, XDI_DeltaQ, message);

    memset(LastMeasurement.Accelerometer, 0, sizeof(LastMeasurement.Accelerometer));
    XbusMessage_getDataItem(LastMeasurement.Accelerometer, XDI_Acceleration, message);

    memset(LastMeasurement.Gyroscope, 0, sizeof(LastMeasurement.Gyroscope));
    XbusMessage_getDataItem(LastMeasurement.Gyroscope, XDI_RateOfTurn, message);

    memset(LastMeasurement.Magnetometer, 0, sizeof(LastMeasurement.Magnetometer));
    XbusMessage_getDataItem(LastMeasurement.Magnetometer, XDI_MagneticField, message);

    LastMeasurement.Status = 0;
    XbusMessage_getDataItem(&LastMeasurement.Status, XDI_StatusWord, message);
}
