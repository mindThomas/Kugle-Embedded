#ifndef LSPC_TEMPLATED_HPP
#define LSPC_TEMPLATED_HPP

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <vector>

#include "lspc/Packet.hpp"
#include "lspc/Serializable.hpp"
#include "lspc/SocketBase.hpp"
#include "cmsis_os.h" // for task creation

#define LSPC_RX_PROCESSING_THREAD_STACK_SIZE		128

namespace lspc
{

template <class COM>
class Socket : public SocketBase
{
public:
  Socket(COM * com, uint32_t processingTaskPriority) : com(com), _processingTaskHandle(0)
  {
	  xTaskCreate(Socket::ProcessingThread, (char *)"LSPC processing", LSPC_RX_PROCESSING_THREAD_STACK_SIZE, (void*) this, processingTaskPriority, &_processingTaskHandle);
  };

  using SocketBase::send;

  // Send a package with lspc
  //
  // @brief Sends a packaged buffer over the USB serial link.
  //
  // @param type The message type. This is user specific; any type between 1-255.
  // @param payload A vector with the serialized payload to be sent.
  //
  // @return True if the packet was sent.
  bool send(uint8_t type, const std::vector<uint8_t> &payload) override
  {
    Packet outPacket(type, payload);

    // Send it
    if (outPacket.encodedDataSize() ==
    		com->Write(outPacket.encodedDataPtr(), outPacket.encodedDataSize()))
      return true;
    else
      return false;
  };


  // Process incoming data on serial link
  //
  // @brief Reads the serial buffer and dispatches the received payload to the
  // relevant message handling callback function.
  void processSerial()
  {
    size_t bytecount = 0;
    while (com->Available() && bytecount < 10)
    {
      processIncomingByte(com->Read());
      bytecount++;
    }
    return;
  };

  static void ProcessingThread(void * pvParameters)
  {
  	Socket<COM> * lspc = (Socket<COM> *)pvParameters;

	// LSPC incoming data processing loop
	while (1)
	{
		if (lspc->com->WaitForNewData(portMAX_DELAY))
			lspc->processSerial();
	}
  }

public:
  COM * com;

private:
  TaskHandle_t _processingTaskHandle;

};

} // namespace lspc

#endif // LSPC_TEMPLATED_HPP
