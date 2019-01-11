#ifndef LSPC_MESSAGE_TYPES_HPP
#define LSPC_MESSAGE_TYPES_HPP

namespace lspc
{

	namespace MessageTypesIn
	{
		typedef enum MessageTypesIn: uint8_t
		{
			Test = 0x01,
			Control = 0x10,
			CalibrateIMU = 0xE0
		} MessageTypesIn_t;
	}

	namespace MessageTypesOut
	{
		typedef enum MessageTypesOut: uint8_t
		{
			Test = 0x01,
			SysInfo = 0x10,
			Debug = 0xFF
		} MessageTypesOut_t;
	}

} // namespace lspc

#endif // LSPC_MESSAGE_TYPES_HPP
