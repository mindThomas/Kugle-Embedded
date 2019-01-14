#ifndef LSPC_MESSAGE_TYPES_HPP
#define LSPC_MESSAGE_TYPES_HPP

#include <cstdint>

namespace lspc
{

	namespace MessageTypesFromPC
	{
		typedef enum MessageTypesFromPC: uint8_t
		{
			Test = 0x01,
			GetParameter = 0x02,
			SetParameter = 0x03,
			StoreParameters = 0x04,
			DumpParameters = 0x05,
			SystemSettings = 0x10,
			EstimatorSettings = 0x11,
			ControllerSettings = 0x12,
			YawCorrection = 0x20,
			PositionCorrection = 0x21,
			AttitudeReference = 0x30,
			AngularVelocityReference_Body = 0x31,
            AngularVelocityReference_Inertial = 0x32,
			VelocityReference_Inertial = 0x33,
			VelocityReference_Heading = 0x34,
			MPCpathReference = 0x35,
            CalibrateIMU = 0xE0,
            CPUload = 0xE1,
            EnterBootloader = 0xF0,
            Reboot = 0xF1,
            Debug = 0xFF
		} MessageTypesFromPC_t;

        typedef struct GetParameter
        {
            uint8_t type;
            uint8_t param;
        } GetParameter_t;

        typedef struct SetParameter
        {
            uint8_t type;
            uint8_t param;
            uint8_t value;
        } SetParameter_t;

        typedef struct EstimatorSettings
        {
            uint16_t estimate_msg_prescaler;
        } EstimatorSettings_t;

        typedef struct ControllerSettings
        {
            uint8_t mode;
        } ControllerSettings_t;

        typedef struct YawCorrection
        {
            float yaw;
        } YawCorrection_t;

        typedef struct PositionCorrection
        {
            float x;
            float y;
        } PositionCorrection_t;

        typedef struct AttitudeReference
        {
            struct q_t
            {
                float w;
                float x;
                float y;
                float z;
            } q;
        } AttitudeReference_t;

        typedef struct AngularVelocityReference_Body
        {
            struct omega_t
            {
                float x;
                float y;
                float z;
            } omega;
        } AngularVelocityReference_Body_t;

        typedef struct AngularVelocityReference_Inertial
        {
            struct omega_t
            {
                float x;
                float y;
                float z;
            } omega;
        } AngularVelocityReference_Inertial_t;

        typedef struct VelocityReference_Inertial
		{
			struct vel_t
			{
				float x;
				float y;
				float yaw;
			} vel;
		} VelocityReference_Inertial_t;

        typedef struct VelocityReference_Heading
        {
            struct vel_t
            {
                float x;
                float y;
                float yaw;
            } vel;
        } VelocityReference_Heading_t;

        typedef struct MPCpathReference
        {
            float desired_velocity;
            float desired_heading;
            float path_coeffs[7];
        } MPCpathReference_t;

        typedef struct CalibrateIMU
        {
            uint32_t magic_key;
        } CalibrateIMU_t;

        typedef struct EnterBootloader
        {
            uint32_t magic_key;
        } EnterBootloader_t;

        typedef struct Reboot
        {
            uint32_t magic_key;
        } Reboot_t;
	}

	namespace MessageTypesToPC
	{
		typedef enum MessageTypesToPC: uint8_t
		{
			Test = 0x01,
			GetParameter = 0x02,
			SetParameterAck = 0x03,
            StoreParametersAck = 0x04,
            DumpParameters = 0x05,
			SystemInfo = 0x10,
			StateEstimates = 0x11,
			ControllerInfo = 0x12,
			AttitudeControllerInfo = 0x13,
            VelocityControllerInfo = 0x14,
            MPCinfo = 0x20,
            PredictedMPCtrajectory = 0x21,
            RawSensor_IMU_MPU9250 = 0x30,
            RawSensor_IMU_MTI200 = 0x31,
            RawSensor_Encoders = 0x32,
            RawSensor_Battery = 0x33,
            CalibrateIMUAck = 0xE0,
			CPUload = 0xE1,
			MathDump = 0xFA,
			Debug = 0xFF
		} MessageTypesToPC_t;

        typedef struct SetParameterAck
        {
            uint8_t type;
            uint8_t param;
            bool acknowledged;
        } SetParameterAck_t;

        typedef struct StoreParametersAck
        {
            bool acknowledged;
        } StoreParametersAck_t;

        typedef struct SystemInfo
        {
            float time;
            float battery_pct;
            float current_consumption;
        } SystemInfo_t;

        typedef struct StateEstimates
        {
            float time;
            struct q_t
            {
                float w;
                float x;
                float y;
                float z;
            } q;
            struct dq_t
            {
                float w;
                float x;
                float y;
                float z;
            } dq;
            struct pos_t
            {
                float x;
                float y;
            } pos;
            struct vel_t
            {
                float x;
                float y;
            } vel;
            struct COM_t
            {
                float x;
                float y;
                float z;
            } COM;
        } StateEstimates_t;

        typedef struct ControllerInfo
        {
            float time;
            uint8_t type;
            uint8_t mode;
            float torque1;
            float torque2;
            float torque3;
            float compute_time;
        } ControllerInfo_t;

        typedef struct AttitudeControllerInfo
        {
            float time;
        } AttitudeControllerInfo_t;

        typedef struct VelocityControllerInfo
        {
            float time;
        } VelocityControllerInfo_t;

        typedef struct MPCinfo
        {
            float time;
        } MPCinfo_t;

        typedef struct PredictedMPCtrajectory
        {
            float time;
            uint8_t horizon_index;
            struct q_t
            {
                float w;
                float x;
                float y;
                float z;
            } q;
            struct dq_t
            {
                float w;
                float x;
                float y;
                float z;
            } dq;
            struct pos_t
            {
                float x;
                float y;
            } pos;
            struct vel_t
            {
                float x;
                float y;
            } vel;
        } PredictedMPCtrajectory_t;

        typedef struct RawSensor_IMU_MPU9250
        {
            float time;
            struct accelerometer_t
            {
                float x;
                float y;
                float z;
            } accelerometer;
            struct gyroscope_t
            {
                float x;
                float y;
                float z;
            } gyroscope;
            struct magnetometer_t
            {
                float x;
                float y;
                float z;
            } magnetometer;
        } RawSensor_IMU_MPU9250_t;

        typedef struct RawSensor_IMU_MTI200
        {
            float time;
            struct accelerometer_t
            {
                float x;
                float y;
                float z;
            } accelerometer;
            struct gyroscope_t
            {
                float x;
                float y;
                float z;
            } gyroscope;
            struct magnetometer_t
            {
                float x;
                float y;
                float z;
            } magnetometer;
        } RawSensor_IMU_MTI200_t;

        typedef struct RawSensor_Encoders
        {
            float time;
            float angle1;
            float angle2;
            float angle3;
        } RawSensor_Encoders_t;

        typedef struct RawSensor_Battery
        {
            float time;
            float vbat1;
            float vbat2;
            float current1;
            float current2;
            float pct1;
            float pct2;
        } RawSensor_Battery_t;

        typedef struct CalibrateIMUAck
        {
            bool acknowledged;
        } CalibrateIMUAck_t;
    }

	namespace ParameterLookup {
		typedef enum ValueType: uint8_t
		{
			_unknown = 0x00,
			_bool = 0x01,
			_float,
			_uint8,
			_uint16,
			_uint32
		} ValueType_t;
		typedef enum Type: uint8_t
		{
			debug = 0x01,
			behavioural,
			controller,
			estimator,
			model,
			test
		} type_t;

		typedef enum debug: uint8_t
		{
			EnableLogOutput = 0x01,
			EnableRawSensorOutput
		} debug_t;

		typedef enum behavioural: uint8_t
		{
			IndependentHeading = 0x01,
			YawVelocityBraking,
			StepTestEnabled,
			VelocityControllerEnabled,
			JoystickVelocityControl
		} behavioural_t;

		typedef enum controller: uint8_t
		{
			ControllerSampleRate = 0x01,
			Type,
			Mode,
			EnableTorqueLPF,
			TorqueLPFtau,
			EnableTorqueSaturation,
			TorqueMax,
			TorqueRampUp,
			TorqueRampUpTime,
			DisableQdot,
			K,
			ContinousSwitching,
			eta,
			epsilon,
			LQR_K,
			LQR_MaxYawError,
			VelocityController_MaxTilt,
			VelocityController_MaxIntegralCorrection,
			VelocityController_VelocityClamp,
			VelocityController_IntegralGain
		} controller_t;

		typedef enum estimator: uint8_t
		{
			EstimatorSampleRate = 0x01,
			EnableSensorLPFfilters,
			EnableSoftwareLPFfilters,
			SoftwareLPFcoeffs_a,
			SoftwareLPFcoeffs_b,
			CreateQdotFromQDifference,
			UseMadgwick,
			EstimateBias,
			Use2Lvelocity,
			UseVelocityEstimator,
			UseCOMestimateInVelocityEstimator,
			EstimateCOM,
			EstimateCOMminVelocity,
			MaxCOMDeviation,
			MadgwickBeta,
			GyroCov_Tuning_Factor,
			AccelCov_Tuning_Factor,
			cov_gyro_mpu,
			cov_acc_mpu,
			sigma2_bias,
			QEKF_P_init_diagonal,
			VelocityEstimator_P_init_diagonal,
			COMEstimator_P_init_diagonal
		} estimator_t;

		typedef enum model: uint8_t
		{
			l = 0x01,
			COM_X,
			COM_Y,
			COM_Z,
			g,
			rk,
			Mk,
			Jk,
			rw,
			Mw,
			i_gear,
			Jow,
			Jm,
			Jw,
			Mb,
			Jbx,
			Jby,
			Jbz,
			Bvk,
			Bvm,
			Bvb,
			EncoderTicksPrRev,
			TicksPrRev
		} model_t;

		typedef enum test: uint8_t
		{
			tmp = 0x01,
			tmp2
		} test_t;
	}

} // namespace lspc

#endif // LSPC_MESSAGE_TYPES_HPP
