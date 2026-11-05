#pragma once

#include <Arduino.h>
#include "i2c_master.h"

/// @brief Class for the TDK InvenSense MPU-6500 6-axis motion sensor
class MPU6500_I2C final : public I2C_Master
{
public:
	enum class I2CAddress : uint8_t
	{
		Addr_0x68 = 0b1101000, // AD0 = 0
		Addr_0x69 = 0b1101001  // AD0 = 1
	};

	enum class PowerMode : uint8_t
	{
		Sleep = 0,				   // Gyro off, accel off
		Standby = 1,			   // Gyro drive on (quick enable), accel off
		LowPowerAccelerometer = 2, // Gyro off, accel duty-cycled
		LowNoiseAccelerometer = 3, // Gyro off, accel on
		Gyroscope = 4,			   // Gyro on, accel off
		SixAxis = 5				   // Gyro on, accel on
	};

	enum class GyroFullScale : uint8_t
	{
		Range_250DPS = 0,  // ±250 dps (default)
		Range_500DPS = 1,  // ±500 dps
		Range_1000DPS = 2, // ±1000 dps
		Range_2000DPS = 3  // ±2000 dps
	};

	enum class AccelFullScale : uint8_t
	{
		Range_2G = 0, // ±2 g (default)
		Range_4G = 1, // ±4 g
		Range_8G = 2, // ±8 g
		Range_16G = 3 // ±16 g
	};

	enum class GyroFilter : uint8_t
	{
		Bandwidth_8800Hz = 0,		  // DLPF bypassed, Fs = 32 kHz, delay 0.064 ms
		Bandwidth_3600Hz_Fs32kHz = 1, // DLPF bypassed, Fs = 32 kHz, delay 0.11 ms
		Bandwidth_250Hz = 2,		  // Fs = 8 kHz, delay 0.97 ms
		Bandwidth_184Hz = 3,		  // Fs = 1 kHz, delay 2.9 ms
		Bandwidth_92Hz = 4,			  // Fs = 1 kHz, delay 3.9 ms
		Bandwidth_41Hz = 5,			  // Fs = 1 kHz, delay 5.9 ms
		Bandwidth_20Hz = 6,			  // Fs = 1 kHz, delay 9.9 ms
		Bandwidth_10Hz = 7,			  // Fs = 1 kHz, delay 17.85 ms
		Bandwidth_5Hz = 8,			  // Fs = 1 kHz, delay 33.48 ms
		Bandwidth_3600Hz_Fs8kHz = 9	  // Fs = 8 kHz, delay 0.17 ms
	};

	enum class AccelFilter : uint8_t
	{
		Bandwidth_1130Hz = 0, // DLPF bypassed, rate 4 kHz, delay 0.75 ms
		Bandwidth_460Hz = 1,  // Rate 1 kHz, delay 1.94 ms
		Bandwidth_184Hz = 2,  // Rate 1 kHz, delay 5.80 ms
		Bandwidth_92Hz = 3,	  // Rate 1 kHz, delay 7.80 ms
		Bandwidth_41Hz = 4,	  // Rate 1 kHz, delay 11.80 ms
		Bandwidth_20Hz = 5,	  // Rate 1 kHz, delay 19.80 ms
		Bandwidth_10Hz = 6,	  // Rate 1 kHz, delay 35.70 ms
		Bandwidth_5Hz = 7	  // Rate 1 kHz, delay 66.96 ms
	};

	enum class LowPowerAccelRate : uint8_t
	{
		Rate_0_24Hz = 0,
		Rate_0_49Hz = 1,
		Rate_0_98Hz = 2,
		Rate_1_95Hz = 3,
		Rate_3_91Hz = 4,
		Rate_7_81Hz = 5,
		Rate_15_63Hz = 6,
		Rate_31_25Hz = 7,
		Rate_62_50Hz = 8,
		Rate_125Hz = 9,
		Rate_250Hz = 10,
		Rate_500Hz = 11
	};

	struct InterruptStatus final
	{
		bool rawDataReady = false; // RAW_DATA_RDY_INT: Sensor data updated and ready to read
		bool dmp = false;		   // DMP_INT: DMP interrupt generated
		bool fsync = false;		   // FSYNC_INT: FSYNC interrupt occurred
		bool fifoOverflow = false; // FIFO_OVERFLOW_INT: FIFO overflow occurred (oldest data dropped)
		bool wakeOnMotion = false; // WOM_INT: Wake on motion interrupt occurred
	};

	enum class AuxI2CMode : uint8_t
	{
		Disabled = 0,	 // I2C master off, bypass off (aux pins float high)
		PassThrough = 1, // Aux bus connected to the primary bus
		Master = 2		 // MPU-6500 is master on the aux bus
	};

	enum class I2CMasterClock : uint8_t
	{
		Clock_348kHz = 0, // Default
		Clock_333kHz = 1,
		Clock_320kHz = 2,
		Clock_308kHz = 3,
		Clock_296kHz = 4,
		Clock_286kHz = 5,
		Clock_276kHz = 6,
		Clock_267kHz = 7,
		Clock_258kHz = 8,
		Clock_500kHz = 9,
		Clock_471kHz = 10,
		Clock_444kHz = 11,
		Clock_421kHz = 12,
		Clock_400kHz = 13,
		Clock_381kHz = 14,
		Clock_364kHz = 15
	};

	enum class AuxSlave : uint8_t
	{
		Slave0 = 0,
		Slave1 = 1,
		Slave2 = 2,
		Slave3 = 3,
		Slave4 = 4 // Single transfers only (see writeSlave4 / readSlave4)
	};

	struct I2CMasterStatus final
	{
		bool slave0Nack = false;	  // I2C_SLV0_NACK: Slave 0 received a NACK
		bool slave1Nack = false;	  // I2C_SLV1_NACK: Slave 1 received a NACK
		bool slave2Nack = false;	  // I2C_SLV2_NACK: Slave 2 received a NACK
		bool slave3Nack = false;	  // I2C_SLV3_NACK: Slave 3 received a NACK
		bool slave4Nack = false;	  // I2C_SLV4_NACK: Slave 4 received a NACK
		bool lostArbitration = false; // I2C_LOST_ARB: Lost arbitration of the aux I2C bus
		bool slave4Done = false;	  // I2C_SLV4_DONE: Slave 4 transfer complete
		bool passThrough = false;	  // PASS_THROUGH: FSYNC interrupt occurred
	};

	static constexpr uint16_t FIFO_SIZE = 512; // FIFO size in bytes

	enum class FifoMode : uint8_t
	{
		Overwrite = 0,	 // When full, new data replaces the oldest (default)
		StopWhenFull = 1 // When full, new data is not written
	};

	struct FifoSources final
	{
		bool temperature = false; // TEMP_OUT
		bool gyroX = false;		  // GYRO_XOUT
		bool gyroY = false;		  // GYRO_YOUT
		bool gyroZ = false;		  // GYRO_ZOUT
		bool accel = false;		  // ACCEL (X, Y and Z)
		bool slave0 = false;	  // SLV_0
		bool slave1 = false;	  // SLV_1
		bool slave2 = false;	  // SLV_2
		bool slave3 = false;	  // SLV_3_FIFO_EN (in I2C_MST_CTRL)
	};

	struct RawSensorData final
	{
		int16_t accelX = 0;
		int16_t accelY = 0;
		int16_t accelZ = 0;
		int16_t temperature = 0;
		int16_t gyroX = 0;
		int16_t gyroY = 0;
		int16_t gyroZ = 0;
	};

	struct SensorData final
	{
		int16_t accelX = 0;		 // mg
		int16_t accelY = 0;		 // mg
		int16_t accelZ = 0;		 // mg
		int32_t temperature = 0; // m°C
		int32_t gyroX = 0;		 // mdps
		int32_t gyroY = 0;		 // mdps
		int32_t gyroZ = 0;		 // mdps
	};

private:
	static constexpr uint8_t IC_ID_VALUE = 0x70; // Value returned from the MPU-6500 when reading WHO_AM_I

	static constexpr uint8_t REG_SELF_TEST_X_GYRO = 0x00;	// X gyro factory self-test code
	static constexpr uint8_t REG_SELF_TEST_Y_GYRO = 0x01;	// Y gyro factory self-test code
	static constexpr uint8_t REG_SELF_TEST_Z_GYRO = 0x02;	// Z gyro factory self-test code
	static constexpr uint8_t REG_SELF_TEST_X_ACCEL = 0x0D;	// X accel factory self-test code
	static constexpr uint8_t REG_SELF_TEST_Y_ACCEL = 0x0E;	// Y accel factory self-test code
	static constexpr uint8_t REG_SELF_TEST_Z_ACCEL = 0x0F;	// Z accel factory self-test code
	static constexpr uint8_t REG_XG_OFFSET_H = 0x13;		// X gyro offset (X_OFFS_USR), high byte
	static constexpr uint8_t REG_XG_OFFSET_L = 0x14;		// X gyro offset (X_OFFS_USR), low byte
	static constexpr uint8_t REG_YG_OFFSET_H = 0x15;		// Y gyro offset (Y_OFFS_USR), high byte
	static constexpr uint8_t REG_YG_OFFSET_L = 0x16;		// Y gyro offset (Y_OFFS_USR), low byte
	static constexpr uint8_t REG_ZG_OFFSET_H = 0x17;		// Z gyro offset (Z_OFFS_USR), high byte
	static constexpr uint8_t REG_ZG_OFFSET_L = 0x18;		// Z gyro offset (Z_OFFS_USR), low byte
	static constexpr uint8_t REG_SMPLRT_DIV = 0x19;			// Sample rate divider (SAMPLE_RATE = 1kHz / (1 + SMPLRT_DIV))
	static constexpr uint8_t REG_CONFIG = 0x1A;				// Configuration
	static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;		// Gyroscope configuration
	static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;		// Accelerometer configuration
	static constexpr uint8_t REG_ACCEL_CONFIG_2 = 0x1D;		// Accelerometer configuration 2
	static constexpr uint8_t REG_LP_ACCEL_ODR = 0x1E;		// Low power accelerometer ODR control
	static constexpr uint8_t REG_WOM_THR = 0x1F;			// Wake-on-motion threshold (4 mg per LSB)
	static constexpr uint8_t REG_FIFO_EN = 0x23;			// FIFO enable
	static constexpr uint8_t REG_I2C_MST_CTRL = 0x24;		// I2C master control
	static constexpr uint8_t REG_I2C_SLV0_ADDR = 0x25;		// I2C slave 0 address
	static constexpr uint8_t REG_I2C_SLV0_REG = 0x26;		// I2C slave 0 register address to begin transfer
	static constexpr uint8_t REG_I2C_SLV0_CTRL = 0x27;		// I2C slave 0 control
	static constexpr uint8_t REG_I2C_SLV1_ADDR = 0x28;		// I2C slave 1 address
	static constexpr uint8_t REG_I2C_SLV1_REG = 0x29;		// I2C slave 1 register address to begin transfer
	static constexpr uint8_t REG_I2C_SLV1_CTRL = 0x2A;		// I2C slave 1 control
	static constexpr uint8_t REG_I2C_SLV2_ADDR = 0x2B;		// I2C slave 2 address
	static constexpr uint8_t REG_I2C_SLV2_REG = 0x2C;		// I2C slave 2 register address to begin transfer
	static constexpr uint8_t REG_I2C_SLV2_CTRL = 0x2D;		// I2C slave 2 control
	static constexpr uint8_t REG_I2C_SLV3_ADDR = 0x2E;		// I2C slave 3 address
	static constexpr uint8_t REG_I2C_SLV3_REG = 0x2F;		// I2C slave 3 register address to begin transfer
	static constexpr uint8_t REG_I2C_SLV3_CTRL = 0x30;		// I2C slave 3 control
	static constexpr uint8_t REG_I2C_SLV4_ADDR = 0x31;		// I2C slave 4 address
	static constexpr uint8_t REG_I2C_SLV4_REG = 0x32;		// I2C slave 4 register address to begin transfer
	static constexpr uint8_t REG_I2C_SLV4_DO = 0x33;		// I2C slave 4 data to write
	static constexpr uint8_t REG_I2C_SLV4_CTRL = 0x34;		// I2C slave 4 control
	static constexpr uint8_t REG_I2C_SLV4_DI = 0x35;		// I2C slave 4 data read
	static constexpr uint8_t REG_I2C_MST_STATUS = 0x36;		// I2C master status (cleared on read)
	static constexpr uint8_t REG_INT_PIN_CFG = 0x37;		// INT pin / bypass enable configuration
	static constexpr uint8_t REG_INT_ENABLE = 0x38;			// Interrupt enable
	static constexpr uint8_t REG_INT_STATUS = 0x3A;			// Interrupt status (cleared on read)
	static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;		// Accel X, high byte
	static constexpr uint8_t REG_ACCEL_XOUT_L = 0x3C;		// Accel X, low byte
	static constexpr uint8_t REG_ACCEL_YOUT_H = 0x3D;		// Accel Y, high byte
	static constexpr uint8_t REG_ACCEL_YOUT_L = 0x3E;		// Accel Y, low byte
	static constexpr uint8_t REG_ACCEL_ZOUT_H = 0x3F;		// Accel Z, high byte
	static constexpr uint8_t REG_ACCEL_ZOUT_L = 0x40;		// Accel Z, low byte
	static constexpr uint8_t REG_TEMP_OUT_H = 0x41;			// Temperature, high byte
	static constexpr uint8_t REG_TEMP_OUT_L = 0x42;			// Temperature, low byte
	static constexpr uint8_t REG_GYRO_XOUT_H = 0x43;		// Gyro X, high byte
	static constexpr uint8_t REG_GYRO_XOUT_L = 0x44;		// Gyro X, low byte
	static constexpr uint8_t REG_GYRO_YOUT_H = 0x45;		// Gyro Y, high byte
	static constexpr uint8_t REG_GYRO_YOUT_L = 0x46;		// Gyro Y, low byte
	static constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;		// Gyro Z, high byte
	static constexpr uint8_t REG_GYRO_ZOUT_L = 0x48;		// Gyro Z, low byte
	static constexpr uint8_t REG_EXT_SENS_DATA_00 = 0x49;	// External sensor data 00
	static constexpr uint8_t REG_EXT_SENS_DATA_01 = 0x4A;	// External sensor data 01
	static constexpr uint8_t REG_EXT_SENS_DATA_02 = 0x4B;	// External sensor data 02
	static constexpr uint8_t REG_EXT_SENS_DATA_03 = 0x4C;	// External sensor data 03
	static constexpr uint8_t REG_EXT_SENS_DATA_04 = 0x4D;	// External sensor data 04
	static constexpr uint8_t REG_EXT_SENS_DATA_05 = 0x4E;	// External sensor data 05
	static constexpr uint8_t REG_EXT_SENS_DATA_06 = 0x4F;	// External sensor data 06
	static constexpr uint8_t REG_EXT_SENS_DATA_07 = 0x50;	// External sensor data 07
	static constexpr uint8_t REG_EXT_SENS_DATA_08 = 0x51;	// External sensor data 08
	static constexpr uint8_t REG_EXT_SENS_DATA_09 = 0x52;	// External sensor data 09
	static constexpr uint8_t REG_EXT_SENS_DATA_10 = 0x53;	// External sensor data 10
	static constexpr uint8_t REG_EXT_SENS_DATA_11 = 0x54;	// External sensor data 11
	static constexpr uint8_t REG_EXT_SENS_DATA_12 = 0x55;	// External sensor data 12
	static constexpr uint8_t REG_EXT_SENS_DATA_13 = 0x56;	// External sensor data 13
	static constexpr uint8_t REG_EXT_SENS_DATA_14 = 0x57;	// External sensor data 14
	static constexpr uint8_t REG_EXT_SENS_DATA_15 = 0x58;	// External sensor data 15
	static constexpr uint8_t REG_EXT_SENS_DATA_16 = 0x59;	// External sensor data 16
	static constexpr uint8_t REG_EXT_SENS_DATA_17 = 0x5A;	// External sensor data 17
	static constexpr uint8_t REG_EXT_SENS_DATA_18 = 0x5B;	// External sensor data 18
	static constexpr uint8_t REG_EXT_SENS_DATA_19 = 0x5C;	// External sensor data 19
	static constexpr uint8_t REG_EXT_SENS_DATA_20 = 0x5D;	// External sensor data 20
	static constexpr uint8_t REG_EXT_SENS_DATA_21 = 0x5E;	// External sensor data 21
	static constexpr uint8_t REG_EXT_SENS_DATA_22 = 0x5F;	// External sensor data 22
	static constexpr uint8_t REG_EXT_SENS_DATA_23 = 0x60;	// External sensor data 23
	static constexpr uint8_t REG_I2C_SLV0_DO = 0x63;		// I2C slave 0 data out
	static constexpr uint8_t REG_I2C_SLV1_DO = 0x64;		// I2C slave 1 data out
	static constexpr uint8_t REG_I2C_SLV2_DO = 0x65;		// I2C slave 2 data out
	static constexpr uint8_t REG_I2C_SLV3_DO = 0x66;		// I2C slave 3 data out
	static constexpr uint8_t REG_I2C_MST_DELAY_CTRL = 0x67; // I2C master delay control
	static constexpr uint8_t REG_SIGNAL_PATH_RESET = 0x68;	// Signal path reset
	static constexpr uint8_t REG_ACCEL_INTEL_CTRL = 0x69;	// Accelerometer interrupt control (wake-on-motion)
	static constexpr uint8_t REG_USER_CTRL = 0x6A;			// User control
	static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;			// Power management 1
	static constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;			// Power management 2
	static constexpr uint8_t REG_FIFO_COUNT_H = 0x72;		// FIFO count, high byte (reading latches both bytes)
	static constexpr uint8_t REG_FIFO_COUNT_L = 0x73;		// FIFO count, low byte
	static constexpr uint8_t REG_FIFO_R_W = 0x74;			// FIFO read / write
	static constexpr uint8_t REG_WHO_AM_I = 0x75;			// Device identity register
	static constexpr uint8_t REG_XA_OFFSET_H = 0x77;		// X accel offset (XA_OFFS), high byte
	static constexpr uint8_t REG_XA_OFFSET_L = 0x78;		// X accel offset (XA_OFFS), low byte
	static constexpr uint8_t REG_YA_OFFSET_H = 0x7A;		// Y accel offset (YA_OFFS), high byte
	static constexpr uint8_t REG_YA_OFFSET_L = 0x7B;		// Y accel offset (YA_OFFS), low byte
	static constexpr uint8_t REG_ZA_OFFSET_H = 0x7D;		// Z accel offset (ZA_OFFS), high byte
	static constexpr uint8_t REG_ZA_OFFSET_L = 0x7E;		// Z accel offset (ZA_OFFS), low byte

	// Configuration bits
	static constexpr uint8_t CONFIG_BIT_DLPF_CFG_0 = (1 << 0);	   // DLPF_CFG <0>: Gyro / temperature DLPF bit 0
	static constexpr uint8_t CONFIG_BIT_DLPF_CFG_1 = (1 << 1);	   // DLPF_CFG <1>: Gyro / temperature DLPF bit 1
	static constexpr uint8_t CONFIG_BIT_DLPF_CFG_2 = (1 << 2);	   // DLPF_CFG <2>: Gyro / temperature DLPF bit 2
	static constexpr uint8_t CONFIG_BIT_EXT_SYNC_SET_0 = (1 << 3); // EXT_SYNC_SET <0>: FSYNC bit location bit 0
	static constexpr uint8_t CONFIG_BIT_EXT_SYNC_SET_1 = (1 << 4); // EXT_SYNC_SET <1>: FSYNC bit location bit 1
	static constexpr uint8_t CONFIG_BIT_EXT_SYNC_SET_2 = (1 << 5); // EXT_SYNC_SET <2>: FSYNC bit location bit 2
	static constexpr uint8_t CONFIG_BIT_FIFO_MODE = (1 << 6);	   // FIFO_MODE: 1 = Stop writing when FIFO full, 0 = Overwrite oldest data
	static constexpr uint8_t CONFIG_BIT_RESERVED_7 = (1 << 7);	   // Reserved

	// Configuration DLPF_CFG values (gyro / temperature, used when FCHOICE_B = 00)
	static constexpr uint8_t CONFIG_DLPF_CFG_BITS_MASK = CONFIG_BIT_DLPF_CFG_2 |
														 CONFIG_BIT_DLPF_CFG_1 |
														 CONFIG_BIT_DLPF_CFG_0;
	static constexpr uint8_t CONFIG_DLPF_CFG_250HZ = 0;																		 // 000: 250 Hz
	static constexpr uint8_t CONFIG_DLPF_CFG_184HZ = CONFIG_BIT_DLPF_CFG_0;													 // 001: 184 Hz
	static constexpr uint8_t CONFIG_DLPF_CFG_92HZ = CONFIG_BIT_DLPF_CFG_1;													 // 010: 92 Hz
	static constexpr uint8_t CONFIG_DLPF_CFG_41HZ = CONFIG_BIT_DLPF_CFG_1 | CONFIG_BIT_DLPF_CFG_0;							 // 011: 41 Hz
	static constexpr uint8_t CONFIG_DLPF_CFG_20HZ = CONFIG_BIT_DLPF_CFG_2;													 // 100: 20 Hz
	static constexpr uint8_t CONFIG_DLPF_CFG_10HZ = CONFIG_BIT_DLPF_CFG_2 | CONFIG_BIT_DLPF_CFG_0;							 // 101: 10 Hz
	static constexpr uint8_t CONFIG_DLPF_CFG_5HZ = CONFIG_BIT_DLPF_CFG_2 | CONFIG_BIT_DLPF_CFG_1;							 // 110: 5 Hz
	static constexpr uint8_t CONFIG_DLPF_CFG_3600HZ = CONFIG_BIT_DLPF_CFG_2 | CONFIG_BIT_DLPF_CFG_1 | CONFIG_BIT_DLPF_CFG_0; // 111: 3600 Hz

	// Gyroscope configuration bits
	static constexpr uint8_t GYRO_CONFIG_BIT_FCHOICE_B_0 = (1 << 0);   // FCHOICE_B <0>: DLPF bypass bit 0
	static constexpr uint8_t GYRO_CONFIG_BIT_FCHOICE_B_1 = (1 << 1);   // FCHOICE_B <1>: DLPF bypass bit 1
	static constexpr uint8_t GYRO_CONFIG_BIT_RESERVED_2 = (1 << 2);	   // Reserved
	static constexpr uint8_t GYRO_CONFIG_BIT_GYRO_FS_SEL_0 = (1 << 3); // GYRO_FS_SEL <0>: Gyro full scale select bit 0
	static constexpr uint8_t GYRO_CONFIG_BIT_GYRO_FS_SEL_1 = (1 << 4); // GYRO_FS_SEL <1>: Gyro full scale select bit 1
	static constexpr uint8_t GYRO_CONFIG_BIT_ZG_ST = (1 << 5);		   // ZG_ST: Z gyro self-test
	static constexpr uint8_t GYRO_CONFIG_BIT_YG_ST = (1 << 6);		   // YG_ST: Y gyro self-test
	static constexpr uint8_t GYRO_CONFIG_BIT_XG_ST = (1 << 7);		   // XG_ST: X gyro self-test

	// Gyro full scale values
	static constexpr uint8_t GYRO_CONFIG_GYRO_FS_SEL_BITS_MASK = GYRO_CONFIG_BIT_GYRO_FS_SEL_1 |
																 GYRO_CONFIG_BIT_GYRO_FS_SEL_0;
	static constexpr uint8_t GYRO_CONFIG_GYRO_FS_SEL_250 = 0;															   // 00: ±250 dps
	static constexpr uint8_t GYRO_CONFIG_GYRO_FS_SEL_500 = GYRO_CONFIG_BIT_GYRO_FS_SEL_0;								   // 01: ±500 dps
	static constexpr uint8_t GYRO_CONFIG_GYRO_FS_SEL_1000 = GYRO_CONFIG_BIT_GYRO_FS_SEL_1;								   // 10: ±1000 dps
	static constexpr uint8_t GYRO_CONFIG_GYRO_FS_SEL_2000 = GYRO_CONFIG_BIT_GYRO_FS_SEL_1 | GYRO_CONFIG_BIT_GYRO_FS_SEL_0; // 11: ±2000 dps

	// Gyroscope configuration FCHOICE_B values
	static constexpr uint8_t GYRO_CONFIG_FCHOICE_B_BITS_MASK = GYRO_CONFIG_BIT_FCHOICE_B_1 |
															   GYRO_CONFIG_BIT_FCHOICE_B_0;
	static constexpr uint8_t GYRO_CONFIG_FCHOICE_B_DLPF = 0;									// 00: DLPF used (set by DLPF_CFG)
	static constexpr uint8_t GYRO_CONFIG_FCHOICE_B_BYPASS_8800HZ = GYRO_CONFIG_BIT_FCHOICE_B_0; // x1: DLPF bypassed, 8800 Hz
	static constexpr uint8_t GYRO_CONFIG_FCHOICE_B_BYPASS_3600HZ = GYRO_CONFIG_BIT_FCHOICE_B_1; // 10: DLPF bypassed, 3600 Hz

	// Accelerometer configuration bits
	static constexpr uint8_t ACCEL_CONFIG_BIT_RESERVED_0 = (1 << 0);	 // Reserved
	static constexpr uint8_t ACCEL_CONFIG_BIT_RESERVED_1 = (1 << 1);	 // Reserved
	static constexpr uint8_t ACCEL_CONFIG_BIT_RESERVED_2 = (1 << 2);	 // Reserved
	static constexpr uint8_t ACCEL_CONFIG_BIT_ACCEL_FS_SEL_0 = (1 << 3); // ACCEL_FS_SEL <0>: Accel full scale select bit 0
	static constexpr uint8_t ACCEL_CONFIG_BIT_ACCEL_FS_SEL_1 = (1 << 4); // ACCEL_FS_SEL <1>: Accel full scale select bit 1
	static constexpr uint8_t ACCEL_CONFIG_BIT_ZA_ST = (1 << 5);			 // ZA_ST: Z accel self-test
	static constexpr uint8_t ACCEL_CONFIG_BIT_YA_ST = (1 << 6);			 // YA_ST: Y accel self-test
	static constexpr uint8_t ACCEL_CONFIG_BIT_XA_ST = (1 << 7);			 // XA_ST: X accel self-test

	// Accel full scale values
	static constexpr uint8_t ACCEL_CONFIG_ACCEL_FS_SEL_BITS_MASK = ACCEL_CONFIG_BIT_ACCEL_FS_SEL_1 |
																   ACCEL_CONFIG_BIT_ACCEL_FS_SEL_0;
	static constexpr uint8_t ACCEL_CONFIG_ACCEL_FS_SEL_2G = 0;																	// 00: ±2 g
	static constexpr uint8_t ACCEL_CONFIG_ACCEL_FS_SEL_4G = ACCEL_CONFIG_BIT_ACCEL_FS_SEL_0;									// 01: ±4 g
	static constexpr uint8_t ACCEL_CONFIG_ACCEL_FS_SEL_8G = ACCEL_CONFIG_BIT_ACCEL_FS_SEL_1;									// 10: ±8 g
	static constexpr uint8_t ACCEL_CONFIG_ACCEL_FS_SEL_16G = ACCEL_CONFIG_BIT_ACCEL_FS_SEL_1 | ACCEL_CONFIG_BIT_ACCEL_FS_SEL_0; // 11: ±16 g

	// Accelerometer configuration 2 bits
	static constexpr uint8_t ACCEL_CONFIG_2_BIT_A_DLPF_CFG_0 = (1 << 0);	// A_DLPF_CFG <0>: Accel DLPF bit 0
	static constexpr uint8_t ACCEL_CONFIG_2_BIT_A_DLPF_CFG_1 = (1 << 1);	// A_DLPF_CFG <1>: Accel DLPF bit 1
	static constexpr uint8_t ACCEL_CONFIG_2_BIT_A_DLPF_CFG_2 = (1 << 2);	// A_DLPF_CFG <2>: Accel DLPF bit 2
	static constexpr uint8_t ACCEL_CONFIG_2_BIT_ACCEL_FCHOICE_B = (1 << 3); // ACCEL_FCHOICE_B: 1 = Accel DLPF bypassed, 0 = DLPF used
	static constexpr uint8_t ACCEL_CONFIG_2_BIT_RESERVED_4 = (1 << 4);		// Reserved
	static constexpr uint8_t ACCEL_CONFIG_2_BIT_RESERVED_5 = (1 << 5);		// Reserved
	static constexpr uint8_t ACCEL_CONFIG_2_BIT_RESERVED_6 = (1 << 6);		// Reserved
	static constexpr uint8_t ACCEL_CONFIG_2_BIT_RESERVED_7 = (1 << 7);		// Reserved

	// Accelerometer configuration 2 A_DLPF_CFG values (used when ACCEL_FCHOICE_B = 0)
	static constexpr uint8_t ACCEL_CONFIG_2_A_DLPF_CFG_BITS_MASK = ACCEL_CONFIG_2_BIT_A_DLPF_CFG_2 |
																   ACCEL_CONFIG_2_BIT_A_DLPF_CFG_1 |
																   ACCEL_CONFIG_2_BIT_A_DLPF_CFG_0;
	static constexpr uint8_t ACCEL_CONFIG_2_A_DLPF_CFG_460HZ = 0;																 // 000: 460 Hz (111 is identical)
	static constexpr uint8_t ACCEL_CONFIG_2_A_DLPF_CFG_184HZ = ACCEL_CONFIG_2_BIT_A_DLPF_CFG_0;									 // 001: 184 Hz
	static constexpr uint8_t ACCEL_CONFIG_2_A_DLPF_CFG_92HZ = ACCEL_CONFIG_2_BIT_A_DLPF_CFG_1;									 // 010: 92 Hz
	static constexpr uint8_t ACCEL_CONFIG_2_A_DLPF_CFG_41HZ = ACCEL_CONFIG_2_BIT_A_DLPF_CFG_1 | ACCEL_CONFIG_2_BIT_A_DLPF_CFG_0; // 011: 41 Hz
	static constexpr uint8_t ACCEL_CONFIG_2_A_DLPF_CFG_20HZ = ACCEL_CONFIG_2_BIT_A_DLPF_CFG_2;									 // 100: 20 Hz
	static constexpr uint8_t ACCEL_CONFIG_2_A_DLPF_CFG_10HZ = ACCEL_CONFIG_2_BIT_A_DLPF_CFG_2 | ACCEL_CONFIG_2_BIT_A_DLPF_CFG_0; // 101: 10 Hz
	static constexpr uint8_t ACCEL_CONFIG_2_A_DLPF_CFG_5HZ = ACCEL_CONFIG_2_BIT_A_DLPF_CFG_2 | ACCEL_CONFIG_2_BIT_A_DLPF_CFG_1;	 // 110: 5 Hz

	// Low power accelerometer ODR control bits
	static constexpr uint8_t LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_0 = (1 << 0); // LPOSC_CLKSEL <0>: Low power accel ODR bit 0
	static constexpr uint8_t LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_1 = (1 << 1); // LPOSC_CLKSEL <1>: Low power accel ODR bit 1
	static constexpr uint8_t LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_2 = (1 << 2); // LPOSC_CLKSEL <2>: Low power accel ODR bit 2
	static constexpr uint8_t LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_3 = (1 << 3); // LPOSC_CLKSEL <3>: Low power accel ODR bit 3
	static constexpr uint8_t LP_ACCEL_ODR_BIT_RESERVED_4 = (1 << 4);	 // Reserved
	static constexpr uint8_t LP_ACCEL_ODR_BIT_RESERVED_5 = (1 << 5);	 // Reserved
	static constexpr uint8_t LP_ACCEL_ODR_BIT_RESERVED_6 = (1 << 6);	 // Reserved
	static constexpr uint8_t LP_ACCEL_ODR_BIT_RESERVED_7 = (1 << 7);	 // Reserved

	// Low power accelerometer ODR LPOSC_CLKSEL values
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_BITS_MASK = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_3 |
																   LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_2 |
																   LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_1 |
																   LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_0;
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_0_24HZ = 0;																									  // 0000: 0.24 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_0_49HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_0;																	  // 0001: 0.49 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_0_98HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_1;																	  // 0010: 0.98 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_1_95HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_1 | LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_0;									  // 0011: 1.95 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_3_91HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_2;																	  // 0100: 3.91 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_7_81HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_2 | LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_0;									  // 0101: 7.81 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_15_63HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_2 | LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_1;									  // 0110: 15.63 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_31_25HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_2 | LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_1 | LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_0; // 0111: 31.25 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_62_50HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_3;																	  // 1000: 62.50 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_125HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_3 | LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_0;									  // 1001: 125 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_250HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_3 | LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_1;									  // 1010: 250 Hz
	static constexpr uint8_t LP_ACCEL_ODR_LPOSC_CLKSEL_500HZ = LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_3 | LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_1 | LP_ACCEL_ODR_BIT_LPOSC_CLKSEL_0;	  // 1011: 500 Hz
																																									  // 1100 to 1111: Reserved

	// FIFO enable bits
	static constexpr uint8_t FIFO_EN_BIT_SLV_0 = (1 << 0);	   // SLV_0: 1 = Slave 0 EXT_SENS_DATA written to FIFO at the sample rate
	static constexpr uint8_t FIFO_EN_BIT_SLV_1 = (1 << 1);	   // SLV_1: 1 = Slave 1 EXT_SENS_DATA written to FIFO at the sample rate
	static constexpr uint8_t FIFO_EN_BIT_SLV_2 = (1 << 2);	   // SLV_2: 1 = Slave 2 EXT_SENS_DATA written to FIFO at the sample rate
	static constexpr uint8_t FIFO_EN_BIT_ACCEL = (1 << 3);	   // ACCEL: 1 = Accel X, Y and Z written to FIFO at the sample rate
	static constexpr uint8_t FIFO_EN_BIT_GYRO_ZOUT = (1 << 4); // GYRO_ZOUT: 1 = Gyro Z written to FIFO at the sample rate
	static constexpr uint8_t FIFO_EN_BIT_GYRO_YOUT = (1 << 5); // GYRO_YOUT: 1 = Gyro Y written to FIFO at the sample rate
	static constexpr uint8_t FIFO_EN_BIT_GYRO_XOUT = (1 << 6); // GYRO_XOUT: 1 = Gyro X written to FIFO at the sample rate
	static constexpr uint8_t FIFO_EN_BIT_TEMP_OUT = (1 << 7);  // TEMP_OUT: 1 = Temperature written to FIFO at the sample rate

	// I2C master control bits
	static constexpr uint8_t I2C_MST_CTRL_BIT_I2C_MST_CLK_0 = (1 << 0); // I2C_MST_CLK <0>: I2C master clock bit 0
	static constexpr uint8_t I2C_MST_CTRL_BIT_I2C_MST_CLK_1 = (1 << 1); // I2C_MST_CLK <1>: I2C master clock bit 1
	static constexpr uint8_t I2C_MST_CTRL_BIT_I2C_MST_CLK_2 = (1 << 2); // I2C_MST_CLK <2>: I2C master clock bit 2
	static constexpr uint8_t I2C_MST_CTRL_BIT_I2C_MST_CLK_3 = (1 << 3); // I2C_MST_CLK <3>: I2C master clock bit 3
	static constexpr uint8_t I2C_MST_CTRL_BIT_I2C_MST_P_NSR = (1 << 4); // I2C_MST_P_NSR: 1 = Stop between slave reads, 0 = Restart
	static constexpr uint8_t I2C_MST_CTRL_BIT_SLV_3_FIFO_EN = (1 << 5); // SLV_3_FIFO_EN: 1 = Slave 3 data written to FIFO, 0 = Disabled
	static constexpr uint8_t I2C_MST_CTRL_BIT_WAIT_FOR_ES = (1 << 6);	// WAIT_FOR_ES: 1 = Data ready interrupt delayed until external sensor data loaded
	static constexpr uint8_t I2C_MST_CTRL_BIT_MULT_MST_EN = (1 << 7);	// MULT_MST_EN: 1 = Multi-master capability enabled

	// I2C master clock values
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_BITS_MASK = I2C_MST_CTRL_BIT_I2C_MST_CLK_3 |
																  I2C_MST_CTRL_BIT_I2C_MST_CLK_2 |
																  I2C_MST_CTRL_BIT_I2C_MST_CLK_1 |
																  I2C_MST_CTRL_BIT_I2C_MST_CLK_0;
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_348KHZ = 0;																								 // 0000: 348 kHz (divider 23)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_333KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_0;																	 // 0001: 333 kHz (divider 24)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_320KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_1;																	 // 0010: 320 kHz (divider 25)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_308KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_1 | I2C_MST_CTRL_BIT_I2C_MST_CLK_0;									 // 0011: 308 kHz (divider 26)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_296KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_2;																	 // 0100: 296 kHz (divider 27)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_286KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_2 | I2C_MST_CTRL_BIT_I2C_MST_CLK_0;									 // 0101: 286 kHz (divider 28)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_276KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_2 | I2C_MST_CTRL_BIT_I2C_MST_CLK_1;									 // 0110: 276 kHz (divider 29)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_267KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_2 | I2C_MST_CTRL_BIT_I2C_MST_CLK_1 | I2C_MST_CTRL_BIT_I2C_MST_CLK_0; // 0111: 267 kHz (divider 30)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_258KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_3;																	 // 1000: 258 kHz (divider 31)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_500KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_3 | I2C_MST_CTRL_BIT_I2C_MST_CLK_0;									 // 1001: 500 kHz (divider 16)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_471KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_3 | I2C_MST_CTRL_BIT_I2C_MST_CLK_1;									 // 1010: 471 kHz (divider 17)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_444KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_3 | I2C_MST_CTRL_BIT_I2C_MST_CLK_1 | I2C_MST_CTRL_BIT_I2C_MST_CLK_0; // 1011: 444 kHz (divider 18)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_421KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_3 | I2C_MST_CTRL_BIT_I2C_MST_CLK_2;									 // 1100: 421 kHz (divider 19)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_400KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_3 | I2C_MST_CTRL_BIT_I2C_MST_CLK_2 | I2C_MST_CTRL_BIT_I2C_MST_CLK_0; // 1101: 400 kHz (divider 20)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_381KHZ = I2C_MST_CTRL_BIT_I2C_MST_CLK_3 | I2C_MST_CTRL_BIT_I2C_MST_CLK_2 | I2C_MST_CTRL_BIT_I2C_MST_CLK_1; // 1110: 381 kHz (divider 21)
	static constexpr uint8_t I2C_MST_CTRL_I2C_MST_CLK_364KHZ = I2C_MST_CTRL_I2C_MST_CLK_BITS_MASK;																 // 1111: 364 kHz (divider 22)

	// I2C slave 0 address bits
	static constexpr uint8_t I2C_SLV0_ADDR_BITS_I2C_ID_0 = 0b1111111;	// I2C_ID_0: Slave 0 I2C address
	static constexpr uint8_t I2C_SLV0_ADDR_BIT_I2C_SLV0_RNW = (1 << 7); // I2C_SLV0_RNW: 1 = Read, 0 = Write

	// I2C slave 0 control bits
	static constexpr uint8_t I2C_SLV0_CTRL_BITS_I2C_SLV0_LENG = 0b1111;		// I2C_SLV0_LENG: Number of bytes to read
	static constexpr uint8_t I2C_SLV0_CTRL_BIT_I2C_SLV0_GRP = (1 << 4);		// I2C_SLV0_GRP: 1 = Registers 1 and 2 grouped, 0 = Registers 0 and 1 grouped
	static constexpr uint8_t I2C_SLV0_CTRL_BIT_I2C_SLV0_REG_DIS = (1 << 5); // I2C_SLV0_REG_DIS: 1 = Transfer data only, no register address
	static constexpr uint8_t I2C_SLV0_CTRL_BIT_I2C_SLV0_BYTE_SW = (1 << 6); // I2C_SLV0_BYTE_SW: 1 = Swap bytes of each word
	static constexpr uint8_t I2C_SLV0_CTRL_BIT_I2C_SLV0_EN = (1 << 7);		// I2C_SLV0_EN: 1 = Read from slave 0 at the sample rate

	// I2C slave 1 address bits
	static constexpr uint8_t I2C_SLV1_ADDR_BITS_I2C_ID_1 = 0b1111111;	// I2C_ID_1: Slave 1 I2C address
	static constexpr uint8_t I2C_SLV1_ADDR_BIT_I2C_SLV1_RNW = (1 << 7); // I2C_SLV1_RNW: 1 = Read, 0 = Write

	// I2C slave 1 control bits
	static constexpr uint8_t I2C_SLV1_CTRL_BITS_I2C_SLV1_LENG = 0b1111;		// I2C_SLV1_LENG: Number of bytes to read
	static constexpr uint8_t I2C_SLV1_CTRL_BIT_I2C_SLV1_GRP = (1 << 4);		// I2C_SLV1_GRP: 1 = Registers 1 and 2 grouped, 0 = Registers 0 and 1 grouped
	static constexpr uint8_t I2C_SLV1_CTRL_BIT_I2C_SLV1_REG_DIS = (1 << 5); // I2C_SLV1_REG_DIS: 1 = Transfer data only, no register address
	static constexpr uint8_t I2C_SLV1_CTRL_BIT_I2C_SLV1_BYTE_SW = (1 << 6); // I2C_SLV1_BYTE_SW: 1 = Swap bytes of each word
	static constexpr uint8_t I2C_SLV1_CTRL_BIT_I2C_SLV1_EN = (1 << 7);		// I2C_SLV1_EN: 1 = Read from slave 1 at the sample rate

	// I2C slave 2 address bits
	static constexpr uint8_t I2C_SLV2_ADDR_BITS_I2C_ID_2 = 0b1111111;	// I2C_ID_2: Slave 2 I2C address
	static constexpr uint8_t I2C_SLV2_ADDR_BIT_I2C_SLV2_RNW = (1 << 7); // I2C_SLV2_RNW: 1 = Read, 0 = Write

	// I2C slave 2 control bits
	static constexpr uint8_t I2C_SLV2_CTRL_BITS_I2C_SLV2_LENG = 0b1111;		// I2C_SLV2_LENG: Number of bytes to read
	static constexpr uint8_t I2C_SLV2_CTRL_BIT_I2C_SLV2_GRP = (1 << 4);		// I2C_SLV2_GRP: 1 = Registers 1 and 2 grouped, 0 = Registers 0 and 1 grouped
	static constexpr uint8_t I2C_SLV2_CTRL_BIT_I2C_SLV2_REG_DIS = (1 << 5); // I2C_SLV2_REG_DIS: 1 = Transfer data only, no register address
	static constexpr uint8_t I2C_SLV2_CTRL_BIT_I2C_SLV2_BYTE_SW = (1 << 6); // I2C_SLV2_BYTE_SW: 1 = Swap bytes of each word
	static constexpr uint8_t I2C_SLV2_CTRL_BIT_I2C_SLV2_EN = (1 << 7);		// I2C_SLV2_EN: 1 = Read from slave 2 at the sample rate

	// I2C slave 3 address bits
	static constexpr uint8_t I2C_SLV3_ADDR_BITS_I2C_ID_3 = 0b1111111;	// I2C_ID_3: Slave 3 I2C address
	static constexpr uint8_t I2C_SLV3_ADDR_BIT_I2C_SLV3_RNW = (1 << 7); // I2C_SLV3_RNW: 1 = Read, 0 = Write

	// I2C slave 3 control bits
	static constexpr uint8_t I2C_SLV3_CTRL_BITS_I2C_SLV3_LENG = 0b1111;		// I2C_SLV3_LENG: Number of bytes to read
	static constexpr uint8_t I2C_SLV3_CTRL_BIT_I2C_SLV3_GRP = (1 << 4);		// I2C_SLV3_GRP: 1 = Registers 1 and 2 grouped, 0 = Registers 0 and 1 grouped
	static constexpr uint8_t I2C_SLV3_CTRL_BIT_I2C_SLV3_REG_DIS = (1 << 5); // I2C_SLV3_REG_DIS: 1 = Transfer data only, no register address (register map prints I2C_SLV0_REG_DIS)
	static constexpr uint8_t I2C_SLV3_CTRL_BIT_I2C_SLV3_BYTE_SW = (1 << 6); // I2C_SLV3_BYTE_SW: 1 = Swap bytes of each word
	static constexpr uint8_t I2C_SLV3_CTRL_BIT_I2C_SLV3_EN = (1 << 7);		// I2C_SLV3_EN: 1 = Read from slave 3 at the sample rate

	// I2C slave 4 address bits
	static constexpr uint8_t I2C_SLV4_ADDR_BITS_I2C_ID_4 = 0b1111111;	// I2C_ID_4: Slave 4 I2C address
	static constexpr uint8_t I2C_SLV4_ADDR_BIT_I2C_SLV4_RNW = (1 << 7); // I2C_SLV4_RNW: 1 = Read, 0 = Write

	// I2C slave 4 control bits
	static constexpr uint8_t I2C_SLV4_CTRL_BITS_I2C_MST_DLY = 0b11111;		// I2C_MST_DLY: Delayed slaves accessed every (1 + I2C_MST_DLY) samples
	static constexpr uint8_t I2C_SLV4_CTRL_BIT_I2C_SLV4_REG_DIS = (1 << 5); // I2C_SLV4_REG_DIS: 1 = Transfer data only, no register address
	static constexpr uint8_t I2C_SLV4_CTRL_BIT_SLV4_DONE_INT_EN = (1 << 6); // SLV4_DONE_INT_EN: 1 = Slave 4 transfer complete causes an interrupt
	static constexpr uint8_t I2C_SLV4_CTRL_BIT_I2C_SLV4_EN = (1 << 7);		// I2C_SLV4_EN: 1 = Start single transfer, cleared when complete

	// I2C master status bits
	static constexpr uint8_t I2C_MST_STATUS_BIT_I2C_SLV0_NACK = (1 << 0); // I2C_SLV0_NACK: 1 = Slave 0 received a NACK
	static constexpr uint8_t I2C_MST_STATUS_BIT_I2C_SLV1_NACK = (1 << 1); // I2C_SLV1_NACK: 1 = Slave 1 received a NACK
	static constexpr uint8_t I2C_MST_STATUS_BIT_I2C_SLV2_NACK = (1 << 2); // I2C_SLV2_NACK: 1 = Slave 2 received a NACK
	static constexpr uint8_t I2C_MST_STATUS_BIT_I2C_SLV3_NACK = (1 << 3); // I2C_SLV3_NACK: 1 = Slave 3 received a NACK
	static constexpr uint8_t I2C_MST_STATUS_BIT_I2C_SLV4_NACK = (1 << 4); // I2C_SLV4_NACK: 1 = Slave 4 received a NACK
	static constexpr uint8_t I2C_MST_STATUS_BIT_I2C_LOST_ARB = (1 << 5);  // I2C_LOST_ARB: 1 = Lost arbitration of the aux I2C bus
	static constexpr uint8_t I2C_MST_STATUS_BIT_I2C_SLV4_DONE = (1 << 6); // I2C_SLV4_DONE: 1 = Slave 4 transfer complete
	static constexpr uint8_t I2C_MST_STATUS_BIT_PASS_THROUGH = (1 << 7);  // PASS_THROUGH: 1 = FSYNC interrupt occurred

	// INT pin / bypass enable configuration bits
	static constexpr uint8_t INT_PIN_CFG_BIT_RESERVED_0 = (1 << 0);		   // Reserved
	static constexpr uint8_t INT_PIN_CFG_BIT_BYPASS_EN = (1 << 1);		   // BYPASS_EN: 1 = Aux I2C pins in bypass mode (when I2C master disabled)
	static constexpr uint8_t INT_PIN_CFG_BIT_FSYNC_INT_MODE_EN = (1 << 2); // FSYNC_INT_MODE_EN: 1 = FSYNC pin used as an interrupt, 0 = Disabled
	static constexpr uint8_t INT_PIN_CFG_BIT_ACTL_FSYNC = (1 << 3);		   // ACTL_FSYNC: 1 = FSYNC interrupt active low, 0 = Active high
	static constexpr uint8_t INT_PIN_CFG_BIT_INT_ANYRD_2CLEAR = (1 << 4);  // INT_ANYRD_2CLEAR: 1 = Status cleared by any read, 0 = Cleared only by reading INT_STATUS
	static constexpr uint8_t INT_PIN_CFG_BIT_LATCH_INT_EN = (1 << 5);	   // LATCH_INT_EN: 1 = INT held until status cleared, 0 = 50 µs pulse
	static constexpr uint8_t INT_PIN_CFG_BIT_OPEN = (1 << 6);			   // OPEN: 1 = INT open drain, 0 = Push-pull
	static constexpr uint8_t INT_PIN_CFG_BIT_ACTL = (1 << 7);			   // ACTL: 1 = INT active low, 0 = Active high

	// Interrupt enable bits
	static constexpr uint8_t INT_ENABLE_BIT_RAW_RDY_EN = (1 << 0);		 // RAW_RDY_EN: 1 = Raw sensor data ready interrupt to INT, 0 = Disabled
	static constexpr uint8_t INT_ENABLE_BIT_RESERVED_1 = (1 << 1);		 // Reserved
	static constexpr uint8_t INT_ENABLE_BIT_RESERVED_2 = (1 << 2);		 // Reserved
	static constexpr uint8_t INT_ENABLE_BIT_FSYNC_INT_EN = (1 << 3);	 // FSYNC_INT_EN: 1 = FSYNC interrupt to INT, 0 = Disabled
	static constexpr uint8_t INT_ENABLE_BIT_FIFO_OVERFLOW_EN = (1 << 4); // FIFO_OVERFLOW_EN: 1 = FIFO overflow interrupt to INT, 0 = Disabled
	static constexpr uint8_t INT_ENABLE_BIT_RESERVED_5 = (1 << 5);		 // Reserved
	static constexpr uint8_t INT_ENABLE_BIT_WOM_EN = (1 << 6);			 // WOM_EN: 1 = Wake on motion interrupt to INT, 0 = Disabled
	static constexpr uint8_t INT_ENABLE_BIT_RESERVED_7 = (1 << 7);		 // Reserved

	// Interrupt status bits
	static constexpr uint8_t INT_STATUS_BIT_RAW_DATA_RDY_INT = (1 << 0);  // RAW_DATA_RDY_INT: 1 = Sensor data updated and ready to read
	static constexpr uint8_t INT_STATUS_BIT_DMP_INT = (1 << 1);			  // DMP_INT: 1 = DMP interrupt generated
	static constexpr uint8_t INT_STATUS_BIT_RESERVED_2 = (1 << 2);		  // Reserved
	static constexpr uint8_t INT_STATUS_BIT_FSYNC_INT = (1 << 3);		  // FSYNC_INT: 1 = FSYNC interrupt occurred
	static constexpr uint8_t INT_STATUS_BIT_FIFO_OVERFLOW_INT = (1 << 4); // FIFO_OVERFLOW_INT: 1 = FIFO overflow occurred (oldest data dropped)
	static constexpr uint8_t INT_STATUS_BIT_RESERVED_5 = (1 << 5);		  // Reserved
	static constexpr uint8_t INT_STATUS_BIT_WOM_INT = (1 << 6);			  // WOM_INT: 1 = Wake on motion interrupt occurred
	static constexpr uint8_t INT_STATUS_BIT_RESERVED_7 = (1 << 7);		  // Reserved

	// I2C master delay control bits
	static constexpr uint8_t I2C_MST_DELAY_CTRL_BIT_I2C_SLV0_DLY_EN = (1 << 0); // I2C_SLV0_DLY_EN: 1 = Slave 0 accessed every (1 + I2C_MST_DLY) samples
	static constexpr uint8_t I2C_MST_DELAY_CTRL_BIT_I2C_SLV1_DLY_EN = (1 << 1); // I2C_SLV1_DLY_EN: 1 = Slave 1 accessed every (1 + I2C_MST_DLY) samples
	static constexpr uint8_t I2C_MST_DELAY_CTRL_BIT_I2C_SLV2_DLY_EN = (1 << 2); // I2C_SLV2_DLY_EN: 1 = Slave 2 accessed every (1 + I2C_MST_DLY) samples
	static constexpr uint8_t I2C_MST_DELAY_CTRL_BIT_I2C_SLV3_DLY_EN = (1 << 3); // I2C_SLV3_DLY_EN: 1 = Slave 3 accessed every (1 + I2C_MST_DLY) samples
	static constexpr uint8_t I2C_MST_DELAY_CTRL_BIT_I2C_SLV4_DLY_EN = (1 << 4); // I2C_SLV4_DLY_EN: 1 = Slave 4 accessed every (1 + I2C_MST_DLY) samples
	static constexpr uint8_t I2C_MST_DELAY_CTRL_BIT_RESERVED_5 = (1 << 5);		// Reserved
	static constexpr uint8_t I2C_MST_DELAY_CTRL_BIT_RESERVED_6 = (1 << 6);		// Reserved
	static constexpr uint8_t I2C_MST_DELAY_CTRL_BIT_DELAY_ES_SHADOW = (1 << 7); // DELAY_ES_SHADOW: 1 = External sensor data shadowed only when all received

	// Signal path reset bits
	static constexpr uint8_t SIGNAL_PATH_RESET_BIT_TEMP_RST = (1 << 0);	  // TEMP_RST: 1 = Reset temperature signal path (sensor registers not cleared)
	static constexpr uint8_t SIGNAL_PATH_RESET_BIT_ACCEL_RST = (1 << 1);  // ACCEL_RST: 1 = Reset accel signal path (sensor registers not cleared)
	static constexpr uint8_t SIGNAL_PATH_RESET_BIT_GYRO_RST = (1 << 2);	  // GYRO_RST: 1 = Reset gyro signal path (sensor registers not cleared)
	static constexpr uint8_t SIGNAL_PATH_RESET_BIT_RESERVED_3 = (1 << 3); // Reserved
	static constexpr uint8_t SIGNAL_PATH_RESET_BIT_RESERVED_4 = (1 << 4); // Reserved
	static constexpr uint8_t SIGNAL_PATH_RESET_BIT_RESERVED_5 = (1 << 5); // Reserved
	static constexpr uint8_t SIGNAL_PATH_RESET_BIT_RESERVED_6 = (1 << 6); // Reserved
	static constexpr uint8_t SIGNAL_PATH_RESET_BIT_RESERVED_7 = (1 << 7); // Reserved

	// Accelerometer interrupt control bits
	static constexpr uint8_t ACCEL_INTEL_CTRL_BIT_RESERVED_0 = (1 << 0);	   // Reserved
	static constexpr uint8_t ACCEL_INTEL_CTRL_BIT_RESERVED_1 = (1 << 1);	   // Reserved
	static constexpr uint8_t ACCEL_INTEL_CTRL_BIT_RESERVED_2 = (1 << 2);	   // Reserved
	static constexpr uint8_t ACCEL_INTEL_CTRL_BIT_RESERVED_3 = (1 << 3);	   // Reserved
	static constexpr uint8_t ACCEL_INTEL_CTRL_BIT_RESERVED_4 = (1 << 4);	   // Reserved
	static constexpr uint8_t ACCEL_INTEL_CTRL_BIT_RESERVED_5 = (1 << 5);	   // Reserved
	static constexpr uint8_t ACCEL_INTEL_CTRL_BIT_ACCEL_INTEL_MODE = (1 << 6); // ACCEL_INTEL_MODE: 1 = Compare the current sample with the previous sample
	static constexpr uint8_t ACCEL_INTEL_CTRL_BIT_ACCEL_INTEL_EN = (1 << 7);   // ACCEL_INTEL_EN: 1 = Wake-on-motion detection logic enabled

	// User control bits
	static constexpr uint8_t USER_CTRL_BIT_SIG_COND_RST = (1 << 0); // SIG_COND_RST: 1 = Reset all sensor signal paths and clear sensor registers
	static constexpr uint8_t USER_CTRL_BIT_I2C_MST_RST = (1 << 1);	// I2C_MST_RST: 1 = Reset I2C master (only when hung), auto-cleared
	static constexpr uint8_t USER_CTRL_BIT_FIFO_RST = (1 << 2);		// FIFO_RST: 1 = Reset FIFO, auto-cleared
	static constexpr uint8_t USER_CTRL_BIT_DMP_RST = (1 << 3);		// DMP_RST: 1 = Reset DMP (when DMP_EN = 0), auto-cleared
	static constexpr uint8_t USER_CTRL_BIT_I2C_IF_DIS = (1 << 4);	// I2C_IF_DIS: 1 = Disable I2C slave, SPI only, auto-cleared. Never set when using I2C
	static constexpr uint8_t USER_CTRL_BIT_I2C_MST_EN = (1 << 5);	// I2C_MST_EN: 1 = I2C master enabled (aux pins isolated), 0 = Aux pins driven by SDA/SCL
	static constexpr uint8_t USER_CTRL_BIT_FIFO_EN = (1 << 6);		// FIFO_EN: 1 = FIFO operation enabled
	static constexpr uint8_t USER_CTRL_BIT_DMP_EN = (1 << 7);		// DMP_EN: 1 = DMP features enabled

	// Power management 1 bits
	static constexpr uint8_t PWR_MGMT_1_BIT_CLKSEL_0 = (1 << 0);	 // CLKSEL <0>: Clock source select bit 0
	static constexpr uint8_t PWR_MGMT_1_BIT_CLKSEL_1 = (1 << 1);	 // CLKSEL <1>: Clock source select bit 1
	static constexpr uint8_t PWR_MGMT_1_BIT_CLKSEL_2 = (1 << 2);	 // CLKSEL <2>: Clock source select bit 2
	static constexpr uint8_t PWR_MGMT_1_BIT_TEMP_DIS = (1 << 3);	 // TEMP_DIS: 1 = Temperature sensor disabled, 0 = Enabled
	static constexpr uint8_t PWR_MGMT_1_BIT_GYRO_STANDBY = (1 << 4); // GYRO_STANDBY: 1 = Gyro drive and PLL enabled, sense paths disabled
	static constexpr uint8_t PWR_MGMT_1_BIT_CYCLE = (1 << 5);		 // CYCLE: 1 = Cycle between sleep and a single sample (when SLEEP and GYRO_STANDBY are 0)
	static constexpr uint8_t PWR_MGMT_1_BIT_SLEEP = (1 << 6);		 // SLEEP: 1 = Sleep mode, 0 = Awake
	static constexpr uint8_t PWR_MGMT_1_BIT_DEVICE_RESET = (1 << 7); // DEVICE_RESET: 1 = Reset all registers to default, auto-cleared

	// Power management 2 bits
	static constexpr uint8_t PWR_MGMT_2_BIT_DISABLE_ZG = (1 << 0);	   // DISABLE_ZG: 1 = Z gyro disabled, 0 = On
	static constexpr uint8_t PWR_MGMT_2_BIT_DISABLE_YG = (1 << 1);	   // DISABLE_YG: 1 = Y gyro disabled, 0 = On
	static constexpr uint8_t PWR_MGMT_2_BIT_DISABLE_XG = (1 << 2);	   // DISABLE_XG: 1 = X gyro disabled, 0 = On
	static constexpr uint8_t PWR_MGMT_2_BIT_DISABLE_ZA = (1 << 3);	   // DISABLE_ZA: 1 = Z accelerometer disabled, 0 = On
	static constexpr uint8_t PWR_MGMT_2_BIT_DISABLE_YA = (1 << 4);	   // DISABLE_YA: 1 = Y accelerometer disabled, 0 = On
	static constexpr uint8_t PWR_MGMT_2_BIT_DISABLE_XA = (1 << 5);	   // DISABLE_XA: 1 = X accelerometer disabled, 0 = On
	static constexpr uint8_t PWR_MGMT_2_BIT_LP_WAKE_CTRL_0 = (1 << 6); // LP_WAKE_CTRL <0>: Accel-only low power wake-up frequency bit 0 (MPU-6050 compatible mode)
	static constexpr uint8_t PWR_MGMT_2_BIT_LP_WAKE_CTRL_1 = (1 << 7); // LP_WAKE_CTRL <1>: Accel-only low power wake-up frequency bit 1 (MPU-6050 compatible mode)

	// Power management 2 values
	static constexpr uint8_t PWR_MGMT_2_DISABLE_GYRO = PWR_MGMT_2_BIT_DISABLE_XG | PWR_MGMT_2_BIT_DISABLE_YG | PWR_MGMT_2_BIT_DISABLE_ZG;  // All gyro axes disabled
	static constexpr uint8_t PWR_MGMT_2_DISABLE_ACCEL = PWR_MGMT_2_BIT_DISABLE_XA | PWR_MGMT_2_BIT_DISABLE_YA | PWR_MGMT_2_BIT_DISABLE_ZA; // All accelerometer axes disabled

	// FIFO count high byte bits
	static constexpr uint8_t FIFO_COUNT_H_BITS_FIFO_CNT = 0b11111;	 // FIFO_CNT (high byte of the count)
	static constexpr uint8_t FIFO_COUNT_H_BIT_RESERVED_5 = (1 << 5); // Reserved
	static constexpr uint8_t FIFO_COUNT_H_BIT_RESERVED_6 = (1 << 6); // Reserved
	static constexpr uint8_t FIFO_COUNT_H_BIT_RESERVED_7 = (1 << 7); // Reserved

	// X accel offset low byte bits
	static constexpr uint8_t XA_OFFSET_L_BIT_RESERVED_0 = (1 << 0); // Reserved
	static constexpr uint8_t XA_OFFSET_L_BITS_XA_OFFS = 0b11111110; // XA_OFFS

	// Y accel offset low byte bits
	static constexpr uint8_t YA_OFFSET_L_BIT_RESERVED_0 = (1 << 0); // Reserved
	static constexpr uint8_t YA_OFFSET_L_BITS_YA_OFFS = 0b11111110; // YA_OFFS

	// Z accel offset low byte bits
	static constexpr uint8_t ZA_OFFSET_L_BIT_RESERVED_0 = (1 << 0); // Reserved
	static constexpr uint8_t ZA_OFFSET_L_BITS_ZA_OFFS = 0b11111110; // ZA_OFFS

	// Accel offset field positions (used to combine / split the H and L registers)
	static constexpr uint8_t XA_OFFSET_L_XA_OFFS_SHIFT = 1; // XA_OFFS starts at bit 1
	static constexpr uint8_t YA_OFFSET_L_YA_OFFS_SHIFT = 1; // YA_OFFS starts at bit 1
	static constexpr uint8_t ZA_OFFSET_L_ZA_OFFS_SHIFT = 1; // ZA_OFFS starts at bit 1

	static constexpr uint8_t START_UP_TIME_MILLIS = 35; // Longest sensor start-up time from sleep mode (gyroscope)
	static constexpr uint8_t RESET_TIME_MILLIS = 100;	// Time for a device reset to complete

	static constexpr uint8_t WOM_THRESHOLD_MG_PER_LSB = 4; // Wake-on-motion threshold resolution
	static constexpr uint16_t WOM_THRESHOLD_MAX_MG = 1020; // Wake-on-motion threshold maximum

	static constexpr uint8_t DATA_REGISTER = REG_ACCEL_XOUT_H;						 // First data register (accel X high byte)
	static constexpr uint8_t DATA_LENGTH = (REG_GYRO_ZOUT_L - REG_ACCEL_XOUT_H) + 1; // Number of data registers (accel X to gyro Z)

	// Data scaling. Fixed point factors avoid a division for every sample
	static constexpr uint16_t MG_PER_G = 1000;			   // Accel is scaled from g to mg
	static constexpr uint8_t SCALE_SHIFT = 8;			   // Fixed point shift used by the gyro and temperature factors
	static constexpr uint8_t ACCEL_SHIFT_2G = 14;		   // 16384 LSB/g
	static constexpr uint8_t ACCEL_SHIFT_4G = 13;		   // 8192 LSB/g
	static constexpr uint8_t ACCEL_SHIFT_8G = 12;		   // 4096 LSB/g
	static constexpr uint8_t ACCEL_SHIFT_16G = 11;		   // 2048 LSB/g
	static constexpr uint16_t GYRO_FACTOR_250DPS = 1954;   // 131 LSB/dps (1000 x 256 / 131)
	static constexpr uint16_t GYRO_FACTOR_500DPS = 3908;   // 65.5 LSB/dps (1000 x 256 / 65.5)
	static constexpr uint16_t GYRO_FACTOR_1000DPS = 7805;  // 32.8 LSB/dps (1000 x 256 / 32.8)
	static constexpr uint16_t GYRO_FACTOR_2000DPS = 15610; // 16.4 LSB/dps (1000 x 256 / 16.4)
	static constexpr uint16_t TEMPERATURE_FACTOR = 767;	   // 333.87 LSB/°C (1000 x 256 / 333.87)
	static constexpr uint16_t TEMPERATURE_OFFSET = 21000;  // Room temperature offset (21 °C in m°C)

	static constexpr uint8_t EXT_SENS_DATA_COUNT = 24; // Number of EXT_SENS_DATA registers (00 to 23)

	static constexpr uint16_t SLAVE4_TRANSFER_TIMEOUT_MILLIS = 300; // Slave 4 transfers happen at the sample rate (slowest 1 kHz / 256 = 3.9 Hz)

	// Self-test configuration, SMPLRT_DIV to ACCEL_CONFIG_2 (from InvenSense app note AN-MPU-6500A-02)
	static constexpr uint8_t SELF_TEST_CONFIG[] = {
		0,							   // SMPLRT_DIV: 1 kHz sample rate
		CONFIG_DLPF_CFG_92HZ,		   // CONFIG: Gyro 92 Hz filter
		GYRO_CONFIG_GYRO_FS_SEL_250,   // GYRO_CONFIG: ±250 dps, self-test off
		ACCEL_CONFIG_ACCEL_FS_SEL_2G,  // ACCEL_CONFIG: ±2 g, self-test off
		ACCEL_CONFIG_2_A_DLPF_CFG_92HZ // ACCEL_CONFIG_2: Accel 92 Hz filter
	};

	// Self-test enabled, GYRO_CONFIG and ACCEL_CONFIG
	static constexpr uint8_t SELF_TEST_ENABLE[] = {
		GYRO_CONFIG_BIT_XG_ST | GYRO_CONFIG_BIT_YG_ST | GYRO_CONFIG_BIT_ZG_ST | GYRO_CONFIG_GYRO_FS_SEL_250,	// GYRO_CONFIG: ±250 dps, self-test on
		ACCEL_CONFIG_BIT_XA_ST | ACCEL_CONFIG_BIT_YA_ST | ACCEL_CONFIG_BIT_ZA_ST | ACCEL_CONFIG_ACCEL_FS_SEL_2G // ACCEL_CONFIG: ±2 g, self-test on
	};

	// The self-test configuration is written in one transfer, in register order
	static_assert((REG_CONFIG == (REG_SMPLRT_DIV + 1)) &&
					  (REG_GYRO_CONFIG == (REG_SMPLRT_DIV + 2)) &&
					  (REG_ACCEL_CONFIG == (REG_SMPLRT_DIV + 3)) &&
					  (REG_ACCEL_CONFIG_2 == (REG_SMPLRT_DIV + 4)),
				  "SMPLRT_DIV to ACCEL_CONFIG_2 must be contiguous");
	static_assert(sizeof(SELF_TEST_CONFIG) == ((REG_ACCEL_CONFIG_2 - REG_SMPLRT_DIV) + 1),
				  "SELF_TEST_CONFIG must hold one value for each register from SMPLRT_DIV to ACCEL_CONFIG_2");
	static_assert(sizeof(SELF_TEST_ENABLE) == ((REG_ACCEL_CONFIG - REG_GYRO_CONFIG) + 1),
				  "SELF_TEST_ENABLE must hold one value for GYRO_CONFIG and one for ACCEL_CONFIG");

	static constexpr uint8_t SELF_TEST_SAMPLE_COUNT = 200;		 // Samples averaged with and without self-test enabled
	static constexpr uint8_t SELF_TEST_SAMPLE_PERIOD_MILLIS = 1; // Test sample rate is 1 kHz
	static constexpr uint8_t SELF_TEST_SETTLE_TIME_MILLIS = 20;	 // Time for the output to settle after a configuration change

	// Self-test response limits (app note limits for a working sensor), in LSB at ±250 dps and ±2 g
	static constexpr uint8_t GYRO_LSB_PER_DPS_250DPS = 131;																				   // Gyro sensitivity at ±250 dps
	static constexpr uint8_t SELF_TEST_GYRO_MIN_DPS = 60;																				   // Minimum gyro response
	static constexpr uint16_t SELF_TEST_ACCEL_MIN_MG = 225;																				   // Minimum accel response
	static constexpr uint16_t SELF_TEST_ACCEL_MAX_MG = 675;																				   // Maximum accel response
	static constexpr uint16_t SELF_TEST_GYRO_MIN_RESPONSE = SELF_TEST_GYRO_MIN_DPS * GYRO_LSB_PER_DPS_250DPS;							   // 7860 LSB
	static constexpr uint16_t SELF_TEST_ACCEL_MIN_RESPONSE = (static_cast<uint32_t>(SELF_TEST_ACCEL_MIN_MG) << ACCEL_SHIFT_2G) / MG_PER_G; // 3686 LSB
	static constexpr uint16_t SELF_TEST_ACCEL_MAX_RESPONSE = (static_cast<uint32_t>(SELF_TEST_ACCEL_MAX_MG) << ACCEL_SHIFT_2G) / MG_PER_G; // 11059 LSB

	// Self-test temperature limits (the datasheet's operating range), in m°C
	static constexpr int32_t SELF_TEST_TEMPERATURE_MIN = -40000; // -40 °C
	static constexpr uint32_t SELF_TEST_TEMPERATURE_MAX = 85000; // 85 °C

	// Registers and bits for slaves 0 to 3 (no default values, so instances can be constexpr within the class)
	struct SlaveRegisters final
	{
		uint8_t regAddr;	   // I2C_SLVx_ADDR register
		uint8_t regReg;		   // I2C_SLVx_REG register
		uint8_t regCtrl;	   // I2C_SLVx_CTRL register
		uint8_t regDo;		   // I2C_SLVx_DO register
		uint8_t addrBitsId;	   // I2C_ID_x bits
		uint8_t addrBitRnw;	   // I2C_SLVx_RNW bit
		uint8_t ctrlBitsLeng;  // I2C_SLVx_LENG bits
		uint8_t ctrlBitGrp;	   // I2C_SLVx_GRP bit
		uint8_t ctrlBitRegDis; // I2C_SLVx_REG_DIS bit
		uint8_t ctrlBitByteSw; // I2C_SLVx_BYTE_SW bit
		uint8_t ctrlBitEn;	   // I2C_SLVx_EN bit
	};

	static constexpr SlaveRegisters SLAVE0_REGISTERS = {
		.regAddr = REG_I2C_SLV0_ADDR,
		.regReg = REG_I2C_SLV0_REG,
		.regCtrl = REG_I2C_SLV0_CTRL,
		.regDo = REG_I2C_SLV0_DO,
		.addrBitsId = I2C_SLV0_ADDR_BITS_I2C_ID_0,
		.addrBitRnw = I2C_SLV0_ADDR_BIT_I2C_SLV0_RNW,
		.ctrlBitsLeng = I2C_SLV0_CTRL_BITS_I2C_SLV0_LENG,
		.ctrlBitGrp = I2C_SLV0_CTRL_BIT_I2C_SLV0_GRP,
		.ctrlBitRegDis = I2C_SLV0_CTRL_BIT_I2C_SLV0_REG_DIS,
		.ctrlBitByteSw = I2C_SLV0_CTRL_BIT_I2C_SLV0_BYTE_SW,
		.ctrlBitEn = I2C_SLV0_CTRL_BIT_I2C_SLV0_EN,
	};

	static constexpr SlaveRegisters SLAVE1_REGISTERS = {
		.regAddr = REG_I2C_SLV1_ADDR,
		.regReg = REG_I2C_SLV1_REG,
		.regCtrl = REG_I2C_SLV1_CTRL,
		.regDo = REG_I2C_SLV1_DO,
		.addrBitsId = I2C_SLV1_ADDR_BITS_I2C_ID_1,
		.addrBitRnw = I2C_SLV1_ADDR_BIT_I2C_SLV1_RNW,
		.ctrlBitsLeng = I2C_SLV1_CTRL_BITS_I2C_SLV1_LENG,
		.ctrlBitGrp = I2C_SLV1_CTRL_BIT_I2C_SLV1_GRP,
		.ctrlBitRegDis = I2C_SLV1_CTRL_BIT_I2C_SLV1_REG_DIS,
		.ctrlBitByteSw = I2C_SLV1_CTRL_BIT_I2C_SLV1_BYTE_SW,
		.ctrlBitEn = I2C_SLV1_CTRL_BIT_I2C_SLV1_EN,
	};

	static constexpr SlaveRegisters SLAVE2_REGISTERS = {
		.regAddr = REG_I2C_SLV2_ADDR,
		.regReg = REG_I2C_SLV2_REG,
		.regCtrl = REG_I2C_SLV2_CTRL,
		.regDo = REG_I2C_SLV2_DO,
		.addrBitsId = I2C_SLV2_ADDR_BITS_I2C_ID_2,
		.addrBitRnw = I2C_SLV2_ADDR_BIT_I2C_SLV2_RNW,
		.ctrlBitsLeng = I2C_SLV2_CTRL_BITS_I2C_SLV2_LENG,
		.ctrlBitGrp = I2C_SLV2_CTRL_BIT_I2C_SLV2_GRP,
		.ctrlBitRegDis = I2C_SLV2_CTRL_BIT_I2C_SLV2_REG_DIS,
		.ctrlBitByteSw = I2C_SLV2_CTRL_BIT_I2C_SLV2_BYTE_SW,
		.ctrlBitEn = I2C_SLV2_CTRL_BIT_I2C_SLV2_EN,
	};

	static constexpr SlaveRegisters SLAVE3_REGISTERS = {
		.regAddr = REG_I2C_SLV3_ADDR,
		.regReg = REG_I2C_SLV3_REG,
		.regCtrl = REG_I2C_SLV3_CTRL,
		.regDo = REG_I2C_SLV3_DO,
		.addrBitsId = I2C_SLV3_ADDR_BITS_I2C_ID_3,
		.addrBitRnw = I2C_SLV3_ADDR_BIT_I2C_SLV3_RNW,
		.ctrlBitsLeng = I2C_SLV3_CTRL_BITS_I2C_SLV3_LENG,
		.ctrlBitGrp = I2C_SLV3_CTRL_BIT_I2C_SLV3_GRP,
		.ctrlBitRegDis = I2C_SLV3_CTRL_BIT_I2C_SLV3_REG_DIS,
		.ctrlBitByteSw = I2C_SLV3_CTRL_BIT_I2C_SLV3_BYTE_SW,
		.ctrlBitEn = I2C_SLV3_CTRL_BIT_I2C_SLV3_EN,
	};

	// setSlaveRead and setSlaveWrite write each slave's ADDR, REG and CTRL in one transfer, in register order
	static_assert((SLAVE0_REGISTERS.regReg == (SLAVE0_REGISTERS.regAddr + 1)) &&
					  (SLAVE0_REGISTERS.regCtrl == (SLAVE0_REGISTERS.regAddr + 2)),
				  "I2C_SLV0_ADDR, I2C_SLV0_REG and I2C_SLV0_CTRL must be contiguous");
	static_assert((SLAVE1_REGISTERS.regReg == (SLAVE1_REGISTERS.regAddr + 1)) &&
					  (SLAVE1_REGISTERS.regCtrl == (SLAVE1_REGISTERS.regAddr + 2)),
				  "I2C_SLV1_ADDR, I2C_SLV1_REG and I2C_SLV1_CTRL must be contiguous");
	static_assert((SLAVE2_REGISTERS.regReg == (SLAVE2_REGISTERS.regAddr + 1)) &&
					  (SLAVE2_REGISTERS.regCtrl == (SLAVE2_REGISTERS.regAddr + 2)),
				  "I2C_SLV2_ADDR, I2C_SLV2_REG and I2C_SLV2_CTRL must be contiguous");
	static_assert((SLAVE3_REGISTERS.regReg == (SLAVE3_REGISTERS.regAddr + 1)) &&
					  (SLAVE3_REGISTERS.regCtrl == (SLAVE3_REGISTERS.regAddr + 2)),
				  "I2C_SLV3_ADDR, I2C_SLV3_REG and I2C_SLV3_CTRL must be contiguous");

	template <AuxSlave>
	static constexpr bool ALWAYS_FALSE = false; // Fails a static_assert for an unimplemented enum value

	uint8_t _accelShift = ACCEL_SHIFT_2G;	   // Set from the device by initialiseFullScales, kept up to date by setAccelFullScale
	uint16_t _gyroFactor = GYRO_FACTOR_250DPS; // Set from the device by initialiseFullScales, kept up to date by setGyroFullScale

	/// @brief Read the chip ID
	/// @return WHO_AM_I value (IC_ID_VALUE for an MPU-6500), or 0 on failure
	uint8_t readChipID() const
	{
		return readReg(REG_WHO_AM_I);
	}

	/// @brief Set the cached data scaling from the device's full scale ranges (e.g. after an MCU-only reset)
	/// @return True if successful
	bool initialiseFullScales();

	/// @brief Write one axis' 15-bit accel offset, keeping the reserved bit in the L register
	/// @param regH The axis' offset high byte register
	/// @param regL The axis' offset low byte register
	/// @param offsetBits Bits of the low byte that hold the offset (bit 0 is reserved)
	/// @param offsetShift Position of the offset's lowest bit in the low byte
	/// @param offset Offset to write (-16384 to 16383)
	/// @return True if successful
	bool writeAccelOffset(const uint8_t regH, const uint8_t regL, const uint8_t offsetBits, const uint8_t offsetShift, const int16_t offset);

	/// @brief Read one axis' 15-bit accel offset
	/// @param regH The axis' offset high byte register
	/// @param regL The axis' offset low byte register
	/// @param offsetBits Bits of the low byte that hold the offset (bit 0 is reserved)
	/// @param offsetShift Position of the offset's lowest bit in the low byte
	/// @param offset Offset read (-16384 to 16383)
	/// @return True if successful
	bool readAccelOffset(const uint8_t regH, const uint8_t regL, const uint8_t offsetBits, const uint8_t offsetShift, int16_t &offset) const;

	/// @brief Get the registers and bits for slaves 0 to 3
	/// @tparam SLAVE The slave (Slave0 to Slave3. Slave4 has different registers)
	/// @return The slave's registers and bits
	template <AuxSlave SLAVE>
	static constexpr SlaveRegisters getSlaveRegisters()
	{
		static_assert(SLAVE != AuxSlave::Slave4, "Slave 4 has different registers - see writeSlave4 / readSlave4");

		if constexpr (SLAVE == AuxSlave::Slave0)
			return SLAVE0_REGISTERS;
		else if constexpr (SLAVE == AuxSlave::Slave1)
			return SLAVE1_REGISTERS;
		else if constexpr (SLAVE == AuxSlave::Slave2)
			return SLAVE2_REGISTERS;
		else if constexpr (SLAVE == AuxSlave::Slave3)
			return SLAVE3_REGISTERS;
		else
			static_assert(ALWAYS_FALSE<SLAVE>, "Slave registers not implemented");
	}

	/// @brief Start a slave 4 transfer and wait for it to complete
	/// @param addr I2C_SLV4_ADDR value (address and RNW)
	/// @param reg Register to transfer to / from
	/// @param data Byte to write (ignored for a read)
	/// @param registerDisabled True = transfer data only, without writing the register address
	/// @return True if the transfer completed without a NACK
	bool transferSlave4(const uint8_t addr, const uint8_t reg, const uint8_t data, const bool registerDisabled);

	/// @brief Parse raw data from the data register bytes
	/// @param data Bytes read from DATA_REGISTER onwards (DATA_LENGTH bytes)
	/// @param raw Raw accel, temperature and gyro data
	void parseRawData(const uint8_t *const data, RawSensorData &raw) const;

	/// @brief Convert a raw temperature to m°C
	/// @param raw Raw temperature (TEMP_OUT)
	/// @return Temperature (m°C)
	static int32_t temperatureFromRaw(const int16_t raw);

	/// @brief Parse scaled data from the data register bytes
	/// @param data Bytes read from DATA_REGISTER onwards
	/// @param length Number of bytes in data (must be DATA_LENGTH)
	/// @param result Accel (mg), temperature (m°C) and gyro (mdps)
	/// @return True if successful
	bool parseData(const uint8_t *const data, const uint8_t length, SensorData &result) const;

	/// @brief Read the average of SELF_TEST_SAMPLE_COUNT samples
	/// @param average Average raw data of the samples
	/// @return True if successful
	bool readSelfTestAverageRawData(RawSensorData &average) const;

	/// @brief Check a gyro axis' self-test response
	/// @param response Self-test output minus normal output (LSB at ±250 dps)
	/// @return True if the response is at least SELF_TEST_GYRO_MIN_RESPONSE
	static bool validGyroSelfTestResponse(const int32_t response);

	/// @brief Check an accel axis' self-test response
	/// @param response Self-test output minus normal output (LSB at ±2 g)
	/// @return True if the response is from SELF_TEST_ACCEL_MIN_RESPONSE to SELF_TEST_ACCEL_MAX_RESPONSE
	static bool validAccelSelfTestResponse(const int32_t response);

	/// @brief Check the self-test temperature
	/// @param rawTemperature Average raw temperature (TEMP_OUT)
	/// @return True if the temperature is from SELF_TEST_TEMPERATURE_MIN to SELF_TEST_TEMPERATURE_MAX
	static bool validSelfTestTemperature(const int16_t rawTemperature);

	/// @brief Apply the self-test configuration and read the averages without and then with self-test enabled
	/// @brief Note: The self-test configuration (±250 dps, ±2 g) is left set, with self-test turned off
	/// @param normal Average raw data with self-test disabled
	/// @param testing Average raw data with self-test enabled
	/// @return True if successful
	bool readSelfTestAverages(RawSensorData &normal, RawSensorData &testing);

protected:
	/// @brief Initialise MPU6500_I2C
	/// @return True if successful
	bool startWire() override;

public:
	/// @param i2cWire The I2C bus the device is connected to
	/// @param i2cAddr The device's I2C address, set by the AD0 pin
	explicit MPU6500_I2C(TwoWire &i2cWire = Wire, const I2CAddress i2cAddr = I2CAddress::Addr_0x68);

	/// @brief Set power mode. Waits for the sensors to start up, except when entering sleep mode
	/// @tparam MODE The power mode (see PowerMode for which sensors each mode runs)
	/// @return True if successful
	template <PowerMode MODE>
	bool setPowerMode()
	{
		// Sensor enables are set before the power mode, as the datasheet does for low-power accelerometer mode.
		// Sleep mode leaves the sensor enables unchanged
		if constexpr (MODE != PowerMode::Sleep)
		{
			uint8_t pwrMgmt2 = 0;
			if (!readReg(REG_PWR_MGMT_2, &pwrMgmt2, 1))
				return false;

			pwrMgmt2 &= ~(PWR_MGMT_2_DISABLE_GYRO | PWR_MGMT_2_DISABLE_ACCEL); // DISABLE_XG/YG/ZG/XA/YA/ZA - set to 0 (all on)

			if constexpr ((MODE == PowerMode::Standby) ||
						  (MODE == PowerMode::Gyroscope))
				pwrMgmt2 |= PWR_MGMT_2_DISABLE_ACCEL; // DISABLE_XA/YA/ZA - set to 1 (accelerometer off)
			else if constexpr ((MODE == PowerMode::LowPowerAccelerometer) ||
							   (MODE == PowerMode::LowNoiseAccelerometer))
				pwrMgmt2 |= PWR_MGMT_2_DISABLE_GYRO; // DISABLE_XG/YG/ZG - set to 1 (gyro off)
			else if constexpr (MODE == PowerMode::SixAxis)
			{
				// Gyro and accelerometer both on
			}
			else
				return false;

			if (!writeReg(REG_PWR_MGMT_2, &pwrMgmt2))
				return false;
		}

		uint8_t pwrMgmt1 = 0;
		if (!readReg(REG_PWR_MGMT_1, &pwrMgmt1, 1))
			return false;

		pwrMgmt1 &= ~(PWR_MGMT_1_BIT_SLEEP | PWR_MGMT_1_BIT_CYCLE | PWR_MGMT_1_BIT_GYRO_STANDBY); // SLEEP, CYCLE, GYRO_STANDBY - set to 0 (awake)

		if constexpr (MODE == PowerMode::Sleep)
			pwrMgmt1 |= PWR_MGMT_1_BIT_SLEEP; // SLEEP - set to 1 (sleep mode)
		else if constexpr (MODE == PowerMode::Standby)
			pwrMgmt1 |= PWR_MGMT_1_BIT_GYRO_STANDBY; // GYRO_STANDBY - set to 1 (gyro drive on, sense paths off)
		else if constexpr (MODE == PowerMode::LowPowerAccelerometer)
			pwrMgmt1 |= PWR_MGMT_1_BIT_CYCLE; // CYCLE - set to 1 (accelerometer duty-cycled)
		else if constexpr ((MODE == PowerMode::LowNoiseAccelerometer) ||
						   (MODE == PowerMode::Gyroscope) ||
						   (MODE == PowerMode::SixAxis))
		{
			// Awake, no cycling or standby
		}
		else
			return false;

		if (!writeReg(REG_PWR_MGMT_1, &pwrMgmt1))
			return false;

		// Allow time to transition
		if constexpr (MODE != PowerMode::Sleep)
			delay(START_UP_TIME_MILLIS);

		return true;
	}

	/// @brief Enable or disable the temperature sensor
	/// @param enable True = enabled, false = disabled (saves power)
	/// @return True if successful
	bool enableTemperatureSensor(const bool enable);

	/// @brief Set gyroscope full scale range. The data scaling used by readData is updated to match
	/// @tparam RANGE The full scale range (±250 to ±2000 dps)
	/// @return True if successful
	template <GyroFullScale RANGE>
	bool setGyroFullScale()
	{
		uint8_t value = 0;
		if (!readReg(REG_GYRO_CONFIG, &value, 1))
			return false;

		value &= ~GYRO_CONFIG_GYRO_FS_SEL_BITS_MASK; // GYRO_FS_SEL - clear bits

		uint16_t gyroFactor = 0;
		if constexpr (RANGE == GyroFullScale::Range_250DPS)
		{
			value |= GYRO_CONFIG_GYRO_FS_SEL_250;
			gyroFactor = GYRO_FACTOR_250DPS;
		}
		else if constexpr (RANGE == GyroFullScale::Range_500DPS)
		{
			value |= GYRO_CONFIG_GYRO_FS_SEL_500;
			gyroFactor = GYRO_FACTOR_500DPS;
		}
		else if constexpr (RANGE == GyroFullScale::Range_1000DPS)
		{
			value |= GYRO_CONFIG_GYRO_FS_SEL_1000;
			gyroFactor = GYRO_FACTOR_1000DPS;
		}
		else if constexpr (RANGE == GyroFullScale::Range_2000DPS)
		{
			value |= GYRO_CONFIG_GYRO_FS_SEL_2000;
			gyroFactor = GYRO_FACTOR_2000DPS;
		}
		else
			return false;

		if (!writeReg(REG_GYRO_CONFIG, &value))
			return false;

		_gyroFactor = gyroFactor;
		return true;
	}

	/// @brief Set accelerometer full scale range. The data scaling used by readData is updated to match
	/// @tparam RANGE The full scale range (±2 to ±16 g)
	/// @return True if successful
	template <AccelFullScale RANGE>
	bool setAccelFullScale()
	{
		uint8_t value = 0;
		if (!readReg(REG_ACCEL_CONFIG, &value, 1))
			return false;

		value &= ~ACCEL_CONFIG_ACCEL_FS_SEL_BITS_MASK; // ACCEL_FS_SEL - clear bits

		uint8_t accelShift = 0;
		if constexpr (RANGE == AccelFullScale::Range_2G)
		{
			value |= ACCEL_CONFIG_ACCEL_FS_SEL_2G;
			accelShift = ACCEL_SHIFT_2G;
		}
		else if constexpr (RANGE == AccelFullScale::Range_4G)
		{
			value |= ACCEL_CONFIG_ACCEL_FS_SEL_4G;
			accelShift = ACCEL_SHIFT_4G;
		}
		else if constexpr (RANGE == AccelFullScale::Range_8G)
		{
			value |= ACCEL_CONFIG_ACCEL_FS_SEL_8G;
			accelShift = ACCEL_SHIFT_8G;
		}
		else if constexpr (RANGE == AccelFullScale::Range_16G)
		{
			value |= ACCEL_CONFIG_ACCEL_FS_SEL_16G;
			accelShift = ACCEL_SHIFT_16G;
		}
		else
			return false;

		if (!writeReg(REG_ACCEL_CONFIG, &value))
			return false;

		_accelShift = accelShift;
		return true;
	}

	/// @brief Set gyroscope (and temperature sensor) filter
	/// @tparam FILTER The filter bandwidth (see GyroFilter for each one's sample rate and delay)
	/// @return True if successful
	template <GyroFilter FILTER>
	bool setGyroFilter()
	{
		uint8_t gyroConfig = 0;
		if (!readReg(REG_GYRO_CONFIG, &gyroConfig, 1))
			return false;

		gyroConfig &= ~GYRO_CONFIG_FCHOICE_B_BITS_MASK; // FCHOICE_B - clear bits

		if constexpr (FILTER == GyroFilter::Bandwidth_8800Hz)
			gyroConfig |= GYRO_CONFIG_FCHOICE_B_BYPASS_8800HZ;
		else if constexpr (FILTER == GyroFilter::Bandwidth_3600Hz_Fs32kHz)
			gyroConfig |= GYRO_CONFIG_FCHOICE_B_BYPASS_3600HZ;
		else
		{
			// DLPF used, so set the DLPF_CFG
			gyroConfig |= GYRO_CONFIG_FCHOICE_B_DLPF;

			uint8_t config = 0;
			if (!readReg(REG_CONFIG, &config, 1))
				return false;

			config &= ~CONFIG_DLPF_CFG_BITS_MASK; // DLPF_CFG - clear bits

			if constexpr (FILTER == GyroFilter::Bandwidth_250Hz)
				config |= CONFIG_DLPF_CFG_250HZ;
			else if constexpr (FILTER == GyroFilter::Bandwidth_184Hz)
				config |= CONFIG_DLPF_CFG_184HZ;
			else if constexpr (FILTER == GyroFilter::Bandwidth_92Hz)
				config |= CONFIG_DLPF_CFG_92HZ;
			else if constexpr (FILTER == GyroFilter::Bandwidth_41Hz)
				config |= CONFIG_DLPF_CFG_41HZ;
			else if constexpr (FILTER == GyroFilter::Bandwidth_20Hz)
				config |= CONFIG_DLPF_CFG_20HZ;
			else if constexpr (FILTER == GyroFilter::Bandwidth_10Hz)
				config |= CONFIG_DLPF_CFG_10HZ;
			else if constexpr (FILTER == GyroFilter::Bandwidth_5Hz)
				config |= CONFIG_DLPF_CFG_5HZ;
			else if constexpr (FILTER == GyroFilter::Bandwidth_3600Hz_Fs8kHz)
				config |= CONFIG_DLPF_CFG_3600HZ;
			else
				return false;

			if (!writeReg(REG_CONFIG, &config))
				return false;
		}

		return writeReg(REG_GYRO_CONFIG, &gyroConfig);
	}

	/// @brief Set accelerometer filter
	/// @tparam FILTER The filter bandwidth (see AccelFilter for each one's rate and delay)
	/// @return True if successful
	template <AccelFilter FILTER>
	bool setAccelFilter()
	{
		uint8_t value = 0;
		if (!readReg(REG_ACCEL_CONFIG_2, &value, 1))
			return false;

		value &= ~(ACCEL_CONFIG_2_BIT_ACCEL_FCHOICE_B | ACCEL_CONFIG_2_A_DLPF_CFG_BITS_MASK); // ACCEL_FCHOICE_B, A_DLPF_CFG - clear bits (DLPF used)

		if constexpr (FILTER == AccelFilter::Bandwidth_1130Hz)
			value |= ACCEL_CONFIG_2_BIT_ACCEL_FCHOICE_B; // ACCEL_FCHOICE_B - set to 1 (DLPF bypassed)
		else if constexpr (FILTER == AccelFilter::Bandwidth_460Hz)
			value |= ACCEL_CONFIG_2_A_DLPF_CFG_460HZ;
		else if constexpr (FILTER == AccelFilter::Bandwidth_184Hz)
			value |= ACCEL_CONFIG_2_A_DLPF_CFG_184HZ;
		else if constexpr (FILTER == AccelFilter::Bandwidth_92Hz)
			value |= ACCEL_CONFIG_2_A_DLPF_CFG_92HZ;
		else if constexpr (FILTER == AccelFilter::Bandwidth_41Hz)
			value |= ACCEL_CONFIG_2_A_DLPF_CFG_41HZ;
		else if constexpr (FILTER == AccelFilter::Bandwidth_20Hz)
			value |= ACCEL_CONFIG_2_A_DLPF_CFG_20HZ;
		else if constexpr (FILTER == AccelFilter::Bandwidth_10Hz)
			value |= ACCEL_CONFIG_2_A_DLPF_CFG_10HZ;
		else if constexpr (FILTER == AccelFilter::Bandwidth_5Hz)
			value |= ACCEL_CONFIG_2_A_DLPF_CFG_5HZ;
		else
			return false;

		return writeReg(REG_ACCEL_CONFIG_2, &value);
	}

	/// @brief Set sample rate divider (SAMPLE_RATE = 1kHz / (1 + divider))
	/// @brief Note: Only effective when the gyro DLPF runs at 1 kHz (Bandwidth_184Hz to Bandwidth_5Hz)
	/// @param divider SMPLRT_DIV (0 to 255, giving 1 kHz to 3.9 Hz)
	/// @return True if successful
	bool setSampleRateDivider(const uint8_t divider)
	{
		return writeReg(REG_SMPLRT_DIV, &divider);
	}

	/// @brief Set the accelerometer output data rate used in low-power accelerometer mode
	/// @tparam RATE The output data rate (0.24 to 500 Hz)
	/// @return True if successful
	template <LowPowerAccelRate RATE>
	bool setLowPowerAccelRate()
	{
		uint8_t value = 0;
		if (!readReg(REG_LP_ACCEL_ODR, &value, 1))
			return false;

		value &= ~LP_ACCEL_ODR_LPOSC_CLKSEL_BITS_MASK; // LPOSC_CLKSEL - clear bits

		if constexpr (RATE == LowPowerAccelRate::Rate_0_24Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_0_24HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_0_49Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_0_49HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_0_98Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_0_98HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_1_95Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_1_95HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_3_91Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_3_91HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_7_81Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_7_81HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_15_63Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_15_63HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_31_25Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_31_25HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_62_50Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_62_50HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_125Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_125HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_250Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_250HZ;
		else if constexpr (RATE == LowPowerAccelRate::Rate_500Hz)
			value |= LP_ACCEL_ODR_LPOSC_CLKSEL_500HZ;
		else
			return false;

		return writeReg(REG_LP_ACCEL_ODR, &value);
	}

	/// @brief Set gyro offsets. Each value is added to the gyro output before it reaches the data registers
	/// @param x X_OFFS_USR (register units)
	/// @param y Y_OFFS_USR (register units)
	/// @param z Z_OFFS_USR (register units)
	/// @return True if successful
	bool setGyroOffsets(const int16_t x, const int16_t y, const int16_t z);

	/// @brief Set accel offsets (15-bit, 0.98 mg steps, ±16 g range in all full scale modes)
	/// @param x XA_OFFS (-16384 to 16383)
	/// @param y YA_OFFS (-16384 to 16383)
	/// @param z ZA_OFFS (-16384 to 16383)
	/// @return True if successful
	bool setAccelOffsets(const int16_t x, const int16_t y, const int16_t z);

	/// @brief Read accel offsets (may hold factory values)
	/// @param x XA_OFFS (-16384 to 16383)
	/// @param y YA_OFFS (-16384 to 16383)
	/// @param z ZA_OFFS (-16384 to 16383)
	/// @return True if successful
	bool readAccelOffsets(int16_t &x, int16_t &y, int16_t &z) const;

	/// @brief Set INT pin behaviour, shared by all interrupts
	/// @param activeHigh True = active high, false = active low
	/// @param openDrain True = open drain, false = push-pull
	/// @param latched True = held until interrupt status is cleared, false = 50 µs pulse
	/// @param clearOnAnyRead True = status cleared by any read, false = cleared only by reading the interrupt status
	/// @return True if successful
	bool setInterruptConfig(const bool activeHigh = true, const bool openDrain = false, const bool latched = false,
							const bool clearOnAnyRead = false);

	/// @brief Enable the raw sensor data ready interrupt on the INT pin
	/// @param enable True = enabled, false = disabled
	/// @return True if successful
	bool enableDataReadyInterrupt(const bool enable);

	/// @brief Enable the FIFO overflow interrupt on the INT pin
	/// @param enable True = enabled, false = disabled
	/// @return True if successful
	bool enableFifoOverflowInterrupt(const bool enable);

	/// @brief Enable the FSYNC pin as an interrupt source, and its interrupt on the INT pin
	/// @param enable True = enabled, false = disabled
	/// @param activeHigh True = FSYNC active high, false = active low
	/// @return True if successful
	bool enableFsyncInterrupt(const bool enable, const bool activeHigh = true);

	/// @brief Read the interrupt status. Reading clears it
	/// @param status The interrupt status
	/// @return True if successful
	bool getInterruptStatus(InterruptStatus &status) const;

	/// @brief Set the auxiliary I2C bus mode. The I2C master and bypass are never on together
	/// @tparam MODE Disabled, PassThrough (aux bus connected to the primary bus) or Master (MPU-6500 drives the aux bus)
	/// @return True if successful
	template <AuxI2CMode MODE>
	bool setAuxI2CMode()
	{
		uint8_t userCtrl = 0;
		if (!readReg(REG_USER_CTRL, &userCtrl, 1))
			return false;

		uint8_t intPinCfg = 0;
		if (!readReg(REG_INT_PIN_CFG, &intPinCfg, 1))
			return false;

		if constexpr (MODE == AuxI2CMode::Disabled)
		{
			userCtrl &= ~USER_CTRL_BIT_I2C_MST_EN;	 // I2C_MST_EN - set to 0 (I2C master off)
			intPinCfg &= ~INT_PIN_CFG_BIT_BYPASS_EN; // BYPASS_EN - set to 0 (bypass off)

			return writeReg(REG_USER_CTRL, &userCtrl) &&
				   writeReg(REG_INT_PIN_CFG, &intPinCfg);
		}
		else if constexpr (MODE == AuxI2CMode::PassThrough)
		{
			// The I2C master is turned off before bypass is turned on, so they are never on together
			userCtrl &= ~USER_CTRL_BIT_I2C_MST_EN;	// I2C_MST_EN - set to 0 (I2C master off)
			intPinCfg |= INT_PIN_CFG_BIT_BYPASS_EN; // BYPASS_EN - set to 1 (bypass on)

			return writeReg(REG_USER_CTRL, &userCtrl) &&
				   writeReg(REG_INT_PIN_CFG, &intPinCfg);
		}
		else if constexpr (MODE == AuxI2CMode::Master)
		{
			// Bypass is turned off before the I2C master is turned on, so they are never on together
			intPinCfg &= ~INT_PIN_CFG_BIT_BYPASS_EN; // BYPASS_EN - set to 0 (bypass off)
			userCtrl |= USER_CTRL_BIT_I2C_MST_EN;	 // I2C_MST_EN - set to 1 (I2C master on)

			return writeReg(REG_INT_PIN_CFG, &intPinCfg) &&
				   writeReg(REG_USER_CTRL, &userCtrl);
		}
		else
			return false;
	}

	/// @brief Set the I2C master clock speed
	/// @tparam CLOCK The aux bus clock speed (258 to 500 kHz)
	/// @return True if successful
	template <I2CMasterClock CLOCK>
	bool setI2CMasterClock()
	{
		uint8_t value = 0;
		if (!readReg(REG_I2C_MST_CTRL, &value, 1))
			return false;

		value &= ~I2C_MST_CTRL_I2C_MST_CLK_BITS_MASK; // I2C_MST_CLK - clear bits

		if constexpr (CLOCK == I2CMasterClock::Clock_348kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_348KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_333kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_333KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_320kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_320KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_308kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_308KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_296kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_296KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_286kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_286KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_276kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_276KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_267kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_267KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_258kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_258KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_500kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_500KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_471kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_471KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_444kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_444KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_421kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_421KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_400kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_400KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_381kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_381KHZ;
		else if constexpr (CLOCK == I2CMasterClock::Clock_364kHz)
			value |= I2C_MST_CTRL_I2C_MST_CLK_364KHZ;
		else
			return false;

		return writeReg(REG_I2C_MST_CTRL, &value);
	}

	/// @brief Set I2C master behaviour
	/// @param waitForExternalSensor True = data ready interrupt delayed until external sensor data is loaded
	/// @param stopBetweenReads True = stop between slave reads, false = restart
	/// @param multiMaster True = multi-master capability enabled
	/// @param delayShadow True = external sensor data shadowed only once all of it has been received
	/// @return True if successful
	bool setI2CMasterConfig(const bool waitForExternalSensor = false, const bool stopBetweenReads = false, const bool multiMaster = false, const bool delayShadow = false);

	/// @brief Set a slave to read from an aux I2C device at the sample rate, storing the data in the EXT_SENS_DATA registers
	/// @tparam SLAVE The slave to use (Slave0 to Slave3)
	/// @param address I2C address of the aux device
	/// @param reg Register to start reading from
	/// @param length Number of bytes to read (1 to 15)
	/// @param swapBytes True = swap the bytes of each word
	/// @param groupOdd True = registers 1 and 2 grouped (even register ends a group), false = registers 0 and 1 grouped
	/// @param registerDisabled True = transfer data only, without writing the register address
	/// @return True if successful
	template <AuxSlave SLAVE>
	bool setSlaveRead(const uint8_t address, const uint8_t reg, const uint8_t length, const bool swapBytes = false, const bool groupOdd = false, const bool registerDisabled = false)
	{
		static_assert(SLAVE != AuxSlave::Slave4, "Slave 4 only supports single transfers - use readSlave4()");
		constexpr SlaveRegisters REGISTERS = getSlaveRegisters<SLAVE>();

		if ((length == 0) || (length > REGISTERS.ctrlBitsLeng))
			return false;

		uint8_t ctrl = REGISTERS.ctrlBitEn | (length & REGISTERS.ctrlBitsLeng); // I2C_SLVx_EN - set to 1, I2C_SLVx_LENG - set to length
		if (swapBytes)
			ctrl |= REGISTERS.ctrlBitByteSw; // I2C_SLVx_BYTE_SW - set to 1 (swap bytes)
		if (groupOdd)
			ctrl |= REGISTERS.ctrlBitGrp; // I2C_SLVx_GRP - set to 1 (registers 1 and 2 grouped)
		if (registerDisabled)
			ctrl |= REGISTERS.ctrlBitRegDis; // I2C_SLVx_REG_DIS - set to 1 (data only)

		const uint8_t data[] = {
			static_cast<uint8_t>(REGISTERS.addrBitRnw | (address & REGISTERS.addrBitsId)), // I2C_SLVx_ADDR: read from address
			reg,																		   // I2C_SLVx_REG
			ctrl																		   // I2C_SLVx_CTRL
		};

		return writeReg(REGISTERS.regAddr, data, sizeof(data));
	}

	/// @brief Set a slave to write a byte to an aux I2C device at the sample rate
	/// @tparam SLAVE The slave to use (Slave0 to Slave3)
	/// @param address I2C address of the aux device
	/// @param reg Register to write to
	/// @param data Byte to write
	/// @param registerDisabled True = transfer data only, without writing the register address
	/// @return True if successful
	template <AuxSlave SLAVE>
	bool setSlaveWrite(const uint8_t address, const uint8_t reg, const uint8_t data, const bool registerDisabled = false)
	{
		static_assert(SLAVE != AuxSlave::Slave4, "Slave 4 only supports single transfers - use writeSlave4()");
		constexpr SlaveRegisters REGISTERS = getSlaveRegisters<SLAVE>();

		// The data out register is set first, so the first transfer writes the correct value
		if (!writeReg(REGISTERS.regDo, &data))
			return false;

		uint8_t ctrl = REGISTERS.ctrlBitEn | (1 & REGISTERS.ctrlBitsLeng); // I2C_SLVx_EN - set to 1, I2C_SLVx_LENG - set to 1 byte
		if (registerDisabled)
			ctrl |= REGISTERS.ctrlBitRegDis; // I2C_SLVx_REG_DIS - set to 1 (data only)

		const uint8_t setup[] = {
			static_cast<uint8_t>(address & REGISTERS.addrBitsId), // I2C_SLVx_ADDR: write to address (RNW = 0)
			reg,												  // I2C_SLVx_REG
			ctrl												  // I2C_SLVx_CTRL
		};

		return writeReg(REGISTERS.regAddr, setup, sizeof(setup));
	}

	/// @brief Enable or disable a slave's transfers at the sample rate
	/// @tparam SLAVE The slave (Slave0 to Slave3)
	/// @param enable True = enabled, false = disabled
	/// @return True if successful
	template <AuxSlave SLAVE>
	bool enableSlave(const bool enable)
	{
		static_assert(SLAVE != AuxSlave::Slave4, "Slave 4 only supports single transfers - use writeSlave4() / readSlave4()");
		constexpr SlaveRegisters REGISTERS = getSlaveRegisters<SLAVE>();

		uint8_t value = 0;
		if (!readReg(REGISTERS.regCtrl, &value, 1))
			return false;

		if (enable)
			value |= REGISTERS.ctrlBitEn; // I2C_SLVx_EN - set to 1 (enabled)
		else
			value &= ~REGISTERS.ctrlBitEn; // I2C_SLVx_EN - set to 0 (disabled)

		return writeReg(REGISTERS.regCtrl, &value);
	}

	/// @brief Set the I2C master delay. Delayed slaves are accessed every (1 + delay) samples
	/// @param delay I2C_MST_DLY (0 to 31)
	/// @return True if successful
	bool setI2CMasterDelay(const uint8_t delay);

	/// @brief Enable or disable the I2C master delay for a slave
	/// @tparam SLAVE The slave (Slave0 to Slave4)
	/// @param enable True = accessed every (1 + I2C_MST_DLY) samples (see setI2CMasterDelay), false = every sample
	/// @return True if successful
	template <AuxSlave SLAVE>
	bool enableSlaveDelay(const bool enable)
	{
		uint8_t value = 0;
		if (!readReg(REG_I2C_MST_DELAY_CTRL, &value, 1))
			return false;

		uint8_t dlyEnBit = 0;
		if constexpr (SLAVE == AuxSlave::Slave0)
			dlyEnBit = I2C_MST_DELAY_CTRL_BIT_I2C_SLV0_DLY_EN;
		else if constexpr (SLAVE == AuxSlave::Slave1)
			dlyEnBit = I2C_MST_DELAY_CTRL_BIT_I2C_SLV1_DLY_EN;
		else if constexpr (SLAVE == AuxSlave::Slave2)
			dlyEnBit = I2C_MST_DELAY_CTRL_BIT_I2C_SLV2_DLY_EN;
		else if constexpr (SLAVE == AuxSlave::Slave3)
			dlyEnBit = I2C_MST_DELAY_CTRL_BIT_I2C_SLV3_DLY_EN;
		else if constexpr (SLAVE == AuxSlave::Slave4)
			dlyEnBit = I2C_MST_DELAY_CTRL_BIT_I2C_SLV4_DLY_EN;
		else
			return false;

		if (enable)
			value |= dlyEnBit; // I2C_SLVx_DLY_EN - set to 1 (delayed)
		else
			value &= ~dlyEnBit; // I2C_SLVx_DLY_EN - set to 0 (every sample)

		return writeReg(REG_I2C_MST_DELAY_CTRL, &value);
	}

	/// @brief Read the data stored by slaves 0 to 3
	/// @param data Buffer for the data
	/// @param index First EXT_SENS_DATA register to read (0 to 23)
	/// @param length Number of bytes to read (1 to 24 - index)
	/// @return True if successful
	bool readExternalSensorData(uint8_t *const data, const uint8_t index, const uint8_t length) const;

	/// @brief Write a single byte to an aux I2C device using slave 4
	/// @param address I2C address of the aux device
	/// @param reg Register to write to
	/// @param data Byte to write
	/// @param registerDisabled True = transfer data only, without writing the register address
	/// @return True if successful
	bool writeSlave4(const uint8_t address, const uint8_t reg, const uint8_t data, const bool registerDisabled = false);

	/// @brief Read a single byte from an aux I2C device using slave 4
	/// @param address I2C address of the aux device
	/// @param reg Register to read from
	/// @param data Byte read
	/// @param registerDisabled True = transfer data only, without writing the register address
	/// @return True if successful
	bool readSlave4(const uint8_t address, const uint8_t reg, uint8_t &data, const bool registerDisabled = false);

	/// @brief Enable the slave 4 transfer complete interrupt
	/// @brief Note: The register map says this also needs I2C_MST_INT_EN in INT_ENABLE, which the MPU-6500 does not have
	/// @param enable True = enabled, false = disabled
	/// @return True if successful
	bool enableSlave4DoneInterrupt(const bool enable);

	/// @brief Read the I2C master status. Reading clears it
	/// @param status The I2C master status
	/// @return True if successful
	bool getI2CMasterStatus(I2CMasterStatus &status) const;

	/// @brief Enable or disable FIFO operation
	/// @param enable True = enabled, false = disabled
	/// @return True if successful
	bool enableFifo(const bool enable);

	/// @brief Reset the FIFO, discarding its contents
	/// @return True if successful
	bool resetFifo();

	/// @brief Set what happens when the FIFO is full
	/// @tparam MODE Overwrite (new data replaces the oldest) or StopWhenFull (new data is not written)
	/// @return True if successful
	template <FifoMode MODE>
	bool setFifoMode()
	{
		uint8_t value = 0;
		if (!readReg(REG_CONFIG, &value, 1))
			return false;

		if constexpr (MODE == FifoMode::Overwrite)
			value &= ~CONFIG_BIT_FIFO_MODE; // FIFO_MODE - set to 0 (overwrite oldest data)
		else if constexpr (MODE == FifoMode::StopWhenFull)
			value |= CONFIG_BIT_FIFO_MODE; // FIFO_MODE - set to 1 (stop writing when full)
		else
			return false;

		return writeReg(REG_CONFIG, &value);
	}

	/// @brief Set which data is written to the FIFO at the sample rate
	/// @param sources The data to write to the FIFO. Sources set to false are not written
	/// @return True if successful
	bool setFifoSources(const FifoSources &sources);

	/// @brief Read the number of bytes in the FIFO
	/// @param count Number of bytes in the FIFO (0 to FIFO_SIZE)
	/// @return True if successful
	bool readFifoCount(uint16_t &count) const;

	/// @brief Read bytes from the FIFO
	/// @param data Buffer for the data
	/// @param length Number of bytes to read (1 to FIFO_SIZE)
	/// @return True if successful
	bool readFifo(uint8_t *const data, const uint16_t length) const;

	/// @brief Reset all registers to their default values. The device is left awake
	/// @return True if successful
	bool reset();

	/// @brief Reset the digital signal paths. The sensor data registers are not cleared (see resetSensors)
	/// @param gyro True = reset the gyro signal path
	/// @param accel True = reset the accel signal path
	/// @param temperature True = reset the temperature signal path
	/// @return True if successful
	bool resetSignalPaths(const bool gyro, const bool accel, const bool temperature);

	/// @brief Reset all digital signal paths and clear the sensor data registers
	/// @return True if successful
	bool resetSensors();

	/// @brief Reset the I2C master
	/// @brief Note: Only use when the I2C master has hung. Resetting during an active transfer will hang the aux device
	/// @return True if successful
	bool resetI2CMaster();

	/// @brief Reset the DMP. Fails if the DMP is enabled, as the reset only takes effect while it is disabled
	/// @return True if successful
	bool resetDmp();

	/// @brief Read raw data from a single burst read, so all values are from the same sample
	/// @param raw Raw accel, temperature and gyro data (LSB at the current full scale ranges)
	/// @return True if successful
	bool readRawData(RawSensorData &raw) const;

	/// @brief Read scaled data from a single burst read, so all values are from the same sample
	/// @param result Accel (mg), temperature (m°C) and gyro (mdps)
	/// @return True if successful
	bool readData(SensorData &result) const;

	/// @brief Check the gyro, accel and temperature sensor are working (not their accuracy). Takes about half a second,
	/// @brief and the device must be still
	/// @brief Note: The self-test turns everything on and changes the configuration, and nothing is restored.
	/// @brief It's therefore recommended to configure the device after the self-test
	/// @return True if every axis and the temperature are within the limits
	bool normalSelfTest();

	/// @brief Enable wake-on-motion: the INT pin signals when any accel axis exceeds the threshold
	/// @brief See enableLowPower for waiting on motion at low power
	/// @tparam THRESHOLD_MG Motion threshold in mg (0 to 1020, multiple of 4)
	/// @return True if successful
	template <uint16_t THRESHOLD_MG>
	bool enableWakeOnMotion()
	{
		static_assert(THRESHOLD_MG <= WOM_THRESHOLD_MAX_MG, "THRESHOLD_MG must be in the range of 0 to 1020");
		static_assert((THRESHOLD_MG % WOM_THRESHOLD_MG_PER_LSB) == 0, "THRESHOLD_MG must be a multiple of 4 mg");

		// Wake-on-motion interrupt on the INT pin
		{
			uint8_t value = 0;
			if (!readReg(REG_INT_ENABLE, &value, 1))
				return false;

			value |= INT_ENABLE_BIT_WOM_EN; // WOM_EN - set to 1 (enabled)
			if (!writeReg(REG_INT_ENABLE, &value))
				return false;
		}

		// Motion detection logic, comparing each sample with the previous one
		{
			uint8_t value = 0;
			if (!readReg(REG_ACCEL_INTEL_CTRL, &value, 1))
				return false;

			value |= (ACCEL_INTEL_CTRL_BIT_ACCEL_INTEL_EN | ACCEL_INTEL_CTRL_BIT_ACCEL_INTEL_MODE); // ACCEL_INTEL_EN, ACCEL_INTEL_MODE - set to 1
			if (!writeReg(REG_ACCEL_INTEL_CTRL, &value))
				return false;
		}

		// Motion threshold
		uint8_t threshold = THRESHOLD_MG / WOM_THRESHOLD_MG_PER_LSB;
		return writeReg(REG_WOM_THR, &threshold);
	}

	/// @brief Disable wake-on-motion (detection logic and interrupt). The power mode is left unchanged
	/// @return True if successful
	bool disableWakeOnMotion();

	/// @brief Run at low power: low-power accelerometer mode (gyro off, accel cycling at 15.63 Hz),
	/// @brief accel DLPF bypassed (the only setting low-power mode supports) and the temperature sensor off
	/// @brief Note: Each setting can be changed back individually
	/// @return True if successful
	bool enableLowPower();

	/// @brief Set up the device in one call. The defaults suit general motion and orientation tracking
	/// @tparam POWER_MODE Power mode (default both sensors on)
	/// @tparam GYRO_RANGE Gyro full scale range (default ±500 dps)
	/// @tparam ACCEL_RANGE Accel full scale range (default ±4 g)
	/// @tparam GYRO_FILTER Gyro filter bandwidth (default 41 Hz)
	/// @tparam ACCEL_FILTER Accel filter bandwidth (default 41 Hz)
	/// @param sampleRateDivider Sample rate = 1 kHz / (1 + divider). The default gives 100 Hz
	/// @return True if successful
	template <PowerMode POWER_MODE = PowerMode::SixAxis,
			  GyroFullScale GYRO_RANGE = GyroFullScale::Range_500DPS,
			  AccelFullScale ACCEL_RANGE = AccelFullScale::Range_4G,
			  GyroFilter GYRO_FILTER = GyroFilter::Bandwidth_41Hz,
			  AccelFilter ACCEL_FILTER = AccelFilter::Bandwidth_41Hz>
	bool quickSetup(const uint8_t sampleRateDivider = 9)
	{
		return setPowerMode<POWER_MODE>() &&
			   setGyroFullScale<GYRO_RANGE>() &&
			   setAccelFullScale<ACCEL_RANGE>() &&
			   setGyroFilter<GYRO_FILTER>() &&
			   setAccelFilter<ACCEL_FILTER>() &&
			   setSampleRateDivider(sampleRateDivider);
	}
};
