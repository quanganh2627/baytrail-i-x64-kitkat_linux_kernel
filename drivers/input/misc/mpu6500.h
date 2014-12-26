#ifndef _MPU_6500_H_
#define _MPU_6500_H_

#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG 			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG	 	0x1C
#define MPUREG_ACCEL_CONFIG2	0x1D
#define MPUREG_LP_ACCEL_ODR   	0x1E

#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_DATA		0x3B
#define MPUREG_TEMP_DATA		0x41
#define MPUREG_GYRO_DATA		0x43
#define MPUREG_ACCEL_INTEL_CTRL 0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_R_W			0x74
#define MPUREG_WHOAMI			0x75

#define DLPF_CFG_250HZ			0x00
#define DLPF_CFG_184HZ			0x01
#define DLPF_CFG_92HZ			0x02
#define DLPF_CFG_41HZ			0x03
#define DLPF_CFG_20HZ			0x04
#define DLPF_CFG_10HZ			0x05
#define DLPF_CFG_5HZ			0x06
#define DLPF_CFG_3600HZ			0x07

#define GFSR_250DPS				(0 <<3)
#define GFSR_500DPS				(1 <<3)
#define GFSR_1000DPS			(2 <<3)
#define GFSR_2000DPS			(3 <<3)

#define AFSR_2G					(0 <<3)
#define AFSR_4G					(1 <<3)
#define AFSR_8G					(2 <<3)
#define AFSR_16G				(3 <<3)

#define A_DLPF_CFG_460HZ		0x00
#define A_DLPF_CFG_184HZ		0x01
#define A_DLPF_CFG_92HZ			0x02
#define A_DLPF_CFG_41HZ			0x03
#define A_DLPF_CFG_20HZ			0x04
#define A_DLPF_CFG_10HZ			0x05
#define A_DLPF_CFG_5HZ			0x06
//#define A_DLPF_CFG_460HZ		0x07

#define BIT_H_RESET				(1<<7)
#define BIT_SLEEP				(1<<6)
#define BIT_CYCLE				(1<<5)
#define BIT_GYRO_STANDBY		(1<<4)
#define BIT_PD_PTAT				(1<<3)
#define BIT_CLKSEL				(1<<2)

#define BIT_ACCEL_STBY       	0x38
#define BIT_GYRO_STBY        	0x07
#define BITS_LPA_WAKE_CTRL	 	0xC0
#define BITS_LPA_WAKE_1HZ 		0x00
#define BITS_LPA_WAKE_2HZ 		0x40
#define BITS_LPA_WAKE_20HZ 		0x80

#define BIT_FIFO_RST			0x04
#define BIT_DMP_RST				0x08
#define BIT_I2C_MST_EN			0x20
#define BIT_FIFO_EN				0x40
#define BIT_DMP_EN				0x80

#define BIT_ACCEL_OUT			0x08
#define BITS_GYRO_OUT			0x70

#define BIT_BYPASS_EN           0x2

#define BIT_INT_STATUS_FIFO_OVERLOW 0x80
#define BIT_MPU_RDY				0x04
#define BIT_DMP_INT				0x02
#define BIT_RAW_RDY				0x01

#define BIT_ACCEL_INTEL_ENABLE	0x80
#define BIT_ACCEL_INTEL_MODE	0x40

#define MPU6050_ID				0x68
#define MPU6500_ID				0x70
#define MPU6515_ID				0x74
#define MPU6880_ID				0x78

#define INVN_I2C_NAME 			"mpu6500"

#define TEST_CH					0
#define ENABLE_CH				1
#define FSR_CH					2
#define DELAY_CH				3
#define RAW_CH					4
#define BIAS_CH					5
#define VALUE_CH				6
#define MATRIX_CH				7

#define DELAY_NORMAL			200
#define DELAY_GAME				60
#define DELAY_UI				20
#define DELAY_FAST				10


struct invsens_platform_data_t {
	signed char orientation[9];
};

struct sensor_data {
	bool	enable;
	int		fsr;
	int		delay;
	int		delay_s;
	int		counter;
	short	raw[3];
	short	value[3];
	short	bias[3];
};

struct INVN_DATA {
	struct	i2c_client	*client;
	struct	mutex invn_mutex;
	struct	delayed_work invn_delay;
	struct	input_dev *input_dev_accel;
	struct	input_dev *input_dev_gyro;
	struct	invsens_platform_data_t inv_sensors_pdata;
	struct	sensor_data accel;
	struct	sensor_data gyro;
	bool	enable;
	int		delay;			//ms
};

#endif

