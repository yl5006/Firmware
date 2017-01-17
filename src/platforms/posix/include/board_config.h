
/*
 * I2C busses
 */
#define PX4_I2C_BUS_ESC		1
//#define PX4_SIM_BUS_TEST	2     //yaoling
//#define PX4_I2C_BUS_EXPANSION	3   
//#define PX4_I2C_BUS_LED		3


/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	2
#define PX4_I2C_BUS_ONBOARD	1
#define PX4_I2C_BUS_LED		PX4_I2C_BUS_ONBOARD
 
#define PX4_I2C_OBDEV_LED	0x55
#define PX4_I2C_OBDEV_HMC5883	0x1e

//#if defined(__PX4_POSIX_RPI2)
//#define PX4_SPI_BUS_SENSORS	0
//#define PX4_I2C_BUS_SENSORS	1
//#define ALARM_GPIO_PORT 4
//#endif
//add for spi custom flags dy
#define SPI_INNER 0
#define SPI_EXTERN 1
#define SPI_CUSTOM 2

#define PX4_SPIDEV_GYRO 1
#define PX4_SPIDEV_ACCEL_MAG 2

#if defined(__PX4_POSIX_TI)
#define PX4_SPI_BUS_SENSORS	1
#define PX4_SPI_BUS_BARO	1
#define PX4_I2C_BUS_SENSORS	2
#define PX4_SPIDEV_MPU		0
#define	PX4FMU_DEVICE_PATH "/dev/px4fmu"
#else
#if defined(__PX4_POSIX_RPI)
#define PX4_SPI_BUS_SENSORS	0
#define PX4_I2C_BUS_SENSORS	1
#define PX4_SPIDEV_MPU		0
#define	PX4FMU_DEVICE_PATH "/dev/px4fmu"
#endif
#endif


/* Use these in place of the spi_dev_e enumeration to select a specific SPI device on SPI1 */
#define PX4_SPIDEV_ACCEL_GYRO	0
#define PX4_SPIDEV_BARO		0

#define PX4_SPIDEV_HMC		3

#define BOARD_NAME "PX4_LINUX"

#define DIRECT_PWM_OUTPUT_CHANNELS 8

#define GPIO_BTN_SAFETY		11
//#define GPIO_PIN(BANK, 8) (BANK*32+8)
#define GPIO_BTN_LED		10
#define	BOARD_HAS_PWM 		8
#define RC_SERIAL_PORT "/dev/ttyO5"
#define RC_SERIAL_PORTs "/dev/ttys5"
