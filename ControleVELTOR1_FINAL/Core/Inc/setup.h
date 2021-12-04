/**
 * @file  setup.h
 * @date  09-September-2021
 * @brief Configuration file.
 *
 * This header file contains definitions that change the firmware behavior.
 * These definitions refer to pins, ports and configuration options for
 * some drivers and devices.
 *
 * @author
 * @author
 */


#ifndef SETUP_H
#define SETUP_H



/******************************************************************************/
/** Configuring the max number of software timers                             */
/******************************************************************************/
#define PIT_MAX_ID                                                            20

#define LIB_DISP_PIT_ID                                                        0
#define LIB_AHRS_PIT_ID                                                        1
#define SDCARD_PIT_ID                                                         19

/******************************************************************************/
/** UART ID used by                                                           */
/******************************************************************************/
#define UART_MAX_ID                                                            1

/******************************************************************************/
/** Configuring the max number of iic IDs that can be configured              */
/******************************************************************************/
#define IIC_MAX_ID                                                             3


/******************************************************************************/
/** UART port parameters for MODBUS server                                    */
/******************************************************************************/
#define MODBUS_RTU_SERVER_UART_PORT                          UART2_RX_PA3_TX_PA2
#define MODBUS_RTU_SERVER_UART_BAUDRATE                          UART_115200_BPS
#define MODBUS_RTU_SERVER_ADDRESS                                             10


/******************************************************************************/
/** MODBUS server registers sizes                                             */
/** Addresses go from 0 to the values defined below                           */
/******************************************************************************/
#define MODBUS_RTU_SERVER_HOLDING_REGISTERS_SIZE                              65
#define MODBUS_RTU_SERVER_OUTPUT_COILS_SIZE                                   65
#define MODBUS_RTU_SERVER_INPUT_REGISTERS_SIZE                                65
#define MODBUS_RTU_SERVER_DISCRETE_INPUTS_SIZE                                65




/** SDCard parameters                                                         */
#define SDCARD_DELAY_TIME                                                    100

#define USE_SDCARD_CARD_DETECT                                                 0

#define SDCARD_SPI_PORT                         SPI3_SCL_PC10_CIPO_PC11_COPI_PB5
#define SDCARD_PRESCALER                                       SPI_PRESCALLER_32
#define SDCARD_CS_PORT                                           SPI_GPIO_PORT_D
#define SDCARD_CS_PIN                                             SPI_GPIO_PIN_0
#define SDCARD_CS_POLARITY                                        SPI_ACTIVE_LOW





/******************************************************************************/
/** PIT time to count used to configure lib_char_disp software timer in ms    */
/******************************************************************************/
#define LIB_DISP_PIT_TIME                                                     20

/******************************************************************************/
/** Defining the configuration parameters for SSD1306 display driver.         */
/** These definitions are used on map_char_disp                               */
/******************************************************************************/
#define DEV_SSD1306_ID                                                         0
#define DEV_SSD1306_PORT                                    IIC3_SDA_PC9_SCL_PA8
#define DEV_SSD1306_ADDR                                            SSD1306_0x3C
#define DEV_SSD1306_PULLCFG                                  IIC_PULLUP_DISABLED




/******************************************************************************/
/** PIT time to count used to configure lib_char_disp software timer in ms    */
/******************************************************************************/
#define LIB_AHRS_PIT_TIME                                                    100

/******************************************************************************/
/** Defining the configuration parameters for MPU6050 accelerometer/gyroscope.*/
/** These definitions are used on map_ahrs.                                   */
/******************************************************************************/
#define DEV_MPU6050_ID                                                         0
#define DEV_MPU6050_PORT                                    IIC3_SDA_PC9_SCL_PA8
#define DEV_MPU6050_ADDR                                            MPU6050_0x68
#define DEV_MPU6050_PULLCFG                                  IIC_PULLUP_DISABLED

/******************************************************************************/
/** Defining the configuration parameters for QMC5883L magnetometer.          */
/** These definitions are used on map_ahrs.                                   */
/******************************************************************************/
#define DEV_QMC5883L_ID                                                        0
#define DEV_QMC5883L_PORT                                   IIC3_SDA_PC9_SCL_PA8
#define DEV_QMC5883L_ADDR                                          QMC5883L_0x0D
#define DEV_QMC5883L_PULLCFG                                 IIC_PULLUP_DISABLED

/******************************************************************************/
/** Defining the configuration parameters motor pins (encoders and PWM).      */
/******************************************************************************/
#define DRV_QDC_RESOLUTION                                                QDC_X4
#define DRV_PWM_MODE                                            PWM_EDGE_ALIGNED
#define DRV_PWM_PREFERENCE                                        PWM_RESOLUTION
/* 15kHz (quite silent)*/
#define DRV_PWM_FREQUENCY                                                15000.0
#define DRV_PWM_RESOLUTION                                             PWM_8BITS


#define DRV_QDC_PORT_1                                    QDC4_CH1_PD12_CH2_PD13
#define DRV_PWM_CHANNEL_1                                         PWM12_CH1_PB14
#define DRV_GPIO_PIN_DIR1                                            GPIO_PIN_12
#define DRV_GPIO_PORT_DIR1                                           GPIO_PORT_B


#define DRV_QDC_PORT_2                                      QDC3_CH1_PB4_CH2_PB5
#define DRV_PWM_CHANNEL_2                                         PWM12_CH2_PB15
#define DRV_GPIO_PIN_DIR2                                            GPIO_PIN_13
#define DRV_GPIO_PORT_DIR2                                           GPIO_PORT_B

#define DRV_QDC_PORT_3                                      QDC8_CH1_PC6_CH2_PC7
#define DRV_PWM_CHANNEL_3                                          PWM10_CH1_PB8
#define DRV_GPIO_PIN_DIR3                                             GPIO_PIN_7
#define DRV_GPIO_PORT_DIR3                                           GPIO_PORT_B



#endif  /* SETUP_H */
