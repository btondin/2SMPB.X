/*****************************************************************************
 * File:    omron_2smpb_02e.h
 * Author:  Bruno Rodriguez tondin
 * Comments: This file contains the definitions and functions for the OMRON
 *           2SMPB-02E barometer sensor.
 * Revision history: 1.01
 * Date: 10/09/2023
 *****************************************************************************/

#ifndef OMROM_2SMPB_02E_H
#define	OMROM_2SMPB_02E_H

#include <stdint.h>
#include <math.h>

#define OMRON_SLAVE_ADDR     0x70   
#define OMRON_RETRY_MAX       100  // Define the retry count
#define OMRON_DEVICE_TIMEOUT  50   // Define slave timeout

#define OMRON_TEMP_TXD0            0xFC  // Temperature DATA [8:1] in 24 bits
#define OMRON_TEMP_TXD1            0xFB  // Temperature DATA [16:9] in 24 bits
#define OMRON_TEMP_TXD2            0xFA  // Temperature DATA [24:17] in 24 bits

#define OMRON_PRESS_TXD0           0xF9  // Pressure DATA [8:1] in 24 bits
#define OMRON_PRESS_TXD1           0xF8  // Pressure DATA [16:9] in 24 bits
#define OMRON_PRESS_TXD2           0xF7  // Pressure DATA [24:17] in 24 bits

#define OMRON_IO_SETUP             0xF5  // t_stanby[2:0] : Standby time setting
                                         // spi3w : SPI mode setting (4 or 3 wire)
                                         // spi3_sdim : Select output type of SDI terminal

#define OMRON_CTRL_MEAS            0xF4  // temp_average[2:0] : Temperature Averaging Times
                                         // press_average[2:0] : Pressure Averaging Times
                                         // power_mode[1:0] : Power mode setting

#define OMRON_DEVICE_STAT          0xF3  // measure : Status of measurement
                                         // otp_update : Status of OTP data access

#define OMRON_I2C_SET              0xF2  // Master code setting at I2C HS mode
#define OMRON_IIR_CNT              0xF1  // IIR filter co-efficient setting
#define OMRON_RESET                0xE0  // When inputting "E6h", a software reset will be occurred.
#define OMRON_CHIP_ID              0xD1  // CHIP_ID : 5Ch

#define OMRON_COE_B00_A0_EX        0xB8  // 24
#define OMRON_COE_A2_0             0xB7  // 23
#define OMRON_COE_A2_1             0xB6  // 22
#define OMRON_COE_A1_0             0xB5  // 21
#define OMRON_COE_A1_1             0xB4  // 20
#define OMRON_COE_A0_0             0xB3  // 19
#define OMRON_COE_A0_1             0xB2  // 18
#define OMRON_COE_BP3_0            0xB1  // 17
#define OMRON_COE_BP3_1            0xB0  // 16
#define OMRON_COE_B21_0            0xAF  // 15
#define OMRON_COE_B21_1            0xAE  // 14
#define OMRON_COE_B12_0            0xAD  // 13
#define OMRON_COE_B12_1            0xAC  // 12
#define OMRON_COE_BP2_0            0xAB  // 11
#define OMRON_COE_BP2_1            0xAA  // 10
#define OMRON_COE_B11_0            0xA9  // 9
#define OMRON_COE_B11_1            0xA8  // 8
#define OMRON_COE_BP1_0            0xA7  // 7
#define OMRON_COE_BP1_1            0xA6  // 6
#define OMRON_COE_BT2_0            0xA5  // 5
#define OMRON_COE_BT2_1            0xA4  // 4
#define OMRON_COE_BT1_0            0xA3  // 3
#define OMRON_COE_BT1_1            0xA2  // 2
#define OMRON_COE_B00_0            0xA1  // 1
#define OMRON_COE_B00_1            0xA0  // 0

/**
 * @brief OMRON 2SMPB-02E description setting.
 * @details Specified setting for the description of OMRON 2SMPB-02E driver.
 */

/**
 * @brief OMRON 2SMPB-02E software reset command.
 * @details Specified software reset command of OMRON 2SMPB-02E driver.
 */
#define OMRON_CMD_RESET                0xE6

/**
 * @brief OMRON 2SMPB-02E average time for measurements.
 * @details Specified setting for the average time for measurements of OMRON 2SMPB-02E driver.
 */
#define OMRON_AVG_SKIP                 0x00
#define OMRON_AVG_1                    0x01
#define OMRON_AVG_2                    0x02
#define OMRON_AVG_4                    0x03
#define OMRON_AVG_8                    0x04
#define OMRON_AVG_16                   0x05
#define OMRON_AVG_32                   0x06
#define OMRON_AVG_64                   0x07

/**
 * @brief OMRON 2SMPB-02E operating mode.
 * @details Specified setting for the operating mode of OMRON 2SMPB-02E driver.
 */
#define OMRON_SLEEP_MODE               0x00
#define OMRON_FORCED_MODE_1            0x01
#define OMRON_FORCED_MODE_2            0x02 // Same as OMRON_FORCED_MODE_1
#define OMRON_NORMAL_MODE              0x03

/**
 * @brief OMRON 2SMPB-02E data ready options.
 * @details Specified options of data ready flag of OMRON 2SMPB-02E driver.
 */
#define OMRON_DATA_NOT_READY           0x00
#define OMRON_DATA_READY               0x01

/**
 * @brief OMRON 2SMPB-02E standby time.
 * @details Specified standby time of OMRON 2SMPB-02E driver.
 */
#define OMRON_STANDBY_1ms              0x00
#define OMRON_STANDBY_5ms              0x01
#define OMRON_STANDBY_50ms             0x02
#define OMRON_STANDBY_250ms            0x03
#define OMRON_STANDBY_500ms            0x04
#define OMRON_STANDBY_1s               0x05
#define OMRON_STANDBY_2s               0x06
#define OMRON_STANDBY_4s               0x07

/**
 * @brief OMRON 2SMPB-02E IIR Filter N coefficient.
 * @details Specified IIR Filter coefficient.
 */
#define OMRON_IIR_COEFF_0              0x00
#define OMRON_IIR_COEFF_2              0x01
#define OMRON_IIR_COEFF_4              0x02
#define OMRON_IIR_COEFF_8              0x03
#define OMRON_IIR_COEFF_16             0x04
#define OMRON_IIR_COEFF_32             0x05

/**
 * @brief OMRON 2SMPB-02E ID register value.
 * @details Specified chip ID of OMRON 2SMPB-02E driver.
 */
#define OMRON_ID_VALUE                 0x5C

// Conversion coefficients
#define A_a1    -0.0063
#define S_a1    0.00043
#define A_a2    -0.000000000019
#define S_a2    0.00000000012
#define A_bt1   0.100000000000000000
#define S_bt1   0.091000000000000000
#define A_bt2   0.000000012000000000
#define S_bt2   0.000001200000000000
#define A_bp1   0.033000000000000000
#define S_bp1   0.019000000000000000
#define A_b11   0.000000210000000000
#define S_b11   0.000000140000000000
#define A_bp2   -0.000000000630000000
#define S_bp2   0.000000000350000000
#define A_b12   0.000000000000290000
#define S_b12   0.000000000000760000
#define A_b21   0.000000000000002100
#define S_b21   0.000000000000012000
#define A_bp3   0.000000000000000130
#define S_bp3   0.000000000000000079

#define OMRON_COEFFICIENT_DIVIDER      32767.0
#define OMRON_MAX_23BIT_VALUE         0x0007FFFF
#define OMRON_MAX_24BIT_VALUE         0x000FFFFF



/**
 * @brief Get the chip ID of the OMRON sensor.
 *
 * @return The chip ID value.
 */
uint8_t OMROM_2SMPB_02E_Get_Chip_ID(void);

/**
 * @brief Reset the OMRON sensor.
 */
void OMROM_2SMPB_02E_Reset(void);

/**
 * @brief Initialize the OMRON sensor.
 *
 * @return 0 if successful, non-zero on failure.
 */
uint16_t OMROM_2SMPB_02E_Init(void);

/**
 * @brief Set the power mode of the OMRON sensor.
 *
 * @param mode The desired power mode.
 */
void OMROM_2SMPB_02E_setPowerMode(uint8_t mode);

/**
 * @brief Read compensated temperature from the sensor.
 *
 * @return The compensated temperature value.
 */
float OMROM_2SMPB_02E_Read_Comp_Temp(void);

/**
 * @brief Read compensated pressure from the sensor.
 *
 * @return The compensated pressure value.
 */
float OMROM_2SMPB_02E_Read_Comp_Press(void);

/**
 * @brief Check if data is ready to read.
 *
 * @return The status OMRON_DATA_NOT_READY (0) or OMRON_DATA_READY (1).
 */
uint8_t OMROM_2SMPB_02E_Check_Ready(void);

/**
 * @brief Set the filter coefficient.
 *
 * @param n_coeff The filter coefficient to set.
 */
void OMROM_2SMPB_02E_Set_Filter(uint8_t n_coeff);

/**
 * @brief Set the standby time.
 *
 * @param T_Standby The standby time to set.
 */
void OMROM_2SMPB_02E_Set_Standby_Time(uint8_t T_Standby);

/**
 * @brief Set the averaging parameters for temperature and pressure.
 *
 * @param Avg_Temp   The averaging parameter for temperature.
 * @param Avg_Press  The averaging parameter for pressure.
 */
void OMROM_2SMPB_02E_set_Avg(uint8_t Avg_Temp, uint8_t Avg_Press);


#endif	/* OMROM_2SMPB_02E_H */
