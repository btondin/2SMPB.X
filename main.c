/**
 * @file main.c
 * @brief Main application for Omron 2SMPB-02E Sensor with PIC16 Microcontroller.
 * @details This application initializes the PIC microcontroller and reads data from the Omron 2SMPB-02E sensor,
 * displaying temperature and pressure readings. It also toggles an LED as a visual indicator.
 *
 * @author Bruno Rodriguez Tondin
 * @date 10/09/2023
 * @version 1.0
 *
 * @note This software is released under the MIT License:
 * @note Permission is hereby granted, free of charge, to any person obtaining a copy
 * @note of this software and associated documentation files (the "Software"), to deal
 * @note in the Software without restriction, including without limitation the rights
 * @note to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * @note copies of the Software, and to permit persons to whom the Software is
 * @note furnished to do so, subject to the following conditions:
 * @note The above copyright notice and this permission notice shall be included
 * @note in all copies or substantial portions of the Software.
 *
 * @note THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * @note EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * @note OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * @note IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * @note DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
 * @note OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
 * @note USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "mcc_generated_files/system.h"
#include "pin_manager.h"
#include "clock.h"
#include <stdio.h>
#define FCY _XTAL_FREQ
#include <libpic30.h>
#include "OMRON_2SMPB_02E.h"

int main(void)
{
    // Initialize the device
    SYSTEM_Initialize();

    __delay_ms(200);
    OMRON_2SMPB_02E_Init();
    printf("\n\n\nStart... Reading Chip ID: %X", OMRON_2SMPB_02E_Get_Chip_ID());
    
    OMRON_2SMPB_02E_set_Avg(OMRON_AVG_64, OMRON_AVG_64); //Set Average times ( Temp, press )    
    OMRON_2SMPB_02E_Set_Filter(OMRON_IIR_COEFF_32); //Set IIR Filtetr    
    OMRON_2SMPB_02E_setPowerMode(OMRON_SLEEP_MODE); //Set Power mode

    while (1)
    {
        LED1_Toggle();
        __delay_ms(1000);
        
        OMRON_2SMPB_02E_setPowerMode(OMRON_FORCED_MODE_1); //Begin measurements
        
        uint16_t attempts = 0;        
        while (OMRON_2SMPB_02E_Check_Ready () == OMRON_DATA_NOT_READY) //Check for data ready
        {            
            attempts++;
            __delay_us(100);
        }

        printf("\n\n Temperature: %.2f °C, Pressure: %.0f hPa", OMRON_2SMPB_02E_Read_Comp_Temp(), OMRON_2SMPB_02E_Read_Comp_Press() / 100);
        printf("\n Attempts: %u", attempts); // Attempts before data is ready

        // printf("\n\nChipID: %X", OMRON_2SMPB_02E_Get_Chip_ID());
    }

    return 1;
}
/**
 End of File
*/
