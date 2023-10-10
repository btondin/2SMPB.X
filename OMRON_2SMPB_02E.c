#include "OMRON_2SMPB_02E.h"
#include "i2c1.h"
#include "clock.h"
#define FCY _XTAL_FREQ
#include <libpic30.h>
#include "pin_manager.h"
#include <stdint.h>

uint8_t NVM_coeffs[25]; //Coeficientes de compensa??o 
float a0, a1, a2, b00, bt1, bt2, bp1, b11, bp2, b12, b21, bp3;


uint16_t OMRON_2SMPB_02E_Read(uint8_t address, uint8_t *pData, uint16_t nCount)
{
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING; // Defines the different message status when processing TRBs.
    uint8_t     writeBuffer[2];   //Sensor tem endereços de apenas 8 bits, portanto foi trocado 3 para 2
    uint16_t    retryTimeOut, slaveTimeOut;
    uint16_t    counter;
    uint8_t     *pD;
    
    pD = pData; //Copia ponteiro (endereço) do vetor de dados

    for (counter = 0; counter < nCount; counter++)
    {

        // build the write buffer first
        // starting address of the EEPROM memory
        //writeBuffer[0] = (address >> 8);                // high address
        //writeBuffer[1] = (uint8_t)(address);            // low low address
        writeBuffer[0] = address; // Sensor tem apenas 8 bits de endereçamento

        // Now it is possible that the slave device will be slow.
        // As a work around on these slaves, the application can
        // retry sending the transaction        
        retryTimeOut = 0;
        while(status != I2C1_MESSAGE_FAIL)
        {
            // write one byte to EEPROM (1 is the count of bytes to write)
            I2C1_MasterWrite(    writeBuffer,
                                    1,
                                    OMRON_SLAVE_ADDR,
                                    &status);   // mudado de 2 para 1 pois dispositivo tem endereço de 8 bit

            // wait for the message to be sent or status has changed.
            slaveTimeOut = 0;
            while(status == I2C1_MESSAGE_PENDING) // Fica nesta rotina até mudança de status por interrupção ou estorurar timeout
            {
                // add some delay here
                __delay_us(100);

                // timeout checking
                // check for max retry and skip this byte
                if (slaveTimeOut == OMRON_RETRY_MAX)
                    return (0);
                else
                    slaveTimeOut++;
            }
            // Terminou de escrever. Na sequencia verifica o status

            if (status == I2C1_MESSAGE_COMPLETE)
                break;

            
            if ((status == I2C1_DATA_NO_ACK)||(status == I2C1_MESSAGE_ADDRESS_NO_ACK))
                {
                // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
                //               or I2C1_DATA_NO_ACK,
                // The device may be busy and needs more time for the last
                // write so we can retry writing the data, this is why we
                // use a while loop here                   
                }

            // check for max retry and skip this byte
            if (retryTimeOut == OMRON_RETRY_MAX) 
                break;            
            else
                retryTimeOut++;
        }

        if (status == I2C1_MESSAGE_COMPLETE)
        {
            // this portion will read the byte from the memory location.
            retryTimeOut = 0;            
            status = I2C1_MESSAGE_PENDING;

            while(status != I2C1_MESSAGE_FAIL)
            {
                // write one byte to EEPROM (2 is the count of bytes to write)
                I2C1_MasterRead(     pD,
                                        1,
                                        OMRON_SLAVE_ADDR,
                                        &status);

                // wait for the message to be sent or status has changed.
                slaveTimeOut = 0;
                while(status == I2C1_MESSAGE_PENDING)
                {
                    __delay_us(100);

                    // timeout checking
                    // check for max retry and skip this byte
                    if (slaveTimeOut == OMRON_DEVICE_TIMEOUT)
                        return (0);
                    else
                        slaveTimeOut++;
                }

                if (status == I2C1_MESSAGE_COMPLETE)
                    break;
                               
                if ((status == I2C1_DATA_NO_ACK)||(status == I2C1_MESSAGE_ADDRESS_NO_ACK))
                {
                    // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
                    //               or I2C1_DATA_NO_ACK,
                    // The device may be busy and needs more time for the last
                    // write so we can retry writing the data, this is why we
                    // use a while loop here                     
                }

                // check for max retry and skip this byte
                if (retryTimeOut == OMRON_RETRY_MAX)
                    break;
                else
                    retryTimeOut++;
            }
        }

        // exit if the last transaction failed
        if (status == I2C1_MESSAGE_FAIL)
        {
            return(0);
            break;
        }

        pD++;
        address++;
    }
    return(1);
}

uint16_t OMRON_2SMPB_02E_Write(uint8_t address, uint8_t *pData, uint16_t nCount)
{
    I2C1_MESSAGE_STATUS status = I2C1_MESSAGE_PENDING;
    uint8_t     writeBuffer[2];   //Sensor tem endereços de apenas 8 bits
    uint16_t    retryTimeOut, slaveTimeOut;
    uint16_t    counter;
    uint8_t     *pD;
    
    pD = pData;

    for (counter = 0; counter < nCount; counter++)
    {
        // build the write buffer first
        // starting address of the EEPROM memory
        //writeBuffer[0] = (address >> 8);                // high address
        //writeBuffer[1] = (uint8_t)(address);            // low low address
        writeBuffer[0] = address; // Sensor tem apenas 8 bits de endereçamento
        
        // data to be written
        writeBuffer[1] = *pD++;

        // Now it is possible that the slave device will be slow.
        // As a work around on these slaves, the application can
        // retry sending the transaction
        retryTimeOut = 0;
        while(status != I2C1_MESSAGE_FAIL)
        {
            // write one byte to EEPROM (1 is the count of bytes to write)
            I2C1_MasterWrite(    writeBuffer,
                                    2,
                                    OMRON_SLAVE_ADDR,
                                    &status);   // mudado de 2 para 1 pois dispositivo tem endereço de 8 bit

            // wait for the message to be sent or status has changed.
            slaveTimeOut = 0;
            while(status == I2C1_MESSAGE_PENDING)
            {
                // add some delay here
                __delay_us(100);

                // timeout checking
                // check for max retry and skip this byte
                if (slaveTimeOut == OMRON_DEVICE_TIMEOUT)
                    return (0);
                else
                    slaveTimeOut++;
            }

            if ((status == I2C1_MESSAGE_COMPLETE) || (slaveTimeOut == OMRON_DEVICE_TIMEOUT))
                break;

            // if status is  I2C1_MESSAGE_ADDRESS_NO_ACK,
            //               or I2C1_DATA_NO_ACK,
            // The device may be busy and needs more time for the last
            // write so we can retry writing the data, this is why we
            // use a while loop here

            // check for max retry and skip this byte
            if (retryTimeOut == OMRON_RETRY_MAX)
                break;
            else
                retryTimeOut++;
        }

        // exit if the last transaction failed
        if (status == I2C1_MESSAGE_FAIL)
        {
            return(0);
            break;
        }

        //pD++; // Tirar ou não?
        address++;

    }
    return(1);
}


uint8_t OMRON_2SMPB_02E_Get_Chip_ID(void)
{
    uint8_t data;
    uint16_t result; 
    result = OMRON_2SMPB_02E_Read(OMRON_CHIP_ID, &data, 1);
    return data;
}

void OMRON_2SMPB_02E_Reset()    //Sensor Reset
{   
    uint8_t data = OMRON_CMD_RESET;
    uint16_t result;
    result = OMRON_2SMPB_02E_Write(OMRON_RESET, &data, 1);
    __delay_ms(10);
} 

/**/
uint8_t OMRON_2SMPB_02E_Teste_RW()    //Teste pra saber se está lendo corretamente
{   
    uint8_t dataW[2] = {0xC0,0x80};
    uint8_t dataR[2] = {0xAB,0xCD};
    uint16_t result;
    result = OMRON_2SMPB_02E_Write( OMRON_CTRL_MEAS, dataW, 2);
    result = OMRON_2SMPB_02E_Read(OMRON_CTRL_MEAS, dataR, 2);
    //I2C_Write1ByteRegister(OMRON_SLAVE_ADDR, OMRON_RESET, 0xE6);
    return(dataR[0]);
}


void OMRON_2SMPB_02E_clearRegisterBit(uint8_t reg, uint8_t bitMask)
{
    uint8_t temper = 0x00;     
    uint16_t result; 
    result = OMRON_2SMPB_02E_Read(reg, &temper, 1);       
    temper &= ~bitMask;                           // Clear the bit from the value
    result = OMRON_2SMPB_02E_Write(reg, &temper, 1);       
}

void OMRON_2SMPB_02E_setRegisterBit(uint8_t reg, uint8_t bitMask)
{
    uint8_t temper = 0x00;     
    uint16_t result; 
    result = OMRON_2SMPB_02E_Read(reg, &temper, 1);       
    temper |= bitMask;                           // Clear the bit from the value
    result = OMRON_2SMPB_02E_Write(reg, &temper, 1);       
}

void OMRON_2SMPB_02E_set_Avg(uint8_t Avg_Temp, uint8_t Avg_Press)
{
    uint8_t temper = 0x00;     
    uint16_t result; 
    result = OMRON_2SMPB_02E_Read(OMRON_CTRL_MEAS, &temper, 1); 
    temper &= 0b00000011; 
    temper |= ( Avg_Temp << 5 );
    temper |= ( Avg_Press << 2 );                              
    result = OMRON_2SMPB_02E_Write(OMRON_CTRL_MEAS, &temper, 1);       
}

void OMRON_2SMPB_02E_setPowerMode(uint8_t mode){    //Define o modo do sensor
                                //OMRON_SLEEP_MODE
                                //OMRON_FORCED_MODE_1
                                //OMRON_FORCED_MODE_2
                                //OMRON_NORMAL_MODE    
    uint8_t aux = 0; 
    uint16_t result; 
    
    result = OMRON_2SMPB_02E_Read(OMRON_CTRL_MEAS, &aux, 1);   
    
    aux &= 0b11111100;                   // Clear out old MODE bits      
    aux |= mode;                 // Mask in new MODE bits    
    result = OMRON_2SMPB_02E_Write(OMRON_CTRL_MEAS, &aux, 1);
} 


void OMRON_2SMPB_02E_Set_Standby_Time(uint8_t T_Standby) 
{
    uint8_t temper = 0x00;     
    uint16_t result; 
    result = OMRON_2SMPB_02E_Read(OMRON_IO_SETUP, &temper, 1); 
    temper &= 0b00011111; 
    temper |= ( T_Standby << 5 );                                  
    result = OMRON_2SMPB_02E_Write(OMRON_IO_SETUP, &temper, 1);       
}

void OMRON_2SMPB_02E_Set_Filter(uint8_t n_coeff)
{  
    uint8_t temper = 0x00; 
    uint16_t result;     
    temper = n_coeff;                                
    result = OMRON_2SMPB_02E_Write(OMRON_IIR_CNT, &temper, 1);
}
 
uint8_t OMRON_2SMPB_02E_Check_Ready ( void )
{
    uint8_t temper;
    uint16_t result;    
    
    result = OMRON_2SMPB_02E_Read(OMRON_DEVICE_STAT, &temper, 1);
    
    if (  ( temper & 0x08 ) >= 0x08 )
    {
        return OMRON_DATA_NOT_READY;
    }
    else
    {        
        return OMRON_DATA_READY;
    }
}

int32_t OMRON_2SMPB_02E_Read_Raw_Temp ( void )
{
    uint16_t result = 0; 
    uint8_t rx_buf[ 3 ] = {0,0,0};
    uint32_t temp_data = 0;         
    
    result = OMRON_2SMPB_02E_Read(OMRON_TEMP_TXD2, rx_buf, 3);
    
    temp_data = ( ( uint32_t ) rx_buf[ 0 ] << 16 ) | ( ( uint32_t ) rx_buf[ 1 ] << 8 ) | rx_buf[ 2 ];       
    
    return (( int32_t )temp_data) - 0x00800000; //Conversion
}

float OMRON_2SMPB_02E_Read_Raw_Press ( void )
{
    uint16_t result; 
    uint8_t rx_buf[ 3 ];
    uint32_t press_data; 
    float prs;    
    
    result = OMRON_2SMPB_02E_Read(OMRON_PRESS_TXD2, rx_buf, 3);
    
    press_data = ( ( uint32_t ) rx_buf[ 0 ] << 16 ) | ( ( uint32_t ) rx_buf[ 1 ] << 8 ) | rx_buf[ 2 ];

    //temp_data = (( int32_t )temp_data) - pow( 2, 23 );  
    prs = ( float )(press_data - pow( 2, 23 ));   
    
    return prs;
}


uint16_t OMRON_2SMPB_02E_Read_OTP(uint8_t *coeffs) //L? coeficientes da NVM
{    
    return(OMRON_2SMPB_02E_Read(OMRON_COE_B00_1, coeffs, 25));       
}

float OMRON_2SMPB_02E_Get_Comp_Coeffs ( uint16_t OTP_val, float A_factor, float S_factor )
{
    
    int16_t OTP_signed = 0;
    
    OTP_signed = (int16_t)OTP_val;
    
    OTP_signed = -(OTP_signed & (1 << 15)) + (OTP_signed & ~(1 << 15)); // 2's complement    
    
    
    return ( (float)(A_factor + ( S_factor *( (( float ) OTP_signed) / OMRON_COEFFICIENT_DIVIDER ) )));
}

float OMRON_2SMPB_02E_Read_Comp_Temp(void )
{
    int32_t Dt = 0;
    float tr = 0;
    
    Dt = OMRON_2SMPB_02E_Read_Raw_Temp();
    
    tr = (float) ( a0 + a1 * Dt + a2 * pow( Dt, 2 ) );     
      
    
    return (tr/256);
}


float OMRON_2SMPB_02E_Read_Comp_Press(void )
{
    int32_t Dt = 0;
    float Dp = 0, Tr = 0, pr = 0;
    
    Dt = OMRON_2SMPB_02E_Read_Raw_Temp(); 
    Dp = OMRON_2SMPB_02E_Read_Raw_Press();
    Tr = (float)(( a0 + a1 * Dt + a2 * pow( Dt, 2 ) ));     
    
        
    pr = (float) ( b00 + bt1 * Tr + bp1 * Dp + b11 * Dp * Tr + bt2 * pow( Tr, 2 ) + bp2 * pow( Dp, 2 ) + b12 * Dp * pow( Tr, 2 ) + b21 * pow( Dp, 2 ) * Tr + bp3 * pow( Dp, 3 ) );    
      
    
    return (pr);
}


uint16_t OMRON_2SMPB_02E_Init(void)
{    
    uint16_t result, aux = 0;   
    int32_t tmp = 0;
    
    __delay_ms(10); //Aguarda POR
    //OMRON_2SMPB_02E_Reset(); //Reseta sensor (opcional pois há POR)  
    
    //uint8_t NVM_coeffs[25]; //Coeficientes de compensa??o 
    result = OMRON_2SMPB_02E_Read_OTP(NVM_coeffs); //Coeficientes de compensa??o); //L? os coeficientes 
    
    OMRON_2SMPB_02E_setPowerMode(OMRON_SLEEP_MODE); //Modo dormindo
    
    //a0    
    
    tmp = (int32_t)(( ( uint32_t ) NVM_coeffs[ 18 ] << 12 ) | ( ( uint32_t ) NVM_coeffs[ 19 ] << 4 ) | ( uint32_t )( NVM_coeffs[ 24 ] & 0x0F ));
    tmp = -(tmp & (uint32_t)1 << 19) + (tmp & ~((uint32_t)1 << 19)); // 2's complement
    
    a0 = ( float ) tmp / 16;
    
    
    //b00
    
    tmp = 0;
    tmp = (int32_t)(( ( uint32_t ) NVM_coeffs[ 0 ] << 12 ) | ( ( uint32_t ) NVM_coeffs[ 1 ] << 4 ) | ( uint32_t )( NVM_coeffs[ 24 ] >> 4 ));
    tmp = -(tmp & (uint32_t)1 << 19) + (tmp & ~((uint32_t)1 << 19)); // 2's complement
    
    b00 = ( float ) tmp / 16;
    //b00 = 286421;
    
    //a1
    aux = NVM_coeffs[ 20 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 21 ];
    a1 = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_a1, S_a1 );
    
    //a2
    aux = NVM_coeffs[ 22 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 23 ];
    a2  = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_a2, S_a2 );
    
    //bt1
    aux = NVM_coeffs[ 2 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 3 ];
    bt1 = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_bt1, S_bt1 );
    
    //bt2
    aux = NVM_coeffs[ 4 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 5 ];
    bt2 = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_bt2, S_bt2 );
    
    //bp1
    aux = NVM_coeffs[ 6 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 7 ];
    bp1 = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_bp1, S_bp1 );
    
    //b11
    aux = NVM_coeffs[ 8 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 9 ];
    b11 = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_b11, S_b11 );
    
    //bp2
    aux = NVM_coeffs[ 10 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 11 ];
    bp2 = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_bp2, S_bp2 );
    
    //b12
    aux = NVM_coeffs[ 12 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 13 ];
    b12 = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_b12, S_b12 );
    
    //b21
    aux = NVM_coeffs[ 14 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 15 ];
    b21 = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_b21, S_b21 );
    
    //bp3
    aux = NVM_coeffs[ 16 ]; aux <<= 8 ;  aux |= NVM_coeffs[ 17 ];
    bp3 = OMRON_2SMPB_02E_Get_Comp_Coeffs ( aux, A_bp3, S_bp3 );
    
    //Set Average times
    OMRON_2SMPB_02E_set_Avg(OMRON_AVG_32, OMRON_AVG_32); // Temp, press  
    
    //Set IIR Filter
    OMRON_2SMPB_02E_Set_Filter(OMRON_IIR_COEFF_32);
    
    
    __delay_ms(1);
    return(result);
}
/*
uint8_t OMRON_2SMPB_02E_dataReady(uint8_t mask)
{
    uint16_t attempts = 0;       
    
    while ((I2C_Read1ByteRegister(OMRON_SLAVE_ADDR, OMRON_DEVICE_STAT) & mask) == mask)
    {
        attempts++;
        
        if(attempts > MAX_DATA_READY_ATTEMPTS) 
            return 0; // Failed            
        __delay_ms(1);
    }
    
    //printf("ATTEMPTS %lu\n", attempts);
    
    return 1; // Success
}

*/