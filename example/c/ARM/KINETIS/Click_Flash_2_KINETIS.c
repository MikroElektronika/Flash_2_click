/*
Example for Flash_2 Click

    Date          : jan 2018.
    Author        : MikroE Team

Test configuration KINETIS :
    
    MCU              : MK64
    Dev. Board       : HEXIWEAR
    ARM Compiler ver : v6.0.0.0

---
Description :

The application is composed of three sections :

- System Initialization - GPIO and SPI module Initalization 
- Application Initialization - Flash Driver Initialization, initialization of click by setting mikorBUS to
  approprieate logic levels, performing global block unlock and chip erase functions.
- Application Task - Writing data to click memory and displaying the read data via UART. 

*/

#include "Click_Flash_2_types.h"
#include "Click_Flash_2_config.h"


char wrData [10] = { 'M', 'i', 'k', 'r', 'o', 'E', 13, 10, 0 };
char rdData [10];

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_PWM_PIN, _GPIO_OUTPUT );
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_RST_PIN, _GPIO_OUTPUT );

    mikrobus_spiInit( _MIKROBUS1, &_FLASH2_SPI_CFG[0] );
    mikrobus_logInit( _MIKROBUS2, 9600 );

    Delay_ms( 100 );
}

void applicationInit()
{
    flash2_spiDriverInit( (T_FLASH2_P)&_MIKROBUS1_GPIO, (T_FLASH2_P)&_MIKROBUS1_SPI );
    flash2_init();
    Delay_ms(300);
    flash2_globalBlockUnlock();
    Delay_ms(400);
    flash2_chipErase();
    Delay_ms(300);
}

void applicationTask()
{
   
    mikrobus_logWrite("Writing MikroE to  Flash memory, from address 0x015015:",_LOG_LINE);
    flash2_write (0x015015, &wrData[0], 9);
    mikrobus_logWrite("Reading 9 bytes of Flash memory, from address 0x015015:",_LOG_LINE);
    flash2_read(0x015015,&rdData[0],9);
    mikrobus_logWrite("Data read: ",_LOG_TEXT);
    mikrobus_logWrite(rdData,_LOG_LINE);
    Delay_ms(1000);

}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}