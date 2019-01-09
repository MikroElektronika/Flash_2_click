![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Flash_2 Click

- **CIC Prefix**  : FLASH2
- **Author**      : MikroE Team
- **Verison**     : 1.0.0
- **Date**        : jan 2018.

---

### Software Support

We provide a library for the Flash_2 Click on our [LibStock](http://libstock.mikroe.com/projects/view/1785/flash-2-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

The library contains all the necessary functions for working with Flash 2 click.

Key functions :

- ```void flash2_read( uint32_t address, uint8_t *buffer,uint32_t data_count );  ``` -  Function that reads data from selected address 
- ```void flash2_write( uint32_t address, uint8_t *buffer,  uint32_t data_count );  ``` - Function that writes data to  selected address 
- ``` void flash2_chipErase( void ); ``` - The chip erase function clears all bits in the device 1.

**Examples Description**

- System Initialization - GPIO and SPI module Initalization 
- Application Initialization - Flash Driver Initialization, initialization of click by setting mikorBUS to
  approprieate logic levels, performing global block unlock and chip erase functions.
- Application Task - Writing data to click memory and displaying the read data via UART.


```.c
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
```

The full application code, and ready to use projects can be found on our 
[LibStock](http://libstock.mikroe.com/projects/view/1785/flash-2-click) page.

Other mikroE Libraries used in the example:

- SPI
- UART

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
