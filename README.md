![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

![Flash 2 Click](http://cdn.mikroe.com/img/banners/news/2016/05/flash-2-click-banner-news.png)

---
[Product Page](http://www.mikroe.com/click/flash-2/)

[Manual Page](http://docs.mikroe.com/Flash_2_click)

[Learn Page](http://learn.mikroe.com/this-nand-nor-that-nand/)

---

### General Description

Flash 2 click is a mikroBUS™ add-on board for adding more Flash Memory to your target board microcontroller. It carries Microchip’s SST26VF064B flash-memory module with 64 Mbits capacity. It’s a highly reliable module with a specified minimum of 100,000 read and write cycles and with over 100 years of Data Retention. For data security, the module features a One-Time Programmable (OTP) 2 KB bit secure ID and a 64 bit unique, factory pre-programmed identifier. Additional software security measures include inidividual-block write Protection with permanent lock-down capability. Flash 2 click communicates with the target MCU through the mikroBUS™ SPI interface (CS#, SCK, MISO, MOSI) with additional functionality provided by the #HOLD pin (in place of default mikroBUS™ RST pin). The board is designed to use a 3.3V power supply.

---

### Example

#### Configuration
* MCU:             STM32F107VC
* Dev.Board:       EasyMx Pro v7
* Oscillator:      72 Mhz internal
* Ext. Modules:    CLICKNAME click
* SW:              MikroC PRO for ARM 4.7.4

```
#include <stdint.h>

/*      Functions
 ****************************/


sbit FLASH_2_WP at GPIOA_ODR.B0;
sbit FLASH_2_CS at GPIOD_ODR.B13;
sbit FLASH_2_HLD at GPIOC_ODR.B2;

void setup()
{

    GPIO_Digital_Output( &GPIOA_BASE, _GPIO_PINMASK_0 );
    GPIO_Digital_Output( &GPIOC_BASE, _GPIO_PINMASK_2 );
    GPIO_Digital_Output( &GPIOD_BASE, _GPIO_PINMASK_13 );

    SPI1_Init_Advanced( _SPI_FPCLK_DIV64,
                          _SPI_MASTER | _SPI_8_BIT | _SPI_CLK_IDLE_LOW |
                          _SPI_FIRST_CLK_EDGE_TRANSITION | _SPI_MSB_FIRST |
                          _SPI_SS_DISABLE | _SPI_SSM_ENABLE |
                          _SPI_SSI_1,
                          &_GPIO_MODULE_SPI3_PC10_11_12 );

    Delay_ms(300);

}


int main(void)
{
    setup();

    flash_2_init();
    Delay_ms(300);
    
    while(1)
    {}

}
```