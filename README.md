> ![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)
> #Flash 2 Click#
> ##By [MikroElektronika](http://www.mikroe.com)
---

## Installation
Use the package manager to install the library for your architecture.

###Example on ARM
```
#include <stdint.h>

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

```

```
// Main function

int main(void)
{
    setup();

    flash_2_init();
    Delay_ms(300);
    
    while(1)
    {}

}
```


