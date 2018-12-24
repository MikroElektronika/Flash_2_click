/*
    __flash2_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__flash2_driver.h"
#include "__flash2_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __FLASH2_DRV_I2C__
static uint8_t _slaveAddress;
#endif


const uint8_t _FLASH2_STATUS_WEL   =  0x02; /**< Write-enable latch status 1 = device is write-enabled
                            0 = device is not write-enabled */
const uint8_t _FLASH2_STATUS_WSE   =  0x04; /**< Write suspend-erase status 1 = erase suspended 0 =
                            erase is not suspended */
const uint8_t _FLASH2_STATUS_WSP   =  0x08; /**< Write suspend-program status 1 = program suspended
                            0 = program is not suspended */
const uint8_t _FLASH2_STATUS_WPLD  =  0x10; /**< Write protections lock-down status 1 = write
                            protection lock-down enabled 0 = write protection
                            lock-down disabled */
const uint8_t _FLASH2_STATUS_SEC   =  0x20; /**< Security ID status 1 = security ID space locked
                            0 = security ID space not locked */
const uint8_t _FLASH2_STATUS_RES   =  0x40;  /**< Reserved for future use */
const uint8_t _FLASH2_STATUS_BUSY  =  0x80;  /**< Write operation status 1 = write in progress 0 =
                            no operation in progress */



const uint8_t _FLASH2_CFG_RES   =  0x01; /**< Reserved */
const uint8_t _FLASH2_CFG_IOC   =  0x02; /**< I/O configuration for SPI mode 1 = WP and
                                    HOLD pins disabled 0 = WP and HOLD enabled*/
const uint8_t _FLASH2_CFG_BPNV  =  0x08; /**< Block-protection volatility state 1 = no
                                   memory block permanently locked 0 = any block
                                    has been permanently locked */

const uint8_t _FLASH2_CFG_WPEN  =  0x80; /**< Write-protection pin (WP) enable 1 = enabled
                                    0 = disabled */


const uint8_t _FLASH2_INSTR_NOP      =    0x00; /**< No Operation */
const uint8_t _FLASH2_INSTR_RSTEN    =    0x66; /**< Reset Enable */
const uint8_t _FLASH2_INSTR_RST      =    0x99; /**< Reset Memory */
const uint8_t _FLASH2_INSTR_EQIO     =    0x38; /**< Enable Quad I/O */
const uint8_t _FLASH2_INSTR_RSTQIO   =    0xFF; /**< Reset Quad I/O */
const uint8_t _FLASH2_INSTR_RDSR     =    0x05; /**< Read Status Register */
const uint8_t _FLASH2_INSTR_WRSR     =    0x01; /**< Write Status Register */
const uint8_t _FLASH2_INSTR_RDCR     =    0x35; /**< Read Configuration Register */
const uint8_t _FLASH2_INSTR_READ     =    0x03; /**< Read Memory */
const uint8_t _FLASH2_INSTR_HS_READ  =    0x0B; /**< Read Memory at Higher Speed */
const uint8_t _FLASH2_INSTR_SQOR     =    0x6B; /**< SPI Quad Output Read */
const uint8_t _FLASH2_INSTR_SQIOR    =    0xEB; /**< SPI Quad I/O Read */
const uint8_t _FLASH2_INSTR_SDOR     =    0x3B; /**< SPI Dual Output Read */
const uint8_t _FLASH2_INSTR_SDIOR    =    0xBB; /**< SPI Dual I/O Read */
const uint8_t _FLASH2_INSTR_SB       =    0xC0; /**< Set Burst Length */
const uint8_t _FLASH2_INSTR_RBSQI    =    0x0C; /**< SQI Read Burst with Wrap */
const uint8_t _FLASH2_INSTR_RBSPI    =    0xEC; /**< SPI Read Burst with Wrap */
const uint8_t _FLASH2_INSTR_JEDECID  =    0x9F; /**< JEDEC-ID Read */
const uint8_t _FLASH2_INSTR_QUAD_JID =    0xAF; /**< Quad I/O J-ID read */
const uint8_t _FLASH2_INSTR_SFDP     =    0x5A; /**< Serial Flash Discoverable Parameters */
const uint8_t _FLASH2_INSTR_WREN     =    0x06; /**< Write Enable */
const uint8_t _FLASH2_INSTR_WRDI     =    0x04; /**< Write Disable */
const uint8_t _FLASH2_INSTR_SE       =    0x20; /**< Erase 4 KBytes of Memory Array */
const uint8_t _FLASH2_INSTR_BE       =    0xD8; /**< Erase 64, 32 or 8 KBytes of Memory Array */
const uint8_t _FLASH2_INSTR_CE       =    0xC7; /**< Erase Full Array */
const uint8_t _FLASH2_INSTR_PP       =    0x02; /**< Page Program */
const uint8_t _FLASH2_INSTR_SPI_QUAD =    0x32; /**< SQI Quad Page Program */
const uint8_t _FLASH2_INSTR_WRSU     =    0xB0; /**< Suspends Program / Erase */
const uint8_t _FLASH2_INSTR_WRRE     =    0x30; /**< Resumes Program / Erase */
const uint8_t _FLASH2_INSTR_RBPR     =    0x72; /**< Read Block-Protection Register */
const uint8_t _FLASH2_INSTR_WBPR     =    0x42; /**< Write Block-Protection Register */
const uint8_t _FLASH2_INSTR_LBPR     =    0x8D; /**< Lock Down Block-Protection Register */
const uint8_t _FLASH2_INSTR_NVWLDR   =    0xE8; /**< Non-Volatile Write Lock-Down Register */
const uint8_t _FLASH2_INSTR_ULBPR    =    0x98; /**< Global Block Protection Unlock */
const uint8_t _FLASH2_INSTR_RSID     =    0x88; /**< Read Security ID */
const uint8_t _FLASH2_INSTR_PSID     =    0xA5; /**< Program User Security ID Area */
const uint8_t _FLASH2_INSTR_LSID     =    0x85; /**< Lockout Security ID Programming */

const uint8_t _FLASH2_START_PAGE_ADDRESS   =   0x010000;
const uint8_t _FLASH2_END_PAGE_ADDRESS     =   0x7FFFFF;
const uint8_t _FLASH2_FLASH_PAGE_SIZE      =   256;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */
void flash2_hal_command( uint8_t command );
void flash2_hal_write( uint8_t *buffer, uint16_t count );
uint8_t flash2_hal_read_byte();
void flash2_hal_read( uint8_t *buffer, uint16_t count );
void flash2_hal_write_address( uint32_t address );

/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */

void flash2_hal_command( uint8_t command )
{
    
    uint8_t temp[1];
    
    temp[0] = command;
    hal_spiWrite(temp,1);

}

void flash2_hal_write( uint8_t *buffer, uint16_t count )
{
    
    uint8_t *dataPtr;

    dataPtr = buffer;

    hal_spiWrite(dataPtr,count);
   
}

uint8_t flash2_hal_read_byte()
{
    uint8_t retVal[1];

    hal_spiRead(retVal,1);

    return retVal[0];

}

void flash2_hal_read( uint8_t *buffer,uint16_t count )
{
    uint8_t *dataPtr;
    
    dataPtr = buffer;
    
    hal_spiRead(dataPtr,count);    
}

void flash2_hal_write_address( uint32_t address )
{    
    uint8_t temp[3];
    temp[0] = (uint8_t)  (address >> 16);
    temp[1] = (uint8_t)  (( address & 0x00FF00 ) >> 8);
    temp[2] = (uint8_t)  ( address & 0x0000FF );

    hal_spiWrite(temp,3);

    return;
}

/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __FLASH2_DRV_SPI__

void flash2_spiDriverInit(T_FLASH2_P gpioObj, T_FLASH2_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );
    hal_gpio_csSet(1);
}

#endif
#ifdef   __FLASH2_DRV_I2C__

void flash2_i2cDriverInit(T_FLASH2_P gpioObj, T_FLASH2_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __FLASH2_DRV_UART__

void flash2_uartDriverInit(T_FLASH2_P gpioObj, T_FLASH2_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif

// GPIO Only Drivers - remove in other cases
void flash2_gpioDriverInit(T_FLASH2_P gpioObj)
{
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
}

/* ----------------------------------------------------------- IMPLEMENTATION */
void flash2_init()
{
    hal_gpio_csSet(1);
    hal_gpio_pwmSet(0);
    hal_gpio_rstSet(1);
}

uint8_t flash2_busy()
{
    uint8_t temp;

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RDSR );
    temp = flash2_hal_read_byte( );
    hal_gpio_csSet(1);
    temp &= _FLASH2_STATUS_BUSY;
    return temp;
}

uint8_t flash2_getStatusReg( void )
{
    uint8_t temp;

    //Status Reg is only instruction that doesn't need to wait for Busy bit
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RDSR );
    temp = flash2_hal_read_byte( );
    hal_gpio_csSet(1);

    return temp;
}

uint8_t flash2_eraseStatus( void )
{
    uint8_t temp;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RDSR );
    flash2_hal_read( &temp, 1);
    hal_gpio_csSet(1);
    temp &= _FLASH2_STATUS_WSE;

    return temp;
}

uint8_t flash2_writeStatus( void )
{
    uint8_t temp;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RDSR );
    flash2_hal_read( &temp, 1);
    hal_gpio_csSet(1);
    temp &= _FLASH2_STATUS_WEL;

    return temp;
}

uint8_t flash2_programStatus( void )
{
    uint8_t temp;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RDSR );
    flash2_hal_read( &temp, 1);
    hal_gpio_csSet(1);
    temp &= _FLASH2_STATUS_WSP;

    return temp;
}

uint8_t flash2_protectStatus( void )
{
    uint8_t temp;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RDSR );
    flash2_hal_read( &temp, 1);
    hal_gpio_csSet(1);
    temp &= _FLASH2_STATUS_WPLD;

    return temp;
}

void flash2_lockSecurityId( void )
{
    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_LSID );
    hal_gpio_csSet(1);

    return;

}

uint8_t flash2_securityStatus( void )
{
    uint8_t temp;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RDSR );
    flash2_hal_read( &temp, 1);
    hal_gpio_csSet(1);
    temp &= _FLASH2_STATUS_SEC;

    return temp;
}

void flash2_writeProtectEnable( void )
{
    hal_gpio_pwmSet(1);
}

void flash2_writeProtectDisable( void )
{
    hal_gpio_pwmSet(0);
}

void flash2_holdEnable( void )
{
    hal_gpio_csSet(0);
    hal_gpio_rstSet( 1 );
}

void flash2_holdDisable( void )
{
    hal_gpio_rstSet( 0 );
    hal_gpio_csSet(1);
}

void flash2_writeSuspend( void )
{
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_WRSU );
    hal_gpio_csSet(1);
}

void flash2_writeResume( void )
{
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_WRRE );
    hal_gpio_csSet(1);
}


void flash2_spiGetSecurityId( uint8_t *buffer, uint32_t data_count)
{

    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RSID );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;

}

void flash2_sqiGetSecurityId( uint8_t *buffer, uint32_t data_count)
{
    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RSID );
    flash2_hal_write( 0x00, 1 );
    flash2_hal_write( 0x00, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;

}

void flash2_setSecurityId( uint8_t *buffer,
                             uint32_t data_count )
{
    uint8_t addressL = 0;
    uint8_t addressH = 7;

    while( flash2_busy() );
    
    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_PSID );
    flash2_hal_write( &addressL, 1);
    flash2_hal_write( &addressH, 1);
    flash2_hal_write( buffer, data_count );
    hal_gpio_csSet(1);

    return;

}

void flash2_writeDisable( void )
{
    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_WRDI );
    hal_gpio_csSet(1);

    return;

}

void flash2_spiGetBpr(  uint8_t *buffer,
                           uint32_t data_count )
{
    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RBPR );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;
}

void flash2_sqiGetBpr( uint8_t *buffer,
                          uint32_t data_count  )
{
    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RBPR );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;
}

void flash2_setBpr(uint8_t *buffer )
{
    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_WBPR );
    flash2_hal_write( buffer, 18 );
    hal_gpio_csSet(1);

    return;
}

void flash2_lockBpr( void )
{
    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_LBPR );
    hal_gpio_csSet(1);

    return;
}

void flash2_nonvolatileWriteLock( uint8_t *buffer)
{
    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_NVWLDR );
    flash2_hal_write( buffer, 18 );
    hal_gpio_csSet(1);

    return;
}


void flash2_globalBlockUnlock( void )
{
    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_ULBPR );
    hal_gpio_csSet(1);

    return;
}

void flash2_read( uint32_t address, uint8_t *buffer,
                        uint32_t data_count )
{
    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_READ );
    flash2_hal_write_address( address );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);
}

void flash2_highspeedRread( uint32_t address, uint8_t *buffer,
                        uint32_t data_count )
{
    while( flash2_busy() )
    
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_HS_READ );
    flash2_hal_write_address( address );
    flash2_hal_write( buffer, data_count );
    hal_gpio_csSet(1);
}
void flash2_quadWrite( uint32_t address, uint8_t *buffer,
                        uint32_t data_count )
{
    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_SPI_QUAD );
    
    if( data_count <= _FLASH2_FLASH_PAGE_SIZE )
    {


        flash2_hal_write_address( address );
        flash2_hal_write( buffer, data_count );

    }
    else
    {
        uint8_t i;
        uint8_t repeat = data_count / _FLASH2_FLASH_PAGE_SIZE;
        uint8_t repeat_over = data_count % _FLASH2_FLASH_PAGE_SIZE;

        for ( i = 0; i < repeat; i++ )
        {


            flash2_hal_write_address( address + ( i * _FLASH2_FLASH_PAGE_SIZE ) );
            flash2_hal_write( buffer + ( i * _FLASH2_FLASH_PAGE_SIZE ),
                                        _FLASH2_FLASH_PAGE_SIZE );
        }
        if ( repeat_over )
        {

            flash2_hal_write_address( address +
                                           ( repeat * _FLASH2_FLASH_PAGE_SIZE ) );
            flash2_hal_write( buffer + ( repeat * _FLASH2_FLASH_PAGE_SIZE ),
                                        repeat_over );
        }
    }
    hal_gpio_csSet(1);
    
    return;
}

void flash2_write( uint32_t address, uint8_t *buffer,
                        uint32_t data_count )
{
    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_PP );
    
    if( data_count <= _FLASH2_FLASH_PAGE_SIZE )
    {

        flash2_hal_write_address( address );
        flash2_hal_write( buffer, data_count );

    }
    else
    {
        uint8_t i;
        uint8_t repeat = data_count / _FLASH2_FLASH_PAGE_SIZE;
        uint8_t repeat_over = data_count % _FLASH2_FLASH_PAGE_SIZE;

        for ( i = 0; i < repeat; i++ )
        {

            flash2_hal_write_address( address + ( i * _FLASH2_FLASH_PAGE_SIZE ) );
            flash2_hal_write( buffer + ( i * _FLASH2_FLASH_PAGE_SIZE ),
                                        _FLASH2_FLASH_PAGE_SIZE );
        }
        if ( repeat_over )
        {


            flash2_hal_write_address( address +
                                           ( repeat * _FLASH2_FLASH_PAGE_SIZE ) );
            flash2_hal_write( buffer + ( repeat * _FLASH2_FLASH_PAGE_SIZE ),
                                        repeat_over );
        }
    }
    hal_gpio_csSet(1);
    
    return;
    
}

void flash2_quadEnable( void )
{
    flash2_hal_command( _FLASH2_INSTR_EQIO );
}

void flash2_quadOutRead( uint32_t address, uint8_t *buffer,
                            uint32_t data_count)
{

    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    hal_gpio_pwmSet(0);
    hal_gpio_rstSet( 0 );
    flash2_hal_command( _FLASH2_INSTR_SQOR );
    flash2_hal_write_address( address );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;
}

void flash2_quadIoRead(  uint32_t address, uint8_t mode,
                            uint8_t *buffer, uint32_t data_count)
{

    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    hal_gpio_pwmSet(0);
    hal_gpio_rstSet( 0 );
    flash2_hal_command( _FLASH2_INSTR_SQIOR );
    flash2_hal_write_address( address );
    flash2_hal_write( &mode, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;
}

void flash2_quadReset( void )
{

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RSTQIO );
    hal_gpio_csSet(1);
}

void flash2_setBurst( uint8_t length )
{
    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_SB );
    flash2_hal_write( &length, 1 );
    hal_gpio_csSet(1);

    return;
}

void flash2_readSqiBurstWrap(uint32_t address, uint8_t *buffer,
                              uint32_t data_count)
{
    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RBSQI );
    flash2_hal_write_address( address );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;

}

void flash2_readSpiBurstWrap( uint32_t address, uint8_t *buffer,
                              uint32_t data_count)
{
    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RBSPI );
    flash2_hal_write_address( address );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;

}

void flash2_readDualOutput( uint32_t address, uint8_t *buffer,
                                uint32_t data_count)
{
    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_SDOR );
    flash2_hal_write_address( address );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;

}

void flash2_readDualIo (uint32_t address, uint8_t *buffer,
                          uint32_t data_count)
{
    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_SDIOR );
    flash2_hal_write_address( address );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;
}

void flash2_sectorErase( uint32_t address )
{
    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_SE );
    flash2_hal_write_address( address );
    hal_gpio_csSet(1);

}

void flash2_blockErase( uint32_t address )
{
    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_BE );
    flash2_hal_write_address( address );
    hal_gpio_csSet(1);

}

void flash2_chipErase( void )
{

    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_CE );
    hal_gpio_csSet(1);
    return;
}

void flash2_getSfdpParams( uint32_t address, uint8_t *buffer,
                         uint32_t data_count)
{
    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_SFDP );
    flash2_hal_write_address( address );
    flash2_hal_write( &dummy_byte, 1 );
    flash2_hal_read( &buffer[0], data_count );
    hal_gpio_csSet(1);

    return;
}

uint8_t flash2_quadDeviceManufac( void )
{
    uint8_t partialInfo[3];

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_QUAD_JID );
    flash2_hal_read( &partialInfo[0], 3 );
    hal_gpio_csSet(1);

    return partialInfo[0];
}

uint8_t flash2_quadDeviceType( void )
{
    uint8_t partialInfo[3];

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_QUAD_JID );
    flash2_hal_read( &partialInfo[0], 3 );
    hal_gpio_csSet(1);

    return partialInfo[1];
}

uint8_t flash2_quadDeviceId( void )
{
    uint8_t partialInfo[3];

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_QUAD_JID );
    flash2_hal_read( &partialInfo[0], 3 );
    hal_gpio_csSet(1);

    return partialInfo[2];
}

uint8_t flash2_deviceManufac( void )
{
    uint8_t partialInfo[3];

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_JEDECID );
    flash2_hal_read( &partialInfo[0], 3 );
    hal_gpio_csSet(1);

    return partialInfo[0];
}

uint8_t flash2_deviceType( void )
{
    uint8_t partialInfo[3];

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_JEDECID );
    flash2_hal_read( &partialInfo[0], 3 );
    hal_gpio_csSet(1);

    return partialInfo[1];
}

uint8_t flash2_deviceId( void )
{
    uint8_t partialInfo[3];

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_JEDECID );
    flash2_hal_read( &partialInfo[0], 3 );
    hal_gpio_csSet(1);

    return partialInfo[2];
}

void flash2_reset( void )
{
    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RSTEN );
    hal_gpio_csSet(1);
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RST );
    hal_gpio_csSet(1);

    return;

}

void flash2_writeStatusReg( uint8_t sReg )
{
    uint8_t dummy_byte = 0x00;

    while( flash2_busy() );

    flash2_writeEnable();
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_WRSR );
    flash2_hal_write( &dummy_byte, 1);
    flash2_hal_write( &sReg, 1);
    hal_gpio_csSet(1);

}

uint8_t flash2_getConfigReg( void )
{
    uint8_t temp;

    while( flash2_busy() );

    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_RDCR );
    flash2_hal_read( &temp, 1);
    hal_gpio_csSet(1);

    return temp;
}

void flash2_writeEnable( void )
{
    hal_gpio_csSet(0);
    flash2_hal_command( _FLASH2_INSTR_WREN );
    hal_gpio_csSet(1);

    return;
}






/* -------------------------------------------------------------------------- */
/*
  __flash2_driver.c

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */