#include "i2c.h"

static int fd = 0;
static const char* device = "/dev/i2c-2" ;

void I2C_Init(uint8_t addr)
{
    wiringPiSetup(); 
    fd = wiringPiI2CSetupInterface (device, addr);
}

void I2C_Read(uint8_t* buff, uint8_t len)
{
    read(fd, buff, len);
}

void I2C_Write(const uint8_t* buff, uint8_t len)
{
    write(fd, buff, len);
}

