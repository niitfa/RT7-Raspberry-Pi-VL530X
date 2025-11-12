#include "i2c.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>

static int fd = 0;
static uint8_t address = 0;
static char* device = "/dev/i2c-2" ;

// I2C definitions

#define I2C_SLAVE	0x0703
#define I2C_SMBUS	0x0720	/* SMBus-level access */

#define I2C_SMBUS_READ	1
#define I2C_SMBUS_WRITE	0

// SMBus transaction types

#define I2C_SMBUS_QUICK		    0
#define I2C_SMBUS_BYTE		    1
#define I2C_SMBUS_BYTE_DATA	    2 
#define I2C_SMBUS_WORD_DATA	    3
#define I2C_SMBUS_PROC_CALL	    4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7		/* SMBus 2.0 */
#define I2C_SMBUS_I2C_BLOCK_DATA    8

// SMBus messages

#define I2C_SMBUS_BLOCK_MAX	32	/* As specified in SMBus standard */	
#define I2C_SMBUS_I2C_BLOCK_MAX	32	/* Not specified but we use same structure */

static int _wiringPiI2CSetupInterface (const char *device, int devId);
static int _wiringPiI2CSetup (const int devId);


void I2C_Init(uint8_t addr)
{
    wiringPiSetup(); 
    fd = _wiringPiI2CSetup(addr);
    address = addr;
}

void I2C_Read(uint8_t* buff, uint8_t len)
{
    read(fd, buff, len);
}

void I2C_Write(const uint8_t* buff, uint8_t len)
{
    write(fd, buff, len);
}


static int _wiringPiI2CSetupInterface (const char *device, int devId)
{
  int fd ;

  if ((fd = open (device, O_RDWR)) < 0)
    return wiringPiFailure (WPI_ALMOST, "Unable to open I2C device: %s\n", strerror (errno)) ;

  if (ioctl (fd, I2C_SLAVE, devId) < 0)
    return wiringPiFailure (WPI_ALMOST, "Unable to select I2C device: %s\n", strerror (errno)) ;

  return fd ;
}

static int _wiringPiI2CSetup (const int devId)
{
	int rev ;
	int model;
	piBoardId(&model);
	return wiringPiI2CSetupInterface (device, devId) ;
}
