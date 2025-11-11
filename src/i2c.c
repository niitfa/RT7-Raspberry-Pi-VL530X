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

union i2c_smbus_data
{
  uint8_t  byte ;
  uint16_t word ;
  uint8_t  block [I2C_SMBUS_BLOCK_MAX + 2] ;	// block [0] is used for length + one more for PEC
} ;

struct i2c_smbus_ioctl_data
{
  char read_write ;
  uint8_t command ;
  int size ;
  union i2c_smbus_data *data ;
} ;

static int wiringPiI2CReadBlockData (int fd, int reg, uint8_t *values, uint8_t size);
static int wiringPiI2CWriteBlockData (int fd, int reg, const uint8_t *values, uint8_t size);
static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data);
static int _wiringPiI2CSetupInterface (const char *device, int devId);
static int _wiringPiI2CSetup (const int devId);


void I2C_Init(uint8_t addr)
{
    wiringPiSetup(); 
    fd = _wiringPiI2CSetup(addr << 1);
   //printf("fd = %i\n", fd);
    address = addr;
}

void I2C_Read(uint8_t* buff, uint8_t len)
{
    int rd = wiringPiI2CReadBlockData (fd, 0, buff, len);   
    //printf("rd = %i\n", rd);
}

void I2C_Write(const uint8_t* buff, uint8_t len)
{
    int wr = wiringPiI2CWriteBlockData (fd, 0, buff, len); 
  // printf("wr = %i\n", wr);
}

static int wiringPiI2CReadBlockData (int fd, int reg, uint8_t *values, uint8_t size)
{
  union i2c_smbus_data data;

  if (size>I2C_SMBUS_BLOCK_MAX) {
    size = I2C_SMBUS_BLOCK_MAX;
  }
  data.block[0] = size;
  int result = i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_I2C_BLOCK_DATA, &data);
  if (result<0) {
    return result;
  }
  memcpy(values, &data.block[1], size);
  return data.block[0];
}

int wiringPiI2CWriteBlockData (int fd, int reg, const uint8_t *values, uint8_t size)
{
    union i2c_smbus_data data;

    if (size>I2C_SMBUS_BLOCK_MAX) {
      size = I2C_SMBUS_BLOCK_MAX;
    }
    data.block[0] = size;
    memcpy(&data.block[1], values, size);
    return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BLOCK_DATA, &data) ;
}

static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args ;

  args.read_write = rw ;
  args.command    = command ;
  args.size       = size ;
  args.data       = data ;

  //int res = ioctl (fd, I2C_SMBUS, &args) ;
  //printf("ioctl: %i\n", res);
  //return res;
  return ioctl (fd, I2C_SMBUS, &args) ;
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


/*
 * wiringPiI2CSetup:
 *	Open the I2C device, and regsiter the target device
 *********************************************************************************
 */

static int _wiringPiI2CSetup (const int devId)
{
	int rev ;
	int model;
	piBoardId(&model);
	return wiringPiI2CSetupInterface (device, devId) ;
}
