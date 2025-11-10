#ifndef RGATE_I2C_H
#define RGATE_I2C_H

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <wiringPi.h>


#define I2C_SDA_GPIO_PIN 0
#define I2C_SCL_GPIO_PIN 1

#define I2C_INIT { pinMode (I2C_SDA_GPIO_PIN, OUTPUT); pinMode (I2C_SCL_GPIO_PIN, OUTPUT); }

#define I2C_CLEAR_SDA  digitalWrite (I2C_SDA_GPIO_PIN, LOW);
#define I2C_SET_SDA  digitalWrite (I2C_SDA_GPIO_PIN, HIGH);
#define I2C_READ_SDA digitalRead(I2C_SDA_GPIO_PIN)
#define I2C_CLEAR_SCL digitalWrite (I2C_SCL_GPIO_PIN, LOW);
#define I2C_SET_SCL digitalWrite (I2C_SCL_GPIO_PIN, HIGH);
#define I2C_DELAY usleep(5); // 5 microsecond delay

//void I2C_bus_init(uint8_t scl_pin, uint8_t sda_pin, uint8_t port);

void I2C_init(void);

void I2C_start_cond(void);

void I2C_stop_cond(void);

void I2C_write_bit(uint8_t b);

uint8_t I2C_read_SDA(void);

// Reading a bit in I2C:
uint8_t I2C_read_bit(void);

_Bool I2C_write_byte(uint8_t B, _Bool start, _Bool stop);

uint8_t I2C_read_byte(_Bool ack, _Bool stop);

_Bool I2C_send_byte(uint8_t address, uint8_t data);

uint8_t I2C_receive_byte(uint8_t address);

_Bool I2C_send_byte_data(uint8_t address, uint8_t reg, uint8_t data);

uint8_t I2C_receive_byte_data(uint8_t address, uint8_t reg);

_Bool I2C_transmit(uint8_t address, uint8_t data[], uint8_t size);

_Bool I2C_receive(uint8_t address, uint8_t reg[], uint8_t *data, uint8_t reg_size, uint8_t size);

#endif