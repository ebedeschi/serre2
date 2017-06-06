
//#include <Wire.h>

#include "../I2C_Wrap/I2C_Wrap.h"

#define I2C_TIMEOUT	1000

//// Legacy Wire.write() function fix
//#if (ARDUINO >= 100)
//  #define __wire_write(d) Wire.write(d)
//#else
//  #define __wire_write(d) Wire.send(d)
//#endif
//
//// Legacy Wire.read() function fix
//#if (ARDUINO >= 100)
//  #define __wire_read() Wire.read()
//#else
//  #define __wire_read() Wire.receive()
//#endif



//////////////////////////////////////////////////////////////////
/////   FUNZIONI I2C WRAP


//------------------------------------
//--  FUNZIONI

//inizializza HW i2c
void i2cBegin()
{
}

//scrive sullo slave
int i2cMasterWrite(I2CHandle* hi2c, uint8_t slaveAddr, uint8_t* data, int dataLength)
{
//	return HAL_I2C_Master_Transmit(hi2c, slaveAddr, data, dataLength, I2C_TIMEOUT);
  //return HAL_I2C_Mem_Write(hi2c, slaveAddr, slaveAddr, 1, data, dataLength, I2C_TIMEOUT);
  slaveAddr <<= 1;
  return HAL_I2C_Master_Transmit(hi2c, slaveAddr, data, dataLength, I2C_TIMEOUT);
}

//legge un certo numero di byte dallo slave
int i2cMasterRead(I2CHandle* hi2c, uint8_t slaveAddr, uint8_t* data, int dataLength)
{
//	return HAL_I2C_Master_Receive(hi2c, slaveAddr, data, dataLength, I2C_TIMEOUT);
//  return HAL_I2C_Mem_Read(hi2c, slaveAddr, 0x00, I2C_MEMADD_SIZE_8BIT, data, dataLength, I2C_TIMEOUT);

  slaveAddr <<= 1;
  slaveAddr += 1;
  return HAL_I2C_Master_Receive(hi2c, slaveAddr, data, dataLength, I2C_TIMEOUT);
}
