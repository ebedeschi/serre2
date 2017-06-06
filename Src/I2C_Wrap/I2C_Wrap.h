#ifndef __I2C_WRAP_H__
#define __I2C_WRAP_H__

////////////////////////////////////////////////////////
/////   Wrapper funzioni I2C per indipendenza HW   /////
////////////////////////////////////////////////////////

#include "../I2C_Wrap/I2C_WrapHandle.h"

//#if (ARDUINO >= 100)
//  #include <Arduino.h>
//#else
//  #include <WProgram.h>
//#endif


//------------------------------------
//--  DEFINIZIONI

//ERRORI
#define I2C_OK 			0



//------------------------------------
//--  FUNZIONI

//inizializza HW i2c
void i2cBegin();
//scrive sullo slave
int i2cMasterWrite(I2CHandle* hi2c, uint8_t slaveAddr, uint8_t* data, int dataLength);
//legge un certo numero di byte dallo slave
int i2cMasterRead(I2CHandle* hi2c, uint8_t slaveAddr, uint8_t* data, int dataLength);


#endif  //__I2C_WRAP_H__
