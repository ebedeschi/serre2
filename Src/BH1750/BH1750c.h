
/*

  This is a library for the BH1750FVI Digital Light Sensor
  breakout board.

  The board uses I2C for communication. 2 pins are required to
  interface to the device.

*/

#ifndef __BH1750C_H__
#define __BH1750C_H__



//#if (ARDUINO >= 100)
//  #include <Arduino.h>
//#else
//  #include <WProgram.h>
//#endif


#include "../I2C_Wrap/I2C_Wrap.h"


//--------------------------------------------------------------
//----  DEFINITIONS

//Device Low address
#define BH1750_LOW_ADDRESS 0x23
//Device High address
#define BH1750_HIGH_ADDRESS 0x5c

//Device sensitivity  range
#define BH1750_MIN_SENSITIVITY      0x1F
#define BH1750_DEFAULT_SENSITIVITY  0x45
#define BH1750_MAX_SENSITIVITY      0xFE

// Uncomment, to enable debug messages
//#define BH1750_DEBUG

//device operation modes
typedef enum
{
  //Shut down the device
  POWER_OFF = 0x00,
  //Device in POWER_ON state, wating for measurment command
  POWER_ON = 0x01,
  //Continuous measurement at 1 lux resolution
  CONTINUOUS_HIGH_RES_MODE_1 = 0x10,
  //Continuous measurement at 0.5 lux resolution
  CONTINUOUS_HIGH_RES_MODE_2 = 0x11,
  //Continuous measurement at 4 lux resolution
  CONTINUOUS_LOW_RES_MODE = 0x13,
  //One shot measurement at 1 lux resolution
  ONE_TIME_HIGH_RES_MODE_1 = 0x20,
  //One shot measurement at 0.5 lux resolution
  ONE_TIME_HIGH_RES_MODE_2 = 0x21,
  //One shot measurement at 4 lux resolution
  ONE_TIME_LOW_RES_MODE = 0x23
} EBH1750_mode;


//Handle for BH1750
typedef struct
{
  //current device mode
  EBH1750_mode CurrentMode;
  //current device sensitivity
  uint8_t CurrentSensitivity;
  //i2c device address
  uint8_t i2cAddress;
  
  //handle i2c device
  I2CHandle* _i2cH;
  
} BH1750;


//crea un handle al BH1750
BH1750 BH1750_CreateHandle(I2CHandle* i2cH, int i2cAddress);
//init i2c connection and set BH1750 into specified operation mode. Default POWER_OFF
void BH1750_Begin(BH1750 handle, EBH1750_mode mode);
//Reset device (reset to 0 the illuminance register). Not allowed in POWER_OFF mode
void BH1750_Reset(BH1750 handle);
//change BH1750 operation mode
void BH1750_ChangeMode (BH1750 handle, EBH1750_mode mode);
//Read last light level measurement. Return measurement in lux
unsigned int BH1750_ReadLightLevel(BH1750 handle);
//change device sensitivity
void BH1750_ChangeSensitivity(BH1750 handle, uint8_t value);
//integration time in ms
int BH1750_GetIntegrationTime(BH1750 handle);




#endif 	//__BH1750C_H__
