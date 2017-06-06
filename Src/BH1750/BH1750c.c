
/*

  Library for the BH1750FVI Digital Light Sensor
  breakout board.

  The board uses I2C for communication. 2 pins are required to
  interface to the device.

*/


#include "../BH1750/BH1750c.h"

//#include "../log.h"


//--------------------------------------------------------------
//----  DECLARATIONS

//valori da aggiungere al valore di sensitivity per impostarlo sul device
#define BH1750_MTREG_HIGH   0x40
#define BH1750_MTREG_LOW    0x60

//Reset command
#define BH1750_RESET  0x07

// Define milliseconds delay for ESP8266 platform
#if defined(ESP8266)
  #include <pgmspace.h>
  #define _delay_ms(ms) delayMicroseconds((ms) * 1000)
// Use _delay_ms from utils for AVR-based platforms
#elif defined(__avr__)
  #include <util/delay.h>
// Use Wiring's delay for compability with another platforms
#else
  #define _delay_ms(ms) HAL_Delay(ms);
#endif

//Funzione di log
#ifdef BH1750_DEBUG
  #define log(x)    SYSTEM_LOG(String(F("[BH1750] ")) + x)
#else
  #define log(x)
#endif

//Forward Declarations
//String ModeToString(EBH1750_mode mode);
void SendCommand(BH1750 handle, uint8_t command);


//--------------------------------------------------------------
//----  FUNCTIONS

//crea un handle al BH1750
BH1750 BH1750_CreateHandle(I2CHandle* i2cH, int i2cAddr)
{
  BH1750 h;
	h.i2cAddress = i2cAddr;
	h._i2cH = i2cH;
	
  return h;
}

//Reset device (reset to 0 the illuminance register). Not allowed in POWER_OFF mode
void BH1750_Reset(BH1750 handle)
{
  //check device POWER_OFF mode
  if (handle.CurrentMode == POWER_OFF)
    return;
  
  //send command
  SendCommand(handle, BH1750_RESET);
  
  //Log
  #ifdef BH1750_DEBUG
    log(F("Device Reset"));
  #endif
}

//init i2c connection and set BH1750 into specified operation mode. Default POWER_OFF
void BH1750_Begin(BH1750 handle, EBH1750_mode mode)
{
  // Initialize I2C
  i2cBegin();
  
  //initialize device
  //turn on device
  BH1750_ChangeMode(handle, POWER_ON);
  //change sensitivity to default value
  BH1750_ChangeSensitivity(handle, BH1750_DEFAULT_SENSITIVITY);
  
  // Configure sensor in specified mode
  BH1750_ChangeMode(handle, mode); 
}

//change BH1750 operation mode
void BH1750_ChangeMode (BH1750 handle, EBH1750_mode mode)
{
  //convert mode to byte
  uint8_t bMode = (uint8_t)mode;
  //Send command to device
  SendCommand(handle, bMode);
  
  //update handle
  handle.CurrentMode = mode;
  
  //Log
  #ifdef BH1750_DEBUG
    log(String(F("Device mode changed to: ")) + ModeToString(mode));
  #endif
}

//Read last light level measurement. Return measurement in lux
unsigned int BH1750_ReadLightLevel(BH1750 handle)
{
  // Measurment result will be stored here
  uint16_t level;
  //allocate a buffer for data
  uint8_t buff[2];
  
  // Read two bytes from sensor
  int rr = i2cMasterRead(handle._i2cH, (uint8_t)handle.i2cAddress, buff, 2);
  
  // Two bytes that are the low and high part of sensor value
  // Retrieve the sensor value composing the 2 bytes
  level = buff[0];
  level <<= 8;
  level |= buff[1];
  
  // Log
  #ifdef BH1750_DEBUG
    log(String(F("Raw value: ")) + level);
  #endif
  
  // Convert raw value to lux
  level /= 1.2;
  
  // Log
  #ifdef BH1750_DEBUG
    log(String(F("Converted value: ")) + level);
  #endif

  return level;
}

//change device sensitivity
void BH1750_ChangeSensitivity(BH1750 handle, uint8_t value)
{
  //il valore di sensitivity è da inserire diviso alto e basso in due registri
  //nel registro alto ci vanno i primi 3 bit, in quello basso i rimanenti 5
  
  //sistema value nel range di valori
  if (value < BH1750_MIN_SENSITIVITY) value = BH1750_MIN_SENSITIVITY;
  if (value > BH1750_MAX_SENSITIVITY) value = BH1750_MAX_SENSITIVITY;
  
  //scompone value in due pezzi
  uint8_t highValue = (value >> 5);
  uint8_t lowValue = (value & 0x1F);
    
  //ricava il valore del registro MTreg 
  uint8_t regHighValue = highValue + BH1750_MTREG_HIGH;
  uint8_t regLowValue = lowValue + BH1750_MTREG_LOW;
  
  //scrive il registro High
  int rr = i2cMasterWrite(handle._i2cH, (uint8_t)handle.i2cAddress, &regHighValue, 1);
  //scrive il registro low
  rr = i2cMasterWrite(handle._i2cH, (uint8_t)handle.i2cAddress, &regLowValue, 1);

  //cambia la variabile locale
  handle.CurrentSensitivity = value;
  
  //Log
  #ifdef BH1750_DEBUG
    log(String("Sensivity changed to ") + value);
    log(String("New integration time: ") + BH1750_GetIntegrationTime(handle));
  #endif
}

//integration time in ms
int BH1750_GetIntegrationTime(BH1750 handle)
{
  //default integration time for device mode
  float time;
  switch (handle.CurrentMode)
  {
    case CONTINUOUS_HIGH_RES_MODE_1:
    case ONE_TIME_HIGH_RES_MODE_1:
      time = 120;
      break;
    case CONTINUOUS_HIGH_RES_MODE_2:
    case ONE_TIME_HIGH_RES_MODE_2:
      time = 120;
      break;
    case CONTINUOUS_LOW_RES_MODE:
    case ONE_TIME_LOW_RES_MODE:
      time = 16;
      break;
    default:
      //not in measurement mode
      return -1;    
  }
  
  //add the sensitivity
  float sens = handle.CurrentSensitivity;
  sens /= BH1750_DEFAULT_SENSITIVITY;
  time *= sens;

  //ceil time
  time += 0.5;
  
  return (int)time;  
}

//--------------------------------------------------------------
//----  PRIVATE FUNCTIONS

//send command to device
void SendCommand(BH1750 handle, uint8_t command)
{
  //Send command to sensor
  int rr = i2cMasterWrite(handle._i2cH, (uint8_t)handle.i2cAddress, &command, 1);
  // Wait few moments for set up
  _delay_ms(10);
}

//Converte EBH1750_mode in stringa
//String ModeToString(EBH1750_mode mode)
//{
//  switch (mode)
//  {
//
//  case POWER_OFF:
//    return F("POWER_OFF");
//  case POWER_ON:
//    return F("POWER_ON");
//  case CONTINUOUS_HIGH_RES_MODE_1:
//    return F("CONTINUOUS_HIGH_RES_MODE_1");
//  case CONTINUOUS_HIGH_RES_MODE_2:
//    return F("CONTINUOUS_HIGH_RES_MODE_2");
//  case CONTINUOUS_LOW_RES_MODE:
//    return F("CONTINUOUS_LOW_RES_MODE");
//  case ONE_TIME_HIGH_RES_MODE_1:
//    return F("ONE_TIME_HIGH_RES_MODE_1");
//  case ONE_TIME_HIGH_RES_MODE_2:
//    return F("ONE_TIME_HIGH_RES_MODE_2");
//  case ONE_TIME_LOW_RES_MODE:
//    return F("ONE_TIME_LOW_RES_MODE");
//
//  }
//}


