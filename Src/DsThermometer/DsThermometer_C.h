

///////////////////////////////////////////////////////////
////       Libreria di interfacciamento                ////
////           termometro Dallas                       ////
///////////////////////////////////////////////////////////

#ifndef __DSTHERMOMETER_C_H__
#define __DSTHERMOMETER_C_H__

#include <stdbool.h>
#include "../OneWire/OneWire_C.h"


////////////////////////////////////////
////    DECLARATIONS

#define DS_ERR_SUCCESS               0
#define DS_ERR_DEVICEDISCONNECTED   -1

// ID dei modelli di termometro
typedef enum
{
  DS18S20     = 0x10,  // also DS1820
  DS18B20     = 0x28,
  DS1822      = 0x22,
  DS1825      = 0x3B,
  DS28EA00    = 0x42
} dsEModel;

//Risoluzione del termometro
typedef enum 
{
  TEMP_9_BIT      = 0x1F,  //  9 bit
  TEMP_10_BIT     = 0x3F,  // 10 bit
  TEMP_11_BIT     = 0x5F,  // 11 bit
  TEMP_12_BIT     = 0x7F   // 12 bit
} dsTempResolution;


//info su un termometro trovato con la search
typedef struct
{
  dsEModel Model;
  owDeviceAddr Address;
} dsSearchThermometerInfo;

//Risultati della ricerca dei termometri
typedef struct
{
  //numero di termometri trovati
  int numDevices;
  //vettore di info sui termometri trovati
  dsSearchThermometerInfo* devices;
} dsSearchResult;


//Handle termometro
typedef struct
{
  ///Modello del termometro
  dsEModel Model;
  ///Indirizzo sul bus OneWire
  owDeviceAddr Address;
  
  // handle bus OneWire
  HOneWire* _wire;
} HDsThermometer;




////////////////////////////////////////
////    FUNCTIONS

//cerca i termometri Dallas presenti sul bus
dsSearchResult dsSearchThermometers(HOneWire* wire);
//elimina il risultato della ricerca
void dsDeleteSearchResult(dsSearchResult* result);


//Crea l'handle DsThermometer agganciato al bus onewire specificato ed al termometro specificato
HDsThermometer dsCreateHandle(HOneWire* wire, dsSearchThermometerInfo thermometer);
//inizializza 
void dsBegin(HDsThermometer* handle);
//ritorna la risoluzione del termometro
int dsGetResolution(HDsThermometer* handle, dsTempResolution* resolution);
//setta la risoluzione del termometro
int dsSetResolution(HDsThermometer* handle, dsTempResolution resolution);
//ritorna la temperatura in °C
int dsGetTemperature(HDsThermometer* handle, float* temperature);
//forza il termometro ad aggiornare il registro di temperatura
int dsRequestTemperature(HDsThermometer* handle);
//se non è usato l'allarme, recupera i dati nei registri di allarme
int dsGetUserData(HDsThermometer* handle, uint16_t* userData);
//se non è usato l'allarme, è possibile settare dei dati utente nei registri dell'allarme
int dsSetUserData(HDsThermometer* handle, uint16_t userData);
//ricava l'impostazione di alarm high temperature
int dsGetHighAlarmTemperature(HDsThermometer* handle, uint8_t* alarm);
//setta l'impostazione di alarm high temperature
int dsSetHighAlarmTemperature(HDsThermometer* handle, uint8_t alarm);
//ricava l'impostazione di alarm low temperature
int dsGetLowAlarmTemperature(HDsThermometer* handle, uint8_t* alarm);
//setta l'impostazione di alarm low temperature
int dsSetLowAlarmTemperature(HDsThermometer* handle, uint8_t alarm);
//determina se la periferica è in allarme
bool dsIsInAlarm(HDsThermometer* handle);
///Forza il device a salvare i dati dello scratchpad sulla EEPROM interna
void dsTransfertToEEPROM(HDsThermometer* handle);



#endif  //__DSTHERMOMETER_C_H__

