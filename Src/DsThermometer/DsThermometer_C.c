
#include <stdlib.h>

#include "DsThermometer_C.h"
#include "stm32l4xx_hal.h"


////////////////////////////////////////
////    DECLARATIONS

// OneWire commands
#define STARTCONVO        0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH       0x48  // Copy EEPROM
#define READSCRATCH       0xBE  // Read EEPROM
#define WRITESCRATCH      0x4E  // Write to EEPROM
#define RECALLSCRATCH     0xB8  // Reload from last known
#define READPOWERSUPPLY   0xB4  // Determine if device needs parasite power
#define ALARMSEARCH       0xEC  // Query bus for devices with an alarm condition

#define _DELAY(x)  HAL_Delay(x)


//ScratchPad del termometro
typedef uint8_t ScratchPad[9];

//struttura della memoria scratchpad del termometro
typedef struct
{
  uint16_t temp;            //temperature
  // uint8_t tempLSB;          //temperature LSB
  // uint8_t tempMSB;          //temperature MSB
  uint8_t highAlarmTemp;    //high alarm temp
  uint8_t lowAlarmTemp;     //low alarm temp
  uint8_t configuration;    //DS18S20: store for crc   /   DS18B20 & DS1822: configuration register
  uint8_t reg5;             //internal use & crc
  uint8_t countRemain;      //DS18S20: COUNT_REMAIN    /   DS18B20 & DS1822: store for crc
  uint8_t countPerC;        //DS18S20: COUNT_PER_C     /   DS18B20 & DS1822: store for crc
  uint8_t crc;              //SCRATCHPAD_CRC
} dsSScratchPad;

//memoria scratchpad 
typedef union
{
  dsSScratchPad structure;
  uint8_t memory[9];
} dsScratchPad;


//FORWARD DECLARATIONS
///Determina se l'indirizzo passato è valido
bool dsCheckValidAddress(owDeviceAddr addr);
///Ritorna il modello del termometro dato l'indirizzo. -1 se non è un termometro Dallas
int dsRetrieveModel(owDeviceAddr addr);
//legge la scratchpad del termometro dato. Ritorna TRUE se ha letto, FALSE altrimenti
bool dsReadScratchPad(HDsThermometer* handle, dsScratchPad* scratchPad);
///scrive lo scratchpad specificato sul device
void dsWriteScratchPad(HDsThermometer* handle, dsScratchPad* scratchPad);
//determina la risoluzione impostata del termometro dallo scratchpad
dsTempResolution dsRetrieveResolution(HDsThermometer* handle, dsScratchPad scratchPad);
//ritorna il dato di temperatura presente sullo scratchpad passato
float dsCalculateTemperature(HDsThermometer* handle, dsScratchPad scratchPad);







////////////////////////////////////////
////    FUNCTIONS

//cerca i termometri Dallas presenti sul bus
dsSearchResult dsSearchThermometers(HOneWire* wire)
{
  dsSearchResult result;

  //inizializza
  result.numDevices = 0;
  result.devices = NULL;
  
  //resetta la ricerca sul bus
  owResetSearch(wire);

  //finchÃ¨ trova dispositivi
  while (owSearch(wire, NORMAL_SEARCH))
  {
    //trovato un dispositivo OneWire

    //controlla se l'indirizzo è valido
    if (dsCheckValidAddress(wire->SelectedDevice))
    {
      //controlla se è un termometro Dallas
      //cerca di ricavarne il modello
      int iModel = dsRetrieveModel(wire->SelectedDevice);
      if (iModel != -1)
      {
        //Ã¨ un termometro Dallas

        //aggiunge ai risultati
        result.numDevices++;
        //alloca lo spazio in memoria
        result.devices = (dsSearchThermometerInfo*)realloc(result.devices,  result.numDevices * sizeof(dsSearchThermometerInfo));
        //aggiorna
        result.devices[result.numDevices - 1].Model = (dsEModel)iModel;
        result.devices[result.numDevices - 1].Address = wire->SelectedDevice;
      }
    }
  }

  return result;
}
//elimina il risultato della ricerca
void dsDeleteSearchResult(dsSearchResult* result)
{
  //se ci sono indirizzi
  if (result->numDevices > 0)
    //dealloca il vettore di info
    free(result->devices);
    
  result->devices = NULL;
}



//Crea l'handle DsThermometer agganciato al bus onewire specificato ed al termometro specificato
HDsThermometer dsCreateHandle(HOneWire* wire, dsSearchThermometerInfo thermometer)
{
  HDsThermometer handle;
  
  handle.Model = thermometer.Model;
  handle.Address = thermometer.Address;
  
  handle._wire = wire;
  
  return handle;
}
//inizializza 
void dsBegin(HDsThermometer* handle)
{
  
}

//ritorna la risoluzione del termometro
int dsGetResolution(HDsThermometer* handle, dsTempResolution* resolution)
{
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconnessa
    return DS_ERR_DEVICEDISCONNECTED;
  }
  
  //determina la resolution
  (*resolution) = dsRetrieveResolution(handle, scratchPad);
  return DS_ERR_SUCCESS;
}
//setta la risoluzione del termometro
int dsSetResolution(HDsThermometer* handle, dsTempResolution resolution)
{
  // DS1820 and DS18S20 have no resolution configuration register
  if (handle->Model == DS18S20)
    return DS_ERR_SUCCESS;
  
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconnessa
    return DS_ERR_DEVICEDISCONNECTED;
  }
  
  //setta il registro di configurazione
  scratchPad.structure.configuration = (uint8_t)resolution;
  //aggiorna la scratchpad sulla periferica
  dsWriteScratchPad(handle, &scratchPad);
  
  return DS_ERR_SUCCESS;
}
//ritorna la temperatura in °C
int dsGetTemperature(HDsThermometer* handle, float* temperature)
{
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconnessa
    return DS_ERR_DEVICEDISCONNECTED;
  }
  
  //calcola la temperatura
  (*temperature) = dsCalculateTemperature(handle, scratchPad);
  return DS_ERR_SUCCESS;
}
//forza il termometro ad aggiornare il registro di temperatura
int dsRequestTemperature(HDsThermometer* handle)
{
  //resetta le periferiche sul bus
  owReset(handle->_wire);
  //seleziona il termometro
  owSelect(handle->_wire, handle->Address);
  //invia il comando di acquisizione della temperatura
  owWrite(handle->_wire, STARTCONVO);
  
  //aspetta il tempo necessario
  //caso peggiore con parasite power
  //TODO: migliorare
  _DELAY(750);   //pausa di 750ms
  
  //temperatura aggiornata e messa in scratchpad
  
  return DS_ERR_SUCCESS;
}
//se non è usato l'allarme, recupera i dati nei registri di allarme
int dsGetUserData(HDsThermometer* handle, uint16_t* userData)
{
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconnessa
    return DS_ERR_DEVICEDISCONNECTED;
  }
  
  //recupera i due byte
  (*userData) = scratchPad.structure.highAlarmTemp << 8;
  (*userData) += scratchPad.structure.lowAlarmTemp;
  
  return DS_ERR_SUCCESS;
}
//se non Ã¨ usato l'allarme, è possibile settare dei dati utente nei registri dell'allarme
int dsSetUserData(HDsThermometer* handle, uint16_t userData)
{
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconnessa
    return DS_ERR_DEVICEDISCONNECTED;
  }
  
  //setta i dati nella scratchpad
  scratchPad.structure.highAlarmTemp = (userData >> 8);
  scratchPad.structure.lowAlarmTemp = (userData & 0xFF);
  //scrive la scratchpad
  dsWriteScratchPad(handle, &scratchPad);
  
  return DS_ERR_SUCCESS;
}
//ricava l'impostazione di alarm high temperature
int dsGetHighAlarmTemperature(HDsThermometer* handle, uint8_t* alarm)
{
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconnessa
    return DS_ERR_DEVICEDISCONNECTED;
  }
  
  //ricava il dato dell'allarme
  (*alarm) = scratchPad.structure.highAlarmTemp;
  
  return DS_ERR_SUCCESS;
}
//setta l'impostazione di alarm high temperature
int dsSetHighAlarmTemperature(HDsThermometer* handle, uint8_t alarm)
{
  //controlla il valore di alarm
  if (alarm > 125) alarm = 125;
  if (alarm < -55) alarm = -55;
  
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconnessa
    return DS_ERR_DEVICEDISCONNECTED;
  }
  
  //imposta il dato
  scratchPad.structure.highAlarmTemp = alarm;
  //aggiorna la scratchpad
  dsWriteScratchPad(handle, &scratchPad);
  
  return DS_ERR_SUCCESS;
}
//ricava l'impostazione di alarm low temperature
int dsGetLowAlarmTemperature(HDsThermometer* handle, uint8_t* alarm)
{
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconnessa
    return DS_ERR_DEVICEDISCONNECTED;
  }
  
  //ricava il dato dell'allarme
  (*alarm) = scratchPad.structure.lowAlarmTemp;
  
  return DS_ERR_SUCCESS;
}
//setta l'impostazione di alarm low temperature
int dsSetLowAlarmTemperature(HDsThermometer* handle, uint8_t alarm)
{
  //controlla il valore di alarm
  if (alarm > 125) alarm = 125;
  if (alarm < -55) alarm = -55;
  
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconnessa
    return DS_ERR_DEVICEDISCONNECTED;
  }
  
  //imposta il dato
  scratchPad.structure.lowAlarmTemp = alarm;
  //aggiorna la scratchpad
  dsWriteScratchPad(handle, &scratchPad);
  
  return DS_ERR_SUCCESS;
}

//determina se la periferica è in allarme
bool dsIsInAlarm(HDsThermometer* handle)
{
  //legge la scratchpad
  dsScratchPad scratchPad;
  if (! dsReadScratchPad(handle, &scratchPad))
  {
    //errore nella lettura scratchpad. Periferica disconness
    return false;
  }
  
  //ricava la parte di temperatura che il termometro usa per l'allarme
  uint16_t temp16 = scratchPad.structure.temp;
  int temp = (int)(temp16 >> 4);
  //confronta con i registri di allarme
  if (temp <= (int)scratchPad.structure.lowAlarmTemp)
    //Ã¨ in allarme per bassa temperatura
    return true;
  if (temp >= (int)scratchPad.structure.highAlarmTemp)
    //Ã¨ in allarme per alta temperatura
    return true;
  
  //nessun allarme
  return false;
}
///Forza il device a salvare i dati dello scratchpad sulla EEPROM interna
void dsTransfertToEEPROM(HDsThermometer* handle)
{
  //resetta il bus
  owReset(handle->_wire);
  //seleziona il device
  owSelect(handle->_wire, handle->Address);
  //invia il comando
  owWrite(handle->_wire, COPYSCRATCH);
  
  //attende la scrittura sulla EEPROM
  //TODO: migliorare
  _DELAY(50);
  
  //resetta il bus
  owReset(handle->_wire);
}





////////////////////////////////////////////////////////////////////////////
////    PRIVATE FUNCTIONS

///Determina se l'indirizzo passato è valido
//TODO: da fare
bool dsCheckValidAddress(owDeviceAddr addr)
{
  return true;
}

///Ritorna il modello del termometro dato l'indirizzo. -1 se non è un termometro Dallas
int dsRetrieveModel(owDeviceAddr addr)
{
  //ricava il primo byte dell'indirizzo
  uint8_t firstByte = addr.rom[0];
  
  switch (firstByte)
  {
    case DS18S20:
    case DS18B20:
    case DS1822:
    case DS1825:
    case DS28EA00:
      return firstByte;
    default:
      //non è un termometro Dallas
      return -1;
  }
}

//legge la scratchpad del termometro dato. Ritorna TRUE se ha letto, FALSE altrimenti
bool dsReadScratchPad(HDsThermometer* handle, dsScratchPad* scratchPad)
{
  //resetta il bus onewire
  if (! owReset(handle->_wire))
    //nessuna periferica, esce
    return false;
    
  //seleziona il termometro sul bus
  owSelect(handle->_wire, handle->Address);
  //invia il comando di lettura dello scratchPad
  owWrite(handle->_wire, READSCRATCH);

  //legge lo scratchpad
  for (int i = 0; i < 9; i++)
    scratchPad->memory[i] = owRead(handle->_wire);
  
  //resetta il bus
  int b = owReset(handle->_wire);

  return (b == 1);
}

///scrive lo scratchpad specificato sul device
void dsWriteScratchPad(HDsThermometer* handle, dsScratchPad* scratchPad)
{
  //solo i registri 2,3 e 4 possono essere scritti
  
  //resetta il bus
  owReset(handle->_wire);
  //seleziona il device
  owSelect(handle->_wire, handle->Address);
  
  //invia comando di write scratchpad
  owWrite(handle->_wire, WRITESCRATCH);
  //invia i registri di allarme
  owWrite(handle->_wire, scratchPad->structure.highAlarmTemp);
  owWrite(handle->_wire, scratchPad->structure.lowAlarmTemp);
  
  // DS1820 and DS18S20 have no configuration register
  if (handle->Model != DS18S20)
    //scrive il registro di configurazione
    owWrite(handle->_wire, scratchPad->structure.configuration);
    
  //resetta il bus
  owReset(handle->_wire);
}
//determina la risoluzione impostata del termometro dallo scratchpad
dsTempResolution dsRetrieveResolution(HDsThermometer* handle, dsScratchPad scratchPad)
{
  // DS1820 and DS18S20 have no resolution configuration register
  //la risoluzione è fissa a 9 bit
  if (handle->Model == DS18S20)
    return TEMP_9_BIT;
  
  //il registro di configurazione contiene la risoluzione del termometro
  uint8_t res = scratchPad.structure.configuration;
  return (dsTempResolution)res;
}

//ritorna il dato di temperatura presente sullo scratchpad passato
float dsCalculateTemperature(HDsThermometer* handle, dsScratchPad scratchPad)
{
  // calcola la temperatura dai registri
  // int16_t temperature = 
    // (int16_t) 

    // int16_t fpTemperature =
    // (((int16_t) scratchPad[TEMP_MSB]) << 11) |
    // (((int16_t) scratchPad[TEMP_LSB]) << 3);

    // /*
    // DS1820 and DS18S20 have a 9-bit temperature register.
    // Resolutions greater than 9-bit can be calculated using the data from
    // the temperature, and COUNT REMAIN and COUNT PER Â°C registers in the
    // scratchpad.  The resolution of the calculation depends on the model.
    // While the COUNT PER Â°C register is hard-wired to 16 (10h) in a
    // DS18S20, it changes with temperature in DS1820.
    // After reading the scratchpad, the TEMP_READ value is obtained by
    // truncating the 0.5Â°C bit (bit 0) from the temperature data. The
    // extended resolution temperature can then be calculated using the
    // following equation:
                                    // COUNT_PER_C - COUNT_REMAIN
    // TEMPERATURE = TEMP_READ - 0.25 + --------------------------
                                            // COUNT_PER_C
    // Hagai Shatz simplified this to integer arithmetic for a 12 bits
    // value for a DS18S20, and James Cameron added legacy DS1820 support.
    // See - http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
    // */

    // if (deviceAddress[0] == DS18S20MODEL){
        // fpTemperature = ((fpTemperature & 0xfff0) << 3) - 16 +
            // (
                // ((scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) << 7) /
                  // scratchPad[COUNT_PER_C]
            // );
    // }

    // return fpTemperature;
    
    
    //recupera il dato di temperatura dallo scratchpad
    uint16_t iTemp = scratchPad.structure.temp;
    
    //se il modello è il DS1820 (o DS18S20)
    if (handle->Model == DS18S20)
    {
      //ha sempre risoluzione 9 bit.
      //per avere una risoluzione maggiore, bisogna usare altri registri dello scratchpad
      
      //tronca il bit meno significativo
      iTemp >>= 1;
      //calcola il valore sotto la virgola usando gli altri registri
      float value = ((float)(scratchPad.structure.countPerC - scratchPad.structure.countRemain)) / scratchPad.structure.countPerC;
      
      //calcola il valore di temperatura finale
      float fTemp = iTemp - 0.25 + value;
      return fTemp;     
    }
    else
    {
      //gli altri tipi di termometro hanno precisione programmabile da 9 a 12 bit
      //a seconda della precisione, bisogna azzerare alcuni bit
      
      //ricava la risoluzione
      dsTempResolution res = dsRetrieveResolution(handle, scratchPad);
      //a seconda della risoluzione
      uint16_t mask;
      switch (res)
      {
        case TEMP_9_BIT:
          mask = 0xFFF8; 
          break;
        case TEMP_10_BIT:
          mask = 0xFFFC; 
          break;
        case TEMP_11_BIT:
          mask = 0xFFFE; 
          break;
        case TEMP_12_BIT:
          mask = 0xFFFF; 
          break;
      }
      //porta a zero i bit fuori maschera
      iTemp &= mask;
      
      //calcola il valore float di temperatura
      float fTemp = (1.0 * iTemp) / 16;
      
      return fTemp;
    }
}


















