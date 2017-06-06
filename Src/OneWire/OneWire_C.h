

///////////////////////////////////////////////////////////
////       Libreria per protocollo OneWire per C       ////
///////////////////////////////////////////////////////////


#ifndef __ONEWIRE_C_H_
#define __ONEWIRE_C_H_


//#include <inttypes.h>
#include "stm32l4xx_hal.h"
//#include "gpio.h"

typedef uint8_t BOOL;

////////////////////////////////////////
////    DECLARATIONS

//struttura indirizzo di un device OneWire
typedef struct
{
  //device type
  uint8_t type;
  //device address
  uint8_t address[6];
  //crc
  uint8_t crc;
  
} owSDeviceAddr;

//indirizzo di un device address
typedef union
{
  owSDeviceAddr fields;
  uint64_t addr;
  uint8_t rom[8];
} owDeviceAddr;


//struttura con i dati per la ricerca di periferiche
typedef struct 
{
  // global search state
  unsigned char ROM_NO[8];
  uint8_t LastDiscrepancy;
  uint8_t LastFamilyDiscrepancy;
  uint8_t LastDeviceFlag;
} owSearchState;


//rappresenta una porta di STM32
typedef struct
{
//  //registro porta
//  GPIO_TypeDef* Reg;
//  //Pin
//  uint16_t Pin;
//
//  GPIO_InitTypeDef _pinSetup; //struttura per il settaggio del pin


  GPIO_TypeDef*         m_Port;
  uint16_t              m_Pin;
  uint32_t              m_BitMaskMode;
  uint8_t               m_PinNumber;

//  GPIO_TypeDef*         m_Port;
  __IO uint32_t*        m_Register;
  uint32_t              m_RegMask;
  uint32_t              m_InputMask;
  uint32_t              m_OutputMask;
  uint16_t m_BitMask;

}owPort;


//struttura handle OneWire
typedef struct 
{
  //Porta a cui � collegato OneWire
  owPort Port;
  //device correntemente selezionata da OneWire
  owDeviceAddr SelectedDevice;
  
  owSearchState _searchState; //stato dell'algoritmo di ricerca di periferiche
  GPIO_InitTypeDef _pinSetup; //struttura per il settaggio del pin
} HOneWire;


//Modalità di ricerca di device
typedef enum 
{
  NORMAL_SEARCH,
  CONDITIONAL_SEARCH
} owESearchMode;


//////////////////////////////////////////////////////////////
////    FUNCTIONS

//Crea l'handle OneWire agganciato al pin specificato
HOneWire owCreateHandle(GPIO_TypeDef* port, uint16_t pin);
//inizializza onewire
void owBegin(HOneWire* handle);

//scrive un byte sulla linea
void owWrite(HOneWire* handle, uint8_t value);
//legge un byte dalla linea
uint8_t owRead(HOneWire* handle);
//scrive un vettore di byte
void owWriteBytes(HOneWire* handle, uint8_t* buf, int size);
//legge un vettore di byte
void owReadBytes(HOneWire* handle, uint8_t* buf, int size);

//funzioni di più alto livello

//Resetta la linea. Ritorna 1 se ci sono slave, 0 se non ci sono. -1 in caso di errore (linea occupata)
int owReset(HOneWire* handle);
//Esegue una SELECT sull'indirizzo specificato
void owSelect(HOneWire* handle, owDeviceAddr device);
//Esegue una SKIP
void owSkip(HOneWire* handle);

// Clear the search state so that if will start from the beginning again.
void owResetSearch(HOneWire* handle);
// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
void owTargetSearch(HOneWire* handle, uint8_t family_code);
// Look for the next device. Returns TRUE if a new address has been
// returned. 
// A FALSE might mean:
//   - the bus is shorted
//   - there are no devices
//   - you have already retrieved all of them.
// It might be a good idea to check the CRC to make sure you didn't
// get garbage.  The order is deterministic. You will always get
// the same devices in the same order.
// If a device is discovered, the OneWire protocol selects it and store it's
// address in "SelectedDevice" property of OneWire handle.
BOOL owSearch(HOneWire* handle, owESearchMode search_mode);

///Calcola il Dallas OneWire CRC del buffer indicato
uint8_t owCRC(uint8_t* buffer, int len);



#endif  //__ONEWIRE_C_H_
