
#include "OneWire_C.h"

//////////////////////////////////////////////////////////////////
///////   OneWireInternal

//COMANDI ONEWIRE
#define COMMAND_SELECT                  0x55
#define COMMAND_SKIP                    0xCC
#define COMMAND_NORMAL_SEARCH           0xF0
#define COMMAND_CONDITIONAL_SEARCH      0xEC


#define ENABLE_INTERRUPTS()     //__disable_irq()
#define DISABLE_INTERRUPTS()    //__enable_irq()
#define DELAY_MICROSECONDS(x)   DelayM(x)

/**
 * @brief  Delays for amount of micro seconds
 * @param  micros: Number of microseconds for delay
 * @retval None
 */
void DelayM(uint32_t micros) {
	uint32_t start = DWT->CYCCNT;

	/* Go to number of cycles for system */
	micros *= (HAL_RCC_GetHCLKFreq() / 1000000);

	/* Delay till end */
	while ((DWT->CYCCNT - start) < micros);
}

//Imposta il pin della porta in input mode
void _port_mode_input(owPort* port)
{
  //modifica il registro di mode della porta
  uint32_t bitmask = 0x3 << (2 * port->m_PinNumber);
  port->m_Port->MODER &= ~bitmask;

//  //sistema la struttura di setup
//  //input mode
//  port->_pinSetup.Mode = GPIO_MODE_INPUT;
//  //attiva il pullup
//  port->_pinSetup.Pull = GPIO_NOPULL;
//  //speed
//  port->_pinSetup.Speed = 0;
//
//  //setta la porta
//  HAL_GPIO_Init(port->Reg, &port->_pinSetup);
}
//Imposta il pin della porta in output mode
void _port_mode_output(owPort* port)
{
  //modifica il registro di mode della porta
  uint32_t bitmask = 0x1 << (2 * port->m_PinNumber);
  port->m_Port->MODER |= bitmask;

//  //sistema la struttura di setup
//  //push-pull output mode
//  port->_pinSetup.Mode = GPIO_MODE_OUTPUT_PP;
//  //disattiva pullup
//  port->_pinSetup.Pull = GPIO_NOPULL;
//  //speed
//  port->_pinSetup.Speed = GPIO_SPEED_FREQ_LOW;
//
//  //setta la porta
//  HAL_GPIO_Init(port->Reg, &port->_pinSetup);
}
//Lettura della porta
uint8_t _port_read(owPort* port)
{
//  return (HAL_GPIO_ReadPin(port->Reg, port->Pin) ? 1 : 0);
  return (port->m_Port->IDR & port->m_Pin) ? 1 : 0;
}
//Scrittura sulla porta di valore LOW
void _port_write_low(owPort* port)
{
//  HAL_GPIO_WritePin(port->Reg, port->Pin, GPIO_PIN_RESET);
  port->m_Port->ODR &= ~port->m_Pin;
}
//Scrittura sulla porta di valore HIGH
void _port_write_high(owPort* port)
{
//  HAL_GPIO_WritePin(port->Reg, port->Pin, GPIO_PIN_SET);
  port->m_Port->ODR |= port->m_Pin;
}



//Write a bit on OneWire port
void owBitWrite(HOneWire* handle, uint8_t value)
{
  owPort* port = &(handle->Port);

  //a seconda del valore
  if (value & 0x01)
  {
    //deve scrivere un 1
    
    DISABLE_INTERRUPTS();
    //abbassa la linea
    _port_write_low(port);
    _port_mode_output(port);
    //pausa
    DELAY_MICROSECONDS(10);
    //alza la linea
    _port_write_high(port);
    ENABLE_INTERRUPTS();
    DELAY_MICROSECONDS(55);
  }
  else
  {
    //deve scrivere un 0
    
    DISABLE_INTERRUPTS();
    //abbassa la linea
    _port_write_low(port);
    _port_mode_output(port);
    //pausa
    DELAY_MICROSECONDS(65);
    //alza la linea
    _port_write_high(port);
    ENABLE_INTERRUPTS();
    DELAY_MICROSECONDS(5);
  }  
}

//Read a bit from OneWire port
uint8_t owBitRead(HOneWire* handle)
{
  owPort* port = &(handle->Port);
  uint8_t value;
  
  DISABLE_INTERRUPTS();
  _port_mode_output(port);
  _port_write_low(port);
  DELAY_MICROSECONDS(3);
  _port_mode_input(port); // let pin float, pull up will raise
  DELAY_MICROSECONDS(10);
  value = _port_read(port); //sample the port
	ENABLE_INTERRUPTS();
	DELAY_MICROSECONDS(53);

  return value;
}







//////////////////////////////////////////////////////////////////
///////   OneWire


//Crea l'handle OneWire agganciato al pin specificato
HOneWire owCreateHandle(GPIO_TypeDef* port, uint16_t pin)
{
  //Crea handle
  HOneWire handle;
  //setta la porta
  owPort pp;
  pp.m_Pin = pin;
  pp.m_Port = port;

  uint16_t appo = 0x1;
  for (pp.m_PinNumber = 0; appo < pin; pp.m_PinNumber++)
    appo <<= 1;

  handle.Port = pp;

  //setta bitmask
  //pp.m_BitMaskMode = 0x


//  pp.Reg = port;
//  pp.Pin = pin;
//  //setta alcuni parametri per il settaggio della porta
//  pp._pinSetup.Pin = pin;
//


//  uint32_t PeriphClock;
//
//  if (port == GPIOA)
//  {
//    PeriphClock = RCC_APB2Periph_GPIOA;
//  }
//  else if (port == GPIOB)
//  {
//    PeriphClock = RCC_APB2Periph_GPIOB;
//  }
//  else if (port == GPIOC)
//  {
//    PeriphClock = RCC_APB2Periph_GPIOC;
//  }
//  else if (port == GPIOD)
//  {
//    PeriphClock = RCC_APB2Periph_GPIOD;
//  }
//  else if (port == GPIOE)
//  {
//    PeriphClock = RCC_APB2Periph_GPIOE;
//  }
  /*else if (port == GPIOF)
  {
    PeriphClock = RCC_APB2Periph_GPIOF;
  }
  else
  {
    if (port == GPIOG)
    {
      PeriphClock = RCC_APB2Periph_GPIOG;
     }
  }*/

//    RCC_APB2PeriphClockCmd(PeriphClock, ENABLE);

    //inizializza la porta
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pin = pin;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &GPIO_InitStructure);

//    //salva alcuni dati della porta
//    pp.m_BitMask = pin;
//    pp.m_Port = port;


//    uint32_t pinpos = 0x00, pos = 0x00, currentpin = 0x00;
//    uint8_t RegShift;
//
//    if((pin & (uint32_t)0x00FF) > 0)
//    {
//      pp->m_Register = &port->CRL;
//
//      for (pinpos = 0x00; pinpos < 0x08; pinpos++)
//      {
//        pos = ((uint32_t)0x01) << pinpos;
//        /* Get the port pins position */
//        currentpin = (uint16_t)((GPIO_Pin) & pos);
//        if (currentpin == pos)
//        {
//          RegShift = (pinpos*4);
//          owire->m_RegMask = ((uint32_t)0x0F) << (pinpos*4);
//          break;
//        }
//       }
//    }
//    else
//    {
//      owire->m_Register = &GPIOx->CRH;
//
//      for (pinpos = 0x00; pinpos < 0x08; pinpos++)
//      {
//        pos = ((uint32_t)0x01) << (pinpos + 0x08);
//        /* Get the port pins position */
//        currentpin = (uint16_t)((GPIO_Pin) & pos);
//        if (currentpin == pos)
//        {
//          RegShift = (pinpos*4);
//          owire->m_RegMask = ((uint32_t)0x0F) << (pinpos*4);
//          break;
//        }
//      }
//    }
//
//    owire->m_InputMask = (((GPIO_Mode_IN_FLOATING) << RegShift) & owire->m_RegMask);
//    owire->m_OutputMask = (((uint32_t)GPIO_Mode_Out_OD|(uint32_t)GPIO_Speed_50MHz) << RegShift) & owire->m_RegMask;
//
//


  return handle;
}

//inizializza onewire
void owBegin(HOneWire* handle)
{
  //pone il pin in input mode
  _port_mode_input(&(handle->Port));

//  _port_mode_output(&(handle->Port));
//  _port_write_high(&(handle->Port));
}

//scrive un byte sulla linea
void owWrite(HOneWire* handle, uint8_t value)
{
  uint8_t bitMask;
  
  //scorre i bit del valore
  for (bitMask = 0x01; bitMask; bitMask <<= 1) 
  {
    owBitWrite(handle, (bitMask & value) ? 1 : 0 );
  }
}

//legge un byte dalla linea
uint8_t owRead(HOneWire* handle)
{
  uint8_t bitMask;
  uint8_t value = 0;

  //scorre i bit di un byte
  for (bitMask = 0x01; bitMask; bitMask <<= 1) 
  {
    uint8_t r = owBitRead(handle);
    //se ha letto un uno
    if (r)
      //aggiunge il relativo bit al valore di uscita
      value |= bitMask;
  }
  
  return value;
}

//scrive un vettore di byte
void owWriteBytes(HOneWire* handle, uint8_t* buf, int size)
{
  for (int i = 0; i < size; i++)
    owWrite(handle, buf[i]);
}
//legge un vettore di byte
void owReadBytes(HOneWire* handle, uint8_t* buf, int size)
{
  for (int i = 0; i < size; i++)
    buf[i] = owRead(handle);
}


//+++++++    funzioni di più alto livello  +++++++++++++++++++++++++++++++++



//Resetta la linea. Ritorna 1 se ci sono slave, 0 se non ci sono. -1 in caso di errore (linea occupata)
int owReset(HOneWire* handle)
{
  int retries = 125;
  uint8_t r;
  
  owPort* port = &(handle->Port);
  
  //porta la linea in ingresso
  DISABLE_INTERRUPTS();
  _port_mode_input(port);
  ENABLE_INTERRUPTS();
  //aspetta un po' nel caso ci sia qualcuno che sta comunicando
  do
  {
    //valuta il contatore di tentativi
    if (retries == 0)
      //finiti i tentativi e la linea è ancora occupata. Esce
      return -1;
    
    //aspetta 2 us
    DELAY_MICROSECONDS(2);
    //decrementa il contatore
    retries--;
  }
  while (! _port_read(port));
  //OK linea libera
  
  //esegue la procedura di reset
  
  //abbassa la linea per 480 us
  DISABLE_INTERRUPTS();
  _port_write_low(port);
  _port_mode_output(port);
  ENABLE_INTERRUPTS();
  DELAY_MICROSECONDS(480);
  //lascia il controllo della linea agli eventuali slave
  DISABLE_INTERRUPTS();
  _port_mode_input(port);
  //aspetta 70 us
  DELAY_MICROSECONDS(70);
  //esegue un campionamento della linea per vedere se gli slave hanno risposto al reset
  r = _port_read(port);
  ENABLE_INTERRUPTS();
  //attende 410 us come da protocollo
  DELAY_MICROSECONDS(410);
  
  //se r == 0, allora ci sono slave sulla linea. Altrimenti no
  return (r == 0) ? 1 : 0;
}

//Esegue una SELECT sull'indirizzo specificato
void owSelect(HOneWire* handle, owDeviceAddr device)
{
  //scrive il comando di select, seguito dall'indirizzo della periferica
  
  owWrite(handle, COMMAND_SELECT);
  for (int i = 0; i < 8; i++)
    owWrite(handle, device.rom[i]);
}
//Esegue una SKIP
void owSkip(HOneWire* handle)
{
  owWrite(handle, COMMAND_SKIP);
}



// Clear the search state so that if will start from the beginning again.
void owResetSearch(HOneWire* handle)
{
  // reset the search state
  handle->_searchState.LastDiscrepancy = 0;
  handle->_searchState.LastDeviceFlag = 0;
  handle->_searchState.LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--) 
  {
    handle->_searchState.ROM_NO[i] = 0;
    if (i == 0) break;
  }
}

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
void owTargetSearch(HOneWire* handle, uint8_t family_code)
{
  // set the search state to find SearchFamily type devices
  handle->_searchState.ROM_NO[0] = family_code;
  for (uint8_t i = 1; i < 8; i++)
    handle->_searchState.ROM_NO[i] = 0;
  handle->_searchState.LastDiscrepancy = 64;
  handle->_searchState.LastFamilyDiscrepancy = 0;
  handle->_searchState.LastDeviceFlag = 0;
}

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
uint8_t owSearch(HOneWire* handle, owESearchMode search_mode)
{

  uint8_t id_bit_number;
  uint8_t last_zero, rom_byte_number;
  uint8_t id_bit, cmp_id_bit;

  unsigned char rom_byte_mask, search_direction;
  uint8_t search_result;

  // initialize for search
  id_bit_number = 1;
  last_zero = 0;
  rom_byte_number = 0;
  rom_byte_mask = 1;
  search_result = 0;

  // if the last call was not the last one
  if (!handle->_searchState.LastDeviceFlag)
  {
    // 1-Wire reset
    if (!owReset(handle))
    {
      //nessun device ha risposto al reset (non ci sono!!)
      // reset the search
      handle->_searchState.LastDiscrepancy = 0;
      handle->_searchState.LastDeviceFlag = 0;
      handle->_searchState.LastFamilyDiscrepancy = 0;
      return 0;
    }

    // issue the search command
    if (search_mode == NORMAL_SEARCH) 
    {
      owWrite(handle, COMMAND_NORMAL_SEARCH);
    } 
    else 
    {
      owWrite(handle, COMMAND_CONDITIONAL_SEARCH);
    }

    // loop to do the search
    do
    {
      // read a bit and its complement
      id_bit = owBitRead(handle);
      cmp_id_bit = owBitRead(handle);

      //nessun device ha risposto
      // check for no devices on 1-wire
      if ((id_bit == 1) && (cmp_id_bit == 1))
        break;
      else
      {
        // all devices coupled have 0 or 1
        if (id_bit != cmp_id_bit)
          search_direction = id_bit;  // bit write value for search
        else
        {
          // if this discrepancy if before the Last Discrepancy
          // on a previous next then pick the same as last time
          if (id_bit_number < handle->_searchState.LastDiscrepancy)
            search_direction = ((handle->_searchState.ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
          else
            // if equal to last pick 1, if not then pick 0
            search_direction = (id_bit_number == handle->_searchState.LastDiscrepancy);

          // if 0 was picked then record its position in LastZero
          if (search_direction == 0)
          {
            last_zero = id_bit_number;

          // check for Last discrepancy in family
          if (last_zero < 9)
            handle->_searchState.LastFamilyDiscrepancy = last_zero;
          }
        }

        // set or clear the bit in the ROM byte rom_byte_number
        // with mask rom_byte_mask
        if (search_direction == 1)
          handle->_searchState.ROM_NO[rom_byte_number] |= rom_byte_mask;
        else
          handle->_searchState.ROM_NO[rom_byte_number] &= ~rom_byte_mask;

        // serial number search direction write bit
        owBitWrite(handle, search_direction);

        // increment the byte counter id_bit_number
        // and shift the mask rom_byte_mask
        id_bit_number++;
        rom_byte_mask <<= 1;

        // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
        if (rom_byte_mask == 0)
        {
          rom_byte_number++;
          rom_byte_mask = 1;
        }
      }
    }
    while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

    // if the search was successful then
    if (!(id_bit_number < 65))
    {
      // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
      handle->_searchState.LastDiscrepancy = last_zero;

      // check for last device
      if (handle->_searchState.LastDiscrepancy == 0)
        handle->_searchState.LastDeviceFlag = 1;

      search_result = 1;
    }
  }

  // if no device found then reset counters so next 'search' will be like a first
  if (!search_result || !handle->_searchState.ROM_NO[0])
  {
    handle->_searchState.LastDiscrepancy = 0;
    handle->_searchState.LastDeviceFlag = 0;
    handle->_searchState.LastFamilyDiscrepancy = 0;
    search_result = 0;
  }
  else 
  {
    //l'indirizzo della device trovata si trova in ROM_NO
    //questo indirizzo indica anche la device correntemente selezionata dal OneWire
    
    //copia l'indirizzo in handle
    for (int i = 0; i < 8; i++)
      handle->SelectedDevice.rom[i] = handle->_searchState.ROM_NO[i];
  }

  return search_result;
}


///Calcola il Dallas OneWire CRC del buffer indicato
uint8_t owCRC(uint8_t* buffer, int len)
{
  uint8_t crc = 0;

  for (int i = 0; i < len; i++)
  {
    uint8_t inbyte = buffer[i];
    for (uint8_t j = 8; j; j--) 
    {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix)
        crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  
  return crc;
}





























