#include "wiring_private.h"
#include "Camino.h"

// PORT should be -1 (for blank), 0, 1, or 2; depending on board
#ifndef PORT
  // We default to the USB port (UART 0). If you want to use something
  // else, you'll need to manually download and include the library,
  // and change this number.
  #define PORT 0
#endif

#if PORT == -1
  #define UCSRNA            UCSRA
  #define UCSRNB            UCSRB
  #define UCSRNC            UCSRC
  #define U2XN              U2X
  #define UBRRNH            UBRRH
  #define UBRRNL            UBRRL
  #define RXENN             RXEN
  #define TXENN             TXEN
  #define RXCIEN            RXCIE
  #define USARTN_RX_vect    USART_RX_vect
  #define USARTN_UDRE_vect  USART_UDRE_vect
  #define UDRN              UDR
  #define UDRIEN            UDRIE
#elif PORT == 0
  #define UCSRNA            UCSR0A
  #define UCSRNB            UCSR0B
  #define UCSRNC            UCSR0C
  #define U2XN              U2X0
  #define UBRRNH            UBRR0H
  #define UBRRNL            UBRR0L
  #define RXENN             RXEN0
  #define TXENN             TXEN0
  #define RXCIEN            RXCIE0
  #define USARTN_RX_vect    USART_RX_vect    // no number for some reason
  #define USARTN_UDRE_vect  USART_UDRE_vect  // no number for some reason
  #define UDRN              UDR0
  #define UDRIEN            UDRIE0
#elif PORT == 1
  #define UCSRNA            UCSR1A
  #define UCSRNB            UCSR1B
  #define UCSRNC            UCSR1C
  #define U2XN              U2X1
  #define UBRRNH            UBRR1H
  #define UBRRNL            UBRR1L
  #define RXENN             RXEN1
  #define TXENN             TXEN1
  #define RXCIEN            RXCIE1
  #define USARTN_RX_vect    USART1_RX_vect
  #define USARTN_UDRE_vect  USART1_UDRE_vect
  #define UDRN              UDR1
  #define UDRIEN            UDRIE1
#elif PORT == 2
  #define UCSRNA            UCSR2A
  #define UCSRNB            UCSR2B
  #define UCSRNC            UCSR2C
  #define U2XN              U2X2
  #define UBRRNH            UBRR2H
  #define UBRRNL            UBRR2L
  #define RXENN             RXEN2
  #define TXENN             TXEN2
  #define RXCIEN            RXCIE2
  #define USARTN_RX_vect    USART2_RX_vect
  #define USARTN_UDRE_vect  USART2_UDRE_vect
  #define UDRN              UDR2
  #define UDRIEN            UDRIE2
#endif

// constants for the command packet that the master sends to the slave
const byte MASTER_COMMAND_HEADER_BYTE_1 = 0xAA;
const byte MASTER_COMMAND_HEADER_BYTE_2 = 0x55;
const unsigned long MASTER_COMMAND_TIMEOUT_PERIOD_MS = 100;
const byte MASTER_COMMAND_MAX_PACKET_BYTES = MASTER_COMMAND_MAX_DATA_BYTES + 4;

// constants for the response packet that the slave sends to the master
const byte SLAVE_RESPONSE_RECEIVED_COMMAND = 0xA9;
const byte SLAVE_RESPONSE_RECEIVED_COMMAND_SENDING_DATA =0xAC;
const byte SLAVE_RESPONSE_RESEND_COMMAND =0xB8;
const byte SLAVE_RESPONSE_MAX_PACKET_BYTES = SLAVE_RESPONSE_MAX_DATA_BYTES + 4;

// values for slaveState
const byte SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1 = 0;
const byte SLAVE_STATE_WAITING_FOR_HEADER_BYTE_2 = 1;
const byte SLAVE_STATE_WAITING_FOR_SLAVE_ADDRESS = 2;
const byte SLAVE_STATE_WAITING_FOR_COMMAND_BYTE = 3;
const byte SLAVE_STATE_WAITING_FOR_DATA_LENGTH_BYTE = 4;
const byte SLAVE_STATE_WAITING_FOR_DATA_BYTES = 5;
const byte SLAVE_STATE_WAITING_FOR_CHECKSUM_BYTE = 6;

// IO pin values
const int RS485_TRANSMIT_ENABLED = HIGH;
const int RS485_TRANSMIT_DISABLED = LOW;

// variables global to this module
byte thisSlavesAddress;
byte slaveState;
unsigned long startTimeForPacketFromHost;
byte slaveAddress;
byte checksum;
byte dataArrayFromMasterIdx;
byte dataLengthToMaster;
byte dataArrayToMaster[SLAVE_RESPONSE_MAX_PACKET_BYTES];
byte dataArrayToMasterIdx;
byte transmitEnablePin;
byte commandByteFromMaster;
byte dataLengthFromMaster;
byte dataArrayFromMaster[MASTER_COMMAND_MAX_DATA_BYTES];

// declare the serial slave object and make it externally available
Camino camino;
Camino::Camino()
{
}


// open the serial slave port
//   baudRate: baud rate (ie 9600)
//   slaveAddr: this slave's address (1 - 255)
//   transmitterEnablePin: pin number to enable/disable the transmit line
void Camino::open(long baudRate, byte slaveAddr, byte transmitterEnablePin)
{
  uint16_t clockRate;

  // remember the address this slave should respond too
  thisSlavesAddress = slaveAddr;

  // configure but disable transmit
  transmitEnablePin = transmitterEnablePin;
  pinMode(transmitEnablePin, OUTPUT);
  digitalWrite(transmitEnablePin, RS485_TRANSMIT_DISABLED);

  clockRate = (uint16_t) ((F_CPU / (8L * baudRate)) - 1L);
  UCSRNA = 1 << U2XN;
  UBRRNH = clockRate >> 8;
  UBRRNL = clockRate & 0xff;

  // enable transmitting and receiving
  sbi(UCSRNB, RXENN);
  sbi(UCSRNB, TXENN);

  // set 8 bit, no parity, 1 stop
  UCSRNC = 0x06;

  // enable interrupts for serial receive complete
  sbi(UCSRNB, RXCIEN);

  // initialize state variables
  slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
  startTimeForPacketFromHost = millis();
}

// send response to master indicating the command was received and no data is being sent
void Camino::respondToCommandSendingNoData()
{
  // build then send packet to master indicate command was received OK
  dataArrayToMaster[0] = SLAVE_RESPONSE_RECEIVED_COMMAND;
  dataArrayToMaster[1] = SLAVE_RESPONSE_RECEIVED_COMMAND;
  dataLengthToMaster = 2;
  sentResponsePacketToMaster();
}



// send response with additional data to master indicating the command was received
//   dataLength: number of data bytes to transmit to the master
//   data: array of bytes to send
void Camino::respondToCommandSendingWithData(byte dataLength, byte data[])
{
  int dataArrayToMasterIdx;
  int i;
  byte checksum;
  byte c;

  // build then packet to master indicate command was received OK with included data
  dataArrayToMasterIdx = 0;
  dataArrayToMaster[dataArrayToMasterIdx] = SLAVE_RESPONSE_RECEIVED_COMMAND_SENDING_DATA;
  dataArrayToMasterIdx++;
  dataArrayToMaster[dataArrayToMasterIdx] = SLAVE_RESPONSE_RECEIVED_COMMAND_SENDING_DATA;
  dataArrayToMasterIdx++;

  dataArrayToMaster[dataArrayToMasterIdx] = dataLength;
  dataArrayToMasterIdx++;
  checksum = dataLength;

  for (i = 0; i < dataLength; i++) {
    c = data[i];
    dataArrayToMaster[dataArrayToMasterIdx] = c;
    dataArrayToMasterIdx++;
    checksum += c;
  }
  dataArrayToMaster[dataArrayToMasterIdx] = checksum;
  dataArrayToMasterIdx++;
  dataLengthToMaster = dataArrayToMasterIdx;

  sentResponsePacketToMaster();
}

// interrupt service routine for characters received from the serial port
ISR(USARTN_RX_vect)
{
  byte c;
  unsigned long periodSinceBeginningOfPacket;

  // check for a timeout receiving data from the host
  periodSinceBeginningOfPacket = millis() - startTimeForPacketFromHost;
  if (periodSinceBeginningOfPacket >= MASTER_COMMAND_TIMEOUT_PERIOD_MS)
    slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;

  // read the byte from the USART
  c = UDRN;

  // select the operation based on the current state
  switch(slaveState)
  {
    // check if waiting for the first header byte
    case SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1:
    {
      if (c == MASTER_COMMAND_HEADER_BYTE_1)
      {
        // received first header byte, start the timeout timer
        startTimeForPacketFromHost = millis();
        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_2;
      }
      break;
    }

    // check if waiting for the second header byte
    case SLAVE_STATE_WAITING_FOR_HEADER_BYTE_2:
    {
      if (c == MASTER_COMMAND_HEADER_BYTE_2)
        slaveState = SLAVE_STATE_WAITING_FOR_SLAVE_ADDRESS;
      else
        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
      break;
    }

    // check if waiting for the slave address
    case SLAVE_STATE_WAITING_FOR_SLAVE_ADDRESS:
    {
      if (c != 0)
      {
        slaveAddress = c;
        checksum = c;
        slaveState = SLAVE_STATE_WAITING_FOR_COMMAND_BYTE;
      }
      else
        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
      break;
    }

    // check if waiting for the command byte
    case SLAVE_STATE_WAITING_FOR_COMMAND_BYTE:
    {
      commandByteFromMaster = c;
      checksum += c;
      slaveState = SLAVE_STATE_WAITING_FOR_DATA_LENGTH_BYTE;
      break;
    }

    // check if waiting for the date length byte
    case SLAVE_STATE_WAITING_FOR_DATA_LENGTH_BYTE:
    {
      dataLengthFromMaster = c;
      checksum += c;
      dataArrayFromMasterIdx = 0;

      if (dataLengthFromMaster == 0) {
        slaveState = SLAVE_STATE_WAITING_FOR_CHECKSUM_BYTE;
      }
      else if (dataLengthFromMaster <= MASTER_COMMAND_MAX_DATA_BYTES)
        slaveState = SLAVE_STATE_WAITING_FOR_DATA_BYTES;
      else
        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
      break;
    }

    // check if waiting for the date bytes
    case SLAVE_STATE_WAITING_FOR_DATA_BYTES:
    {
      dataArrayFromMaster[dataArrayFromMasterIdx] = c;
      dataArrayFromMasterIdx++;
      checksum += c;
      if (dataArrayFromMasterIdx == dataLengthFromMaster)
        slaveState = SLAVE_STATE_WAITING_FOR_CHECKSUM_BYTE;
      break;
    }

    // check if waiting for the checksum byte
    case SLAVE_STATE_WAITING_FOR_CHECKSUM_BYTE:
    {
      if (c == checksum)
      {
        // verify this packet is for this slave
        if (slaveAddress == thisSlavesAddress)
        {
          // execute the command received from the host if
          processCommandFromMaster(commandByteFromMaster, dataLengthFromMaster, dataArrayFromMaster);
        }

        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
      }

      else
      {
        // checksum error, request that the command be resent
        camino.sendResendCommandToMaster();
        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
      }
      break;
    }
  }
}

// send response to master indicating the command should be resent
void Camino::sendResendCommandToMaster(void)
{
  // build then send packet to master indicate command should be resent
  dataArrayToMaster[0] = SLAVE_RESPONSE_RESEND_COMMAND;
  dataArrayToMaster[1] = SLAVE_RESPONSE_RESEND_COMMAND;
  dataLengthToMaster = 2;
  sentResponsePacketToMaster();
}

// begin sending the response packet to the master
//   dataLengthToMaster: number of data bytes
//   dataArrayToMaster: data array with data to send to the master
void Camino::sentResponsePacketToMaster(void)
{
  // enable the RS485 transmit lines for this board
  digitalWrite(transmitEnablePin, RS485_TRANSMIT_ENABLED);
  delayMicroseconds(18);

  // This is a bit of a hack.  Add one byte to the number of characters transmitted.  This way
  // the transmit lines can be disabled as the last character is streamed out.  I tried
  // using the TXC interrupt, but had no success.
  dataLengthToMaster++;

  // transmit the first byte in the packet
  dataArrayToMasterIdx = 0;
  UDRN = dataArrayToMaster[dataArrayToMasterIdx];
  dataArrayToMasterIdx++;

  // enable the interrupt that triggers when the transmit buffer is empty
  sbi(UCSRNB, UDRIEN);
}

// interrupt service routine indicating the USART is ready to transmit the next byte
ISR(USARTN_UDRE_vect)
{
  // check if there is any more data in the packet to send
  if (dataArrayToMasterIdx >= dataLengthToMaster)
  {
    // nothing left to transmit, disable the interrupt and disable driving the RS-485 TX lines
    cbi(UCSRNB, UDRIEN);
    digitalWrite(transmitEnablePin, RS485_TRANSMIT_DISABLED);
    return;
  }

  // transmit the next byte in the packet
  UDRN = dataArrayToMaster[dataArrayToMasterIdx];
  dataArrayToMasterIdx++;
}

Callable internalCallables[] = {
  {"num_calls", numberOfCallables},
  {"get_nth_call", getNthCallable},
  {"echo", echo},
  {"pin_mode", _pinMode},
  {"digital_write", _digitalWrite},
  {"digital_read", _digitalRead},
  {"analog_read", _analogRead},
  {"analog_write", _analogWrite},
};

boolean returnWithData;
byte returnLength;
byte returnData[16];

extern Callable callables[];
extern byte numberOfExternalCallables;

byte numberOfInternalCallables = sizeof(internalCallables) / sizeof(Callable);

void numberOfCallables(byte dataLength, byte *dataArray) {
  returns(numberOfInternalCallables + numberOfExternalCallables);
}

void getNthCallable(byte dataLength, byte *dataArray) {
  byte nth = dataArray[0];
  if(nth < numberOfInternalCallables) {
    returns(internalCallables[nth].shortName);
  } else {
    returns(callables[nth - numberOfInternalCallables].shortName);
  }
}

void echo(byte dataLength, byte *dataArray) {
  returns(dataLength, dataArray);
}


void _pinMode(byte dataLength, byte *dataArray) {
  pinMode(dataArray[0], dataArray[1]);
}
void _digitalWrite(byte dataLength, byte *dataArray) {
  digitalWrite(dataArray[0], dataArray[1]);
}
void _digitalRead(byte dataLength, byte *dataArray) {
  returns(digitalRead(dataArray[0]));
}
void _analogWrite(byte dataLength, byte *dataArray) {
  analogWrite(dataArray[0], dataArray[1]*256 + dataArray[2]);
}
void _analogRead(byte dataLength, byte *dataArray) {
  int value = digitalRead(dataArray[0]);
  byte arr[2] = {(byte) (value % 256), (byte) value};
  returns(2, arr);
}

byte lengthOf(const char* string) {
  // assuming string is null terminated
  byte l = 0;
  while(string[l] != 0) {
    l++;
  }
  return l;
}

void returns(const char* string) {
  returnLength = lengthOf(string);
  for(byte i = 0; i < returnLength; i++) {
    returnData[i] = string[i];
  }
  returnWithData = true;
}

void returns(byte dataLength, byte *dataArray) {
  returnLength = dataLength;
  for(byte i = 0; i < returnLength; i++) {
    returnData[i] = dataArray[i];
  }
  returnWithData = true;
}

void returns(byte v) {
  returnLength = 1;
  returnData[0] = v;
  returnWithData = true;
}

void processCommandFromMaster(byte commandByteFromMaster,
                              byte dataLengthFromMaster,
                              byte dataArrayFromMaster[]){
  //by default we want no data returned
  returnWithData = false;
  Callable c;
  if(commandByteFromMaster < numberOfInternalCallables) {
    c = internalCallables[commandByteFromMaster];
  } else {
    c = callables[commandByteFromMaster - numberOfInternalCallables];
  }
  c.call(dataLengthFromMaster, dataArrayFromMaster);

  respondAccordingly();
}

void respondAccordingly() {
  if(returnWithData) {
    byte l = min(SLAVE_RESPONSE_MAX_DATA_BYTES, returnLength);
    camino.respondToCommandSendingWithData(l, returnData);
  } else {
    camino.respondToCommandSendingNoData();
  }
}
