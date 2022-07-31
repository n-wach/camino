#include "wiring_private.h"
#include "Camino.h"

// Configure register and interrupt names by port
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
  #ifdef ARDUINO_AVR_UNO
    #define USARTN_RX_vect    USART_RX_vect
    #define USARTN_UDRE_vect  USART_UDRE_vect
  #else
    #define USARTN_RX_vect    USART0_RX_vect
    #define USARTN_UDRE_vect  USART0_UDRE_vect
  #endif
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

// Constants for the command packet that the master sends to the slave
const byte COMMAND_HEADER_BYTE_1 = 0xAA;
const byte COMMAND_HEADER_BYTE_2 = 0x55;

// Constants for the response packet that the slave sends to the master
const byte RESPONSE_HEADER_WITH_NO_DATA = 0xA9;
const byte RESPONSE_HEADER_WITH_DATA = 0xAC;
const byte RESPONSE_HEADER_RESEND_REQUEST = 0xB8;

// Values for slaveState
const byte STATE_WAITING_FOR_HEADER_BYTE_1 = 0;
const byte STATE_WAITING_FOR_HEADER_BYTE_2 = 1;
const byte STATE_WAITING_FOR_SLAVE_ADDRESS = 2;
const byte STATE_WAITING_FOR_COMMAND_BYTE = 3;
const byte STATE_WAITING_FOR_DATA_LENGTH_BYTE = 4;
const byte STATE_WAITING_FOR_DATA_BYTES = 5;
const byte STATE_WAITING_FOR_CHECKSUM_BYTE = 6;

// Variables global to this module
byte thisAddress;
byte slaveState;
// We don't use packetDataLength == 0, so there remains the distinction of
// commands returning no data vs returning a data array that's empty.
byte responseHasData;

// This buffer stores the *entire* packet for a response. Its size is at most
// 2 bytes for header, 1 byte for data length, MAX_DATA_LENGTH bytes for data,
// and 1 byte for checksum.
byte packetArray[2 + 1 + MAX_DATA_LENGTH + 1];
// When composing a response with data, we write directly to the response packet.
// Data array begins at offset 3.
byte *responseDataArray = &packetArray[3];

// Variables per packet
unsigned long packetStartTimeMs;
byte packetAddress;
byte packetCommand;
byte packetDataArray[MAX_DATA_LENGTH];
byte packetDataLength;
byte packetLength;
byte packetChecksum;
byte packetTransmitIdx;
byte packetReceiveIdx;

// Declare internal callables
Handler numberOfCallables;
Handler getNthCallable;
Handler echo;
Handler _pinMode;
Handler _digitalWrite;
Handler _digitalRead;
Handler _analogWrite;
Handler _analogRead;

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
#define NUM_INTERNAL_CALLABLES (sizeof(internalCallables) / sizeof(Callable))

// Declare public camino object.
Camino camino;
Camino::Camino()
{
}

// Begin listening to serial port, with default address 0.
//   baudRate: baud rate (ie 9600)
void Camino::begin(long baudRate) {
    begin(baudRate, 0);
}

// Begin listening to serial port.
//   baudRate: baud rate (ie 9600)
//   address: this slave's address (0 - 255)
void Camino::begin(long baudRate, byte address) {
  uint16_t clockRate;

  // remember the address this slave should respond too
  thisAddress = address;

  // init transmission hooks
  initTransmissions();

  // configure baudrate registers of UART
  clockRate = (uint16_t) ((F_CPU / (8L * baudRate)) - 1L);
  UCSRNA = 1 << U2XN; // enable double rate (2X flag)
  UBRRNH = clockRate >> 8;
  UBRRNL = clockRate & 0xff;

  // set 8 bit, no parity, 1 stop
  UCSRNC = 0x06;

  // enable RX and TX interrupts
  sbi(UCSRNB, RXENN);
  sbi(UCSRNB, TXENN);

  // enable serial receive complete interrupt
  sbi(UCSRNB, RXCIEN);

  // initialize state
  slaveState = STATE_WAITING_FOR_HEADER_BYTE_1;
  packetStartTimeMs = 0;
}

// Send response indicating no data is being sent
void Camino::respondWithNoData() {
  packetArray[0] = RESPONSE_HEADER_WITH_NO_DATA;
  packetArray[1] = RESPONSE_HEADER_WITH_NO_DATA;
  packetLength = 2;
  sendResponsePacket();
}

// Send response with additional data. The data should already be written
// to packetDataArray.
//   dataLength: number of data bytes to send (at most MAX_DATA_LENGTH)
void Camino::respondWithData(byte dataLength) {
  int dataArrayToMasterIdx;
  int i;
  byte checksum;
  byte c;

  packetArray[0] = RESPONSE_HEADER_WITH_DATA;
  packetArray[1] = RESPONSE_HEADER_WITH_DATA;
  packetArray[2] = dataLength;

  // calculate checksum
  checksum = dataLength;
  for (i = 0; i < dataLength; i++) {
    checksum += responseDataArray[i];
  }
  packetArray[3 + dataLength] = checksum;
  packetLength = 4 + dataLength;

  sendResponsePacket();
}

// Send response indicating the command should be resent
void Camino::respondWithResendRequest() {
  packetArray[0] = RESPONSE_HEADER_RESEND_REQUEST;
  packetArray[1] = RESPONSE_HEADER_RESEND_REQUEST;
  packetLength = 2;
  sendResponsePacket();
}

// Begin sending the response packet
void Camino::sendResponsePacket() {
  // maintain transmission hooks
  beginTransmission();

  // transmit the first byte in the packet
  packetTransmitIdx = 0;
  UDRN = packetArray[packetTransmitIdx];
  packetTransmitIdx++;

  // enable the interrupt that triggers when the transmit buffer is empty
  sbi(UCSRNB, UDRIEN);
}

// Dispatch correct handler, and respond according to handler return
void Camino::processCommand(byte command, byte dataLength){
  // by default, return no data
  responseHasData = 0;
  Callable c;
  if(command < NUM_INTERNAL_CALLABLES) {
    c = internalCallables[command];
  } else {
    c = callables[command - NUM_INTERNAL_CALLABLES];
  }

  // make call to handler. for an incoming command, packetArray holds the
  // data section.
  c.call(dataLength, packetDataArray);

  // the handler calls some returns(...) functions, which set responseHasData
  // and packetDataLength.
  if(responseHasData) {
    respondWithData(packetDataLength);
  } else {
    respondWithNoData();
  }
}

// Interrupt service routine for characters received from the serial port
ISR(USARTN_RX_vect) {
  byte c;

  // check for a timeout receiving data
  if ((millis() - packetStartTimeMs) >= COMMAND_TIMEOUT_MS) {
    slaveState = STATE_WAITING_FOR_HEADER_BYTE_1;
  }

  // read the byte from the USART
  c = UDRN;

  // select the operation based on the current state
  switch(slaveState) {
    case STATE_WAITING_FOR_HEADER_BYTE_1: {
      if (c == COMMAND_HEADER_BYTE_1) {
        // received first header byte, start the timeout timer
        packetStartTimeMs = millis();
        slaveState = STATE_WAITING_FOR_HEADER_BYTE_2;
      }
      break;
    }

    case STATE_WAITING_FOR_HEADER_BYTE_2: {
      if (c == COMMAND_HEADER_BYTE_2) {
        slaveState = STATE_WAITING_FOR_SLAVE_ADDRESS;
      } else {
        slaveState = STATE_WAITING_FOR_HEADER_BYTE_1;
      }
      break;
    }

    case STATE_WAITING_FOR_SLAVE_ADDRESS: {
      packetAddress = c;
      packetChecksum = c;
      slaveState = STATE_WAITING_FOR_COMMAND_BYTE;
      break;
    }

    case STATE_WAITING_FOR_COMMAND_BYTE: {
      packetCommand = c;
      packetChecksum += c;
      slaveState = STATE_WAITING_FOR_DATA_LENGTH_BYTE;
      break;
    }

    case STATE_WAITING_FOR_DATA_LENGTH_BYTE: {
      packetDataLength = c;
      packetChecksum += c;
      packetReceiveIdx = 0;

      if (packetDataLength == 0) {
        slaveState = STATE_WAITING_FOR_CHECKSUM_BYTE;
      } else {
        slaveState = STATE_WAITING_FOR_DATA_BYTES;
      }
      break;
    }

    case STATE_WAITING_FOR_DATA_BYTES: {
      if(packetReceiveIdx < MAX_DATA_LENGTH) {
        // only save data if it will fit in buffer
        packetDataArray[packetReceiveIdx] = c;
      }
      packetReceiveIdx++;
      packetChecksum += c;
      if (packetReceiveIdx == packetDataLength) {
        // done getting data
        slaveState = STATE_WAITING_FOR_CHECKSUM_BYTE;
      }
      break;
    }

    case STATE_WAITING_FOR_CHECKSUM_BYTE: {
      if (c == packetChecksum) {
        // verify this packet is for this address
        if (packetAddress == thisAddress) {
          // execute the command (and respond)
          camino.processCommand(packetCommand, packetDataLength);
        }
      } else {
        // checksum error, request that the command be resent
        camino.respondWithResendRequest();
      }
      // ready for next packet
      slaveState = STATE_WAITING_FOR_HEADER_BYTE_1;
      break;
    }
  }
}

// Interrupt service routine triggered when the transmit buffer is empty
ISR(USARTN_UDRE_vect) {
  // check if there is any more data in the packet to send
  if (packetTransmitIdx >= packetLength) {
    // nothing left to transmit
    // disable the interrupt that triggers when the transmit buffer is empty
    cbi(UCSRNB, UDRIEN);
    // call hook
    endTransmission();
    return;
  }

  // transmit the next byte in the packet
  UDRN = packetArray[packetTransmitIdx];
  packetTransmitIdx++;
}

void numberOfCallables(byte dataLength, byte *dataArray) {
  returns(NUM_INTERNAL_CALLABLES + numberOfExternalCallables);
}

void getNthCallable(byte dataLength, byte *dataArray) {
  byte nth = dataArray[0];
  if(nth < NUM_INTERNAL_CALLABLES) {
    returns(internalCallables[nth].shortName);
  } else {
    returns(callables[nth - NUM_INTERNAL_CALLABLES].shortName);
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

void returns(const char* string) {
  packetDataLength = strlen(string);
  for(byte i = 0; i < packetDataLength; i++) {
    responseDataArray[i] = string[i];
  }
  responseHasData = 1;
}

void returns(byte dataLength, byte *dataArray) {
  packetDataLength = dataLength;
  for(byte i = 0; i < packetDataLength; i++) {
    responseDataArray[i] = dataArray[i];
  }
  responseHasData = 1;
}

void returns(byte v) {
  packetDataLength = 1;
  responseDataArray[0] = v;
  responseHasData = 1;
}
