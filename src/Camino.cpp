#define IN_CAMINO_LIBRARY
#include "Camino.h"


void __attribute__ ((weak)) initTransmissions() {};
void __attribute__ ((weak)) beginTransmission() {};
void __attribute__ ((weak)) endTransmission() {};

extern void Camino_InitPort(unsigned long);
extern void Camino_SendByte(byte b);
extern void Camino_EnableByteSentISR();
extern void Camino_DisableByteSentISR();
extern byte Camino_ReadByte();

extern const byte Camino_MAX_DATA_LENGTH;
#define MAX_DATA_LENGTH Camino_MAX_DATA_LENGTH
extern const byte Camino_COMMAND_TIMEOUT_MS;
#define COMMAND_TIMEOUT_MS Camino_COMMAND_TIMEOUT_MS


// Implementation. This was originally in a separate .cpp file. It's been
// moved in order to allow users to #define options without modifying any
// files.

// Constants for the command packet that python sends to the arduino
#define COMMAND_HEADER_BYTE_1 0xAA
#define COMMAND_HEADER_BYTE_2 0x55

// Constants for the response packet that the arduino sends to python
#define RESPONSE_HEADER_WITH_NO_DATA    0xA9
#define RESPONSE_HEADER_WITH_DATA       0xAC
#define RESPONSE_HEADER_RESEND_REQUEST  0xB8

// Values for receiveState
#define WAITING_FOR_HEADER_BYTE_1     0
#define WAITING_FOR_HEADER_BYTE_2     1
#define WAITING_FOR_ADDRESS           2
#define WAITING_FOR_COMMAND_BYTE      3
#define WAITING_FOR_DATA_LENGTH_BYTE  4
#define WAITING_FOR_DATA_BYTES        5
#define WAITING_FOR_CHECKSUM_BYTE     6

// State when receiving/sending a packet.
byte receiveState;
unsigned long packetStartTimeMs;
byte packetAddress;
byte packetCommand;
//byte packetDataArray[MAX_DATA_LENGTH];
extern byte packetDataArray[];
byte packetDataLength;
byte packetLength;
byte packetChecksum;
byte packetTransmitIdx;
byte packetReceiveIdx;

// Arduino address. Only messages with this address will be processed.
byte thisAddress;
// We use a variable instead of packetDataLength == 0. We want there to be a
// distinction between returning no data and return an empty data array.
byte responseHasData;
// This buffer stores the *entire* packet for a response. Its size is at most
// 2 bytes for header, 1 byte for data length, MAX_DATA_LENGTH bytes for data,
// and 1 byte for checksum.
//byte packetArray[2 + 1 + MAX_DATA_LENGTH + 1];
extern byte packetArray[];
// When composing a response with data, we write directly to the response packet.
// Data array begins at offset 3.
byte *responseDataArray = &packetArray[3];


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



Camino camino;
Camino::Camino(){}

// Begin listening to serial port, with default address 0.
//   baudRate: baud rate (ie 9600)
void Camino::begin(long baudRate) {
    begin(baudRate, 0);
}

// Begin listening to serial port.
//   baudRate: baud rate (ie 9600)
//   address: this arduino's address (0 - 255)
void Camino::begin(long baudRate, byte address) {
  // remember the address we should respond too
  thisAddress = address;

  // init transmission hooks
  initTransmissions();

  // configure UART
  //Camino_InitPort(baudRate);
  Camino_InitPort(baudRate);

  // initialize state
  receiveState = WAITING_FOR_HEADER_BYTE_1;
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
  int i;
  byte checksum;

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
  Camino_SendByte(packetArray[packetTransmitIdx]);
  packetTransmitIdx++;

  // enable the interrupt that triggers when the transmit buffer is empty
  Camino_EnableByteSentISR();
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
ISR(Camino_ByteReadable_vect) {
  byte c;


  // check for a timeout receiving data
  if ((millis() - packetStartTimeMs) >= COMMAND_TIMEOUT_MS) {
    receiveState = WAITING_FOR_HEADER_BYTE_1;
  }

  // read the byte from the USART
  c = Camino_ReadByte();

  // select the operation based on the current state
  switch(receiveState) {
    case WAITING_FOR_HEADER_BYTE_1: {
      if (c == COMMAND_HEADER_BYTE_1) {
        // received first header byte, start the timeout timer
        packetStartTimeMs = millis();
        receiveState = WAITING_FOR_HEADER_BYTE_2;
      }
      break;
    }

    case WAITING_FOR_HEADER_BYTE_2: {
      if (c == COMMAND_HEADER_BYTE_2) {
        receiveState = WAITING_FOR_ADDRESS;
      } else {
        receiveState = WAITING_FOR_HEADER_BYTE_1;
      }
      break;
    }

    case WAITING_FOR_ADDRESS: {
      packetAddress = c;
      packetChecksum = c;
      receiveState = WAITING_FOR_COMMAND_BYTE;
      break;
    }

    case WAITING_FOR_COMMAND_BYTE: {
      packetCommand = c;
      packetChecksum += c;
      receiveState = WAITING_FOR_DATA_LENGTH_BYTE;
      break;
    }

    case WAITING_FOR_DATA_LENGTH_BYTE: {
      packetDataLength = c;
      packetChecksum += c;
      packetReceiveIdx = 0;

      if (packetDataLength == 0) {
        receiveState = WAITING_FOR_CHECKSUM_BYTE;
      } else {
        receiveState = WAITING_FOR_DATA_BYTES;
      }
      break;
    }

    case WAITING_FOR_DATA_BYTES: {
      if(packetReceiveIdx < MAX_DATA_LENGTH) {
        // only save data if it will fit in buffer
        packetDataArray[packetReceiveIdx] = c;
      }
      packetReceiveIdx++;
      packetChecksum += c;
      if (packetReceiveIdx == packetDataLength) {
        // done getting data
        receiveState = WAITING_FOR_CHECKSUM_BYTE;
      }
      break;
    }

    case WAITING_FOR_CHECKSUM_BYTE: {
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
      receiveState = WAITING_FOR_HEADER_BYTE_1;
      break;
    }
  }

}

// Interrupt service routine triggered when the transmit buffer is empty
ISR(Camino_ByteSent_vect) {

  // check if there is any more data in the packet to send
  if (packetTransmitIdx >= packetLength) {
    // nothing left to transmit
    // disable the interrupt that triggers when the transmit buffer is empty
    Camino_DisableByteSentISR();

    // call hook
    endTransmission();
    return;
  }


  // transmit the next byte in the packet
  Camino_SendByte(packetArray[packetTransmitIdx]);
  packetTransmitIdx++;
}

void numberOfCallables(byte dataLength, byte *dataArray) {
  returns((byte) (NUM_INTERNAL_CALLABLES + numberOfExternalCallables));
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
  analogWrite(dataArray[0], dataArray[1]);
}

void _analogRead(byte dataLength, byte *dataArray) {
  int value = analogRead(dataArray[0]);
  returns((short) value);
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

#define returnsType(type) void returns(type v) {\
  packetDataLength = sizeof(type);\
  for(byte i = 0; i < packetDataLength; i++) {\
    responseDataArray[i] = (byte) ((v & ((type) 0xff << (i * 8))) >> (i * 8));\
  }\
  responseHasData = 1;\
}

returnsType(byte);
returnsType(char);
returnsType(unsigned short);
returnsType(short);
returnsType(unsigned int);
returnsType(int);
returnsType(unsigned long);
returnsType(long);


