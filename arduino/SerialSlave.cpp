#include "wiring_private.h"
#include "SerialSlave.h"


//
// constants for the command packet that the master sends to the slave
//
const byte MASTER_COMMAND_HEADER_BYTE_1 = 0xAA;
const byte MASTER_COMMAND_HEADER_BYTE_2 = 0x55;
const unsigned long MASTER_COMMAND_TIMEOUT_PERIOD_MS = 100;
const byte MASTER_COMMAND_MAX_PACKET_BYTES = MASTER_COMMAND_MAX_DATA_BYTES + 4;


//
// constants for the response packet that the slave sends to the master
//
const byte SLAVE_RESPONSE_RECEIVED_COMMAND = 0xA9;
const byte SLAVE_RESPONSE_RECEIVED_COMMAND_SENDING_DATA =0xAC;
const byte SLAVE_RESPONSE_RESEND_COMMAND =0xB8;
const byte SLAVE_RESPONSE_MAX_PACKET_BYTES = SLAVE_RESPONSE_MAX_DATA_BYTES + 4;


//
// values for slaveState
//
const byte SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1 = 0;
const byte SLAVE_STATE_WAITING_FOR_HEADER_BYTE_2 = 1;
const byte SLAVE_STATE_WAITING_FOR_SLAVE_ADDRESS = 2;
const byte SLAVE_STATE_WAITING_FOR_COMMAND_BYTE = 3;
const byte SLAVE_STATE_WAITING_FOR_DATA_LENGTH_BYTE = 4;
const byte SLAVE_STATE_WAITING_FOR_DATA_BYTES = 5;
const byte SLAVE_STATE_WAITING_FOR_CHECKSUM_BYTE = 6;


//
// IO pin values
//
const int RS485_TRANSMIT_ENABLED = HIGH;
const int RS485_TRANSMIT_DISABLED = LOW;


//
// variables global to this module
//
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


//
// declare the serial slave object and make it externally available
//
SerialSlave serialSlave;
SerialSlave::SerialSlave()
{
}


//
// open the serial slave port
//    Enter:  baudRate = baud rate (ie 9600)
//            slaveAddr = this slave's address (1 - 255)
//            transmitterEnablePin = pin number to enable/disable the transmit line
//
void SerialSlave::open(long baudRate, byte slaveAddr, byte transmitterEnablePin)
{
  uint16_t clockRate;

  
  //
  // remember the address this slave should respond too
  //
  thisSlavesAddress = slaveAddr;
  
  //
  // configure but disable transmit
  //
  transmitEnablePin = transmitterEnablePin;
  pinMode(transmitEnablePin, OUTPUT);
  digitalWrite(transmitEnablePin, RS485_TRANSMIT_DISABLED);
  
  //
  // set the baud rate assuming a 16Mhz clock and double speed operation
  //
  clockRate = (uint16_t) ((16000000 / 8L / baudRate) - 1L);
  UCSR2A = 1 << U2X2;
  UBRR2H = clockRate >> 8;
  UBRR2L = clockRate & 0xff;

  //
  // enable transmitting and receiving
  //
  sbi(UCSR2B, RXEN2);
  sbi(UCSR2B, TXEN2);

  //
  // set 8 bit, no parity, 1 stop
  //
  UCSR2C = 0x06;

  //
  // enable interrupts for serial receive complete
  //
  sbi(UCSR2B, RXCIE2);
 
  //
  // initialize state variables
  //
  slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
  startTimeForPacketFromHost = millis();
}



//
// send response to master indicating the command was received and no data is being sent
//
void SerialSlave::respondToCommandSendingNoData()
{
  //
  // build then send packet to master indicate command was received OK
  //
  dataArrayToMaster[0] = SLAVE_RESPONSE_RECEIVED_COMMAND;
  dataArrayToMaster[1] = SLAVE_RESPONSE_RECEIVED_COMMAND;
  dataLengthToMaster = 2;
  sentResponsePacketToMaster();
}



//
// send response with additional data to master indicating the command was received
//    Enter:  dataLength = number of data bytes to transmit to the master
//            data -> array of bytes to send
//
void SerialSlave::respondToCommandSendingWithData(byte dataLength, byte data[])
{
  int dataArrayToMasterIdx;
  int i;
  byte checksum;
  byte c;
  
  //
  // build then packet to master indicate command was received OK with included data
  //
  dataArrayToMasterIdx = 0;
  dataArrayToMaster[dataArrayToMasterIdx] = SLAVE_RESPONSE_RECEIVED_COMMAND_SENDING_DATA;
  dataArrayToMasterIdx++;
  dataArrayToMaster[dataArrayToMasterIdx] = SLAVE_RESPONSE_RECEIVED_COMMAND_SENDING_DATA;
  dataArrayToMasterIdx++;
  
  dataArrayToMaster[dataArrayToMasterIdx] = dataLength;
  dataArrayToMasterIdx++;
  checksum = dataLength;
  
  for (i = 0; i < dataLength; i++)
  {
    c = data[i];
    dataArrayToMaster[dataArrayToMasterIdx] = c;
    dataArrayToMasterIdx++;
    checksum += c;
  }
  
  dataArrayToMaster[dataArrayToMasterIdx] = checksum;
  dataArrayToMasterIdx++;
  
  dataLengthToMaster = dataArrayToMasterIdx;
  
  //
  // send the packet to the master
  //
  sentResponsePacketToMaster();
}

//
// interrupt service routine for characters received from the serial port
//
ISR(USART2_RX_vect)
{
  byte c;
  unsigned long periodSinceBeginningOfPacket;



  //
  // check for a timeout receiving data from the host
  //
  periodSinceBeginningOfPacket = millis() - startTimeForPacketFromHost;
  if (periodSinceBeginningOfPacket >= MASTER_COMMAND_TIMEOUT_PERIOD_MS)
    slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
  
  
  //
  // read the byte from the USART
  //
  c = UDR2;
  
  //
  // select the operation based on the current state
  //
  switch(slaveState)
  {
    //
    // check if waiting for the first header byte
    //
    case SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1:
    {
      if (c == MASTER_COMMAND_HEADER_BYTE_1)
      {
        //
        // received first header byte, start the timeout timer
        //
        startTimeForPacketFromHost = millis();
        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_2;
      }
      break;
    }

    //
    // check if waiting for the second header byte
    //
    case SLAVE_STATE_WAITING_FOR_HEADER_BYTE_2:
    {
      if (c == MASTER_COMMAND_HEADER_BYTE_2)
        slaveState = SLAVE_STATE_WAITING_FOR_SLAVE_ADDRESS;
      else
        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
      break;
    }

    //
    // check if waiting for the slave address
    //
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

    //
    // check if waiting for the command byte
    //
    case SLAVE_STATE_WAITING_FOR_COMMAND_BYTE:
    {
      commandByteFromMaster = c;
      checksum += c;
      slaveState = SLAVE_STATE_WAITING_FOR_DATA_LENGTH_BYTE;
      break;
    }

    //
    // check if waiting for the date length byte
    //
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

    //
    // check if waiting for the date bytes
    //
    case SLAVE_STATE_WAITING_FOR_DATA_BYTES:
    {
      dataArrayFromMaster[dataArrayFromMasterIdx] = c;
      dataArrayFromMasterIdx++;
      checksum += c;
      if (dataArrayFromMasterIdx == dataLengthFromMaster)
        slaveState = SLAVE_STATE_WAITING_FOR_CHECKSUM_BYTE;
      break;
    }

    //
    // check if waiting for the checksum byte
    //
    case SLAVE_STATE_WAITING_FOR_CHECKSUM_BYTE:
    {
      if (c == checksum)
      {
        //
        // verify this packet is for this slave
        //
        if (slaveAddress == thisSlavesAddress)
        {
          //
          // execute the command received from the host if
          //
          processCommandFromMaster(commandByteFromMaster, dataLengthFromMaster, dataArrayFromMaster);
        }
        
        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1; 
      }
      
      else
      {
        //
        // checksum error, request that the command be resent
        //
        serialSlave.sendResendCommandToMaster();
        slaveState = SLAVE_STATE_WAITING_FOR_HEADER_BYTE_1;
      }
      break;
    }
  }
}

//
// send response to master indicating the command should be resent
//
void SerialSlave::sendResendCommandToMaster(void)
{
  //
  // build then send packet to master indicate command should be resent
  //
  dataArrayToMaster[0] = SLAVE_RESPONSE_RESEND_COMMAND;
  dataArrayToMaster[1] = SLAVE_RESPONSE_RESEND_COMMAND;
  dataLengthToMaster = 2;
  sentResponsePacketToMaster();
}



//
// begin sending the response packet to the master
//    Enter:  dataLengthToMaster = number of data bytes
//            dataArrayToMaster -> data array with data to send to the master
//
void SerialSlave::sentResponsePacketToMaster(void)
{
  //
  // enable the RS485 transmit lines for this board
  //
  digitalWrite(transmitEnablePin, RS485_TRANSMIT_ENABLED);
  delayMicroseconds(18); 

  //
  // This is a bit of a hack.  Add one byte to the number of characters transmitted.  This way
  // the transmit lines can be disabled as the last character is streamed out.  I tried
  // using the TXC interrupt, but had no success.
  //
  dataLengthToMaster++;

  //
  // transmit the first byte in the packet
  //
  dataArrayToMasterIdx = 0;
  UDR2 = dataArrayToMaster[dataArrayToMasterIdx];
  dataArrayToMasterIdx++;
  
  //
  // enable the interrupt that triggers when the transmit buffer is empty
  //  
  sbi(UCSR2B, UDRIE2);
}



//
// interrupt service routine indicating the USART is ready to transmit the next byte
//
ISR(USART2_UDRE_vect)
{
  //
  // check if there is any more data in the packet to send
  //
  if (dataArrayToMasterIdx >= dataLengthToMaster)
  {
    //
    // nothing left to transmit, disable the interrupt and disable driving the RS-485 TX lines
    //
    cbi(UCSR2B, UDRIE2);
    digitalWrite(transmitEnablePin, RS485_TRANSMIT_DISABLED);
    return;
  }
  
  //
  // transmit the next byte in the packet
  //
  UDR2 = dataArrayToMaster[dataArrayToMasterIdx];
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
  byte arr[2] = {value%256, value};
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
    serialSlave.respondToCommandSendingWithData(l, returnData);
  } else {
    serialSlave.respondToCommandSendingNoData();
  }
}
