#ifndef Camino_h
#define Camino_h

#include <Arduino.h>

// This file defines a number of configuration options for Camino.
// If you want to use other values, you'll need to manually download
// and include the library in your sketch, and change them. The
// defaults are used in all examples and should work for most users.

// The UART port used by Camino. Can be -1, 0, 1, 2. We default to
// UART0, which is usually the USB port. This means you can't use
// Serial in your sketch.
// See Camino.cpp for what this actually changes.
#define PORT 0

// The maximum length of the data section for packets. Value must
// be at least 16 and at most 250. Making it smaller reduces the
// memory footprint of Camino.
#define MAX_DATA_LENGTH 250

// Command timeout in milliseconds. When a new byte in a packet is received,
// if this period has elapsed since the start of the packet, the earlier
// bytes are discarded and Camino assumes that a new packet is beginning.
#define COMMAND_TIMEOUT_MS 100

// Macros for transmission hooks. Sometimes you may need to do some
// preparation before a packet can be sent. For instance, you may have
// a pin that enables a pull down resistor to remove noise from your
// transmission line when not sending anything. Or when debugging, it
// may be useful to shine an LED when packets are being sent.
#define initTransmissions() {} // called in begin()
#define beginTransmission() {} // called before the first byte is sent in a response packet.
#define endTransmission() {} // called after the last byte is sent in a response packet.


class Camino
{
  public:
    Camino();
    void begin(long baudRate);
    void begin(long baudRate, byte address);
    void processCommand(byte command, byte dataLength);
    void respondWithResendRequest();
  private:
    void respondWithNoData();
    void respondWithData(byte dataLength);
    void sendResponsePacket();
};
extern Camino camino;

typedef void Handler(byte dataLength, byte dataArray[]);
typedef struct callable {
  const char * shortName; //must be <= 16 characters
  Handler * call;
} Callable;
extern Callable callables[];
extern byte numberOfExternalCallables;

#define BEGIN_CALLABLES Callable callables[] =
#define END_CALLABLES ; \
byte numberOfExternalCallables = sizeof(callables) / sizeof(Callable);

void returns(const char* string);
void returns(byte dataLength, byte dataArray[]);
void returns(byte v);
void returns(short v);
void returns(int v);
void returns(long v);

#endif
