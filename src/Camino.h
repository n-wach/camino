#ifndef Camino_h
#define Camino_h

#include <Arduino.h>

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
  const char * shortName; //must be <= MAX_DATA_LENGTH characters
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
void returns(char v);
void returns(unsigned short v);
void returns(short v);
void returns(unsigned int v);
void returns(int v);
void returns(unsigned long v);
void returns(long v);

#endif
