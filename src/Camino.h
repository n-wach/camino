#ifndef Camino_h
#define Camino_h

#ifdef IN_CAMINO_LIBRARY
  #include "arch.h"
#else
  #include "arch_user.h"
#endif


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



#ifdef IN_CAMINO_LIBRARY
  extern void Camino_InitPort(unsigned long);
  extern void Camino_SendByte(byte b);
  extern void Camino_EnableByteSentISR();
  extern void Camino_DisableByteSentISR();
  extern byte Camino_ReadByte();

  extern const byte Camino_MAX_DATA_LENGTH;
  #define MAX_DATA_LENGTH Camino_MAX_DATA_LENGTH
  extern const byte Camino_COMMAND_TIMEOUT_MS;
  #define COMMAND_TIMEOUT_MS Camino_COMMAND_TIMEOUT_MS

#else
  byte packetDataArray[MAX_DATA_LENGTH];
  byte packetArray[2 + 1 + MAX_DATA_LENGTH + 1];
  byte Camino_COMMAND_TIMEOUT_MS  __attribute__((used, retain)) = COMMAND_TIMEOUT_MS;
  byte Camino_MAX_DATA_LENGTH __attribute__((used, retain)) = MAX_DATA_LENGTH;
  extern Camino camino;
#endif


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

// Declare internal callables
Handler numberOfCallables;
Handler getNthCallable;
Handler echo;
Handler _pinMode;
Handler _digitalWrite;
Handler _digitalRead;
Handler _analogWrite;
Handler _analogRead;

extern Callable internalCallables[];
#define NUM_INTERNAL_CALLABLES (sizeof(internalCallables) / sizeof(Callable))


#endif
