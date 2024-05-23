#include "Camino.h"


void setup() {
  camino.begin(115200);
}

void loop() {}

void add(byte dataLength, byte *dataArray) {
  byte sum = 0;
  for(byte i = 0; i < dataLength; i++){
    sum += dataArray[i];
  }
  returns(sum);
}

void sayHi(byte dataLength, byte *dataArray) {
  returns("I can say hi!");
}

void returnByte(byte dataLength, byte *dataArray) {
  byte val = 229;
  returns(val);
}

void returnChar(byte dataLength, byte *dataArray) {
  char val = -127;
  returns(val);
}

void returnUShort(byte dataLength, byte *dataArray) {
  unsigned short val = 6000;
  returns(val);
}

void returnShort(byte dataLength, byte *dataArray) {
  short val = -1000;
  returns(val);
}

void returnUInt(byte dataLength, byte *dataArray) {
  unsigned int val = 6000;
  returns(val);
}

void returnInt(byte dataLength, byte *dataArray) {
  int val = -1000;
  returns(val);
}

void returnULong(byte dataLength, byte *dataArray) {
  unsigned long val = 4294967295UL;
  returns(val);
}

void returnLong(byte dataLength, byte *dataArray) {
  long val = -2147483648L;
  returns(val);
}

BEGIN_CALLABLES {
  {"add", add},
  {"say_hi", sayHi},
  {"return_byte", returnByte},
  {"return_char", returnChar},
  {"return_ushort", returnUShort},
  {"return_short", returnShort},
  {"return_uint", returnUInt},
  {"return_int", returnInt},
  {"return_ulong", returnULong},
  {"return_long", returnLong},
} END_CALLABLES;
