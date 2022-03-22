#include "Camino.h"

void setup() {
  camino.begin(9600);
}

void loop() {}

// Example that returns the sum of some data (%256)
void add(byte dataLength, byte *dataArray) {
  byte sum = 0;
  for(byte i = 0; i < dataLength; i++){
    sum += dataArray[i];
  }
  returns(sum);
}

// Example that returns a string
void sayHi(byte dataLength, byte *dataArray) {
  returns("I can say hi!");
}

BEGIN_CALLABLES {
  {"say_hi", sayHi},
  {"add", add},
} END_CALLABLES;
