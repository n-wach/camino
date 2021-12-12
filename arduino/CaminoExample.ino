#include "Camino.h"

#define ADDRESS 15

void setup() {
  camino.open(9600, ADDRESS, 40);
}

void loop() {
  
}

Func sayHi;

Callable callables[] = {
  {"say_hi", sayHi},
  {"add", add},
};

byte numberOfExternalCallables = sizeof(callables) / sizeof(Callable);

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
