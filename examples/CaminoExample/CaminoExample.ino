/*
    CaminoExample

    This sketch is demonstrates how to create custom callables, that can
    be executed from within Python.

    See the Camino README for full details on running this example.

    Connect your Arduino to a computer running Python 3. Then run the following code:
      >>> import camino
      >>> connection = camino.SerialConnection(port='/dev/ttyACM0', baud=9600)
      >>> arduino = camino.Arduino(connection)
      >>> arduino.pin_mode(12, 0) # set pin 12 to INPUT
      >>> arduino.pin_mode(16, 1) # set pin 16 to OUTPUT
      >>> value = arduino.digital_read(12, out=int)
      >>> arduino.digital_write(16, value)
      >>> print(arduino.say_hi(out=str))
      I can say hi!
      >>> print(arduino.add(1, 2, 3, out=int))
      6

    Created 22 March 2022
    By Nathan Wachholz
*/

#include "Camino.h"

void setup() {
  camino.begin(9600);
}

void loop() {}

// Example that returns the sum of some data
void add(byte dataLength, byte *dataArray) {
  int sum = 0;
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
