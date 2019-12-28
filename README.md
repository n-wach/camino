# arduslave
This library allows Python 3 to communicate with up to 255 Arduino Megas over a serial connection.  It provides a simple way to call functions remotely on an Arduino.  These calls can send and receive data.  Besides built in functions (`digital_write`, `pin_mode`, `analog_read`, etc.), approximately 230 user-defined functions can be defined.

There are two completmentary parts of this repo:
 - `/arduino` for the Arduino library, including an example project
 - `/arduslave` for the Python 3 library


## Setup and testing a connection:

### Arduino

Here is a simple starting Arduino setup.  Upload it to your Arduino Mega, along with `SerialSalve.h` and `SerialSlave.cpp`.
```arduino
#include "SerialSlave.h"
#define ADDRESS XX //replace XX with an address other than 0
#define RATE 9600 //higher rates are supported
#define DISABLE_PIN 40 //it's recommened to connect a pin (D40) to a pulldown resistor that disables the serial transmission line

//Required by SerialSlave:
Callable callables[] = {
    //Your callables here
    //Format:
    //{"python_name", functionToRunOnArduino},
};

//Required by SerialSlave:
byte numberOfExternalCallables = sizeof(callables) / sizeof(Callable);

void setup() {
    //Initialize and begin listening
    serialSlave.open(RATE, ADDRESS, DISABLE_PIN); 
}

void loop() {
    //Nothing needed here...
}
```

### Python
Next, connect the Arduino Megas's `Serial2` Port to the python host.  Ideally this is to `/dev/ttyS0`, but any serial port should work.  If using `/dev/ttyS0` on a Raspberry Pi, you may need to disable the serial console.

Install the `python_arduslave` package, then open a shell and run the following:

```python
from python_arduslave import SerialMaster, Arduino

# Create connection
arduino = Arduino(SerialMaster(dev='/dev/ttyS0', baud=9600), ADDRESS)

# Test connection
print(arduino.echo("Hello World!", format_out=FORMAT_STRING))

# Should print "Hello World!" to the console
# Now you can control the Arduino however you'd like:
arduino.pin_mode([12, 0]) # set pin 12 to INPUT
arduino.pin_mode([16, 1]) # set pin 16 to OUTPUT

value = arduino.digital_read(12, format_out=FORMAT_BYTE) # we want to get a single value; without format_out this would return a list with a single element

arduino.digital_write([16, value]) 
```

## Callables

Callables act as a link between an Arduino function and the corresponding Python function.  The python names are sent from the Arduino when they establish a connection, and automatically added to the `Arduino` python object.

### Built-in Callables

There are a few internal Callables that expose some simple Arduino functions:
```arduino
Callable internalCallables[] = {
  {"num_calls", numberOfCallables}, //returns number of callables
  {"get_nth_call", getNthCallable}, //returns shortName of nth callable
  {"echo", echo}, //for testing; sends back received data
  {"pin_mode", _pinMode},
  {"digital_write", _digitalWrite},
  {"digital_read", _digitalRead},
  {"analog_read", _analogRead},
  {"analog_write", _analogWrite},
};
```

### User-defined Callables

Users can also easily create their own Callables.

Create a function of the type `void myFunction(byte dataLength, byte *data){...}`.  

Create a new Callable in callables.  Make sure your short name is no more than 16 characters long.  For instance:

```
Callable callables[] = {
    //Your callables here
    //Format:
    //{"short_name", functionToRunOnArduino},
    {"my_function", myFunction),
};
```
You may have to declare your function before adding it to the array if you define it after the array.  This can be done succinctly using `Func myFunction;`.  

### Doing stuff with your Callable:

Callables **MUST** return within 100ms.  Ideally, they should return in under 20ms.  You can `digitalWrite`, `pinMode`, `digitalRead`, etc.  Because Callables are called from an interrupt routine, they can't use `delay` or other interrupt-dependent functions.  If you want to do interrupt-based stuff or expect it to take more than ~20ms, have code running in `loop` dependent on some global variables.  You can change global variables from inside your Callable.

Data will be passed from the RPi to your function as an array with a given length.  Even if 1 or no parameters are expected, your function must accept a `byte` and `byte *` as parameters.

All functions must be `void`.  To return something to the RPi, use `returns(...)`.  This sets what data will be sent back to the RPi when your function finishes.  There are several possible arguments for `returns(...)`:

```arduino
void returns(const char* string); //returns("hello");
void returns(byte v); //returns(1);
void returns(byte dataLength, data *dataArray); //returns(dataLength, data); as in echo
```

You do not need to call `returns(...)`.  If a function does not call `returns(...)`, the Arduino simply responds with no data.  Calling `returns(...)` multiple times overwrites the return value.

**Important:** Because packets can be sent up to 3 times, it is important that your code not rely on the number of times it is called.  For instance, it would be a bad idea to have a function toggle a pin.  It would be better to keep track of the pin value on the RPi and send commands that set it.  See below for handling situations where function calls fail all 3 times.

Of course, if you are following this guide and getting any failed function calls, something is probably amiss.

## Protocol

### Packet structure

By default, packets can contain at most 16 bytes of user data.

Everything starts with a Master to Slave Command Packet:

|1 byte|1 byte|1 byte       | 1 byte | 1 byte     |`data length` bytes|1 byte  |
|------|------|-------------|--------|------------|-------------------|--------|
|0xAA  |0x55  |slave address|command |data length |data               |checksum|

The slave can respond with either a Slave to Master with Data Packet:

|1 byte|1 byte|1 byte     |`data length` bytes|1 byte  |
|------|------|-----------|-------------------|--------|
| 0xAC | 0xAC |data length|data               |checksum|

or a Slave to Master No Data Packet:

|1 byte|1 byte|
|------|------|
|0xA9  |0xA9  |

If some part of the transmission fails (such as an invalid checksum), the slave may send a Slave to Master Resend Request:

|1 byte|1 byte|
|------|------|
|0xA9  |0xA9  |

### Communication flow

For any given packet sent from the Master:
 * Send to Arduino with a specific address
 * Wait up to 100ms for a response
 * If we got a response:
    * If requesting retransmission (invalid checksum), send again
    * If successful, collect data, check checksum, and return
 * If no response received, send again
 * A packet will be sent 3 times before giving up

## Python

Python will automatically connect to the `Arduino` device at the given address.  The same `SerialMaster` instance should be passed to all `Arduino` objects on the Serial port.

After connecting to the device, Python will get the names of every `Callable` declared in `callables`.  It will add these functions to your `Arduino` instance. You can call these functions as you would call any other function, although only one command can be sent and received at a time across all `Arduino` instances on a given `SerialMaster`.

To help you send and receive different types of data, you can specify a `format_out` as one of the following:
 * `FORMAT_BYTE` - single value (0-255) as a python `int`
 * `FORMAT_LIST` - array of up to 16 values (0-255) as a python `list` of `int`s
 * `FORMAT_STRING` - string of up to 16 chars as a python `str` 

If a function fails all 3 times, `Arduino.serial.status` will be set to `MASTER_STATUS_SENDING_COMMAND_FAILED (4)` and the call will return `-1`.

## More information

This project was originally developed for communicating with an Arduino Mega from a Raspberry Pi.  We used RS-485, with some voltage steppers and other custom circuits.  For a more detailed description of that, see the beginning of [this page](https://github.com/n-wach/Portfolio/wiki/Recap).

