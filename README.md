# Camino

Camino is a library allowing communication over a serial connection between Python 3 
and up to 256 Arduino Megas. It provides a simple way to call functions on a remote
Arduino, sending and receiving up to 255 bytes of data. Besides the built in functions 
(`digital_write`, `pin_mode`, `analog_read`, etc.), approximately 250 user-defined 
functions can be defined.


## Table of Contents

- [Installation](#Installation)
- [Example](#Example)
- [Adding Custom Functions](#Adding-Custom-Functions)
- [Features and Limitations](#Features-and-Limitations)
- [Options](#Options)
- [Protocol](#Protocol)
- [Contributing](#Contributing)

## Installation

### Python

To install the latest Python release of `camino`, simply run `pip install camino`. The
library requires `pyserial`. This should be installed automatically, but additional
configuration may be required for some setups. See the 
[pyserial docs](https://pyserial.readthedocs.io/en/latest/pyserial.html#requirements)
for details.

To install the latest development version (for testing or contributing), clone this 
repository and run `pip install -e .` from inside the root directory.

### Arduino

Camino is distributed as part of the Arduino Library Manager. The version is shipped
with the default options, and should work for most users. See the 
[Features and Options section](#Options).

Using the Arduino IDE, it is easy to install the library with default options:

To change any options, you'll need to manually include the library files. Download
this repository. Then copy `src/Camino.cpp` and `src/Camino.h` into your sketch.
If using the Arduino IDE, you can use the "Add File" option. To avoid conflicts,
make sure you uninstall the library if you installed it via the IDE.

Installing manually also means you won't have the example sketches mentioned below.
It's recommended to use the defaults options (and install normally) to get familiar 
with the library before changing them.

## Example

Once the Python and Arduino libraries are installed, you should be able to run the
examples.

### Arduino

Install `examples/CaminoExample` to your Arduino. It is copied below:

```c++
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
```

When uploading the sketch, take note of which serial port the Arduino is connected 
to. Be sure to keep it plugged in.

### Python

Open a Python shell.

```python3
>>> import camino
```

Create a `SerialConnection` for the Arduino's port. On Linux these look like 
`/dev/ttyS0`, `/dev/ttyACM0`, etc. On Windows they look like `COM1`, `COM2`, etc.
Make sure the baud rate matches what is in the sketch.

```python3
>>> connection = camino.SerialConnection(port='/dev/ttyACM0', baud=9600)
```

Connect to an Arduino.

```python3
>>> arduino = camino.Arduino(connection)
```

This will begin communicating with the Arduino to discover which functions are
available. This should take less than a second. Now you can use the built in 
methods:

```python3
>>> arduino.pin_mode(12, 0) # set pin 12 to INPUT
>>> arduino.pin_mode(16, 1) # set pin 16 to OUTPUT
>>> value = arduino.digital_read(12, out=int)
>>> arduino.digital_write(16, value)
```

As well as the custom functions defined:
```python3
>>> print(arduino.say_hi(out=str))
I can say hi!
>>> print(arduino.add(1, 2, 3, out=int))
6
```

## Adding Custom Functions

The following functions are defined for all Arduinos:

```
pin_mode
digital_write
digital_read
analog_read
analog_write
```

As demonstrated in the example, if you want to do something else, you can 
define functions specific to your application.

A *callable* is a function that can be called by Camino from Python.

In order to be callable, a function must have return type `void` and accept
exactly `byte dataLength, byte *data` as arguments.

Then, you must include the callable in an entry of the `CALLABLES` section.
In this entry, you specify the Python name for the callable, and a
reference to the function. The Python name should be no longer than 
`MAX_DATA_LENGTH - 1` characters, because it is sent in a packet.

Assume you have a single callable called `triggerPiston`, and want to be able
to call it from python as `arduino.trigger_piston()`. At the bottom of your 
Arduino code, you'll write the following:

```c
BEGIN_CALLABLES {
  {"trigger_piston", triggerPiston},
} END_CALLABLES;
```

### Returning Data

To allow all callables to have the same type, data is returned using a number
of helper functions, rather than `return` statements.

To return data to Python, use one of the available `returns(...)` methods. 
At the moment, there are 3 possible arguments for `returns(...)`:

```C
void returns(const char* string); // returns("hello");
void returns(byte v); // returns(1);
void returns(byte dataLength, data *dataArray); // returns(dataLength, data); as in echo
```

You do not need to call `returns(...)`.  If a function does not call `returns(...)`,
the Arduino simply responds with no data (`None` in Python).  Calling `returns(...)` 
multiple times overwrites the return value.

### Limitations to Callable Functions

Callables have limitations. 

They are executed within an interrupt. This means they should be fast, and
they can't use `delay`, `millis`, `Serial`, or other interrupt-dependent
functions. For a good overview of the limitations to interrupts on Arduino,
see [Nick Gammon's notes](http://gammon.com.au/interrupts). If you want to use 
interrupt-dependent functions or expect the call to take a long time, you can
have code running in `loop` dependent on some global variables. You can, of course,
change global variables from inside a Callable.

When a problem occurs with a transmission, a command packet may be resent up
to 3 times. Therefore, you should design your callables to be 
[idempotent](https://en.wikipedia.org/wiki/Idempotence#Computer_science_meaning). 
Repeat calls with the same arguments should have the same effect as a single call.
For instance, instead of creating a `toggle_led` callable, you should create 
`set_led` and keep track of the state in Python. Of course, if you are getting
any failed function calls, something is probably amiss with your serial connection.

### Using Custom Functions from Python

Callables act as a link between an Arduino function and the corresponding Python
function. The Python names are sent from the Arduino when they establish a connection, 
and automatically added to the `Arduino` python object.

You can call these functions as you would call any other function. Note that Camino
is *not* thread safe--only call one function at a time.

Callables automatically convert input arguments to a `bytes` object. For instance,
a single `str` is converted using `ord` on each character, or a bunch of `int`s are
appended into a `bytes` object. If the automatic conversion doesn't work as expected, 
you can pass a `bytes` object directly.

If the callable returns no data, Python returns `None`. Otherwise, the return type 
defaults to `bytes`. You can have Camino automatically convert the `bytes` to 
another type by passing the `out` keyword argument. The options are:

 * `bytes` - raw `bytes` returned by the Callable
 * `int` - convert the bytes to a single Python `int` using little endian.
 * `str` - convert the bytes to a Python `str`

## Features and Limitations

One Python host can control up to 256 individually addressable Arduinos. Pass
the desired address of the Arduino as the second parameter to `camino.open`. In
Python, pass the address as the second argument to the `Arduino` constructor.

Each packet may contain between 0 and 255 bytes of data. It is a known issue
that packets with more than around 100 bytes of data are unreliable in some 
setups.

Camino operates entirely through interrupts, so does not require any code in 
`loop`. For instance, you can easily collect data from sensors in `loop` (using
delays, serial writes, etc.), and still be able to quickly report aggregate data 
when polled using a callable.

Lastly, Camino is incredibly fast, with low overhead. Even at low baud-rates, it is
possible to PWM digital pins using only `digital_write` from within Python.

## Options

A few options are listed in `Camino.h` for more advanced use cases.

If you want to use values other than the defaults, you'll need to manually download
and include the library in your sketch, and change them in `Camino.h`.

### UART Port: `PORT`

The UART port used by Camino. Can be `-1`, `0`, `1`, `2`. We default to UART0, which is 
usually the USB port. This means you can't use Serial in your sketch. For
other boards, like the Arduino Mega, you could have Camino listen on UART2 
(`Serial2`) and use the normal Serial port in parallel.

### Maximum Data Length in Packets: `MAX_DATA_LENGTH`

The maximum length of the data section for packets (to and from the Arduino). 
Value must be at least 16 and at most 255. Making it smaller reduces the
memory footprint of Camino.

### Command Timeout: `COMMAND_TIMEOUT_MS`.

The command timeout in milliseconds. When a new byte in a packet is received,
if this period has elapsed since the start of the packet, the earlier
bytes are discarded and Camino assumes that a new packet is beginning.

Defaults to 100 milliseconds. With a well-behaved connection, this should 
never matter.

### Transmission Hooks

There are three possible macros that act as hooks into the lifecycle of a 
packet transmission:

- `initTransmissions()` is called in begin()
- `beginTransmission()`is called before the first byte is sent in a response packet.
- `endTransmission()` is called after the last byte is sent in a response packet.

Sometimes you may need to do some preparation before a packet can be sent. 
For instance, you may have a pin that enables a pull down resistor to remove 
noise from your transmission line when not sending anything. Or when debugging, it
may be useful to shine an LED when packets are being sent.

## Protocol

The protocol refers to the host Python machine as the Master, and the Arduino
as the Slave.

The protocol follows a simple rule that no slave may transmit unless it is in
response to a command from the master, and only one response per command is 
allowed.

The master's command packet always takes the same format. It specifies which
Arduino is being addressed, which command should be run, and what data to
pass to the command.

The slave can respond in one of three ways:

- Response with data, for instance with `digital_read`
- Response with no data, for instance with `digital_write`
- Resend request, if there is a problem understanding the command.

### Single Command Communication Flow

- Python sends a command packet containing an address, command, and data.
- All Arduinos connected to the serial port read the entire packet.
- If the checksum is invalid, Arduino transmits a Resend Request.
- Otherwise, check if the address matches the Arduino. If it doesn't,
ignore the command.
- Otherwise, if the checksum is valid, and the address matches the
Arduino, run the specified command with the given data. Then,
respond with either data or no data.

Python will attempt to send a command packet up to 3 times before giving up
and raising a `CaminoError`.

### Packet Structure

#### Command packet

|1 byte|1 byte|1 byte       | 1 byte | 1 byte     |`data length` bytes| 1 byte  |
|------|------|-------------|--------|------------|-------------------|---------|
|0xAA  |0x55  |address|command |data length |data               | checksum|

Checksum is the sum of address, command, data length, and all bytes of data, mod 256.

#### Response with data

|1 byte|1 byte|1 byte     |`data length` bytes| 1 byte    |
|------|------|-----------|-------------------|-----------|
| 0xAC | 0xAC |data length|data               | checksum* |

Checksum is the sum of data length and all bytes of data, mod 256.

#### Response with no data

|1 byte|1 byte|
|------|------|
|0xA9  |0xA9  |

#### Resend request

|1 byte|1 byte|
|------|------|
|0xB8  |0xB8  |

# Contributing

Please visit the [Issues Page](https://github.com/n-wach/camino/issues) for a list of 
known issues. Feel free to add issues, or contribute comments to issues that affect
you.

Contributions in the form of PRs are also welcome, either adding new features or 
solving existing issues.

This project was originally developed in the 
[Dos Pueblos Engineering Academy](https://www.dpengineering.org/about.html). Specifically,
it was used to communicate between an Arduino Mega and a Raspberry Pi in the Air Guitar
project. We used RS-485, with some voltage steppers and other custom circuits. For a more 
detailed description of that, see
[this page](https://github.com/n-wach/Portfolio/wiki/Recap).

