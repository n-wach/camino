from camino import Arduino, SerialConnection
import time

serial = SerialConnection("/dev/ttyACM0", 115200)
arduino = Arduino(serial)

assert(arduino.echo("hi", out=str) == "hi")
assert(arduino.echo(1, 2, 3) == bytes([1, 2, 3]))

assert(arduino.add(1, 2, 3, out=int) == 6)
assert(arduino.add([1, 2, 3], out=int) == 6)

assert(arduino.say_hi(out=str) == "I can say hi!")

assert(arduino.return_byte(out=int, signed=False) == 229)
assert(arduino.return_char(out=int) == -127)

assert(arduino.return_ushort(out=int, signed=False) == 6000)
assert(arduino.return_short(out=int) == -1000)

assert(arduino.return_uint(out=int, signed=False) == 6000)
assert(arduino.return_int(out=int) == -1000)

assert(arduino.return_ulong(out=int, signed=False) == 4294967295)
assert(arduino.return_long(out=int) == -2147483648)

arduino.pin_mode(13, 1)
for i in range(1000):
    arduino.digital_write(13, 0)
    arduino.digital_write(13, 1)
