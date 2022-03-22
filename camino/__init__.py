from serial import Serial
import time

MAX_DATA_LENGTH = 255
SEND_ATTEMPTS = 3
COMMAND_TIMEOUT_PERIOD_S = 1

COMMAND_HEADER_BYTE_1 = 0xAA
COMMAND_HEADER_BYTE_2 = 0x55

RESPONSE_HEADER_WITH_NO_DATA = 0xA9
RESPONSE_HEADER_WITH_DATA = 0xAC
RESPONSE_HEADER_RESEND_REQUEST = 0xB8


class CaminoException(Exception):
    pass


class CaminoResendException(CaminoException):
    pass


class SerialConnection:
    def __init__(self, port="/dev/ttyS0", baud=115200):
        self.port = Serial(port=port, baudrate=baud, timeout=COMMAND_TIMEOUT_PERIOD_S)
        self.port.set_input_flow_control(True)

    def read_byte(self):
        b = self.port.read(1)
        if len(b) != 1:
            raise CaminoException(f"Nothing sent when a response was expected.")
        return b[0]

    def read_packet(self):
        header_byte_1 = self.read_byte()
        header_byte_2 = self.read_byte()
        if header_byte_1 != header_byte_2:
            raise CaminoException(
                f"Mismatched header bytes: {header_byte_1} vs {header_byte_2}"
            )

        if header_byte_1 == RESPONSE_HEADER_WITH_NO_DATA:
            return None
        elif header_byte_1 == RESPONSE_HEADER_WITH_DATA:
            # slave is going to send some data with the response code
            data_length = self.read_byte()
            checksum = data_length
            data = self.port.read(data_length)
            for b in data:
                checksum += b
            checksum = checksum % 256
            received_checksum = self.read_byte()
            if received_checksum != checksum:
                raise CaminoException(
                    f"Invalid checksum: {checksum} vs {received_checksum}"
                )
            return data
        elif header_byte_1 == RESPONSE_HEADER_RESEND_REQUEST:
            raise CaminoResendException()
        else:
            raise CaminoException(f"Unexpected header value: {header_byte_1}")

    def send_command(self, address, command, data):
        if len(data) > MAX_DATA_LENGTH:
            raise CaminoException(
                f"Data length ({len(data)}) larger than max ({MAX_DATA_LENGTH})"
            )

        packet = [COMMAND_HEADER_BYTE_1, COMMAND_HEADER_BYTE_2]

        checksum = 0

        packet.append(address)
        checksum += address

        packet.append(command)
        checksum += command

        data_length = len(data)
        packet.append(data_length)
        checksum += data_length

        for byte in data:
            packet.append(byte)
            checksum += byte

        packet.append(checksum % 256)

        last_exception = None
        for attempt_number in range(SEND_ATTEMPTS):
            self.port.flushInput()
            self.port.write(bytes(packet))
            self.port.flushOutput()
            time.sleep(0.05)
            try:
                response = self.read_packet()
                return response
            except CaminoException as e:
                last_exception = e
                print(f"Got error on attempt {attempt_number + 1}/{SEND_ATTEMPTS}: {e}")
                # flushing
                self.port.flush()
        raise CaminoException(
            f"All {SEND_ATTEMPTS} attempts to communicate with device failed."
        ) from last_exception


class Callable:
    def __init__(self, arduino, command, name=None):
        self.command = command
        self.arduino = arduino
        if name is None:
            self.name = self.arduino.get_nth_call(command, out=str)
        else:
            self.name = name

    def call(self, *args, out=bytes):
        """
        format_out: int, str, bytes, None
        """
        serial = self.arduino.serial
        data = []
        for arg in args:
            if isinstance(arg, int):
                data.append(arg)
            elif isinstance(arg, str):
                if len(args) > 1:
                    raise CaminoException(f"str must be only argument.")
                data.extend([ord(c) for c in arg])
            elif isinstance(arg, list):
                if len(args) > 1:
                    raise CaminoException(f"list must be only argument.")
                data = list(arg)
            else:
                raise CaminoException(f"Unknown arg type: {type(arg)}")

        response = serial.send_command(self.arduino.address, self.command, data)

        if response is None:
            return None

        if out == int:
            return int.from_bytes(response, "little")
        elif out == str:
            return "".join(chr(val) for val in response)
        elif out == bytes:
            return response

        raise CaminoException(f"Unknown output format: {out}")


class Arduino:
    def __init__(self, serial, address=0):
        self.serial = serial
        self.address = address
        self.callables = {}
        self._add_callable(Callable(self, 0, "num_calls"))
        self._add_callable(Callable(self, 1, "get_nth_call"))
        self._fetch_callables()

    def _add_callable(self, _callable):
        self.callables[_callable.name] = _callable
        setattr(self, _callable.name, _callable.call)
        print("Callable added: {}".format(_callable.name))

    def _fetch_callables(self):
        callable_count = self.num_calls(out=int)
        print("There are", callable_count, "callables")
        for i in range(len(self.callables), callable_count):
            self._add_callable(Callable(self, i))
