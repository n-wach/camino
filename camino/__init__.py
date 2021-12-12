from serial import Serial

M_MASTER_COMMAND_MAX_DATA_BYTES = 64
M_SLAVE_RESPONSE_MAX_DATA_BYTES = 64

MASTER_COMMAND_HEADER_BYTE_1 = 0xAA
MASTER_COMMAND_HEADER_BYTE_2 = 0x55
MASTER_COMMAND_TIMEOUT_PERIOD_S = 0.1
MASTER_COMMAND_MAX_PACKET_BYTES = M_MASTER_COMMAND_MAX_DATA_BYTES + 5

SLAVE_RESPONSE_RECIEVED_COMMAND = 0xA9
SLAVE_RESPONSE_RECIEVED_COMMAND_SENDING_DATA = 0xAC
SLAVE_RESPONSE_RESEND_COMMAND = 0xB8
SLAVE_RESPONSE_MAX_PACKET_BYTES = M_SLAVE_RESPONSE_MAX_DATA_BYTES + 4

MASTER_STATUS_READY_TO_SEND_COMMAND = 1
MASTER_STATUS_BUSY_SENDING_COMMAND = 2
MASTER_STATUS_SENDING_COMMAND_SUCCEEDED = 3
MASTER_STATUS_SENDING_COMMAND_FAILED = 4

READ_FAILURE = 0
READ_SUCCESS_NO_DATA = 1
READ_SUCCESS_DATA = 2

SEND_ATTEMPTS = 3

"""
From slave:
if no response:
SLAVE_RESPONSE_RECEIVED_COMMAND
SLAVE_RESPONSE_RECEIVED_COMMAND

if resend:
SLAVE_RESPONSE_RESEND_COMMAND
SLAVE_RESPONSE_RESEND_COMMAND

if response:
SLAVE_RESPONSE_RECEIVED_COMMAND_SENDING_DATA
SLAVE_RESPONSE_RECEIVED_COMMAND_SENDING_DATA
dataLength
data where len(data)is dataLength
checksum

"""


class SerialConnection:
    def __init__(self, port="/dev/ttyS0", baud=115200):
        self.port = Serial(port=port, baudrate=baud, timeout=MASTER_COMMAND_TIMEOUT_PERIOD_S)
        self.port.set_input_flow_control(True)
        self.data_from_slave = []
        self.data_length_from_slave = 0
        self.checksum_from_slave = 0
        self.status = MASTER_STATUS_READY_TO_SEND_COMMAND

    def read_byte(self):
        b = self.port.read()
        # print("read_byte", b)
        return int.from_bytes(b, "big")

    # called when we expect a packet
    def read_packet(self):
        response_type_first = self.read_byte()
        # make sure 2nd byte is the same
        response_type_repeat = self.read_byte()
        if response_type_repeat != response_type_first:
            # uh oh. failure!
            # print("non-matching response codes", response_type_first, response_type_repeat)
            self.read_byte()
            #return READ_FAILURE

        if response_type_repeat == SLAVE_RESPONSE_RECIEVED_COMMAND:
            self.data_length_from_slave = 0
            # done
            # print("read success")
            return READ_SUCCESS_NO_DATA
        elif response_type_repeat == SLAVE_RESPONSE_RECIEVED_COMMAND_SENDING_DATA:
            # slave is going to send some data with the response code
            data_length = self.read_byte()
            if data_length >= 1 and data_length <= M_SLAVE_RESPONSE_MAX_DATA_BYTES:
                self.data_length_from_slave = data_length
                self.checksum = data_length
                self.data_from_slave = []
                while len(self.data_from_slave) < self.data_length_from_slave:
                    next_byte = self.read_byte()
                    self.checksum += next_byte
                    self.data_from_slave.append(next_byte)
                # check checksum
                checksum = self.read_byte()
                if self.checksum % 256 != checksum:
                    # problem
                    print("invalid checksum: {} vs {}".format(self.checksum % 256, checksum))
                    return READ_FAILURE
                else:
                    # print("read success w/ data")
                    return READ_SUCCESS_DATA
            else:
                print("invalid data length:", data_length)
                return READ_FAILURE

        elif response_type_repeat == SLAVE_RESPONSE_RESEND_COMMAND:
            print("resend")
            return READ_FAILURE

        else:
            print("unexpected response type:", response_type_first)
            return READ_FAILURE

    def send_command_to_slave(self, slave_address, command, command_data, response):
        if self.status == MASTER_STATUS_BUSY_SENDING_COMMAND:
            raise RuntimeError("Cannot send command: BUSY")
        self.status = MASTER_STATUS_BUSY_SENDING_COMMAND

        if len(command_data) > M_MASTER_COMMAND_MAX_DATA_BYTES:
            raise ValueError(
                "Data length ({}) cannot be greater than {}".format(len(command_data), M_MASTER_COMMAND_MAX_DATA_BYTES))

        packet = []
        self.packet_to_slave = packet
        data_length = len(command_data)
        checksum = 0
        packet.append(MASTER_COMMAND_HEADER_BYTE_1)
        packet.append(MASTER_COMMAND_HEADER_BYTE_2)
        packet.append(slave_address)
        checksum += slave_address
        packet.append(command)
        checksum += command
        packet.append(data_length)
        checksum += data_length

        for byte in command_data:
            packet.append(byte)
            checksum += byte

        packet.append(checksum % 256)

        for attempt_number in range(SEND_ATTEMPTS):
            self.port.write(bytes(self.packet_to_slave))
            # print("writing: " + str(self.packet_to_slave))
            status = self.read_packet()
            if status == READ_SUCCESS_DATA:
                self.status = MASTER_STATUS_SENDING_COMMAND_SUCCEEDED
                return self.data_from_slave
            elif status == READ_SUCCESS_NO_DATA:
                self.status = MASTER_STATUS_SENDING_COMMAND_SUCCEEDED
                return 1
            else:
                # prepare for next by clearing what's waiting on the line
                # WARNING: this will pause until read timeout
                print("read attempt", attempt_number, "failed")
                self.port.read(size=M_SLAVE_RESPONSE_MAX_DATA_BYTES)

        # after SEND_ATTEMPTS attempts
        self.status = MASTER_STATUS_SENDING_COMMAND_FAILED
        return -1


class Callable:
    def __init__(self, arduino, command, name=None):
        self.command = command
        self.arduino = arduino
        if name is None:
            self.name = self.arduino.get_nth_call(command, format_out=str)
        else:
            self.name = name

    def call(self, data=None, format_out=int):
        """
        format_out: int, str, bytes, None
        """
        serial = self.arduino.serial
        to_send = data
        if isinstance(data, int):
            to_send = [data]
        elif isinstance(data, str):
            to_send = [ord(c) for c in data]
        elif data is None:
            to_send = []

        # call
        response = False if format_out is None else True
        out = serial.send_command_to_slave(self.arduino.address, self.command, to_send, response)

        if isinstance(out, int):
            return None

        if format_out == int:
            return out[0]
        elif format_out == str:
            return "".join(chr(val) for val in out)
        else:
            return out


class Arduino:
    def __init__(self, serial, address):
        self.serial = serial
        self.address = address
        self.callables = {}
        self.add_callable(Callable(self, 0, "num_calls"))
        self.add_callable(Callable(self, 1, "get_nth_call"))
        self.fetch_callables()
        # test
        print("Arduino at {}...".format(address))
        print(self.echo("Ready!", format_out=str))

    def add_callable(self, callable):
        self.callables[callable.name] = callable
        setattr(self, callable.name, callable.call)
        print("Callable added: {}".format(callable.name))

    def fetch_callables(self):
        callable_count = self.num_calls()
        print("There are", callable_count, "callables")
        for i in range(len(self.callables), callable_count):
            self.add_callable(Callable(self, i))

