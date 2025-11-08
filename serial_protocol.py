import struct
from enum import Enum, IntEnum

class SerialMessage:
    """
    Base class for creating and parsing serial messages with a simple protocol.

    The protocol is a fixed-size struct with a start marker (0xCAFE), a message code,
    the payload, and an end marker (0xFACE).
    """
    format: str | None = None
    code: int | None = None

    def _get_payload_data(self) -> tuple:
        """
        Returns a tuple of the data to be packed into the message payload.
        This method should be overridden by subclasses.
        """
        return tuple()

    @classmethod
    def parse(cls, buffer: bytes) -> "SerialMessage":
        """
        Parses a byte buffer into a message object.

        Args:
            buffer: The raw byte buffer to parse.

        Returns:
            An instance of the message class.

        Raises:
            ValueError: If the buffer is malformed (e.g., incorrect markers or code).
            NotImplementedError: If the subclass does not define a 'format'.
        """
        if cls.format is not None:
            try:
                fields = struct.unpack(cls.format, buffer)
                # Check for start marker, message code, and end marker
                if fields[0] != 0xCAFE or fields[-1] != 0xFACE or fields[1] != cls.code:
                    raise ValueError('Bad buffer: Invalid markers or message code.')
                # Return a new instance with the payload fields
                return cls(*fields[2:-1])
            except struct.error as e:
                raise ValueError(f'Bad buffer: Struct unpacking failed. {e}')
        raise NotImplementedError("Message format is not defined.")


    def serialize(self) -> bytes:
        """
        Serializes the message object into a byte buffer.

        Returns:
            The packed byte buffer ready for transmission.

        Raises:
            ValueError: If the message format or code is not defined.
        """
        if not (self.format and self.code is not None):
            raise ValueError('Cannot serialize message: format or code is not defined.')

        return struct.pack(
            self.format,
            0xCAFE,
            self.code,
            *self._get_payload_data(),
            0xFACE
        )

class TargetMessage(SerialMessage):
    """
    Message to transmit target information.

    Attributes:
        target_id (int): The unique ID of the person being targeted.
        is_valid (bool): Whether the target is confirmed and valid for engagement.
        x (int): The x-coordinate of the target in millimeters.
        y (int): The y-coordinate of the target in millimeters.
        z (int): The z-coordinate of the target in millimeters.
    """
    format = '<HBI?HHHH'  # <H=start, B=code, I=id, ?=valid, H=x,H=y,H=z, H=end>
    code = 0

    def __init__(self, target_id: int, is_valid: bool, x: int, y: int, z: int):
        self.target_id = target_id
        self.is_valid = is_valid
        self.x = x
        self.y = y
        self.z = z

    def _get_payload_data(self) -> tuple:
        """Returns the payload data for the TargetMessage."""
        return (self.target_id, self.is_valid, self.x, self.y, self.z)

class TargetSource(IntEnum):
    STATIC = 0
    RADAR = 1
    CV = 2

class SetTargetSourceMessage(SerialMessage):
    """
    Message to set the active target source.
    """
    format = '<HBBH'  # <H=start, B=code, B=source, H=end>
    code = 2

    def __init__(self, source: TargetSource):
        self.source = source

    def _get_payload_data(self) -> tuple:
        """Returns the payload data for the SetTargetSourceMessage."""
        return (self.source,)

class StaticTargetMessage(SerialMessage):
    """
    Message to set a static target position.
    """
    format = '<HBHHH H' # <H=start, B=code, H=x, H=y, H=z, H=end>
    code = 3

    def __init__(self, x: int, y: int, z: int):
        self.x = x
        self.y = y
        self.z = z

    def _get_payload_data(self) -> tuple:
        """Returns the payload data for the StaticTargetMessage."""
        return (self.x, self.y, self.z)

class ParseState(Enum):
    WAITING_FOR_HEADER = 1
    READING_PAYLOAD = 2

class MessageParser:
    """
    A state machine for parsing binary messages from a stream.

    This class reads bytes from a stream-like object and searches for message
    frames delimited by a header (0xCAFE) and a footer (0xFACE). When a
    complete frame is found, it uses a factory to construct a message object.
    """
    def __init__(self, message_factory):
        self.buffer = bytearray()
        self.state = ParseState.WAITING_FOR_HEADER
        self.message_factory = message_factory

    def parse(self, data):
        """
        Processes incoming data from the stream and yields any complete messages.

        Args:
            data (bytes): A chunk of data read from the serial stream.

        Yields:
            SerialMessage: A message object if a complete frame is parsed.
        """
        self.buffer.extend(data)
        while len(self.buffer) > 0:
            if self.state == ParseState.WAITING_FOR_HEADER:
                header_index = self.buffer.find(b'\xfe\xca')
                if header_index != -1:
                    self.buffer = self.buffer[header_index:]
                    self.state = ParseState.READING_PAYLOAD
                else:
                    if len(self.buffer) > 1:
                        self.buffer = self.buffer[-1:]
                    break

            elif self.state == ParseState.READING_PAYLOAD:
                if len(self.buffer) < 3:
                    break

                type_code = self.buffer[2]
                if type_code in self.message_factory:
                    msg_class, msg_size = self.message_factory[type_code]
                    if len(self.buffer) >= msg_size:
                        message_data = self.buffer[:msg_size]
                        try:
                            parsed_msg = msg_class.parse(message_data)
                            yield parsed_msg
                            self.buffer = self.buffer[msg_size:]
                        except ValueError:
                            self.buffer = self.buffer[2:]
                        finally:
                            self.state = ParseState.WAITING_FOR_HEADER
                    else:
                        break
                else:
                    self.buffer = self.buffer[2:]
                    self.state = ParseState.WAITING_FOR_HEADER

MESSAGE_CLASSES = [TargetMessage, SetTargetSourceMessage, StaticTargetMessage]
MESSAGE_FACTORY = {msg.code: (msg, struct.calcsize(msg.format)) for msg in MESSAGE_CLASSES if msg.code is not None}
