import unittest
import struct
from serial_protocol import (
    TargetMessage,
    SetTargetSourceMessage,
    StaticTargetMessage,
    TargetSource,
    MessageParser,
    MESSAGE_FACTORY
)

class TestSerialization(unittest.TestCase):

    def test_target_message_serialization(self):
        msg = TargetMessage(target_id=123, is_valid=True, x=1024, y=2048, z=3072)
        packed = msg.serialize()
        # <HBI?HHHH
        # 0xCAFE, 0, 123, True, 1024, 2048, 3072, 0xFACE
        expected = struct.pack('<HBI?HHHH', 0xCAFE, 0, 123, True, 1024, 2048, 3072, 0xFACE)
        self.assertEqual(packed, expected)

    def test_set_target_source_message_serialization(self):
        msg = SetTargetSourceMessage(source=TargetSource.RADAR)
        packed = msg.serialize()
        # <HBBH
        # 0xCAFE, 2, 1, 0xFACE
        expected = struct.pack('<HBBH', 0xCAFE, 2, TargetSource.RADAR, 0xFACE)
        self.assertEqual(packed, expected)

    def test_static_target_message_serialization(self):
        msg = StaticTargetMessage(x=1, y=2, z=3)
        packed = msg.serialize()
        # <HBHHH H
        # 0xCAFE, 3, 1, 2, 3, 0xFACE
        expected = struct.pack('<HBHHHH', 0xCAFE, 3, 1, 2, 3, 0xFACE)
        self.assertEqual(packed, expected)


class TestMessageParser(unittest.TestCase):

    def setUp(self):
        self.parser = MessageParser(MESSAGE_FACTORY)

    def test_parse_single_target_message(self):
        original_message = TargetMessage(target_id=123, is_valid=True, x=10, y=20, z=30)
        binary_data = original_message.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 1)
        parsed_message = parsed_messages[0]

        self.assertIsInstance(parsed_message, TargetMessage)
        self.assertEqual(parsed_message.target_id, original_message.target_id)
        self.assertEqual(parsed_message.is_valid, original_message.is_valid)
        self.assertEqual(parsed_message.x, original_message.x)
        self.assertEqual(parsed_message.y, original_message.y)
        self.assertEqual(parsed_message.z, original_message.z)

    def test_parse_multiple_messages(self):
        msg1 = TargetMessage(target_id=1, is_valid=True, x=10, y=20, z=30)
        msg2 = SetTargetSourceMessage(source=TargetSource.CV)
        binary_data = msg1.serialize() + msg2.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 2)
        self.assertIsInstance(parsed_messages[0], TargetMessage)
        self.assertEqual(parsed_messages[0].target_id, 1)
        self.assertIsInstance(parsed_messages[1], SetTargetSourceMessage)
        self.assertEqual(parsed_messages[1].source, TargetSource.CV)

    def test_parse_with_leading_junk_data(self):
        junk = b'\x01\x02\x03\x04'
        msg = TargetMessage(target_id=123, is_valid=False, x=1, y=2, z=3)
        binary_data = junk + msg.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 1)
        self.assertEqual(parsed_messages[0].target_id, 123)

    def test_parse_with_junk_data_between_messages(self):
        msg1 = TargetMessage(target_id=1, is_valid=True, x=10, y=20, z=30)
        junk = b'\xde\xad\xbe\xef'
        msg2 = TargetMessage(target_id=2, is_valid=False, x=40, y=50, z=60)
        binary_data = msg1.serialize() + junk + msg2.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 2)
        self.assertEqual(parsed_messages[0].target_id, 1)
        self.assertEqual(parsed_messages[1].target_id, 2)

    def test_incremental_parsing(self):
        msg = TargetMessage(target_id=123, is_valid=True, x=10, y=20, z=30)
        binary_data = msg.serialize()

        # Feed data byte by byte
        parsed_messages = []
        for i in range(len(binary_data)):
            parsed_messages.extend(list(self.parser.parse(binary_data[i:i+1])))

        self.assertEqual(len(parsed_messages), 1)
        self.assertEqual(parsed_messages[0].target_id, 123)

    def test_corrupted_footer(self):
        msg_ok1 = TargetMessage(target_id=1, is_valid=True, x=1, y=1, z=1)
        msg_bad_bytes = TargetMessage(target_id=2, is_valid=True, x=2, y=2, z=2).serialize()
        # Corrupt the footer
        msg_bad = msg_bad_bytes[:-2] + b'\x00\x00'
        msg_ok2 = TargetMessage(target_id=3, is_valid=True, x=3, y=3, z=3)

        binary_data = msg_ok1.serialize() + msg_bad + msg_ok2.serialize()
        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 2)
        self.assertEqual(parsed_messages[0].target_id, 1)
        self.assertEqual(parsed_messages[1].target_id, 3)

    def test_unknown_type_code(self):
        # Header, Type Code = 99 (unknown), Footer
        bad_type_packet = struct.pack('<HBH', 0xCAFE, 99, 0xFACE)
        msg_ok = TargetMessage(target_id=123, is_valid=True, x=1, y=2, z=3)
        binary_data = bad_type_packet + msg_ok.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 1)
        self.assertEqual(parsed_messages[0].target_id, 123)

if __name__ == '__main__':
    unittest.main()
