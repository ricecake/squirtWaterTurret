import unittest
import struct
from cvTargetAssist import (
    TargetMessage,
    ConfigMessage,
    SetTargetSourceMessage,
    StaticTargetMessage,
    TargetSource,
    MessageParser,
    SerialMessage
)

class TestSerialization(unittest.TestCase):

    def test_target_message_serialization(self):
        msg = TargetMessage(id=123, valid=True, x=1024, y=2048, z=3072)
        packed = msg.serialize()
        # <H B I ? H H H H
        # 0xCAFE, 0, 123, True, 1024, 2048, 3072, 0xFACE
        expected = struct.pack('<HBI?HHH', 0xCAFE, 0, 123, True, 1024, 2048, 3072) + b'\xce\xfa'
        self.assertEqual(packed, expected)

    def test_config_message_serialization(self):
        msg = ConfigMessage(projectile_speed=15.5, turret_height=1.2, max_speed=1000, acceleration=500)
        packed = msg.serialize()
        # <H B f f H H H
        # 0xCAFE, 1, 15.5, 1.2, 1000, 500, 0xFACE
        expected = struct.pack('<HBffHH', 0xCAFE, 1, 15.5, 1.2, 1000, 500) + b'\xce\xfa'
        self.assertEqual(packed, expected)

    def test_set_target_source_message_serialization(self):
        msg = SetTargetSourceMessage(source=TargetSource.RADAR)
        packed = msg.serialize()
        # <H B B H
        # 0xCAFE, 2, 1, 0xFACE
        expected = struct.pack('<HBB', 0xCAFE, 2, TargetSource.RADAR) + b'\xce\xfa'
        self.assertEqual(packed, expected)

    def test_static_target_message_serialization(self):
        msg = StaticTargetMessage(x=1, y=2, z=3)
        packed = msg.serialize()
        # <H B H H H H
        # 0xCAFE, 3, 1, 2, 3, 0xFACE
        expected = struct.pack('<HBHHH', 0xCAFE, 3, 1, 2, 3) + b'\xce\xfa'
        self.assertEqual(packed, expected)


class TestMessageParser(unittest.TestCase):

    def setUp(self):
        self.parser = MessageParser(SerialMessage.message_factory)

    def test_parse_single_target_message(self):
        original_message = TargetMessage(id=123, valid=True, x=10, y=20, z=30)
        binary_data = original_message.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 1)
        parsed_message = parsed_messages[0]

        self.assertIsInstance(parsed_message, TargetMessage)
        self.assertEqual(parsed_message.id, original_message.id)
        self.assertEqual(parsed_message.valid, original_message.valid)
        self.assertEqual(parsed_message.x, original_message.x)
        self.assertEqual(parsed_message.y, original_message.y)
        self.assertEqual(parsed_message.z, original_message.z)

    def test_parse_multiple_messages(self):
        msg1 = TargetMessage(id=1, valid=True, x=10, y=20, z=30)
        msg2 = ConfigMessage(projectile_speed=10.0, turret_height=1.0, max_speed=100, acceleration=50)
        msg3 = SetTargetSourceMessage(source=TargetSource.CV)
        binary_data = msg1.serialize() + msg2.serialize() + msg3.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 3)
        self.assertIsInstance(parsed_messages[0], TargetMessage)
        self.assertEqual(parsed_messages[0].id, 1)
        self.assertIsInstance(parsed_messages[1], ConfigMessage)
        self.assertAlmostEqual(parsed_messages[1].projectile_speed, 10.0)
        self.assertIsInstance(parsed_messages[2], SetTargetSourceMessage)
        self.assertEqual(parsed_messages[2].source, TargetSource.CV)

    def test_parse_with_leading_junk_data(self):
        junk = b'\x01\x02\x03\x04'
        msg = TargetMessage(id=123, valid=False, x=1, y=2, z=3)
        binary_data = junk + msg.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 1)
        self.assertEqual(parsed_messages[0].id, 123)

    def test_parse_with_junk_data_between_messages(self):
        msg1 = TargetMessage(id=1, valid=True, x=10, y=20, z=30)
        junk = b'\xde\xad\xbe\xef'
        msg2 = TargetMessage(id=2, valid=False, x=40, y=50, z=60)
        binary_data = msg1.serialize() + junk + msg2.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 2)
        self.assertEqual(parsed_messages[0].id, 1)
        self.assertEqual(parsed_messages[1].id, 2)

    def test_incremental_parsing(self):
        msg = TargetMessage(id=123, valid=True, x=10, y=20, z=30)
        binary_data = msg.serialize()

        # Feed data byte by byte
        parsed_messages = []
        for i in range(len(binary_data)):
            parsed_messages.extend(list(self.parser.parse(binary_data[i:i+1])))

        self.assertEqual(len(parsed_messages), 1)
        self.assertEqual(parsed_messages[0].id, 123)

    def test_corrupted_footer(self):
        msg_ok1 = TargetMessage(id=1, valid=True, x=1, y=1, z=1)
        msg_bad = TargetMessage(id=2, valid=True, x=2, y=2, z=2).serialize()[:-2] + b'\x00\x00'
        msg_ok2 = TargetMessage(id=3, valid=True, x=3, y=3, z=3)

        binary_data = msg_ok1.serialize() + msg_bad + msg_ok2.serialize()
        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 2)
        self.assertEqual(parsed_messages[0].id, 1)
        self.assertEqual(parsed_messages[1].id, 3)

    def test_unknown_type_code(self):
        # Header, Type Code = 99 (unknown), Footer
        bad_type_packet = struct.pack('<HB', 0xCAFE, 99)
        msg_ok = TargetMessage(id=123, valid=True, x=1, y=2, z=3)
        binary_data = bad_type_packet + msg_ok.serialize()

        parsed_messages = list(self.parser.parse(binary_data))

        self.assertEqual(len(parsed_messages), 1)
        self.assertEqual(parsed_messages[0].id, 123)


if __name__ == '__main__':
    unittest.main()
