#include <iostream>
#include <sstream>
#include <vector>

#include "common.h"
#include "serializer.hpp"

// Helper function to assert that two Target messages are equal.
void require_messages_equal(const cerializer::Target& a, const cerializer::Target& b) {
	CHECK(a.id == b.id);
	CHECK(a.valid == b.valid);
	CHECK(a.x == b.x);
	CHECK(a.y == b.y);
	CHECK(a.z == b.z);
}

TEST_CASE("Deserialize single valid message") {
	// Arrange: Create a message and serialize it into a stream.
	const cerializer::Target original_message(123, true, 10, 20, 30);
	auto                     binary_message = original_message.ToBinary();

	std::stringstream stream;
	stream.write(binary_message.data(), binary_message.size());

	cerializer::StreamHandler<std::stringstream> handler(stream);
	bool                                         message_received = false;

	// Act: Parse the stream.
	handler.ParseStream<cerializer::BasePacket>([&](cerializer::BasePointer& msg) {
		message_received = true;

		// Assert: Check that the deserialized message is correct.
		CHECK(msg != nullptr);
		CHECK(msg->Code() == cerializer::Target::Type());

		auto received_message = dynamic_cast<cerializer::Target*>(msg.get());
		CHECK(received_message != nullptr);

		require_messages_equal(original_message, *received_message);
	});

	CHECK(message_received);
}

TEST_CASE("Deserialize multiple valid messages") {
	// Arrange: Create multiple messages and serialize them into a single stream.
	const std::vector<cerializer::Target> original_messages = {
		cerializer::Target(1, true, 10, 20, 30),
		cerializer::Target(2, false, 40, 50, 60),
		cerializer::Target(3, true, 70, 80, 90),
	};

	std::stringstream stream;
	for (const auto& msg : original_messages) {
		auto binary_message = msg.ToBinary();
		stream.write(binary_message.data(), binary_message.size());
	}

	cerializer::StreamHandler<std::stringstream> handler(stream);
	std::vector<cerializer::Target>              received_messages;

	// Act: Parse the stream.
	handler.ParseStream<cerializer::BasePacket>([&](cerializer::BasePointer& msg) {
		CHECK(msg != nullptr);
		CHECK(msg->Code() == cerializer::Target::Type());

		auto received_message = dynamic_cast<cerializer::Target*>(msg.get());
		CHECK(received_message != nullptr);

		received_messages.push_back(*received_message);
	});

	// Assert: Check that all messages were received and are correct.
	CHECK(received_messages.size() == original_messages.size());
	for (size_t i = 0; i < original_messages.size(); ++i) {
		require_messages_equal(original_messages[i], received_messages[i]);
	}
}

TEST_CASE("Deserialize with invalid data") {
	// Arrange: Create a valid message and surround it with junk data.
	const cerializer::Target original_message(456, true, 11, 22, 33);
	auto                     binary_message = original_message.ToBinary();
	std::string              junk_data = "xxx";

	std::stringstream stream;
	stream << junk_data;
	stream.write(binary_message.data(), binary_message.size());
	stream << junk_data;

	cerializer::StreamHandler<std::stringstream> handler(stream);
	bool                                         message_received = false;

	// Act: Parse the stream.
	handler.ParseStream<cerializer::BasePacket>([&](cerializer::BasePointer& msg) {
		message_received = true;

		// Assert: Check that the deserialized message is correct.
		CHECK(msg != nullptr);
		auto received_message = dynamic_cast<cerializer::Target*>(msg.get());
		require_messages_equal(original_message, *received_message);
	});

	CHECK(message_received);
}

TEST_CASE("Deserialize incomplete message") {
	// Arrange: Create an incomplete message.
	const cerializer::Target original_message(789, false, 44, 55, 66);
	auto                     binary_message = original_message.ToBinary();

	std::stringstream stream;
	stream.write(binary_message.data(), binary_message.size() - 1); // Write all but the last byte

	cerializer::StreamHandler<std::stringstream> handler(stream);
	bool                                         message_received = false;

	// Act: Parse the stream.
	handler.ParseStream<cerializer::BasePacket>([&](cerializer::BasePointer& msg) {
		message_received = bool(msg); // This should not be called
	});

	// Assert: No message should have been deserialized.
	CHECK(!message_received);
}

TEST_CASE("Serialize and deserialize a message") {
	// Arrange: Create a message and a stream.
	const cerializer::Target original_message(101, true, 12, 13, 14);
	std::stringstream        stream;

	cerializer::StreamHandler<std::stringstream> handler(stream);
	bool                                         message_received = false;

	// Act: Write the message to the stream, then parse it back.
	handler.Write(original_message);

	handler.ParseStream<cerializer::BasePacket>([&](cerializer::BasePointer& msg) {
		message_received = true;

		// Assert: Check that the deserialized message is correct.
		CHECK(msg != nullptr);
		auto received_message = dynamic_cast<cerializer::Target*>(msg.get());
		require_messages_equal(original_message, *received_message);
	});

	CHECK(message_received);
}
