#include "serializer.hpp"
#include <cassert>
#include <iostream>
#include <sstream>

// A wrapper for std::stringstream to make it conform to the IOAble concept
class StringStreamWrapper {
public:
    std::stringstream& ss;

    StringStreamWrapper(std::stringstream& stream) : ss(stream) {}

    size_t readsome(char* buf, size_t count) {
        ss.read(buf, count);
        return ss.gcount();
    }

    size_t write(const char* buf, size_t count) {
        ss.write(buf, count);
        return count;
    }

    bool good() {
        return ss.good();
    }
};

void test_serializer() {
    std::stringstream ss;
    StringStreamWrapper wrapper(ss);
    cerializer::Serializer<StringStreamWrapper> serializer(wrapper);

    cerializer::Target target(1, true, 10, 20, 30);
    serializer.Write(target);

    auto binary_message = target.ToBinary();
    std::string expected(binary_message.begin(), binary_message.end());

    assert(ss.str() == expected);
    std::cout << "test_serializer passed" << std::endl;
}

void test_stream_handler_write_read() {
    std::stringstream ss;
    StringStreamWrapper wrapper(ss);
    cerializer::StreamHandler<StringStreamWrapper> handler(wrapper);

    cerializer::Target sent_target(1, true, 10, 20, 30);
    handler.Write(sent_target);

    handler.ParseStream([&](cerializer::BasePointer& received_msg) {
        assert(received_msg);
        assert(received_msg->Code() == cerializer::Target::Type());
        auto received_target = static_cast<cerializer::Target*>(received_msg.get());
        assert(received_target->id == sent_target.id);
        assert(received_target->valid == sent_target.valid);
        assert(received_target->x == sent_target.x);
        assert(received_target->y == sent_target.y);
        assert(received_target->z == sent_target.z);
    });

    std::cout << "test_stream_handler_write_read passed" << std::endl;
}

void run_serializer_tests() {
    test_serializer();
    test_stream_handler_write_read();
}