#pragma once
#include <array>
#include <cstdint>
#include <iostream>
#include <source_location>
#include <sstream>
#include <string>
using namespace std::literals; // required for ""sv

namespace logger {
	// A constexpr-friendly string wrapper for NTTP
	template <size_t N>
	struct FixedString {
		std::array<char, N> data{};

		// Constexpr constructor to build from a literal
		consteval FixedString(const char (&str)[N]) { std::copy_n(str, N, data.begin()); }

		// Get as a string_view
		constexpr std::string view() const {
			// -1 to exclude the null terminator
			return std::string{data.data(), N - 1};
		}

		// Get size (excluding null)
		static constexpr size_t size = N - 1;
	};

	// Deduction guide to make it easier to use
	template <size_t N>
	FixedString(const char (&str)[N]) -> FixedString<N>;

	enum class LogLevel : uint8_t { INFO, WARN, TEST };

	constexpr inline static std::string_view levelString(const LogLevel& level) {
		switch (level) {
		case LogLevel::WARN:
			return "[WARN]"sv;
		case LogLevel::TEST:
			return "[TEST]"sv;
		case LogLevel::INFO:
		default:
			return "[INFO]"sv;
		}
	}

	consteval inline static std::string_view levelString2(const LogLevel& level) {
		switch (level) {
		case LogLevel::WARN:
			return "[WARN]"sv;
		case LogLevel::TEST:
			return "[TEST]"sv;
		case LogLevel::INFO:
		default:
			return "[INFO]"sv;
		}
	}

	struct LogMessage {
		const LogLevel     level = LogLevel::INFO;
		const std::string  message;
		const std::string  file_name;
		const std::string  function_name;
		const unsigned int line_number;
	};

	const std::string format(const LogMessage& msg) {
		std::stringstream str;
		str << "[" << std::string(levelString(msg.level)) << "] " << msg.message << " (" << msg.file_name << ": "
			<< msg.line_number << " :: " << msg.function_name;

		return str.str();
	}

	class Backend { // abstract base class for backend
	protected:
		virtual ~Backend() = default;
		virtual bool render(const std::string& str) = 0;
	};

	class ConsoleBackend: public Backend {
	public:
		bool render(const std::string& str) override {
			std::cout << str << std::endl;
			return true;
		}
	};

	template <class B>
		requires std::derived_from<B, Backend>
	class Logger {
		B backend;

		void doLogging(const LogLevel level, const std::string& msg, const std::source_location& loc) {
			LogMessage log{
				.level = level,
				.message = msg,
				.file_name = loc.file_name(),
				.function_name = loc.function_name(),
				.line_number = loc.line(),
			};
			std::string logStr = format(log);
			backend.render(logStr);
		}

	public:
		void INFO(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
			doLogging(LogLevel::INFO, msg, loc);
		};

		void WARN(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
			doLogging(LogLevel::WARN, msg, loc);
		};

		void TEST(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
			doLogging(LogLevel::TEST, msg, loc);
		};
	};

	inline static Logger<ConsoleBackend> defaultLogger;

	void INFO(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
		defaultLogger.INFO(msg, loc);
	};

	void WARN(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
		defaultLogger.WARN(msg, loc);
	};

	void TEST(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
		defaultLogger.TEST(msg, loc);
	};

	// We template this on the log level and the message
	template <LogLevel L, FixedString Msg>
	consteval auto build_log_entry() {
		// 1. Get all the string_view pieces
		constexpr auto level_sv = levelString(L);
		constexpr auto msg_sv = Msg.view();

		// 2. Calculate the total size of the final string
		// Format: "[LEVEL] Message (file:line :: function)\n"
		constexpr size_t total_size = 1 + level_sv.size() + 2 // "[LEVEL] "
			+ msg_sv.size();                              // "Message"

		// 3. Create the destination array
		std::array<char, total_size> buffer{};
		size_t                       offset = 0;

		// 4. Populate the array, piece by piece
		offset = copy_to_array(buffer, offset, "["sv);
		offset = copy_to_array(buffer, offset, level_sv);
		offset = copy_to_array(buffer, offset, "] "sv);
		offset = copy_to_array(buffer, offset, msg_sv);
		// Note: We don't add a final null terminator, as we'll use string_view

		return buffer;
	}


	template <size_t N, typename... Ts>
		requires(std::is_trivially_copyable_v<Ts> && ...)
	constexpr inline std::array<char, ((sizeof(levelString2(LogLevel::INFO)) + N) + ... + sizeof(Ts))>
	pack(const char (&msg)[N], const Ts&... args) {
		constexpr auto level = levelString2(LogLevel::INFO);

		std::array<char, ((sizeof(levelString2(LogLevel::INFO)) + N) + ... + sizeof(Ts))> dest;
		auto                                                                              offset = 0;

		 std::memcpy(dest.data() + offset, &level, sizeof(level));
		offset += sizeof(level);

		std::memcpy(dest.data() + offset, &msg, N);
		offset += N;

		(
			[&]() {
				std::memcpy(dest.data() + offset, &args, sizeof(Ts));
				offset += sizeof(Ts);
			}(),
			...
		);
		return dest;
	};

}; // namespace logger