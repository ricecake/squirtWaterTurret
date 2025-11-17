#pragma once
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

	enum class LogLevel : uint8_t { INFO, WARN, DEBUG };

 inline static std::string levelString(const LogLevel& level) {
		switch (level) {
		case LogLevel::INFO:
			return "INFO";
		case LogLevel::WARN:
			return "WARN";
		case LogLevel::DEBUG:
			return "DEBUG";
		}
		return "INFO";
	}

	struct LogMessage {
		const LogLevel         level = LogLevel::INFO;
		const std::string message;
		const std::string file_name;
		const std::string function_name;
		const unsigned int     line_number;
	};

	const std::string format(const LogMessage& msg) {
			std::stringstream str;
			str
			<< "[" << std::string(levelString(msg.level)) << "] "
			<< msg.message << " ("
			<< msg.file_name << ": "
			<< msg.line_number << " :: "
			<< msg.function_name;

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

		void DEBUG(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
			doLogging(LogLevel::DEBUG, msg, loc);
		};
	};

	inline static Logger<ConsoleBackend> defaultLogger;

	void INFO(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
		defaultLogger.INFO(msg, loc);
	};

	void WARN(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
		defaultLogger.WARN(msg, loc);
	};

	void DEBUG(const std::string& msg, const std::source_location& loc = std::source_location::current()) {
		defaultLogger.DEBUG(msg, loc);
	};

}; // namespace logger