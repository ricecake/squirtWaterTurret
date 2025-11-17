#pragma once
#include <cstdint>
#include <iostream>
#include <source_location>
#include <sstream>
#include <string>
using namespace std::literals; // required for ""sv

namespace logger {
	enum class LogLevel : uint8_t { INFO, WARN, DEBUG };

	constexpr inline static std::string_view levelString(const LogLevel& level) {
		switch (level) {
		case LogLevel::INFO:
			return "INFO";
		case LogLevel::WARN:
			return "WARN";
		case LogLevel::DEBUG:
			return "DEBUG";
		}
	}

	struct LogMessage {
		LogLevel         level = LogLevel::INFO;
		std::string_view message;
	};

	class Backend { // abstract base class for backend
	protected:
		virtual ~Backend() = default;
		virtual constexpr const std::string_view format(const LogMessage& msg, std::source_location loc) = 0;
		virtual bool                             render(const std::string& str) = 0;
	};

	class ConsoleBackend: public Backend {
		constexpr const std::string_view format(const LogMessage& msg, std::source_location loc) override {
			// std::string str;
			// str
			// + "[" + levelString(msg.level) + "] "
			// + msg.message + " "
			// + loc.file_name + "::"
			// + loc.function_name + "::"
			// + loc.line << "::";

			std::string_view str{"["sv + levelString(msg.level)};

			return str;
		}

		bool render(const std::string& str) override {
			std::cout << str << std::endl;
			return true;
		}
	};

	template <class B>
		requires std::derived_from<B, Backend>
	class Logger {
		B backend;

		void doLogging(LogLevel level, const std::string& msg, const auto& loc) {
			LogMessage log{level, msg};
			auto       logStr = backend.format(log, loc);
			backend.render(logStr);
		}

	public:
		void INFO(const std::string& msg, const auto& loc = std::source_location::current()) {
			doLogging(LogLevel::INFO, msg, loc);
		};

		void WARN(const std::string& msg, const auto& loc = std::source_location::current()) {
			doLogging(LogLevel::WARN, msg, loc);
		};

		void DEBUG(const std::string& msg, const auto& loc = std::source_location::current()) {
			doLogging(LogLevel::DEBUG, msg, loc);
		};
	};

	inline static Logger<ConsoleBackend> defaultLogger;

	void INFO(const std::string& msg, const auto& loc = std::source_location::current()) {
		defaultLogger.INFO(msg, loc);
	};

	void WARN(const std::string& msg, const auto& loc = std::source_location::current()) {
		defaultLogger.WARN(msg, loc);
	};

	void DEBUG(const std::string& msg, const auto& loc = std::source_location::current()) {
		defaultLogger.DEBUG(msg, loc);
	};

}; // namespace logger