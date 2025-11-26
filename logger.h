#pragma once

#include <atomic>
#include <cstdint>
#include <iostream>
#include <source_location>
#include <sstream>
#include <string>
#include <string_view>         // Add this
using namespace std::literals; // required for ""sv

namespace logger {
	// Avoid the plain identifier `DEBUG` (can collide with build-system macros).
	enum class LogLevel : uint8_t { LogLvl, ErrorLvl, DebugLvl };

	constexpr std::string_view levelString(const LogLevel& level) {
		switch (level) {
		case LogLevel::LogLvl:
			return "LOG"sv;
		case LogLevel::ErrorLvl:
			return "ERROR"sv;
		case LogLevel::DebugLvl:
			return "DEBUG"sv;
		}
		return "LOG"sv;
	}

	struct LogMessage {
		const LogLevel         level = LogLevel::LogLvl;
		const std::string_view message;   // View of the original message
		const std::string_view file_name; // View of the const char*
#ifdef LOGGER_CAPTURE_CALLER
		const std::string_view function_name; // View of the const char*
#endif
		const std::string  tags;
		const unsigned int line_number;
	};

	inline const std::string format(const LogMessage& msg) {
		std::stringstream str;
		str << "[" << levelString(msg.level) << "] " << msg.message;
		if (!msg.tags.empty()) {
			str << " " << msg.tags;
		}
		// Always include file:line when available
		if (!msg.file_name.empty() || msg.line_number) {
			str << " (" << msg.file_name << ":" << msg.line_number << ")";
		}

#ifdef LOGGER_CAPTURE_CALLER
		// Optionally include function name when present
		if (!msg.file_name.empty() && msg.line_number && !msg.function_name.empty()) {
			str << " :: " << msg.function_name;
		}
#endif
		return str.str();
	}

	class Backend { // abstract base class for backend
	protected:
		virtual ~Backend() = default;
		virtual bool render(const std::string_view& str) = 0;
	};

	class ConsoleBackend: public Backend {
	public:
		bool render(const std::string_view& str) override {
			std::cout << str << std::endl;
			return true;
		}
	};

	// Runtime toggle for including caller information (only meaningful when LOGGER_CAPTURE_CALLER is enabled)
	inline std::atomic<bool> include_caller_info{true};

	inline void setIncludeCallerInfo(bool v) {
		include_caller_info.store(v);
	}

	inline bool getIncludeCallerInfo() {
		return include_caller_info.load();
	}

	// LogSource collects the message and (optionally) the caller location. When
	// LOGGER_CAPTURE_CALLER is NOT defined the constructor does not capture a
	// std::source_location so there is zero runtime cost for collecting caller
	// info.
#ifdef LOGGER_CAPTURE_CALLER
	struct LogSource {
		std::string_view     msg;
		std::source_location loc;

		template <typename StringType>
		constexpr LogSource(const StringType& m, const std::source_location& l = std::source_location::current()):
			msg(m), loc(l) {}
	};
#else
	struct LogSource {
		std::string_view msg;

		template <typename StringType>
		constexpr LogSource(const StringType& m): msg(m) {}
	};
#endif

	template <class B>
		requires std::derived_from<B, Backend>
	class Logger {
		B backend;

		template <typename... Ts>
		void doLogging(const LogLevel& level, const LogSource& src, Ts&&... flags) {
			std::stringstream tags;
			((tags << "[" << flags << "] "), ...);

			// Fill LogMessage fields conditionally. When LOGGER_CAPTURE_CALLER is
			// defined we have access to src.loc; otherwise we supply empty/zero
			// values to avoid any capture cost.
#ifdef LOGGER_CAPTURE_CALLER
			if (include_caller_info.load()) {
				LogMessage log{
					.level = level,
					.message = src.msg,
					.file_name = src.loc.file_name(),
					.function_name = src.loc.function_name(),
					.tags = tags.str(),
					.line_number = src.loc.line(),
				};

				std::string logStr = format(log);
				backend.render(logStr);
				return;
			} else {
				LogMessage log{
					.level = level,
					.message = src.msg,
					.file_name = ""sv,
					.function_name = ""sv,
					.tags = tags.str(),
					.line_number = 0,
				};
				std::string logStr = format(log);
				backend.render(logStr);
				return;
			}
#else
			LogMessage log{
				.level = level,
				.message = src.msg,
				.file_name = ""sv,
				.tags = tags.str(),
				.line_number = 0,
			};

			std::string logStr = format(log);
			backend.render(logStr);
#endif
		}

	public:
		template <typename... Ts>
		void Log(LogSource& src, Ts&&... flags) {
			doLogging(LogLevel::LogLvl, src, std::forward<Ts>(flags)...);
		};

		template <typename... Ts>
		void Error(LogSource& src, Ts&&... flags) {
			doLogging(LogLevel::ErrorLvl, src, std::forward<Ts>(flags)...);
		};

		template <typename... Ts>
		void Debug(LogSource& src, Ts&&... flags) {
			doLogging(LogLevel::DebugLvl, src, std::forward<Ts>(flags)...);
		};
	};

	inline static Logger<ConsoleBackend> defaultLogger;

	template <typename... Ts>
	void Log(LogSource src, Ts&&... flags) {
		defaultLogger.Log(src, std::forward<Ts>(flags)...);
	};

	template <typename... Ts>
	void Error(LogSource src, Ts&&... flags) {
		defaultLogger.Error(src, std::forward<Ts>(flags)...);
	};

	template <typename... Ts>
	void Debug(LogSource src, Ts&&... flags) {
		defaultLogger.Debug(src, std::forward<Ts>(flags)...);
	};

	// What if these were classes, whose initializers did the logging?  A lot more would be definitively known at
	// compile time...

}; // namespace logger