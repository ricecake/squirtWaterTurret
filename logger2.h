#pragma once
#include <array>
#include <cstdint>
#include <iostream>
#include <source_location>
#include <sstream>
#include <string>
#include <string_view>
using namespace std::literals; // required for ""sv
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

// + loc.file_name + "::"
// + loc.function_name + "::"
// + loc.line << "::";

// A constexpr-friendly string wrapper for NTTP
template <size_t N>
struct FixedString {
	std::array<char, N> data{};

	// Constexpr constructor to build from a literal
	consteval FixedString(const char (&str)[N]) { std::copy_n(str, N, data.begin()); }

	// Get as a string_view
	constexpr std::string_view view() const {
		// -1 to exclude the null terminator
		return std::string_view{data.data(), N - 1};
	}

	// Get size (excluding null)
	static constexpr size_t size = N - 1;
};

// Deduction guide to make it easier to use
template <size_t N>
FixedString(const char (&str)[N]) -> FixedString<N>;

template <FixedString file, FixedString function, int sline>
struct SourceLoc {
	const FixedString<file>     file_name;
	const FixedString<function> function_name;
	const int                   line{sline};

	consteval SourceLoc(const auto& file_name = file, const auto& function_name = function, int line = sline): file_name(file), function_name(function), line(line) {}
};

class Backend {
public:
	virtual ~Backend() = default;
	// Just renders a pre-formatted string view
	virtual bool render(std::string_view str) = 0;
};

class ConsoleBackend: public Backend {
public:
	bool render(std::string_view str) override {
		// We use std::cout.write for string_view, or just print it
		std::cout << str << std::endl;
		return true;
	}
};

#include <algorithm>
#include <array>
#include <charconv> // For std::to_chars

namespace logger {
	// --- Helpers ---

	// Helper to copy a string_view into a char array at a specific offset
	// Returns the new offset after copying
	template <size_t N>
	constexpr size_t copy_to_array(std::array<char, N>& arr, size_t offset, std::string_view sv) {
		std::copy_n(sv.data(), sv.size(), arr.begin() + offset);
		return offset + sv.size();
	}

	// Helper to convert an integer (like line number) to chars in our array
	// Returns the new offset after writing the number
	template <size_t N>
	constexpr size_t int_to_array(std::array<char, N>& arr, size_t offset, uint32_t line) {
		// std::to_chars is constexpr-friendly
		auto [ptr, ec] = std::to_chars(arr.data() + offset, arr.data() + N, line);
		if (ec == std::errc()) {
			return ptr - arr.data(); // Return new offset
		}
		return offset; // Failed, return old offset
	}

	// Helper to get the length of an integer when converted to a string
	constexpr size_t int_length(uint32_t n) {
		if (n == 0)
			return 1;
		size_t len = 0;
		while (n > 0) {
			n /= 10;
			len++;
		}
		return len;
	}

	// --- The Main Compile-Time Formatter ---

	// We template this on the log level, the message, and the source location
	template <LogLevel L, FixedString Msg, SourceLoc Loc>
	consteval auto build_log_entry() {
		// 1. Get all the string_view pieces
		constexpr auto level_sv = levelString(L);
		constexpr auto msg_sv = Msg.view();
		constexpr auto file_sv = std::string_view{Loc.file_name};
		constexpr auto func_sv = std::string_view{Loc.function_name};
		constexpr auto line_num = Loc.line;

		// 2. Calculate the total size of the final string
		// Format: "[LEVEL] Message (file:line :: function)\n"
		constexpr size_t total_size = 1 + level_sv.size() + 2 // "[LEVEL] "
			+ msg_sv.size() + 2                               // "Message ("
			+ file_sv.size() + 1                              // "file:"
			+ int_length(line_num)                            // "line"
			+ 4                                               // " :: "
			+ func_sv.size() + 2;                             // "function)\n"

		// 3. Create the destination array
		std::array<char, total_size> buffer{};
		size_t                       offset = 0;

		// 4. Populate the array, piece by piece
		offset = copy_to_array(buffer, offset, "["sv);
		offset = copy_to_array(buffer, offset, level_sv);
		offset = copy_to_array(buffer, offset, "] "sv);
		offset = copy_to_array(buffer, offset, msg_sv);
		offset = copy_to_array(buffer, offset, " ("sv);
		offset = copy_to_array(buffer, offset, file_sv);
		offset = copy_to_array(buffer, offset, ":"sv);
		offset = int_to_array(buffer, offset, line_num);
		offset = copy_to_array(buffer, offset, " :: "sv);
		offset = copy_to_array(buffer, offset, func_sv);
		offset = copy_to_array(buffer, offset, ")\n"sv);
		// Note: We don't add a final null terminator, as we'll use string_view

		return buffer;
	}

} // namespace logger

namespace logger {

	template <class B>
		requires std::derived_from<B, Backend>
	class Logger {
		B backend;

	public:
		// The INFO/WARN/DEBUG functions are now templates
		// They take the static string literal via the FixedString NTTP
		template <FixedString Msg, SourceLoc Loc>
		void INFO() {
			// 1. Build the log entry AT COMPILE TIME
			static constexpr auto entry = logger::build_log_entry<LogLevel::INFO, Msg, Loc>();

			// 2. Render the pre-formatted view AT RUNTIME
			backend.render(std::string_view{entry.data(), entry.size()});
		};

		template <FixedString Msg, SourceLoc Loc>
		void WARN() {
			static constexpr auto entry = build_log_entry<LogLevel::WARN, Msg, Loc>();
			backend.render(std::string_view{entry.data(), entry.size()});
		};

		template <FixedString Msg, SourceLoc Loc>
		void DEBUG() {
			static constexpr auto entry = build_log_entry<LogLevel::DEBUG, Msg, Loc>();
			backend.render(std::string_view{entry.data(), entry.size()});
		};
	};

	inline static Logger<ConsoleBackend> defaultLogger;

	// The free functions are also templates on the FixedString
	template <FixedString Msg, SourceLoc<"A", "B", 3> Loc>
	void INFO() {
		defaultLogger.INFO<Msg, Loc>();
	};

	template <FixedString Msg, SourceLoc Loc>
	void WARN() {
		defaultLogger.WARN<Msg, Loc>();
	};

	template <FixedString Msg, SourceLoc Loc>
	void DEBUG() {
		defaultLogger.DEBUG<Msg, Loc>();
	};

} // namespace logger

// // --- How to call it ---
// int main() {
//     // The string literal is passed as a template parameter
//     logger::INFO<"This is a static log message">();
//     logger::WARN<"Something might be wrong">();

//     // This will fail to compile, as "s" is not a compile-time constant
//     // std::string s = "hello";
//     // logger::INFO<s>();
// }