#pragma once
#include <algorithm>
#include <array>
#include <charconv> // For std::to_chars
#include <cstdint>
#include <iostream>
#include <source_location>
#include <sstream>
#include <string>
#include <string_view>

namespace logger {

	using namespace std::literals; // required for ""sv
	enum class LogLevel : uint8_t { INFO, WARN, DEBUG };

	consteval inline static std::string_view levelString(const LogLevel& level) {
		switch (level) {
		case LogLevel::INFO:
			return "INFO";
		case LogLevel::WARN:
			return "WARN";
		case LogLevel::DEBUG:
			return "DEBUG";
		}
	}

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

	class Backend {
	public:
		virtual ~Backend() = default;
		virtual bool render(std::string_view str) = 0;
	};

	class ConsoleBackend: public Backend {
	public:
		bool render(std::string_view str) override {
			std::cout << str << std::endl;
			return true;
		}
	};

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

	template <LogLevel L, FixedString Msg>
	consteval auto build_log_entry() {
		// 1. Get all the string_view pieces
		constexpr auto level_sv = levelString(L);
		constexpr auto msg_sv = Msg.view();

		constexpr size_t total_size = 1 + level_sv.size() + 2 // "[LEVEL] "
			+ msg_sv.size();                                  // "Message"

		std::array<char, total_size> buffer{};
		size_t                       offset = 0;

		offset = copy_to_array(buffer, offset, "["sv);
		offset = copy_to_array(buffer, offset, level_sv);
		offset = copy_to_array(buffer, offset, "] "sv);
		offset = copy_to_array(buffer, offset, msg_sv);

		return buffer;
	}

	template <class B>
		requires std::derived_from<B, Backend>
	class Logger {
		B backend;

	public:
		template <FixedString Msg>
		void INFO() {
			static constexpr auto entry = logger::build_log_entry<LogLevel::INFO, Msg>();
			backend.render(std::string_view{entry.data(), entry.size()});
		};

		template <FixedString Msg>
		void WARN() {
			static constexpr auto entry = build_log_entry<LogLevel::WARN, Msg>();
			backend.render(std::string_view{entry.data(), entry.size()});
		};

		template <FixedString Msg>
		void DEBUG() {
			static constexpr auto entry = build_log_entry<LogLevel::DEBUG, Msg>();
			backend.render(std::string_view{entry.data(), entry.size()});
		};
	};

	inline static Logger<ConsoleBackend> defaultLogger;

	template <FixedString Msg>
	void INFO(std::source_location loc = std::source_location::current()) {
		defaultLogger.INFO<Msg>();
	};

	template <FixedString Msg>
	void WARN() {
		defaultLogger.WARN<Msg>();
	};

	template <FixedString Msg>
	void DEBUG() {
		defaultLogger.DEBUG<Msg>();
	};

} // namespace logger
