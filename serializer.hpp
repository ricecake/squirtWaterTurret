/**
 * @file serializer.hpp
 * @brief A header-only serialization library for creating and parsing binary messages.
 *
 * This library provides a framework for defining message structures, serializing them
 * into a binary format, and deserializing them from a stream. It is designed to be
 * type-safe and extensible.
 */
#pragma once
#include <array>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <span>
#include <tuple>

#include <stdint.h>

/**
 * @brief The main namespace for the serialization library.
 */
namespace cerializer {
	const uint16_t magicHead = 0xCAFE; ///< Magic number to signify the start of a message.
	const uint16_t magicFoot = 0xFACE; ///< Magic number to signify the end of a message.

	/**
	 * @brief Converts a block of data into a hexadecimal string representation.
	 */
	std::string hexify(auto data) {
		std::stringstream out;
		for (char c : data) {
			out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(c));
		}
		return out.str();
	}

	/**
	 * @brief Converts an object into a character array.
	 */
	template <typename T>
	constexpr auto toCharArray(const T& thing) {
		return std::bit_cast<std::array<char, sizeof(T)>>(thing);
	}

	/**
	 * @brief Represents the current state of the deserializer's parsing process.
	 */
	enum ParseMode {
		START,   ///< Looking for the start of a message.
		PRE_END, ///< Found the start, looking for the end.
		END,     ///< Found a potential end, confirming.
		EMIT,    ///< A complete message is ready to be emitted.
	};

	/**
	 * @brief Type trait to check if a type is a std::array.
	 */
	template <typename T>
	struct is_std_array: std::false_type {};

	template <typename T, std::size_t N>
	struct is_std_array<std::array<T, N>>: std::true_type {};

	template <typename T>
	constexpr bool is_std_array_v = is_std_array<T>::value;

	/**
	 * @brief Concept to identify types that are indexable (like arrays).
	 */
	template <class T>
	concept Indexable = requires { requires std::is_array_v<T> || is_std_array_v<T>; };

	/**
	 * @brief Concept to identify container-like types.
	 */
	template <class T, typename V = nullptr_t, typename N = int>
	concept Container = requires(T obj, V, N idx) {
		typename T::value_type;
		{ obj.data() } -> std::same_as<std::conditional_t<!std::is_same_v<V, nullptr_t>, V, typename T::value_type>*>;
		{ obj[idx] } -> std::same_as<std::conditional_t<!std::is_same_v<V, nullptr_t>, V, typename T::value_type>&>;
	};

	/**
	 * @brief A utility function to perform a postfix addition.
	 */
	template <std::integral T, std::integral Y>
	constexpr inline std::common_type<T, Y>::type postfixAdd(T& initial, const Y& add) {
		T x = initial;
		initial += add;
		return x;
	}

	/**
	 * @brief Packs a series of arguments into a single character array.
	 */
	template <typename... Ts>
		requires(std::is_trivially_copyable_v<Ts> && ...)

	constexpr inline std::array<char, (sizeof(Ts) + ...)> pack(const Ts&... args) {
		std::array<char, (sizeof(Ts) + ...)> dest;
		auto                                 offset = 0;
		(
			[&]() {
				if constexpr ((std::endian::native == std::endian::big) && sizeof(Ts) > 1 && !Indexable<Ts>) {
					auto bits = std::bit_cast<std::array<char, sizeof(Ts)>>(args);
					std::copy(bits.rbegin(), bits.rend(), dest.data() + offset);
				} else {
					std::memcpy(dest.data() + offset, &args, sizeof(Ts));
				}
				offset += sizeof(Ts);
			}(),
			...
		);
		return dest;
	};

	/**
	 * @brief Unpacks a character array into a specified destination type.
	 */
	template <typename Dest, typename... Ts, typename Cont>
		requires(std::is_trivially_copyable_v<Ts> && ...) && Container<Cont, char>
	constexpr inline Dest unpack(const Cont& binaryData) {
		if constexpr (sizeof...(Ts) < 1) {
			return *reinterpret_cast<Dest*>(binaryData.data());
		} else {
			auto            offset = 0;
			std::span<char> dataView(binaryData);
			return Dest{*reinterpret_cast<Ts*>(dataView.subspan(postfixAdd(offset, sizeof(Ts)), sizeof(Ts)).data())...};
		}
	};

	using TypeCharSpec = std::tuple<uint8_t, uint8_t, char>;

	/**
	 * @brief Gets the format size and type character for a given type.
	 */
	template <typename T>
		requires std::is_trivially_copyable_v<T> && (!std::is_bounded_array_v<T>)
	constexpr TypeCharSpec formatSize() {
		return {ceil(log10(sizeof(T))), sizeof(T), 'P'};
	}

	template <typename T>
		requires std::is_bounded_array_v<T> && std::same_as<char, std::remove_all_extents_t<T>>
	constexpr TypeCharSpec formatSize() {
		return {ceil(log10(sizeof(T))), sizeof(T), 's'};
	}

	template <>
	constexpr TypeCharSpec formatSize<char>() {
		return {1, 1, 'c'};
	}

	template <>
	constexpr TypeCharSpec formatSize<signed char>() {
		return {1, 1, 'b'};
	}

	template <>
	constexpr TypeCharSpec formatSize<unsigned char>() {
		return {1, 1, 'B'};
	}

	template <>
	constexpr TypeCharSpec formatSize<bool>() {
		return {1, 1, '?'};
	}

	template <>
	constexpr TypeCharSpec formatSize<short>() {
		return {1, 2, 'h'};
	}

	template <>
	constexpr TypeCharSpec formatSize<unsigned short>() {
		return {1, 2, 'H'};
	}

	template <>
	constexpr TypeCharSpec formatSize<int>() {
		return {1, 4, 'i'};
	}

	template <>
	constexpr TypeCharSpec formatSize<unsigned int>() {
		return {1, 4, 'I'};
	}

	template <>
	constexpr TypeCharSpec formatSize<long>() {
		return {1, 4, 'l'};
	}

	template <>
	constexpr TypeCharSpec formatSize<unsigned long>() {
		return {1, 4, 'L'};
	}

	template <>
	constexpr TypeCharSpec formatSize<long long>() {
		return {1, 8, 'q'};
	}

	template <>
	constexpr TypeCharSpec formatSize<unsigned long long>() {
		return {1, 8, 'Q'};
	}

	template <>
	constexpr TypeCharSpec formatSize<float>() {
		return {1, 4, 'f'};
	}

	template <>
	constexpr TypeCharSpec formatSize<double>() {
		return {1, 8, 'd'};
	}

	template <typename... Ts>
	constexpr size_t formatStringSize() {
		return (1 + ... + std::get<0>(formatSize<Ts>()));
	}

	/**
	 * @brief A type alias for the character array that holds a rendered format string.
	 */
	template <typename... Ts>
	using RenderedFormatString = std::array<char, formatStringSize<Ts...>()>;

	/**
	 * @brief Renders the format string for a series of types.
	 */
	template <typename... Ts>
		requires(std::is_trivially_copyable_v<Ts> && ...)
	constexpr inline RenderedFormatString<Ts...> renderFormat() {
		return RenderedFormatString<Ts...>{char('<'), char(std::get<2>(formatSize<Ts>()))...};
	}

	/**
	 * @brief Base class for all serializable packets.
	 */
	class BasePacket {
	public:
		virtual ~BasePacket() = default;
		virtual constexpr uint8_t Code() = 0;
	};

	using BasePointer = std::unique_ptr<BasePacket>;

	/**
	 * @brief A factory class for creating message objects from binary data.
	 */
	class MessageMaker {
	public:
		using TCreateMethod = std::function<BasePointer(const std::span<char>& binaryData)>;

		static bool Register(const uint8_t typeVal, TCreateMethod builder) {
			MessageMaker::makerMap[typeVal] = builder;
			return true;
		}

		static BasePointer Create(const uint8_t typeVal, const std::span<char>& binaryData) {
			if (auto it = MessageMaker::makerMap.find(typeVal); it != makerMap.end()) {
				return it->second(binaryData);
			}
			return nullptr;
		}

	private:
		static inline std::map<uint8_t, TCreateMethod> makerMap;
	};

	/**
	 * @brief A template class for defining serializable messages.
	 *
	 * This class uses the Curiously Recurring Template Pattern (CRTP) to provide a
	 * common interface for all messages. It handles automatic registration with the
	 * MessageMaker factory.
	 *
	 * @tparam Derived The derived message class.
	 * @tparam TypeVal A unique identifier for the message type.
	 * @tparam FieldTypes The types of the fields in the message.
	 */
	template <typename Derived, uint8_t TypeVal, typename... FieldTypes>
	class Message: public BasePacket {
	public:
		static bool registered;

	public:
		constexpr uint8_t Code() override { return TypeVal; };

		constexpr static uint8_t Type() { return TypeVal; };

		constexpr static unsigned int Size() { return (sizeof(FieldTypes) + ...); };

		using MessageFormat = std::array<char, 4 + (std::get<0>(formatSize<FieldTypes>()) + ...)>;
		using BinaryMessage = std::array<char, (sizeof(FieldTypes) + ...) + 2 * sizeof(uint16_t) + sizeof(uint8_t)>;

		constexpr static MessageFormat Format() { return renderFormat<uint16_t, uint8_t, FieldTypes..., uint16_t>(); };

		constexpr BinaryMessage ToBinary() const {
			auto encodedData = static_cast<const Derived*>(this)->encode();
			return pack(magicHead, Type(), encodedData, magicFoot);
		};

		constexpr static Derived LoadBinary(BinaryMessage& binaryData) {
			return LoadBinary(std::span(binaryData.begin(), binaryData.end()));
		}

		constexpr static Derived LoadBinary(const std::span<char>& binaryData) {
			std::span<char> dataView(binaryData);
			uint16_t        headCheck = unpack<uint16_t>(dataView.first(sizeof(magicHead)));
			uint16_t        footCheck = unpack<uint16_t>(dataView.last(sizeof(footCheck)));

			assert(headCheck == magicHead);
			assert(footCheck == magicFoot);

			return unpack<Derived, FieldTypes...>(dataView.subspan(sizeof(uint16_t) + sizeof(uint8_t), Size()));
		}
	};

	template <typename Derived, uint8_t TypeVal, typename... FieldTypes>
	bool Message<Derived, TypeVal, FieldTypes...>::registered = MessageMaker::Register(
		TypeVal,
		[](const std::span<char>& binaryData) -> BasePointer {
			auto obj = Derived::LoadBinary(binaryData);
			return std::make_unique<Derived>(obj);
		}
	);

	/**
	 * @brief An example message class for representing a target.
	 */
	class Target: public Message<Target, 0, uint32_t, bool, uint16_t, uint16_t, uint16_t> {
	public:
		const uint32_t id;
		const bool     valid;
		const uint16_t x;
		const uint16_t y;
		const uint16_t z;

	public:
		constexpr inline Target(uint32_t id, bool valid, uint16_t x, uint16_t y, uint16_t z) noexcept:
			id(id), valid(valid), x(x), y(y), z(z) {
			assert(registered);
		}

		constexpr std::array<char, Size()> encode() const { return pack(id, valid, x, y, z); }
	};

	/**
	 * @brief A message for setting runtime configuration parameters.
	 *
	 * This message is used to send updated configuration values to the firmware,
	 * allowing for on-the-fly tuning of the system's behavior. The float values
	 * are intended to be converted to fixed-point format on the device.
	 */
	class Config: public Message<Config, 1, float, float, uint16_t, uint16_t> {
	public:
		const float    projectile_speed;
		const float    turret_height;
		const uint16_t max_speed;
		const uint16_t acceleration;

	public:
		constexpr inline Config(
			float    projectile_speed,
			float    turret_height,
			uint16_t max_speed,
			uint16_t acceleration
		) noexcept:
			projectile_speed(projectile_speed),
			turret_height(turret_height),
			max_speed(max_speed),
			acceleration(acceleration) {
			assert(registered);
		}

		constexpr std::array<char, Size()> encode() const {
			return pack(projectile_speed, turret_height, max_speed, acceleration);
		}
	};

	/**
	 * @brief An enum to specify the source of targeting data.
	 */
	enum class TargetSource : uint8_t {
		STATIC, ///< Target is a fixed, statically defined point.
		RADAR,  ///< Target is provided by the onboard radar sensor.
		CV,     ///< Target is provided by the external computer vision system.
	};

	/**
	 * @brief A message to set the active target source.
	 *
	 * This message is sent to the microcontroller to command it to switch
	 * between different targeting systems like static, radar, or CV.
	 */
	class SetTargetSourceMessage: public Message<SetTargetSourceMessage, 2, TargetSource> {
	public:
		const TargetSource source;

	public:
		constexpr inline SetTargetSourceMessage(TargetSource source) noexcept: source(source) { assert(registered); }

		constexpr std::array<char, Size()> encode() const { return pack(source); }
	};

	/**
	 * @brief A message for setting a static target position.
	 *
	 * This message provides explicit coordinates for the turret to aim at when
	 * in STATIC target mode.
	 */
	class StaticTargetMessage: public Message<StaticTargetMessage, 3, uint16_t, uint16_t, uint16_t> {
	public:
		const uint16_t x;
		const uint16_t y;
		const uint16_t z;

	public:
		constexpr inline StaticTargetMessage(uint16_t x, uint16_t y, uint16_t z) noexcept: x(x), y(y), z(z) {
			assert(registered);
		}

		constexpr std::array<char, Size()> encode() const { return pack(x, y, z); }
	};

	/**
	 * @brief Concept to identify types that support I/O operations.
	 */
	template <typename T>
	concept IOAble = requires(T io, char* buf, size_t count) {
		{ io.readsome(buf, count) } -> std::convertible_to<size_t>;
		{ io.good() } -> std::convertible_to<bool>;
	};

	/**
	 * DEPRECATE
	 * This class is not implemented and is not used anywhere in the codebase.
	 */
	class Serializer {
	public:
		void Write();
	};

	/**
	 * @brief A class for deserializing messages from an input stream.
	 *
	 * This class reads from an input stream, identifies message boundaries using
	 * magic numbers, and uses the MessageMaker factory to construct message objects.
	 *
	 * @tparam T The type of the input stream.
	 */
	template <typename T>
		requires IOAble<T>

	class Deserializer {
	protected:
		T&                    input;
		std::array<char, 128> buf;
		char*                 offset = buf.begin();
		char*                 end_offset = buf.begin();

		const std::array<char, 2> header_bytes = toCharArray(magicHead);
		const std::array<char, 2> footer_bytes = toCharArray(magicFoot);
		std::size_t               read_size = header_bytes.size();
		std::array<char, 2>       token = header_bytes;
		ParseMode                 state = START;
		ParseMode                 success = PRE_END;
		ParseMode                 fail = START;

		constexpr auto findToken(
			const std::span<char>& buffer,
			const std::span<char>& value,
			const ParseMode&       fail_state,
			const ParseMode&       success_state
		) {
			auto next_state = fail_state;
			auto next_size = value.size();
			auto next_offset = buffer.begin();

			if (buffer.size() < value.size()) {
				next_size = value.size();
			} else if (buffer.size() >= value.size()) {
				auto index = std::search(buffer.begin(), buffer.end(), value.begin(), value.end());
				if (index == buffer.end()) {
					next_offset = buffer.end() - (value.size() - 1);
				} else {
					next_offset = index;
					next_state = success_state;
				}
			}

			return std::tuple{next_state, next_size, next_offset};
		}

	public:
		Deserializer(T& readStream): input(readStream) {};

		template <std::derived_from<BasePacket> Type>
		void ParseStream(std::function<void(std::unique_ptr<Type>&)> callback) {
			auto read = 0;

			read = input.readsome(end_offset, read_size);
			end_offset += read;

			auto result = findToken(std::span(offset, end_offset), std::span(token), fail, success);
			state = std::get<0>(result);
			read_size = std::get<1>(result);
			offset = std::get<2>(result).base();

			while (input.good() && (read > 0 || state == EMIT)) {
				switch (state) {
				case START:
					token = header_bytes;
					success = PRE_END;
					fail = START;
					if (offset > buf.begin()) {
						auto shift = offset - buf.begin();
						std::copy(offset, end_offset, buf.begin());
						end_offset -= shift;
						offset = buf.begin();
					}
					break;
				case PRE_END:
					token = footer_bytes;
					success = EMIT;
					fail = END;
					if (offset > buf.begin()) {
						auto shift = offset - buf.begin();
						std::copy(offset, end_offset, buf.begin());
						end_offset -= shift;
						offset = buf.begin();
					}
					break;
				case END:
					token = footer_bytes;
					success = EMIT;
					fail = END;
					break;
				case EMIT:
					uint8_t typeCode = buf.begin()[2];
					auto found = MessageMaker::Create(typeCode, std::span(buf.begin(), offset + sizeof(footer_bytes)));

					callback(found);

					offset = buf.begin();
					end_offset = buf.begin();
					state = START;
					continue;
				}

				auto result = findToken(std::span<char>(offset, end_offset), std::span(token), fail, success);
				state = std::get<0>(result);
				read_size = std::get<1>(result);
				offset = std::get<2>(result).base();

				read = input.readsome(end_offset, read_size);
				end_offset += read;
			}
		}
	};

	/**
	 * DEPRECATE
	 * This class is not implemented and is not used anywhere in the codebase.
	 */
	class StreamHandler {
	public:
	};
} // namespace cerializer