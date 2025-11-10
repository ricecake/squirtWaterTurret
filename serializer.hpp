/**
 * @file serializer.hpp
 * @brief A header-only serialization library for creating and parsing binary messages.
 *
 * This library provides a framework for defining message structures, serializing them
 * into a binary format, and deserializing them from a stream. It is designed to be
 * type-safe and extensible.
 */
#pragma once
#include <algorithm>
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
#include <algorithm>

#include "shared_types.h"
#include <stdint.h>

/**
 * @brief The main namespace for the serialization library.
 */
namespace cerializer {
	const uint16_t magicHead = 0xCAFE; ///< Magic number to signify the start of a message.
	const uint16_t magicFoot = 0xFACE; ///< Magic number to signify the end of a message.

	/**
	 * @brief Converts a block of data into a hexadecimal string representation.
	 * @param data The data to be converted, expected to be a container of characters.
	 * @return A string containing the hexadecimal representation of the input data.
	 */
	std::string hexify(auto data) {
		std::stringstream out;
		for (char c : data) {
			out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(c));
		}
		return out.str();
	}

	/**
	 * @brief Casts an object to its raw byte representation in a std::array.
	 * @tparam T The type of the object to convert.
	 * @param thing The object to be converted.
	 * @return A std::array<char, sizeof(T)> containing the byte representation of the object.
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
	 * @brief A utility function to perform a postfix addition, returning the original value.
	 *
	 * This is useful for advancing an offset or pointer while using its previous value in an expression.
	 * @tparam T The type of the value to be incremented.
	 * @tparam Y The type of the value to add.
	 * @param initial The value to be incremented, passed by reference.
	 * @param add The value to add.
	 * @return The value of `initial` before the addition.
	 */
	template <std::integral T, std::integral Y>
	constexpr inline std::common_type<T, Y>::type postfixAdd(T& initial, const Y& add) {
		T x = initial;
		initial += add;
		return x;
	}

	/**
	 * @brief Packs a series of trivially copyable arguments into a single character array.
	 *
	 * This function performs serialization by copying the binary representation of each argument
	 * into a contiguous byte array. It handles endianness conversion for multi-byte types
	 * if the native endianness is big-endian.
	 *
	 * @tparam Ts The types of the arguments to pack.
	 * @param args The arguments to be packed.
	 * @return A std::array<char, ...> containing the serialized data.
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
	 * @brief Safely unpacks a single trivially copyable type from a container of bytes.
	 *
	 * This function uses `std::memcpy` to avoid misaligned reads and handles endianness
	 * conversion for multi-byte types if the native endianness is big-endian.
	 *
	 * @tparam T The type of the object to unpack.
	 * @tparam Cont The container type for the binary data.
	 * @param data The container holding the bytes.
	 * @return An object of type `T`.
	 */
	template <typename T, typename Cont>
		requires Container<Cont, char>
	constexpr inline T unpack_one(const Cont& data) {
		T value;
		std::memcpy(&value, data.data(), sizeof(T));
		if constexpr ((std::endian::native == std::endian::big) && sizeof(T) > 1 && !Indexable<T>) {
			auto bytes = std::bit_cast<std::array<char, sizeof(T)>>(value);
			std::reverse(bytes.begin(), bytes.end());
			return std::bit_cast<T>(bytes);
		}
		return value;
	}

	/**
	 * @brief Unpacks a character array into a specified destination type or a struct.
	 *
	 * This function can operate in two modes:
	 * 1. If only `Dest` is provided, it unpacks the binary data directly into an object of type `Dest`.
	 * 2. If `Dest` and `Ts...` are provided, it unpacks the binary data into individual values of types `Ts...`
	 *    and uses them to construct an object of type `Dest`.
	 *
	 * It uses `memcpy` to ensure safe access to potentially misaligned data and handles
	 * endianness conversion.
	 *
	 * @tparam Dest The destination type to create.
	 * @tparam Ts The types of the fields to unpack for constructing `Dest`.
	 * @tparam Cont A container type for the binary data, like std::span or std::array.
	 * @param binaryData The container holding the binary data.
	 * @return An object of type `Dest` constructed from the unpacked data.
	 */
	template <typename Dest, typename... Ts, typename Cont>
		requires(std::is_trivially_copyable_v<Ts> && ...) && Container<Cont, char>
	constexpr inline Dest unpack(const Cont& binaryData) {
		if constexpr (sizeof...(Ts) < 1) {
			return unpack_one<Dest>(binaryData);
		} else {
			auto            offset = 0;
			std::span<char> dataView(binaryData);
			return Dest{unpack_one<Ts>(dataView.subspan(postfixAdd(offset, sizeof(Ts)), sizeof(Ts)))...};
		}
	};

	/**
	 * @brief A type alias for a tuple describing a type's format: {size_digits, size_in_bytes, type_char}.
	 */
	using TypeCharSpec = std::tuple<uint8_t, uint8_t, char>;

	/**
	 * @brief Gets the format size and type character for a given type.
	 *
	 * This function provides the necessary information for building a format string
	 * similar to Python's `struct` module. Specializations are provided for common types.
	 * The generic fallback treats the type as a packed binary structure.
	 *
	 * @tparam T The type to inspect.
	 * @return A `TypeCharSpec` tuple containing {size_digits, size_in_bytes, type_char}.
	 */
	template <typename T>
		requires std::is_trivially_copyable_v<T> && (!std::is_bounded_array_v<T>)
	constexpr TypeCharSpec formatSize() {
		return {ceil(log10(sizeof(T))), sizeof(T), 'P'};
	}

	/**
	 * @brief Specialization for character arrays, representing them as a string.
	 */
	template <typename T>
		requires std::is_bounded_array_v<T> && std::same_as<char, std::remove_all_extents_t<T>>
	constexpr TypeCharSpec formatSize() {
		return {ceil(log10(sizeof(T))), sizeof(T), 's'};
	}

	/**
	 * @brief Specialization for char.
	 */
	template <>
	constexpr TypeCharSpec formatSize<char>() {
		return {1, 1, 'c'};
	}

	/**
	 * @brief Specialization for signed char.
	 */
	template <>
	constexpr TypeCharSpec formatSize<signed char>() {
		return {1, 1, 'b'};
	}

	/**
	 * @brief Specialization for unsigned char.
	 */
	template <>
	constexpr TypeCharSpec formatSize<unsigned char>() {
		return {1, 1, 'B'};
	}

	/**
	 * @brief Specialization for bool.
	 */
	template <>
	constexpr TypeCharSpec formatSize<bool>() {
		return {1, 1, '?'};
	}

	/**
	 * @brief Specialization for short.
	 */
	template <>
	constexpr TypeCharSpec formatSize<short>() {
		return {1, 2, 'h'};
	}

	/**
	 * @brief Specialization for unsigned short.
	 */
	template <>
	constexpr TypeCharSpec formatSize<unsigned short>() {
		return {1, 2, 'H'};
	}

	/**
	 * @brief Specialization for int.
	 */
	template <>
	constexpr TypeCharSpec formatSize<int>() {
		return {1, 4, 'i'};
	}

	/**
	 * @brief Specialization for unsigned int.
	 */
	template <>
	constexpr TypeCharSpec formatSize<unsigned int>() {
		return {1, 4, 'I'};
	}

	/**
	 * @brief Specialization for long.
	 */
	template <>
	constexpr TypeCharSpec formatSize<long>() {
		return {1, 4, 'l'};
	}

	/**
	 * @brief Specialization for unsigned long.
	 */
	template <>
	constexpr TypeCharSpec formatSize<unsigned long>() {
		return {1, 4, 'L'};
	}

	/**
	 * @brief Specialization for long long.
	 */
	template <>
	constexpr TypeCharSpec formatSize<long long>() {
		return {1, 8, 'q'};
	}

	/**
	 * @brief Specialization for unsigned long long.
	 */
	template <>
	constexpr TypeCharSpec formatSize<unsigned long long>() {
		return {1, 8, 'Q'};
	}

	/**
	 * @brief Specialization for float.
	 */
	template <>
	constexpr TypeCharSpec formatSize<float>() {
		return {1, 4, 'f'};
	}

	/**
	 * @brief Specialization for double.
	 */
	template <>
	constexpr TypeCharSpec formatSize<double>() {
		return {1, 8, 'd'};
	}

	/**
	 * @brief A type alias for the character array that holds a rendered format string.
	 */
	template <typename... Ts>
	using RenderedFormatString = std::array<char, 1 + (std::get<0>(formatSize<Ts>()) + ...)>;

	/**
	 * @brief Renders the format string for a series of types.
	 */
	template <typename... Ts>
		requires(std::is_trivially_copyable_v<Ts> && ...)
	constexpr inline RenderedFormatString<Ts...> renderFormat() {
		return RenderedFormatString<Ts...>{char('<'), char(std::get<2>(formatSize<Ts>()))...};
	}

	/**
	 * @brief An abstract base class for all serializable message packets.
	 *
	 * This class defines the fundamental interface for all messages, requiring
	 * them to provide a unique type code. It ensures that different message
	 * types can be handled polymorphically.
	 */
	class BasePacket {
	public:
		virtual ~BasePacket() = default;
		/**
		 * @brief Gets the unique 8-bit code that identifies the message type.
		 * @return The type code.
		 */
		virtual constexpr uint8_t Code() = 0;
	};

	/**
	 * @brief Type alias for a unique pointer to a BasePacket, used for polymorphism.
	 */
	using BasePointer = std::unique_ptr<BasePacket>;

	/**
	 * @brief A factory class for creating message objects from binary data.
	 *
	 * This class uses the factory pattern to decouple the message parsing logic
	 * from the concrete message classes. It maintains a map of message type codes
	 * to constructor functions, allowing for dynamic creation of message objects
	 * based on a type code read from a stream.
	 */
	class MessageMaker {
	public:
		/// A function pointer type for a method that creates a message object.
		using TCreateMethod = std::function<BasePointer(const std::span<char>& binaryData)>;

		/**
		 * @brief Registers a message type with the factory.
		 * @param typeVal The unique identifier for the message type.
		 * @param builder The function that constructs the message object.
		 * @return True if registration was successful.
		 */
		static bool Register(const uint8_t typeVal, TCreateMethod builder) {
			MessageMaker::makerMap[typeVal] = builder;
			return true;
		}

		/**
		 * @brief Creates a message object of the specified type.
		 * @param typeVal The unique identifier for the message type.
		 * @param binaryData The binary data to deserialize.
		 * @return A unique pointer to the created message object, or nullptr if the type is not registered.
		 */
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
	 * @brief A template base class for defining serializable messages.
	 *
	 * This class uses the Curiously Recurring Template Pattern (CRTP) to provide a
	 * common interface and implementation for all message types. It automatically
	 * handles registration with the `MessageMaker` factory, serialization to a
	 * binary format (including headers and footers), and deserialization.
	 *
	 * To define a new message, a class should inherit from this template, providing
	 * itself as the `Derived` parameter, a unique `TypeVal`, and a list of the
	 * `FieldTypes` that make up its payload.
	 *
	 * @tparam Derived The derived message class that is being defined.
	 * @tparam TypeVal A unique 8-bit identifier for this message type.
	 * @tparam FieldTypes A parameter pack of the types of the fields in the message payload.
	 */
	template <typename Derived, uint8_t TypeVal, typename... FieldTypes>
	class Message: public BasePacket {
	public:
		/// @brief A static flag that triggers registration with the MessageMaker factory upon program start.
		static const bool registered;

	public:
		/**
		 * @brief Gets the unique code for this message type.
		 * @return The 8-bit type code.
		 */
		constexpr uint8_t Code() override { return TypeVal; };

		/**
		 * @brief Gets the unique code for this message type (static version).
		 * @return The 8-bit type code.
		 */
		constexpr static uint8_t Type() { return TypeVal; };

		/**
		 * @brief Gets the total size of the message payload in bytes.
		 * @return The size of the message payload.
		 */
		constexpr static unsigned int Size() { return (sizeof(FieldTypes) + ...); };

		/// @brief A type alias for the array holding the message's format string.
		using MessageFormat = std::array<char, 4 + (std::get<0>(formatSize<FieldTypes>()) + ...)>;
		/// @brief A type alias for the array holding the full binary message, including headers and footers.
		using BinaryMessage = std::array<char, (sizeof(FieldTypes) + ...) + 2 * sizeof(uint16_t) + sizeof(uint8_t)>;

		/**
		 * @brief Generates the format string for the message.
		 * @return An array of characters representing the format.
		 */
		constexpr static MessageFormat Format() { return renderFormat<uint16_t, uint8_t, FieldTypes..., uint16_t>(); };

		/**
		 * @brief Generates the binary representation of the message.
		 * This method serializes the message fields, adds the magic head/foot and type code.
		 * @return An array of characters containing the full binary message.
		 */
		constexpr BinaryMessage ToBinary() const {
			auto encodedData = static_cast<const Derived*>(this)->encode();
			return pack(magicHead, Type(), encodedData, magicFoot);
		};

		/**
		 * @brief Deserializes a message from a character array.
		 * @param binaryData The array containing the binary message.
		 * @return An instance of the derived message class.
		 */
		constexpr static Derived LoadBinary(BinaryMessage& binaryData) {
			return LoadBinary(std::span(binaryData.begin(), binaryData.end()));
		}

		/**
		 * @brief Deserializes a message from a span of characters.
		 * @param binaryData The span containing the binary message.
		 * @return An instance of the derived message class.
		 */
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
	const bool Message<Derived, TypeVal, FieldTypes...>::registered = MessageMaker::Register(
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
		const uint32_t id;    ///< A unique identifier for the target.
		const bool     valid; ///< Flag indicating if the target data is valid.
		const uint16_t x;     ///< The x-coordinate of the target.
		const uint16_t y;     ///< The y-coordinate of the target.
		const uint16_t z;     ///< The z-coordinate of the target.

	public:
		/**
		 * @brief Constructs a new Target message.
		 * @param id A unique identifier for the target.
		 * @param valid Flag indicating if the target data is valid.
		 * @param x The x-coordinate of the target.
		 * @param y The y-coordinate of the target.
		 * @param z The z-coordinate of the target.
		 */
		constexpr inline Target(uint32_t id, bool valid, uint16_t x, uint16_t y, uint16_t z) noexcept:
			id(id), valid(valid), x(x), y(y), z(z) {
			assert(registered);
		}

		/**
		 * @brief Encodes the message fields into a character array.
		 * @return A std::array<char, Size()> containing the serialized payload.
		 */
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
		const float    projectile_speed; ///< The initial speed of the projectile.
		const float    turret_height;    ///< The height of the turret from the ground.
		const uint16_t max_speed;        ///< The maximum rotational speed of the motors.
		const uint16_t acceleration;     ///< The acceleration of the motors.

	public:
		/**
		 * @brief Constructs a new Config message.
		 * @param projectile_speed The initial speed of the projectile.
		 * @param turret_height The height of the turret from the ground.
		 * @param max_speed The maximum rotational speed of the motors.
		 * @param acceleration The acceleration of the motors.
		 */
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

		/**
		 * @brief Encodes the message fields into a character array.
		 * @return A std::array<char, Size()> containing the serialized payload.
		 */
		constexpr std::array<char, Size()> encode() const {
			return pack(projectile_speed, turret_height, max_speed, acceleration);
		}
	};

	/**
	 * @brief A message to set the active target source.
	 *
	 * This message is sent to the microcontroller to command it to switch
	 * between different targeting systems like static, radar, or CV.
	 */
	class SetTargetSourceMessage: public Message<SetTargetSourceMessage, 2, TargetSource> {
	public:
		const TargetSource source; ///< The selected target source.

	public:
		/**
		 * @brief Constructs a new SetTargetSourceMessage.
		 * @param source The target source to be set.
		 */
		constexpr inline SetTargetSourceMessage(TargetSource source) noexcept: source(source) { assert(registered); }

		/**
		 * @brief Encodes the message fields into a character array.
		 * @return A std::array<char, Size()> containing the serialized payload.
		 */
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
		const uint16_t x; ///< The x-coordinate of the static target.
		const uint16_t y; ///< The y-coordinate of the static target.
		const uint16_t z; ///< The z-coordinate of the static target.

	public:
		/**
		 * @brief Constructs a new StaticTargetMessage.
		 * @param x The x-coordinate of the static target.
		_           * @param y The y-coordinate of the static target.
		_           * @param z The z-coordinate of the static target.
		 */
		constexpr inline StaticTargetMessage(uint16_t x, uint16_t y, uint16_t z) noexcept: x(x), y(y), z(z) {
			assert(registered);
		}

		/**
		 * @brief Encodes the message fields into a character array.
		 * @return A std::array<char, Size()> containing the serialized payload.
		 */
		constexpr std::array<char, Size()> encode() const { return pack(x, y, z); }
	};

	/**
	 * @brief A message to set the turret's targeting strategy.
	 */
	class SetStrategyMessage: public Message<SetStrategyMessage, 4, TurretStrategy> {
	public:
		const TurretStrategy strategy;

	public:
		constexpr inline SetStrategyMessage(TurretStrategy strategy) noexcept: strategy(strategy) {
			assert(registered);
		}

		constexpr std::array<char, Size()> encode() const { return pack(strategy); }
	};

	/**
	 * @brief A message to set the turret's firing stance.
	 */
	class SetStanceMessage: public Message<SetStanceMessage, 5, TurretStance> {
	public:
		const TurretStance stance;

	public:
		constexpr inline SetStanceMessage(TurretStance stance) noexcept: stance(stance) { assert(registered); }

		constexpr std::array<char, Size()> encode() const { return pack(stance); }
	};

	/**
	 * @brief Concept to identify types that support I/O operations.
	 */
	template <typename T>
	concept IOAble = requires(T io, char* buf, const char* cbuf, size_t count) {
		{ io.readsome(buf, count) } -> std::convertible_to<size_t>;
		{ io.write(cbuf, count) };
		{ io.good() } -> std::convertible_to<bool>;
	};

	/**
	 * @brief A class for serializing messages to a generic output stream.
	 *
	 * @tparam T The type of the output stream, which must satisfy the `IOAble` concept.
	 */
	template <typename T>
		requires IOAble<T>
	class Serializer {
	private:
		T& output;

	public:
		/**
		 * @brief Constructs a Serializer.
		 * @param outputStream The output stream to write to.
		 */
		Serializer(T& outputStream): output(outputStream) {};

		/**
		 * @brief Writes a message to the output stream.
		 *
		 * This method serializes the given message into its binary format and
		 * writes the result to the stream.
		 *
		 * @tparam M The derived message class.
		 * @tparam U The unique message type code.
		 * @tparam Fs The field types of the message.
		 * @param message The message object to serialize and write.
		 */
		template <typename M, uint8_t U, typename... Fs>
		void Write(const Message<M, U, Fs...>& message) {
			auto binaryMessage = message.ToBinary();
			output.write(binaryMessage.data(), binaryMessage.size());
		}
	};

	/**
	 * @brief A class for deserializing messages from a generic input stream.
	 *
	 * This class implements a state machine to parse a stream of characters,
	 * identify message frames (delimited by `magicHead` and `magicFoot`), and
	 * deserialize the enclosed payload into a message object using the `MessageMaker`
	 * factory. It is designed to handle continuous, asynchronous data streams.
	 *
	 * @tparam T The type of the input stream, which must satisfy the `IOAble` concept.
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
		/**
		 * @brief Constructs a Deserializer.
		 * @param readStream The input stream to read from.
		 */
		Deserializer(T& readStream): input(readStream) {};

		/**
		 * @brief Parses the input stream and invokes a callback for each complete message found.
		 *
		 * This method continuously reads from the stream, searches for message frames
		 * (delimited by magic numbers), and deserializes them.
		 *
		 * @tparam Type The base type of the messages to expect (e.g., BasePacket).
		 * @param callback A function to be called with each successfully parsed message.
		 */
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
				case EMIT: {
					uint8_t typeCode = buf.begin()[2];
					char*   message_end_ptr = offset + sizeof(footer_bytes);
					auto    found = MessageMaker::Create(typeCode, std::span(buf.begin(), message_end_ptr));

					callback(found);

					// Shift the remaining buffer content to the beginning
					auto remaining_size = end_offset - message_end_ptr;
					std::copy(message_end_ptr, end_offset, buf.begin());

					// Reset pointers and state for the next message
					offset = buf.begin();
					end_offset = buf.begin() + remaining_size;
					state = START;
					token = header_bytes;
					success = PRE_END;
					fail = START;
					continue;
				}
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
	 * @brief A class for handling both serialization and deserialization on a single stream.
	 *
	 * This class provides a convenient wrapper around a Serializer and a Deserializer,
	 * allowing for easy reading and writing of messages from the same IOAble stream.
	 *
	 * @tparam T The type of the stream, which must satisfy the `IOAble` concept.
	 */
	template <typename T>
		requires IOAble<T>
	class StreamHandler {
	private:
		Serializer<T>   serializer;
		Deserializer<T> deserializer;

	public:
		/**
		 * @brief Constructs a StreamHandler.
		 * @param stream The stream to read from and write to.
		 */
		StreamHandler(T& stream): serializer(stream), deserializer(stream) {};

		/**
		 * @brief Writes a message to the stream.
		 * @see Serializer::Write
		 */
		template <typename M, uint8_t U, typename... Fs>
		void Write(const Message<M, U, Fs...>& message) {
			serializer.Write(message);
		}

		/**
		 * @brief Parses the input stream and invokes a callback for each complete message found.
		 * @see Deserializer::ParseStream
		 */
		template <std::derived_from<BasePacket> Type>
		void ParseStream(std::function<void(std::unique_ptr<Type>&)> callback) {
			deserializer.ParseStream(callback);
		}
	};
} // namespace cerializer