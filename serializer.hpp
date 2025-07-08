#pragma once
#include <stdint.h>
#include <array>
#include <bit>
#include <cstring>
#include <span>

const uint16_t magicHead = 0xCAFE;
const uint16_t magicFoot = 0xFACE;

namespace cerializer
{
	template <typename T>
	struct is_std_array : std::false_type
	{
	};
	template <typename T, std::size_t N>
	struct is_std_array<std::array<T, N>> : std::true_type
	{
	};
	template <typename T>
	constexpr bool is_std_array_v = is_std_array<T>::value;

	template <class T>
	concept Indexable = requires {
		requires std::is_array_v<T> || is_std_array_v<T>;
	};

	template <class T, typename V = nullptr_t, typename N = int>
	concept Container = requires(T obj, V, N idx) {
		typename T::value_type;
		{ obj.data() } -> std::same_as<std::conditional_t<!std::is_same_v<V, nullptr_t>, V, typename T::value_type> *>;
		{ obj[idx] } -> std::same_as<std::conditional_t<!std::is_same_v<V, nullptr_t>, V, typename T::value_type> &>;
	};

	template <std::integral T, std::integral Y>
	constexpr inline std::common_type<T, Y>::type postfixAdd(T &initial, const Y &add)
	{
		T x = initial;
		initial += add;
		return x;
	}

	template <typename... Ts>
		requires(std::is_trivially_copyable_v<Ts> && ...)
	constexpr inline std::array<std::byte, (sizeof(Ts) + ...)> pack(const Ts &...args)
	{
		std::array<std::byte, (sizeof(Ts) + ...)> dest;
		auto offset = 0;
		([&]()
		 {
			if constexpr ((std::endian::native == std::endian::big) && sizeof(Ts) > 1 && !Indexable<Ts>) {
				auto bits = std::bit_cast<std::array<std::byte, sizeof(Ts)>>(args);
				std::copy(bits.rbegin(), bits.rend(), dest.data()+offset);
			}
			else {
				std::memcpy(dest.data() + offset, &args, sizeof(Ts));
			}
			offset += sizeof(Ts); }(), ...);
		return dest;
	};

	template <typename Dest, typename... Ts, typename Cont>
		requires
			(std::is_trivially_copyable_v<Ts> && ...)
			&& (std::is_trivially_constructible_v<Ts> && ...)
			&& (std::is_constructible_v<Dest, Ts...>)
			&& Container<Cont, std::byte>
	constexpr inline Dest unpack(const Cont &binaryData)
	{
		if (sizeof...(Ts) < 1)
		{
			return *reinterpret_cast<Dest *>(binaryData.data());
		}
		auto count = 0;
		std::span<std::byte> dataView(binaryData);
		return Dest{
			*reinterpret_cast<Ts *>(
				dataView.subspan(
							postfixAdd(
								count, sizeof(Ts)),
							sizeof(Ts))
					.data())...};
	};

	template <typename T>
		requires std::is_trivially_copyable_v<T>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize()
	{
		return {ceil(log10(sizeof(T))), sizeof(T), 'P'};
	}

	template <typename T>
		requires std::is_bounded_array_v<T> && std::same_as<char, std::remove_all_extents_t<T>>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize()
	{
		return {ceil(log10(sizeof(T))), sizeof(T), 's'};
	}

	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<char>()
	{
		return {1, 1, 'c'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<signed char>()
	{
		return {1, 1, 'b'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<unsigned char>()
	{
		return {1, 1, 'B'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<bool>()
	{
		return {1, 1, '?'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<short>()
	{
		return {1, 2, 'h'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<unsigned short>()
	{
		return {1, 2, 'H'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<int>()
	{
		return {1, 4, 'i'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<unsigned int>()
	{
		return {1, 4, 'I'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<long>()
	{
		return {1, 4, 'l'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<unsigned long>()
	{
		return {1, 4, 'L'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<long long>()
	{
		return {1, 8, 'q'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<unsigned long long>()
	{
		return {1, 8, 'Q'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<float>()
	{
		return {1, 4, 'f'};
	}
	template <>
	constexpr std::tuple<uint8_t, uint8_t, char> formatSize<double>()
	{
		return {1, 8, 'd'};
	}

	template <typename... Ts>
		requires(std::is_trivially_copyable_v<Ts> && ...)
	constexpr inline std::array<std::byte, 1 + (std::get<0>(formatSize<Ts>()) + ...)> renderFormat()
	{
		return std::array<std::byte, 1 + (std::get<0>(formatSize<Ts>()) + ...)>{std::byte('<'), std::byte(std::get<2>(formatSize<Ts>()))...};
	}

	template <typename Derived, uint8_t TypeVal, typename... FieldTypes>
	class Message
	{
	public:
		inline constexpr static uint8_t Type() { return TypeVal; };
		inline constexpr static unsigned int Size() { return (sizeof(FieldTypes) + ...); };
		constexpr inline static std::array<std::byte, 4 + (std::get<0>(formatSize<FieldTypes>()) + ...)> Format()
		{
			return renderFormat<uint16_t, uint8_t, FieldTypes..., uint16_t>();
		};
		constexpr std::array<std::byte, (sizeof(FieldTypes) + ...) + 2 * sizeof(uint16_t) + sizeof(uint8_t)> ToBinary() const
		{
			auto encodedData = static_cast<const Derived *>(this)->encode();
			return pack(magicHead, Type(), encodedData, magicFoot);
		};

		static Derived LoadBinary(std::array<std::byte, (sizeof(FieldTypes) + ...) + 2 * sizeof(uint16_t) + sizeof(uint8_t)> &binaryData)
		{
			std::span<std::byte> dataView(binaryData);
			uint16_t headCheck = unpack<uint16_t>(dataView.first(sizeof(magicHead)));
			uint16_t footCheck = unpack<uint16_t>(dataView.last(sizeof(footCheck)));

			assert(headCheck == magicHead);
			assert(footCheck == magicFoot);

			auto body = std::span(binaryData.begin() + sizeof(uint16_t) + sizeof(uint8_t), Size());
			return unpack<Derived, FieldTypes...>(dataView.subspan(sizeof(uint16_t) + sizeof(uint8_t), Size()));
		}
	};

	class Target : public Message<Target, 0, uint32_t, bool, uint16_t, uint16_t, uint16_t>
	{
	private:
		const uint32_t id;
		const bool valid;
		const uint16_t x;
		const uint16_t y;
		const uint16_t z;

	public:
		constexpr inline Target(uint32_t id, bool valid, uint16_t x, uint16_t y, uint16_t z) noexcept : id(id), valid(valid), x(x), y(y), z(z) {}

		constexpr std::array<std::byte, Size()> encode() const
		{
			return pack(id, valid, x, y, z);
		}
	};

	class Serializer
	{
	public:
		void Write();
	};

	class Deserializer
	{
	public:
	};

	class StreamHandler // This one should have a callback for what to do when it deserializes a message
	{
	public:
	};
}