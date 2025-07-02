#pragma once
#include <stdint.h>
// #include <cstddef>
#include <array>
#include <bit>
#include <cstring>
// #include <concepts>
// #include <ranges>
#include <span>

#include <iostream>

const uint16_t magicHead = 0xCAFE;
const uint16_t magicFoot = 0xFACE;

namespace cerializer
{
	// Custom type trait to check for std::array
	template <typename T>
	struct is_std_array : std::false_type
	{
	};

	template <typename T, std::size_t N>
	struct is_std_array<std::array<T, N>> : std::true_type
	{
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
	constexpr inline typename std::array<std::byte, (sizeof(Ts) + ...)> pack(const Ts &...args)
	{
		std::array<std::byte, (sizeof(Ts) + ...)> dest;
		auto offset = 0;
		([&]()
		 {
			if constexpr ((std::endian::native == std::endian::big) && sizeof(Ts) > 1 && !is_std_array<Ts>::value) {
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
		requires(std::is_trivially_copyable_v<Ts> && ...) && (std::is_trivially_constructible_v<Ts> && ...)
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

	template <typename Derived, uint8_t TypeVal, typename... FieldTypes>
	class Message
	{
	public:
		inline constexpr static uint8_t Type() { return TypeVal; };
		inline constexpr static unsigned int Size() { return (sizeof(FieldTypes) + ...); };
		constexpr std::array<std::byte, (sizeof(FieldTypes) + ...) + 2 * sizeof(uint16_t) + sizeof(uint8_t)> ToBinary() const
		{
			auto encodedData = static_cast<const Derived *>(this)->encode();
			return pack(magicHead, Type(), encodedData, magicFoot);
		};

		static Derived LoadBinary(std::array<std::byte, (sizeof(FieldTypes) + ...) + 2 * sizeof(uint16_t) + sizeof(uint8_t)> &binaryData)
		{
			std::span<std::byte> dataView(binaryData);
			uint16_t headCheck = unpack<uint16_t>(dataView.first(sizeof(magicHead)));
			uint16_t footCheck = unpack<uint16_t, uint16_t>(dataView.last(sizeof(footCheck)));

			std::cout << headCheck << "\t" << magicHead << std::endl;
			std::cout << footCheck << "\t" << magicFoot << std::endl;
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