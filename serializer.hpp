#pragma once
#include <stdint.h>
#include <vector>
#include <cstddef>
#include <array>
#include <bit>
#include <cstring>
#include <concepts>

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

	template <typename... Ts>
		requires(std::is_trivially_copyable_v<Ts> && ...)
	constexpr inline std::array<std::byte, (sizeof(Ts) + ...)> pack(const Ts &...args)
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

	template <typename Derived, uint8_t TypeVal, unsigned int SizeVal>
	class Message
	{
	public:
		inline constexpr uint8_t Type() const { return TypeVal; };
		inline constexpr unsigned int Size() const { return SizeVal; };
		constexpr std::array<std::byte, SizeVal + 2 * sizeof(uint16_t) + sizeof(uint8_t)> ToBinary() const
		{
			auto encodedData = static_cast<const Derived *>(this)->encode();
			return pack(magicHead, Type(), encodedData, magicFoot);
		};
		Derived LoadBinary(const std::array<std::byte, SizeVal> & binaryData);
	};

	class Target : public Message<Target, 0, 11>
	{
	private:
		const uint32_t id;
		const bool valid;
		const uint16_t x;
		const uint16_t y;
		const uint16_t z;

	public:
		constexpr inline Target(uint32_t id, bool valid, uint16_t x, uint16_t y, uint16_t z) noexcept : id(id), valid(valid), x(x), y(y), z(z) {}

		constexpr std::array<std::byte, 11> encode() const
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