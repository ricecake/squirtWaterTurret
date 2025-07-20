#pragma once
#include <stdint.h>
#include <array>
#include <bit>
#include <cstring>
#include <span>
#include <tuple>
#include <iostream>
#include <memory>
#include <map>

namespace cerializer
{
	const uint16_t magicHead = 0xCAFE;
	const uint16_t magicFoot = 0xFACE;

	std::string hexify(auto data)
	{
		std::stringstream out;
		for (char c : data)
		{
			out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(c));
		}
		return out.str();
	}

	template <typename T>
	constexpr auto toCharArray(const T &thing)
	{
		return std::bit_cast<std::array<char, sizeof(T)>>(thing);
	}

	enum ParseMode
	{
		START,
		PRE_END,
		END,
		EMIT,
	};

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
	constexpr inline std::array<char, (sizeof(Ts) + ...)> pack(const Ts &...args)
	{
		std::array<char, (sizeof(Ts) + ...)> dest;
		auto offset = 0;
		([&]()
		 {
			if constexpr ((std::endian::native == std::endian::big) && sizeof(Ts) > 1 && !Indexable<Ts>) {
				auto bits = std::bit_cast<std::array<char, sizeof(Ts)>>(args);
				std::copy(bits.rbegin(), bits.rend(), dest.data()+offset);
			}
			else {
				std::memcpy(dest.data() + offset, &args, sizeof(Ts));
			}
			offset += sizeof(Ts); }(), ...);
		return dest;
	};

	template <typename Dest, typename... Ts, typename Cont>
		requires(std::is_trivially_copyable_v<Ts> && ...)
				// && (std::is_trivially_constructible_v<Ts> && ...)
				// && (std::is_constructible_v<Dest, Ts...>)
				&& Container<Cont, char>
	constexpr inline Dest unpack(const Cont &binaryData)
	{
		if (sizeof...(Ts) < 1)
		{
			return *reinterpret_cast<Dest *>(binaryData.data());
		}
		auto count = 0;
		std::span<char> dataView(binaryData);
		return Dest{
			*reinterpret_cast<Ts *>(
				dataView.subspan(
							postfixAdd(
								count, sizeof(Ts)),
							sizeof(Ts))
					.data())...};
	};

	template <typename T>
		requires std::is_trivially_copyable_v<T> && (!std::is_bounded_array_v<T>)
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
	constexpr inline std::array<char, 1 + (std::get<0>(formatSize<Ts>()) + ...)> renderFormat()
	{
		return std::array<char, 1 + (std::get<0>(formatSize<Ts>()) + ...)>{char('<'), char(std::get<2>(formatSize<Ts>()))...};
	}

	// using creatorFunc = void(*)(void);
	// using infoFunc = std::tuple<uint8_t, creatorFunc>(*)(void);
	// infoFunc creators[32];

	// class Registry
	// {
	// 	public:
	// 	constexpr Registry() {
	// 		 bool registered = register_class();
	// 	}
	// 	protected:
	// 	static bool register_class() {
	// 		auto n= [] (const auto& ... args) -> Base { return Base((args, ...)); };
	// 	creators[Derived::Type()] =

	// 		[] (const auto& ... args) -> Derived { return std::make_unique<Derived>((args, ...)); };
	// 		return true;
	// 	}
	// };

	// class Base {
	// 	public:
	// 	// std::map<uint8_t, std::function<Base(auto&...)>> registry;
	// };

	template <typename Derived, uint8_t TypeVal, typename... FieldTypes>
	class Message
	{
	public:
		// static constexpr uint8_t TypeCode = TypeVal;
		constexpr static uint8_t Type() { return TypeVal; };
		constexpr static unsigned int Size() { return (sizeof(FieldTypes) + ...); };
		constexpr static std::array<char, 4 + (std::get<0>(formatSize<FieldTypes>()) + ...)> Format()
		{
			return renderFormat<uint16_t, uint8_t, FieldTypes..., uint16_t>();
		};
		constexpr std::array<char, (sizeof(FieldTypes) + ...) + 2 * sizeof(uint16_t) + sizeof(uint8_t)> ToBinary() const
		{
			auto encodedData = static_cast<const Derived *>(this)->encode();
			return pack(magicHead, Type(), encodedData, magicFoot);
		};

		constexpr static Derived LoadBinary(std::array<char, (sizeof(FieldTypes) + ...) + 2 * sizeof(uint16_t) + sizeof(uint8_t)> &binaryData)
		{
			return LoadBinary(std::span(binaryData.begin(), binaryData.end()));
		}
		constexpr static Derived LoadBinary(const std::span<char> &binaryData)
		{
			std::span<char> dataView(binaryData);
			uint16_t headCheck = unpack<uint16_t>(dataView.first(sizeof(magicHead)));
			uint16_t footCheck = unpack<uint16_t>(dataView.last(sizeof(footCheck)));

			assert(headCheck == magicHead);
			assert(footCheck == magicFoot);

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
		constexpr std::array<char, Size()> encode() const
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
		std::istream input;
		std::array<char, 128> buf;
		char *offset = buf.begin();
		char *end_offset = buf.begin();

		constexpr auto findToken(const std::span<char> &buffer, const std::span<char> &value, const ParseMode &fail_state, const ParseMode &success_state)
		{
			auto next_state = fail_state;
			auto next_size = value.size();
			auto next_offset = buffer.begin();

			if (buffer.size() < value.size())
			{
				next_size = value.size();
				// next_size += value.size() - buffer.size();
			}
			else if (buffer.size() >= value.size())
			{
				auto index = std::search(buffer.begin(), buffer.end(), value.begin(), value.end());
				if (index == buffer.end())
				{
					next_offset = buffer.end() - (value.size() - 1);
				}
				else
				{
					next_offset = index;
					next_state = success_state;
				}
			}

			return std::tuple{next_state, next_size, next_offset};
		}

	public:
		auto ParseStream(std::istream &input)
		{
			constexpr const auto header_bytes = toCharArray(magicHead);
			constexpr const auto footer_bytes = toCharArray(magicFoot);
			auto state = START;
			auto read_size = header_bytes.size();
			auto token = header_bytes;
			auto success = PRE_END;
			auto fail = START;

			auto read = 0;

			read = input.readsome(end_offset, read_size);
			end_offset += read;

			auto result = findToken(std::span(offset, end_offset), std::span(token), fail, success);
			state = std::get<0>(result);
			read_size = std::get<1>(result);
			offset = std::get<2>(result).base();

			while (input.good() && !input.eof())
			{
				switch (state)
				{
				case START:
					token = header_bytes;
					success = PRE_END;
					fail = START;
					if (offset > buf.begin())
					{
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
					if (offset > buf.begin())
					{
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
					switch (offset[2])
					{
					case 0:
						return Target::LoadBinary(std::span(offset, end_offset));
					}
					// // std::cout << "FOUND IT" << hexify(std::span(buf.begin(), end_offset)) << std::endl;
					// offset = buf.begin();
					// end_offset = buf.begin();
					// state = START;
					// continue;
				}

				auto result = findToken(std::span<char>(offset, end_offset), std::span(token), fail, success);
				state = std::get<0>(result);
				read_size = std::get<1>(result);
				offset = std::get<2>(result).base();

				read = input.readsome(end_offset, read_size);
				end_offset += read;
			}
			return Target(1, true, 0, 0, 0);
		}
	};

	class StreamHandler // This one should have a callback for what to do when it deserializes a message
	{
	public:
	};
}
