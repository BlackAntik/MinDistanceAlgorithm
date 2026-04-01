#pragma once

#include "common/macros.h"

#include <bit>
#include <bitset>


namespace sdc::geometry {

    template <size_t max_size>
    class BitsetWrapper {
    public:
        static constexpr size_t max_stored_objects_count = max_size;

        BitsetWrapper() noexcept
            : bits_{}
        {
        }

        explicit BitsetWrapper(const std::bitset<max_size>& bits) noexcept
            : bits_{bits}
        {
        }

        ALWAYS_INLINE inline void add(uint32_t pos) noexcept {
            VERIFY(pos < max_size);
            bits_[pos] = true;
        }

        ALWAYS_INLINE
        inline BitsetWrapper operator|(const BitsetWrapper& object) const noexcept {
            return BitsetWrapper(bits_ | object.bits_);
        }

        ALWAYS_INLINE
        inline BitsetWrapper& operator|=(const BitsetWrapper& object) noexcept {
            bits_ |= object.bits_;
            return *this;
        }

        ALWAYS_INLINE
        inline BitsetWrapper operator&(const BitsetWrapper& object) const noexcept {
            return BitsetWrapper(bits_ & object.bits_);
        }

        ALWAYS_INLINE
        inline BitsetWrapper& operator&=(const BitsetWrapper& object) noexcept {
            bits_ &= object.bits_;
            return *this;
        }

        ALWAYS_INLINE
        inline bool operator==(const BitsetWrapper& object) const noexcept {
            return bits_ == object.bits_;
        }

        template <typename Func>
        ALWAYS_INLINE inline void for_each(Func&& func) const noexcept {
            const uint64_t* data = reinterpret_cast<const uint64_t*>(&bits_);
            constexpr uint64_t and_mask = (uint64_t(-1) << 1);

            for (size_t i = 0; i < (max_size + 63) / 64; ++i) {
                uint64_t w = data[i];
                size_t j = i * 64;
                while (w) {
                    size_t const s = std::countr_zero(w);
                    w &= and_mask << s;
                    func(j + s);
                }
            }
        }

    private:
        std::bitset<max_size> bits_;
    };
} // namespace sdc::geometry
