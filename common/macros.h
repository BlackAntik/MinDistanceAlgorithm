#pragma once

#include <exception>

#define ALWAYS_INLINE __attribute__((always_inline))
#define PURE __attribute__((pure))
#define FLOAT_OPTIMIZE _Pragma("clang fp contract(fast) reassociate(on) exceptions(ignore)")

#define VERIFY(expr) \
    ([](auto&& _v) -> decltype(auto) { \
        if (!_v) std::terminate(); \
        return static_cast<decltype(_v)>(_v); \
    }(expr))

