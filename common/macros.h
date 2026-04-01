#pragma once

#include <exception>
#include <iostream>
#include <sstream>

#define ALWAYS_INLINE __attribute__((always_inline))
#define PURE __attribute__((pure))
#define FLOAT_OPTIMIZE _Pragma("clang fp contract(fast) reassociate(on) exceptions(ignore)")

#define VERIFY(expr) \
    ([](auto&& _v) -> decltype(auto) { \
        if (!_v) std::terminate(); \
        return static_cast<decltype(_v)>(_v); \
    }(expr))

#define VERIFY_STREAM(expr, msg) \
    do { \
        if (!(expr)) { \
            std::ostringstream _sdc_oss; \
            _sdc_oss << msg; \
            std::cerr << "VERIFY_STREAM failed: " << #expr \
                      << "\n  " << _sdc_oss.str() \
                      << "\n  at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::terminate(); \
        } \
    } while (false)

