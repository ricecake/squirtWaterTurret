#ifndef DOCTEST_FPM_HELPER_H
#define DOCTEST_FPM_HELPER_H

#include "doctest.h"
#include "fpm/fixed.hpp"
#include "fpm/ios.hpp" // Required for the ostream operator<< overload
#include <sstream>

// This namespace-level specialization tells doctest how to print fpm::fixed types
namespace doctest {
    template<typename I, typename L, unsigned int F, bool S>
    struct StringMaker<fpm::fixed<I, L, F, S>> {
        static String convert(const fpm::fixed<I, L, F, S>& value) {
            std::stringstream ss;
            ss << value; // Use the existing ostream overload from fpm/ios.hpp
            return ss.str().c_str();
        }
    };
}

#endif // DOCTEST_FPM_HELPER_H
