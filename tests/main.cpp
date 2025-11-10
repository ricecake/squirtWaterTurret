#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <iostream>
#include <ostream>

#include "doctest/doctest.h"
#include "doctest_fpm_adapter.hpp"
#include "mock_time.h"

TestClock mock_clock;
