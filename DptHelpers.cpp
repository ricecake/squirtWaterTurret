/**
 * @file DptHelpers.cpp
 * @brief Implementation of DPT (Dynamic Positioning and Tracking) helper functions.
 *
 * This file is intended to house implementations of helper utilities for the DPT system.
 * It is currently a placeholder and will be populated with functionality in future development.
 */

#ifdef ARDUINO
	#include "HardwareSerial.h"
	#include "esp32-hal-gpio.h"
	#include "esp_timer.h"

	#include <Arduino.h>
#endif

#include "DptHelpers.h"

#include <climits>
#include <stdint.h>
