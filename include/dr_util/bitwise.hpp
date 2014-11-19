#pragma once
#include <stdint.h>

namespace dr {
	constexpr uint32_t bitMask(unsigned int bit) { return 1 << bit; };

	constexpr bool testBit(uint32_t values, unsigned int bit) { return values & bitMask(bit); };

	constexpr bool testRising(uint32_t values, uint32_t old, unsigned int bit) { return (values & bitMask(bit)) && !(old & bitMask(bit)); };
};
