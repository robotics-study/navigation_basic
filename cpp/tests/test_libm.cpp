// The scalar-libm contract: these constants ARE the cross-language bit-identical
// pin — identical to the Python goldens in python/tests/test_libm.py; if a golden
// changes, every language mirror breaks on purpose.

#include <cstdlib>

#include <gtest/gtest.h>

#include "navigation/core/libm.hpp"

TEST(Libm, ScalarGoldenAtDivergentInput) {
  // Apple clang folds same-argument (sin, cos) call pairs into __sincos_stret — a
  // SIMD variant that differs from the scalar libSystem sin/cos (what Python's
  // math.sin/math.cos call) by 1 ulp on rare inputs. libm_sin/libm_cos route
  // through dlsym-resolved pointers so no fold can happen; this input is a KNOWN
  // divergence point — code calling std::sin/std::cos as a pair fails these
  // goldens (the Python test_libm pins the same values for math.sin/math.cos).
  double x = std::strtod("0x1.f9cbc4269ab30p-2", nullptr);
  EXPECT_DOUBLE_EQ(navigation::core::libm_sin(x), std::strtod("0x1.e57a6c8be62efp-2", nullptr));
  EXPECT_DOUBLE_EQ(navigation::core::libm_cos(x), std::strtod("0x1.c2cd1ba67a4fdp-1", nullptr));
}
