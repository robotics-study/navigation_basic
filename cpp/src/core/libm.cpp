#include "navigation/core/libm.hpp"

#include <dlfcn.h>

#include <cstdio>
#include <cstdlib>

namespace navigation::core {
namespace {

using MathFn = double (*)(double);

MathFn resolve(const char* name) {
  void* sym = dlsym(RTLD_DEFAULT, name);
  if (sym == nullptr) {
    std::fprintf(stderr, "libm: cannot resolve '%s'\n", name);
    std::abort();
  }
  return reinterpret_cast<MathFn>(sym);
}

}  // namespace

// The static init-once pattern is deliberate: dlsym's RESULT is opaque to the
// optimizer (an external call), so the pointer can never be constant-propagated
// back into a direct libcall that Apple clang would re-fold into __sincos_stret.
double libm_sin(double x) {
  static const MathFn fn = resolve("sin");
  return fn(x);
}

double libm_cos(double x) {
  static const MathFn fn = resolve("cos");
  return fn(x);
}

}  // namespace navigation::core
