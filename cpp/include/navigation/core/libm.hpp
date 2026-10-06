#pragma once

// Scalar libm sin/cos — routed through dlsym-resolved function pointers, and WHY
// (this is part of the bit-identical cross-language contract): Apple clang's
// optimizer folds same-argument (sin, cos) call pairs into __sincos_stret, a SIMD
// sincos whose results differ from the scalar sin/cos that CPython's math.sin /
// math.cos call by 1 ulp on rare inputs (~0.4% of angles; verified:
// sin(0x1.f9cbc4269ab30p-2) differs between the two variants). A dlsym-resolved
// pointer is opaque to the optimizer — the call stays an indirect call to
// libSystem's scalar implementation, the same code path Python takes. atan2 and
// log need no such routing (no pair fold exists for them; they bind directly to
// _atan2 / _log); sqrt and floor are hardware instructions.

namespace navigation::core {

double libm_sin(double x);
double libm_cos(double x);

}  // namespace navigation::core
