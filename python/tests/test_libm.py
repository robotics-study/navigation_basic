"""Scalar libm goldens — the cross-language bit-identical contract for sin/cos.

math.sin/math.cos ARE the scalar libSystem implementations; that is what C++ must
reach too. Apple clang folds same-argument (sin, cos) call pairs into
__sincos_stret, whose SIMD result differs from the scalar one by 1 ulp on rare
inputs — C++ therefore routes every sin/cos through dlsym-resolved pointers
(core/libm). This input is a KNOWN divergence point: the pinned values here are
what the scalar variant returns and what the SIMD fold does NOT (the C++
test_libm pins the same constants through libm_sin/libm_cos).
"""

import math


def test_libm_scalar_golden_at_divergent_input() -> None:
    x = float.fromhex("0x1.f9cbc4269ab30p-2")
    assert math.sin(x) == float.fromhex("0x1.e57a6c8be62efp-2")
    assert math.cos(x) == float.fromhex("0x1.c2cd1ba67a4fdp-1")
