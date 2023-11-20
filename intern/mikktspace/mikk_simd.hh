#pragma once

namespace mikk {

/* Only functions using v8sf require AVX2 (and some of those only require AVX).
 * The rest requires at most SSE4.1, which has been included in all x86 CPUs since
 * 2013 or so, and may be assumed to exist.
 *
 * We must separately test for AVX2 CPU support at runtime, becase Intel
 * was still making non-AVX2 CPUs as late as 2020. */

#ifdef __GNUC__
#  pragma GCC push_options
#  pragma GCC target("avx2", "sse4.1")
#endif

#ifdef __GNUC__
typedef float v4sf __attribute__((vector_size(16)));
typedef float v8sf __attribute__((vector_size(32)));

inline v4sf to_float4(const float3 &x)
{
  return v4sf{x.x, x.y, x.z, 0.0f};
}

inline v4sf to_float4_masked(const float3 &x)
{
  typedef int v4si __attribute__((vector_size(16)));
  return __builtin_ia32_maskloadps((const v4sf *)&x, v4si{-1, -1, -1, 0});
}

inline float3 to_float3(v4sf x)
{
  return float3(x[0], x[1], x[2]);
}
inline v8sf from_2xv4sf(v4sf a, v4sf b)
{
  v8sf a2 = __builtin_ia32_ps256_ps(a);
  return __builtin_ia32_vinsertf128_ps256(a2, b, 1);
}

inline v8sf from_v4sf(v4sf a)
{
  v8sf a2 = __builtin_ia32_ps256_ps(a);
  return __builtin_ia32_vinsertf128_ps256(a2, a, 1);
}

inline v4sf v4sf_set(float x)
{
  return v4sf{x, x, x, x};
}
inline v4sf v4sf_setzero()
{
  return v4sf{0, 0, 0, 0};
}
inline v8sf v8sf_set(float x)
{
  return v8sf{x, x, x, x, x, x, x, x};
}
inline v8sf v8sf_setzero()
{
  return v8sf{0, 0, 0, 0, 0, 0, 0, 0};
}
inline v8sf v8sf_set2(float x, float y)
{
  return v8sf{x, x, x, x, y, y, y, y};
}

inline float extract(v4sf v, uint x)
{
  return v[x];
}
inline float extract(v8sf v, uint x)
{
  return v[x];
}

inline v4sf insert(v4sf v, int x, float y)
{
  v[x] = y;
  return v;
}
inline v8sf insert(v8sf v, int x, float y)
{
  v[x] = y;
  return v;
}

#elif _MSC_VER

/* Without this line, the compiler may use FMA ops to rewrite float arithmetics.
 * Sometimes this results in slight changes in numerical results.
 *
 * For example,
 * 'a*b-c*d' might compile into  "temp = mul(c*d); out = fmsub(a,b,temp);"
 *
 * Here, temp is truncated to float32, but fmsub does everything internally in
 * float64 precision, so executing this for a=c and b=d no longer produces a zero
 *   (but something on the order of |ab| * 10^-13).
 *
 * This would break our reference compatibility, because, e.g., the output of initTriangle()
 * changes fundamentally if some internal values go from exactly zero to almost zero. */
#  pragma fp_contract(off)

typedef __m128 v4sf;
typedef __m128i v4si;
typedef __m256 v8sf;

inline v4sf to_float4(const float3 &x)
{
  return _mm_set_ps(0, x.z, x.y, x.x);
}
inline v4sf to_float4_masked(const float3 &x)
{
  return _mm_maskload_ps(&x.x, _mm_set_epi32(0, -1, -1, -1));
}

inline float3 to_float3(v4sf x)
{
  return float3(x.m128_f32[0], x.m128_f32[1], x.m128_f32[2]);
}
inline v8sf from_2xv4sf(v4sf a, v4sf b)
{
  return _mm256_set_m128(b, a);
}

inline v8sf from_v4sf(v4sf a)
{
  return _mm256_set_m128(a, a);
}

inline v4sf v4sf_set(float x)
{
  return _mm_set1_ps(x);
}
inline v4si v4si_set(int x)
{
  return _mm_set1_epi32(x);
}
inline v4sf v4sf_setzero()
{
  return _mm_setzero_ps();
}
inline v8sf v8sf_setzero()
{
  return _mm256_setzero_ps();
}
inline v8sf v8sf_set(float x)
{
  return _mm256_set1_ps(x);
}
inline v8sf v8sf_set2(float x, float y)
{
  return _mm256_set_ps(y, y, y, y, x, x, x, x);
}

#  define __builtin_ia32_dpps _mm_dp_ps
#  define __builtin_ia32_hsubps _mm_hsub_ps
#  define __builtin_ia32_maxss _mm_max_ss
#  define __builtin_ia32_maxps _mm_max_ps
#  define __builtin_ia32_minps _mm_min_ps
#  define __builtin_ia32_mulps _mm_mul_ps
#  define __builtin_ia32_shufps _mm_shuffle_ps
#  define __builtin_ia32_cmpltps _mm_cmplt_ps
#  define __builtin_ia32_andps _mm_and_ps
#  define __builtin_ia32_andnps _mm_andnot_ps
#  define __builtin_ia32_rsqrtss _mm_rsqrt_ss
#  define __builtin_ia32_sqrtps _mm_sqrt_ps
#  define __builtin_ia32_maxps256 _mm256_max_ps
#  define __builtin_ia32_sqrtps256 _mm256_sqrt_ps
#  define __builtin_ia32_dpps256 _mm256_dp_ps
#  define __builtin_ia32_vpermilps256 _mm256_permute_ps
#  define __builtin_ia32_vextractf128_ps256 _mm256_extractf128_ps
#  define __builtin_ia32_vperm2f128_ps256 _mm256_permute2f128_ps
#  define __builtin_ia32_blendps256 _mm256_blend_ps
#  define __builtin_ia32_permvarsf256 _mm256_permutevar8x32_ps
#  define __builtin_ia32_cmpgtps _mm_cmpgt_ps
#  define __builtin_ia32_cmpps256 _mm256_cmp_ps
#  define __builtin_ia32_andps256 _mm256_and_ps
#  define __builtin_ia32_andnps256 _mm256_andnot_ps
#  define __builtin_ia32_haddps256 _mm256_hadd_ps

inline float extract(__m128 v, uint x)
{
  return v.m128_f32[x];
}
inline int extract(__m128i v, uint x)
{
  return v.m128i_i32[x];
}
inline float extract(__m256 v, uint x)
{
  return v.m256_f32[x];
}

inline __m128 insert(__m128 v, int x, float y)
{
  v.m128_f32[x] = y;
  return v;
}
inline __m256 insert(__m256 v, int x, float y)
{
  v.m256_f32[x] = y;
  return v;
}

// GCC provides all these automatically
inline v4sf operator*(v4sf a, v4sf b)
{
  return _mm_mul_ps(a, b);
}
inline v4sf operator*(v4sf a, float b)
{
  return _mm_mul_ps(a, v4sf_set(b));
}
inline v4sf operator+(float a, v4sf b)
{
  return _mm_add_ps(v4sf_set(a), b);
}
inline v4sf operator-(float a, v4sf b)
{
  return _mm_sub_ps(v4sf_set(a), b);
}
inline v8sf operator*(v8sf a, v8sf b)
{
  return _mm256_mul_ps(a, b);
}
inline v4sf operator-(v4sf a, v4sf b)
{
  return _mm_sub_ps(a, b);
}
inline v4sf operator+(v4sf a, v4sf b)
{
  return _mm_add_ps(a, b);
}
inline v4si operator+(v4si a, v4si b)
{
  return _mm_add_epi32(a, b);
}
inline v8sf operator+(v8sf a, v8sf b)
{
  return _mm256_add_ps(a, b);
}
inline v8sf operator-(v8sf a, v8sf b)
{
  return _mm256_sub_ps(a, b);
}
inline v8sf operator/(v8sf a, v8sf b)
{
  return _mm256_div_ps(a, b);
}
inline v4sf operator/(float a, v4sf b)
{
  return _mm_div_ps(v4sf_set(a), b);
}
inline v8sf operator/(float a, v8sf b)
{
  return _mm256_div_ps(v8sf_set(a), b);
}
inline void operator+=(v8sf &a, v8sf b)
{
  a = _mm256_add_ps(a, b);
}
inline v4si operator*(v4si a, int b)
{
  return _mm_mul_epi32(a, v4si_set(b));
}
inline v4si operator^(v4si a, v4si b)
{
  return _mm_xor_si128(a, b);
}

#endif

inline v4sf to_float4(v4sf x)
{
  return x;
}

inline void unpack(v4sf &p0, v4sf &p1, v8sf v)
{
  p0 = __builtin_ia32_vextractf128_ps256(v, 0);
  p1 = __builtin_ia32_vextractf128_ps256(v, 1);
}

template<int i> inline v4sf bcast(v4sf x)
{
  return __builtin_ia32_shufps(x, x, i * 0x55);
}

inline float dot(const v4sf &a, const v4sf &b)
{
  return extract(__builtin_ia32_dpps(a, b, 255), 0);
}

inline v8sf dot_single(v8sf a, v8sf b)
{
  return __builtin_ia32_dpps256(a, b, 0xFF);
}

inline float dot_inner(const v8sf &a)
{
  return extract(dot_single(a, __builtin_ia32_vperm2f128_ps256(a, a, 1)), 0);
}

/* Potentially somewhat expensive (creates a global constant and adds a memory access, though the
 * constant may be expected to stay in cache), use sparingly. Sometimes unavoidable since few AVX2
 * ops send data across the 128-bit boundary */
inline v8sf cross_lane_permute(
    v8sf v, int i0, int i1, int i2, int i3, int i4, int i5, int i6, int i7)
{
#if __GNUC__
  typedef int v8si __attribute__((vector_size(32)));
  v8si perm_mask{i0, i1, i2, i3, i4, i5, i6, i7};
  return __builtin_ia32_permvarsf256(v, perm_mask);
#else
  return __builtin_ia32_permvarsf256(v, _mm256_set_epi32(i7, i6, i5, i4, i3, i2, i1, i0));
#endif
}

/* input: [... i j ..... ], with <i> in position <offset> and <j> in position <offset+1>
 * output: [ i i i i j j j j ] */
template<int offset> inline v8sf unpack4x(v8sf a)
{
  return cross_lane_permute(
      a, offset, offset, offset, offset, offset + 1, offset + 1, offset + 1, offset + 1);
}

inline float length_squared(const v4sf &x)
{
  return dot(x, x);
}
inline float length(const v4sf &x)
{
  v4sf v = __builtin_ia32_dpps(x, x, 255);
  v = __builtin_ia32_sqrtps(v);
  return extract(v, 0);
}

inline float inv_length(const v4sf &x)
{
  v4sf v = __builtin_ia32_dpps(x, x, 255);
  v4sf one = v4sf_set(1.0f), zero = v4sf_setzero();
  v4sf rv = 1.0f / __builtin_ia32_sqrtps(v);
  v4sf nonzero = __builtin_ia32_cmpgtps(v, zero);
  rv = __builtin_ia32_andps(nonzero, rv) + __builtin_ia32_andnps(nonzero, one);
  return extract(rv, 0);
}

/* input: 8 float4's packed into 4 v8sf
 * output: 8 floats containing inverse lengths of each */
inline v8sf inv_length8(v8sf px0, v8sf px1, v8sf px2, v8sf px3)
{
  v8sf v;
  v = __builtin_ia32_dpps256(px0, px0, 0xF1);
  v += __builtin_ia32_dpps256(px1, px1, 0xF2);
  v += __builtin_ia32_dpps256(px2, px2, 0xF4);
  v += __builtin_ia32_dpps256(px3, px3, 0xF8);

  /* v is [0 2 4 6 1 3 5 7], and we want [0 1 2 3 4 5 6 7] */
  v = cross_lane_permute(v, 0, 4, 1, 5, 2, 6, 3, 7);

  /* There is an intrinsic for rsqrt, but it has relative accuracy of ~1/1000 */

  v8sf one = v8sf_set(1.0f), zero = v8sf_setzero();
  v8sf nonzero = __builtin_ia32_cmpps256(v, zero, 4);
  v8sf rv = 1.0f / __builtin_ia32_sqrtps256(v);
  /* we must match reference behavior (return 1 if input is zero) or else
   * we'll have significant mismatches in some situations */
  return __builtin_ia32_andps256(nonzero, rv) + __builtin_ia32_andnps256(nonzero, one);
}

/* Projects v onto the surface with normal n. */
inline v4sf project1(v4sf n, v4sf v)
{
  return v - (n * dot(n, v));
}

/* Projects two packed float3's in v8 onto surfaces in n8. */
inline v8sf project2(v8sf n8, v8sf v8)
{
  v8sf d8 = dot_single(n8, v8);
  return v8 - n8 * d8;
}

inline v4sf fast_acosf_4x(v4sf x)
{
  v4sf f = __builtin_ia32_maxps(x, x * -1.0f);
  v4sf one = v4sf_set(1.0f), zero = v4sf_setzero();
  v4sf m = __builtin_ia32_minps(f, one);
  v4sf a = __builtin_ia32_sqrtps(one - m) *
           (1.5707963267f + m * (-0.213300989f + m * (0.077980478f + m * -0.02164095f)));

  v4sf sign = __builtin_ia32_cmpltps(x, zero);
  return __builtin_ia32_andps(sign, 3.1415926535897932f - a) + __builtin_ia32_andnps(sign, a);
}

#ifdef __GNUC__
#  pragma GCC pop_options
#endif

}  // namespace mikk
