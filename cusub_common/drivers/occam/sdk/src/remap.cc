/*
Copyright 2011 - 2017 Occam Robotics Inc - All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of Occam Vision Group, Occam Robotics Inc, nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL OCCAM ROBOTICS INC BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// CPU bilinear remap derived from OpenCV 1bdd86edeba4babff7a78586beba20841bb87fa9
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2010, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "remap.h"
#include "system.h"
#include <algorithm>
#include <iostream>
#include <assert.h>
#include <string.h>
#include <math.h>
#undef min
#undef max

static const int REMAP_BORDER_TRANSPARENT = 0;
static const int REMAP_BORDER_CONSTANT = 1;
static const int REMAP_BORDER_REPLICATE = 2;

static const int INTER_RESIZE_COEF_BITS = 11;
static const int INTER_RESIZE_COEF_SCALE = 1 << INTER_RESIZE_COEF_BITS;
static const int INTER_REMAP_COEF_BITS = 15;
static const int INTER_REMAP_COEF_SCALE = 1 << INTER_REMAP_COEF_BITS;

static const int INTER_BITS = 5;
static const int INTER_BITS2 = INTER_BITS * 2;
static const int INTER_TAB_SIZE = 1 << INTER_BITS;
static const int INTER_TAB_SIZE2 = INTER_TAB_SIZE * INTER_TAB_SIZE;

static float BilinearTab_f[INTER_TAB_SIZE2][2][2];
static short BilinearTab_i[INTER_TAB_SIZE2][2][2];

#if OCCAM_SSE2 || OCCAM_NEON
static short BilinearTab_iC4_buf[INTER_TAB_SIZE2+2][2][8];
static short (*BilinearTab_iC4)[2][8] = (short (*)[2][8])occamAlignPtr(BilinearTab_iC4_buf, 16);
#endif

#if OCCAM_SSE2

static int RemapVec_8u(int channels,
		       const uint8_t* srcp, int src_step,
		       void* _dst,
		       const short* XY, const unsigned short* FXY,
		       int width) {
  if ((channels != 1 &&
       channels != 3 &&
       channels != 4) ||
      !occamHardwareSupport(OCCAM_CPU_SSE2) ||
      src_step > 0x8000)
    return 0;

  const unsigned char *S0 = srcp, *S1 = srcp + src_step;
  const short* wtab = channels == 1 ? &BilinearTab_i[0][0][0] : &BilinearTab_iC4[0][0][0];
  unsigned char* D = (unsigned char*)_dst;
  __m128i delta = _mm_set1_epi32(INTER_REMAP_COEF_SCALE/2);
  __m128i xy2ofs = _mm_set1_epi32(channels + (src_step << 16));
  __m128i z = _mm_setzero_si128();
  int OCCAM_DECL_ALIGNED(16) iofs0[4];
  int OCCAM_DECL_ALIGNED(16) iofs1[4];

  int x = 0;
  if (channels == 1) {
    for (; x <= width - 8; x += 8) {
      __m128i xy0 = _mm_loadu_si128((const __m128i*)(XY + x*2));
      __m128i xy1 = _mm_loadu_si128((const __m128i*)(XY + x*2 + 8));
      __m128i v0, v1, v2, v3, a0, a1, b0, b1;
      unsigned i0, i1;

      xy0 = _mm_madd_epi16(xy0, xy2ofs);
      xy1 = _mm_madd_epi16(xy1, xy2ofs);
      _mm_store_si128((__m128i*)iofs0, xy0);
      _mm_store_si128((__m128i*)iofs1, xy1);

      i0 = *(unsigned short*)(S0 + iofs0[0]) + (*(unsigned short*)(S0 + iofs0[1]) << 16);
      i1 = *(unsigned short*)(S0 + iofs0[2]) + (*(unsigned short*)(S0 + iofs0[3]) << 16);
      v0 = _mm_unpacklo_epi32(_mm_cvtsi32_si128(i0), _mm_cvtsi32_si128(i1));
      i0 = *(unsigned short*)(S1 + iofs0[0]) + (*(unsigned short*)(S1 + iofs0[1]) << 16);
      i1 = *(unsigned short*)(S1 + iofs0[2]) + (*(unsigned short*)(S1 + iofs0[3]) << 16);
      v1 = _mm_unpacklo_epi32(_mm_cvtsi32_si128(i0), _mm_cvtsi32_si128(i1));
      v0 = _mm_unpacklo_epi8(v0, z);
      v1 = _mm_unpacklo_epi8(v1, z);

      a0 = _mm_unpacklo_epi32(_mm_loadl_epi64((__m128i*)(wtab+FXY[x]*4)),
			      _mm_loadl_epi64((__m128i*)(wtab+FXY[x+1]*4)));
      a1 = _mm_unpacklo_epi32(_mm_loadl_epi64((__m128i*)(wtab+FXY[x+2]*4)),
			      _mm_loadl_epi64((__m128i*)(wtab+FXY[x+3]*4)));
      b0 = _mm_unpacklo_epi64(a0, a1);
      b1 = _mm_unpackhi_epi64(a0, a1);
      v0 = _mm_madd_epi16(v0, b0);
      v1 = _mm_madd_epi16(v1, b1);
      v0 = _mm_add_epi32(_mm_add_epi32(v0, v1), delta);

      i0 = *(unsigned short*)(S0 + iofs1[0]) + (*(unsigned short*)(S0 + iofs1[1]) << 16);
      i1 = *(unsigned short*)(S0 + iofs1[2]) + (*(unsigned short*)(S0 + iofs1[3]) << 16);
      v2 = _mm_unpacklo_epi32(_mm_cvtsi32_si128(i0), _mm_cvtsi32_si128(i1));
      i0 = *(unsigned short*)(S1 + iofs1[0]) + (*(unsigned short*)(S1 + iofs1[1]) << 16);
      i1 = *(unsigned short*)(S1 + iofs1[2]) + (*(unsigned short*)(S1 + iofs1[3]) << 16);
      v3 = _mm_unpacklo_epi32(_mm_cvtsi32_si128(i0), _mm_cvtsi32_si128(i1));
      v2 = _mm_unpacklo_epi8(v2, z);
      v3 = _mm_unpacklo_epi8(v3, z);

      a0 = _mm_unpacklo_epi32(_mm_loadl_epi64((__m128i*)(wtab+FXY[x+4]*4)),
			      _mm_loadl_epi64((__m128i*)(wtab+FXY[x+5]*4)));
      a1 = _mm_unpacklo_epi32(_mm_loadl_epi64((__m128i*)(wtab+FXY[x+6]*4)),
			      _mm_loadl_epi64((__m128i*)(wtab+FXY[x+7]*4)));
      b0 = _mm_unpacklo_epi64(a0, a1);
      b1 = _mm_unpackhi_epi64(a0, a1);
      v2 = _mm_madd_epi16(v2, b0);
      v3 = _mm_madd_epi16(v3, b1);
      v2 = _mm_add_epi32(_mm_add_epi32(v2, v3), delta);

      v0 = _mm_srai_epi32(v0, INTER_REMAP_COEF_BITS);
      v2 = _mm_srai_epi32(v2, INTER_REMAP_COEF_BITS);
      v0 = _mm_packus_epi16(_mm_packs_epi32(v0, v2), z);
      _mm_storel_epi64( (__m128i*)(D + x), v0 );
    }
  } else if (channels == 3) {
    for (; x <= width - 5; x += 4, D += 12) {
      __m128i xy0 = _mm_loadu_si128( (const __m128i*)(XY + x*2));
      __m128i u0, v0, u1, v1;

      xy0 = _mm_madd_epi16(xy0, xy2ofs);
      _mm_store_si128((__m128i*)iofs0, xy0);
      const __m128i *w0, *w1;
      w0 = (const __m128i*)(wtab + FXY[x]*16);
      w1 = (const __m128i*)(wtab + FXY[x+1]*16);

      u0 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int*)(S0 + iofs0[0])),
			     _mm_cvtsi32_si128(*(int*)(S0 + iofs0[0] + 3)));
      v0 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int*)(S1 + iofs0[0])),
			     _mm_cvtsi32_si128(*(int*)(S1 + iofs0[0] + 3)));
      u1 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int*)(S0 + iofs0[1])),
			     _mm_cvtsi32_si128(*(int*)(S0 + iofs0[1] + 3)));
      v1 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int*)(S1 + iofs0[1])),
			     _mm_cvtsi32_si128(*(int*)(S1 + iofs0[1] + 3)));
      u0 = _mm_unpacklo_epi8(u0, z);
      v0 = _mm_unpacklo_epi8(v0, z);
      u1 = _mm_unpacklo_epi8(u1, z);
      v1 = _mm_unpacklo_epi8(v1, z);
      u0 = _mm_add_epi32(_mm_madd_epi16(u0, w0[0]), _mm_madd_epi16(v0, w0[1]));
      u1 = _mm_add_epi32(_mm_madd_epi16(u1, w1[0]), _mm_madd_epi16(v1, w1[1]));
      u0 = _mm_srai_epi32(_mm_add_epi32(u0, delta), INTER_REMAP_COEF_BITS);
      u1 = _mm_srai_epi32(_mm_add_epi32(u1, delta), INTER_REMAP_COEF_BITS);
      u0 = _mm_slli_si128(u0, 4);
      u0 = _mm_packs_epi32(u0, u1);
      u0 = _mm_packus_epi16(u0, u0);
      _mm_storel_epi64((__m128i*)D, _mm_srli_si128(u0,1));

      w0 = (const __m128i*)(wtab + FXY[x+2]*16);
      w1 = (const __m128i*)(wtab + FXY[x+3]*16);

      u0 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int*)(S0 + iofs0[2])),
			     _mm_cvtsi32_si128(*(int*)(S0 + iofs0[2] + 3)));
      v0 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int*)(S1 + iofs0[2])),
			     _mm_cvtsi32_si128(*(int*)(S1 + iofs0[2] + 3)));
      u1 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int*)(S0 + iofs0[3])),
			     _mm_cvtsi32_si128(*(int*)(S0 + iofs0[3] + 3)));
      v1 = _mm_unpacklo_epi8(_mm_cvtsi32_si128(*(int*)(S1 + iofs0[3])),
			     _mm_cvtsi32_si128(*(int*)(S1 + iofs0[3] + 3)));
      u0 = _mm_unpacklo_epi8(u0, z);
      v0 = _mm_unpacklo_epi8(v0, z);
      u1 = _mm_unpacklo_epi8(u1, z);
      v1 = _mm_unpacklo_epi8(v1, z);
      u0 = _mm_add_epi32(_mm_madd_epi16(u0, w0[0]), _mm_madd_epi16(v0, w0[1]));
      u1 = _mm_add_epi32(_mm_madd_epi16(u1, w1[0]), _mm_madd_epi16(v1, w1[1]));
      u0 = _mm_srai_epi32(_mm_add_epi32(u0, delta), INTER_REMAP_COEF_BITS);
      u1 = _mm_srai_epi32(_mm_add_epi32(u1, delta), INTER_REMAP_COEF_BITS);
      u0 = _mm_slli_si128(u0, 4);
      u0 = _mm_packs_epi32(u0, u1);
      u0 = _mm_packus_epi16(u0, u0);
      _mm_storel_epi64((__m128i*)(D + 6), _mm_srli_si128(u0,1));
    }
  }

  return x;
}

#else

static int RemapVec_8u(int channels,
		       const uint8_t* srcp, int src_step,
		       void* _dst,
		       const short* XY, const unsigned short* FXY,
		       int width) {
  return 0;
}

#endif

static void initInterTab2D() {
  static bool inittab = false;
  if (inittab)
    return;

  float _tab[2*INTER_TAB_SIZE];
  for (int j=0;j<INTER_TAB_SIZE;++j) {
    float scale = 1.f/INTER_TAB_SIZE;
    float x = j * scale;
    _tab[j*2+0] = 1.f - x;
    _tab[j*2+1] = x;
  }

  const int ksize = 2;
  float* tab = BilinearTab_f[0][0];
  short* itab = BilinearTab_i[0][0];

  for (int i = 0; i < INTER_TAB_SIZE; i++)
    for (int j = 0; j < INTER_TAB_SIZE; j++, tab += ksize*ksize, itab += ksize*ksize) {
      int isum = 0;

      for (int k1 = 0; k1 < ksize; k1++) {
	float vy = _tab[i*ksize + k1];
	for (int k2 = 0; k2 < ksize; k2++) {
	  float v = vy*_tab[j*ksize + k2];
	  tab[k1*ksize + k2] = v;
	  int v0 = int(std::round(v*INTER_REMAP_COEF_SCALE));
	  v0 = std::min(int(std::numeric_limits<short>::max()),
			std::max(int(std::numeric_limits<short>::min()),v0));
	  isum += itab[k1*ksize + k2] = v0;
	}
      }

      if (isum != INTER_REMAP_COEF_SCALE) {
	int diff = isum - INTER_REMAP_COEF_SCALE;
	int ksize2 = ksize/2, Mk1=ksize2, Mk2=ksize2, mk1=ksize2, mk2=ksize2;
	for (int k1 = ksize2; k1 < ksize2+2; k1++)
	  for (int k2 = ksize2; k2 < ksize2+2; k2++) {
	    if (itab[k1*ksize+k2] < itab[mk1*ksize+mk2])
	      mk1 = k1, mk2 = k2;
	    else if (itab[k1*ksize+k2] > itab[Mk1*ksize+Mk2])
	      Mk1 = k1, Mk2 = k2;
	  }
	if (diff < 0)
	  itab[Mk1*ksize + Mk2] = (short)(itab[Mk1*ksize + Mk2] - diff);
	else
	  itab[mk1*ksize + mk2] = (short)(itab[mk1*ksize + mk2] - diff);
      }
    }

#if OCCAM_SSE2 || OCCAM_NEON
  for (int i = 0; i < INTER_TAB_SIZE2; i++)
    for (int j = 0; j < 4; j++) {
      BilinearTab_iC4[i][0][j*2] = BilinearTab_i[i][0][0];
      BilinearTab_iC4[i][0][j*2+1] = BilinearTab_i[i][0][1];
      BilinearTab_iC4[i][1][j*2] = BilinearTab_i[i][1][0];
      BilinearTab_iC4[i][1][j*2+1] = BilinearTab_i[i][1][1];
    }
#endif

  inittab = true;
}

//////////////////////////////////////////////////////////////////////////////////
// ImageRemap

ImageRemap::ImageRemap(int _map_width,
		       int _map_height)
  : map_width(_map_width),
    map_height(_map_height) {
  initInterTab2D();
}

int ImageRemap::mapWidth() const {
  return map_width;
}

int ImageRemap::mapHeight() const {
  return map_height;
}

int ImageRemap::addImage(int width, int height) {
  int src_index = images.size();
  Image& img = *images.emplace(images.end(),Image());
  img.width = width;
  img.height = height;
  return src_index;
}

void ImageRemap::map(int dst_x, int dst_y, int src_index, float src_x, float src_y) {
  map(dst_x, dst_y, src_index, src_x, src_y, -1, 0, 0, 0);
}

void ImageRemap::map(int dst_x, int dst_y,
		     int src_index0, float src_x0, float src_y0,
		     int src_index1, float src_x1, float src_y1,
		     float fade0) {
  assert(dst_x>=0&&dst_x<map_width);
  assert(dst_y>=0&&dst_y<map_height);

  auto push_xy = [&](float src_x, float src_y, int src_width, int src_height, bool& inlier){
    int sx = int(std::round(src_x*INTER_TAB_SIZE));
    int sy = int(std::round(src_y*INTER_TAB_SIZE));
    int v = (sy & (INTER_TAB_SIZE-1))*INTER_TAB_SIZE + (sx & (INTER_TAB_SIZE-1));
    int sx0 = sx >> INTER_BITS;
    int sy0 = sy >> INTER_BITS;
    if (sx0 < 0 || sy0 < 0 || sx0 >= src_width-1 || sy0 >= src_height-1) {
      inlier = false;
      src_x = -1;
      src_y = -1;
      sx = -1;
      sy = -1;
      v = 0;
    }
    xy.push_back(src_x);
    xy.push_back(src_y);
    ixy.push_back(std::min(int(std::numeric_limits<short>::max()),
			  std::max(int(std::numeric_limits<short>::min()),sx0)));
    ixy.push_back(std::min(int(std::numeric_limits<short>::max()),
			  std::max(int(std::numeric_limits<short>::min()),sy0)));
    fxy.push_back((unsigned short)v);
  };

  bool inlier = true;
  push_xy(src_x0,src_y0,images[src_index0].width,images[src_index0].height,inlier);
  if (src_index1>=0) {
    push_xy(src_x1,src_y1,images[src_index1].width,images[src_index1].height,inlier);
    fade.push_back(fade0);
  }

  if (!segments.empty()) {
    Segment& s = segments.end()[-1];
    if (s.dst_y == dst_y &&
	s.dst_x+s.length == dst_x &&
	s.src_indices[0] == src_index0 &&
	s.src_indices[1] == src_index1 &&
	s.inlier == inlier) {
      ++s.length;
      return;
    }
  }

  Segment& s = *segments.emplace(segments.end(),Segment());
  s.dst_x = dst_x;
  s.dst_y = dst_y;
  s.src_indices[0] = src_index0;
  s.src_indices[1] = src_index1;
  s.length = 1;
  s.inlier = inlier;
}

int ImageRemap::operator() (OccamImageFormat format,
			    const uint8_t* const* srcpp,const int* src_stepp,
			    uint8_t* dstp0,int dst_step) {
  if (format != OCCAM_GRAY8 &&
      format != OCCAM_RGB24 &&
      format != OCCAM_SHORT1)
    return OCCAM_API_INVALID_FORMAT;

  auto castOp = [](int v) {
    v += 1<<(INTER_REMAP_COEF_BITS-1);
    v >>= INTER_REMAP_COEF_BITS;
    return (unsigned char)((unsigned)v <= 255 ? v : v > 0 ? 255 : 0);
  };
  auto clip = [](int x, int min_x, int max_x) {
    return x < min_x ? min_x : (x >= max_x ? max_x - 1 : x);
  };

  if (format == OCCAM_SHORT1)
    for (int y=0;y<map_height;++y)
      memset(dstp0+dst_step*y,255,dst_step);

  int channels = format == OCCAM_RGB24 ? 3 : 1;
  int bpp = 1;
  occamImageFormatBytesPerPixel(format, &bpp);
  const short* wtab = &BilinearTab_i[0][0][0];
  const float* tab = &BilinearTab_f[0][0][0];

  short* ixyp = ixy.empty()?0:&ixy[0];
  unsigned short* fxyp = fxy.empty()?0:&fxy[0];
  float* fadep = fade.empty()?0:&fade[0];
  for (const Segment& s : segments) {
    uint8_t* dstp = dstp0+dst_step*s.dst_y+s.dst_x*bpp;
    int length = s.length;

    int src_count = ((s.src_indices[0]>=0)?1:0) + ((s.src_indices[1]>=0)?1:0);
    if (src_count == 1) {
      int src_index = s.src_indices[0];
      const uint8_t* srcp = srcpp[src_index];
      int src_step = src_stepp[src_index];
      int src_width = images[src_index].width;
      int src_height = images[src_index].height;

      if (s.inlier) {
	if (format == OCCAM_GRAY8 || format == OCCAM_RGB24) {
	  const short* wtab = channels == 1 ? &BilinearTab_i[0][0][0] : &BilinearTab_iC4[0][0][0];
	  int vec_length = RemapVec_8u(channels,srcp,src_step,dstp,ixyp,fxyp,length);
	  length -= vec_length;
	  dstp += vec_length * channels;
	  ixyp += vec_length * 2;
	  fxyp += vec_length;
	}

	if (format == OCCAM_GRAY8) {
	  for (int j=0;j<length;++j,++dstp,ixyp+=2,++fxyp) {
	    int sx = ixyp[0];
	    int sy = ixyp[1];
	    const short* w = wtab + fxyp[0]*4;
	    const uint8_t* S = srcp + sy*src_step + sx;
	    *dstp = castOp(int(S[0]*w[0] + S[1]*w[1] + S[src_step]*w[2] + S[src_step+1]*w[3]));
	  }
	} else if (format == OCCAM_RGB24) {
	  for (int j=0;j<length;++j,dstp+=3,ixyp+=2,++fxyp) {
	    int sx = ixyp[0];
	    int sy = ixyp[1];
	    const short* w = wtab + fxyp[0]*4;
	    const uint8_t* S = srcp + sy*src_step + sx*3;
	    dstp[0] = castOp(S[0]*w[0] + S[3]*w[1] + S[src_step]*w[2] + S[src_step+3]*w[3]);
	    dstp[1] = castOp(S[1]*w[0] + S[4]*w[1] + S[src_step+1]*w[2] + S[src_step+4]*w[3]);
	    dstp[2] = castOp(S[2]*w[0] + S[5]*w[1] + S[src_step+2]*w[2] + S[src_step+5]*w[3]);
	  }
	} else if (format == OCCAM_SHORT1) {
	  for (int j=0;j<length;++j,dstp+=2,ixyp+=2,++fxyp) {
	    int sx = ixyp[0];
	    int sy = ixyp[1];
	    const float* w = tab + fxyp[0]*4;
	    const short* S0 = (short*)(srcp + sy*src_step) + sx;
	    const short* S1 = (short*)(srcp + sy*src_step + src_step) + sx;
	    float v = 0;
	    if (S0[0]>=0&&S0[1]>=0&&S1[0]>=0&&S1[1]>=0)
	      v = S0[0]*w[0] + S0[1]*w[1] + S1[0]*w[2] + S1[1]*w[3];
	    else
	      v = S0[0];
	    if (v <= 0)
	      v = -999;
	    *((short*)dstp) = short(v);
	  }
	}
      } else {
	if (format == OCCAM_GRAY8 || format == OCCAM_RGB24) {
	  for (int j=0;j<length;++j,dstp+=channels,ixyp+=2,++fxyp) {
	    int sx = ixyp[0];
	    int sy = ixyp[1];
	    if ((sx >= src_width || sx+1 < 0 ||
		 sy >= src_height || sy+1 < 0)) {
	      for (int k=0;k<channels;++k)
		dstp[k] = 128;
	    } else {
	      const short* w = wtab + fxyp[0]*4;
	      int sx0 = clip(sx, 0, src_width);
	      int sx1 = clip(sx+1, 0, src_width);
	      int sy0 = clip(sy, 0, src_height);
	      int sy1 = clip(sy+1, 0, src_height);
	      const uint8_t* v0 = srcp + sy0*src_step + sx0;
	      const uint8_t* v1 = srcp + sy0*src_step + sx1;
	      const uint8_t* v2 = srcp + sy1*src_step + sx0;
	      const uint8_t* v3 = srcp + sy1*src_step + sx1;
	      for (int k=0;k<channels;++k)
		dstp[k] = castOp(int(v0[k]*w[0] + v1[k]*w[1] + v2[k]*w[2] + v3[k]*w[3]));
	    }
	  }
	} else if (format == OCCAM_SHORT1) {
	  for (int j=0;j<length;++j,dstp+=2,ixyp+=2,++fxyp) {
	    int sx = ixyp[0];
	    int sy = ixyp[1];
	    if ((sx >= src_width || sx+1 < 0 ||
		 sy >= src_height || sy+1 < 0)) {
	      *((short*)dstp) = -999;
	    } else {
	      const float* w = tab + fxyp[0]*4;
	      int sx0 = clip(sx, 0, src_width);
	      int sx1 = clip(sx+1, 0, src_width);
	      int sy0 = clip(sy, 0, src_height);
	      int sy1 = clip(sy+1, 0, src_height);
	      const short* v0 = (short*)(srcp + sy0*src_step) + sx0;
	      const short* v1 = (short*)(srcp + sy0*src_step) + sx1;
	      const short* v2 = (short*)(srcp + sy1*src_step) + sx0;
	      const short* v3 = (short*)(srcp + sy1*src_step) + sx1;
	      *((short*)dstp) = short(v0[0]*w[0] + v1[0]*w[1] + v2[0]*w[2] + v3[0]*w[3]);
	    }
	  }
	}
      }
    } else if (src_count == 2) {
      int src_index0 = s.src_indices[0];
      int src_index1 = s.src_indices[1];
      const uint8_t* srcp0 = srcpp[src_index0];
      const uint8_t* srcp1 = srcpp[src_index1];
      int src_step0 = src_stepp[src_index0];
      int src_step1 = src_stepp[src_index1];
      int src_width0 = images[src_index0].width;
      int src_width1 = images[src_index1].width;
      int src_height0 = images[src_index0].height;
      int src_height1 = images[src_index1].height;

      if (format == OCCAM_GRAY8 || format == OCCAM_RGB24) {
	for (int j=0;j<length;++j,dstp+=channels,ixyp+=4,fxyp+=2,++fadep) {
	  int sx0 = ixyp[0];
	  int sy0 = ixyp[1];
	  int sx1 = ixyp[2];
	  int sy1 = ixyp[3];
	  if ((sx0 >= src_width0 || sx0+1 < 0 ||
	       sy0 >= src_height0 || sy0+1 < 0) ||
	      (sx1 >= src_width1 || sx1+1 < 0 ||
	       sy1 >= src_height1 || sy1+1 < 0)) {
	    for (int k=0;k<channels;++k)
	      dstp[k] = 0;
	  } else {
	    const short* w0 = wtab + fxyp[0]*4;
	    const short* w1 = wtab + fxyp[1]*4;
	    int sx00 = clip(sx0, 0, src_width0);
	    int sx10 = clip(sx0+1, 0, src_width0);
	    int sy00 = clip(sy0, 0, src_height0);
	    int sy10 = clip(sy0+1, 0, src_height0);
	    const uint8_t* v00 = srcp0 + sy00*src_step0 + sx00*channels;
	    const uint8_t* v10 = srcp0 + sy00*src_step0 + sx10*channels;
	    const uint8_t* v20 = srcp0 + sy10*src_step0 + sx00*channels;
	    const uint8_t* v30 = srcp0 + sy10*src_step0 + sx10*channels;
	    int sx01 = clip(sx1, 0, src_width1);
	    int sx11 = clip(sx1+1, 0, src_width1);
	    int sy01 = clip(sy1, 0, src_height1);
	    int sy11 = clip(sy1+1, 0, src_height1);
	    const uint8_t* v01 = srcp1 + sy01*src_step1 + sx01*channels;
	    const uint8_t* v11 = srcp1 + sy01*src_step1 + sx11*channels;
	    const uint8_t* v21 = srcp1 + sy11*src_step1 + sx01*channels;
	    const uint8_t* v31 = srcp1 + sy11*src_step1 + sx11*channels;
	    for (int k=0;k<channels;++k) {
	      uint8_t v0 = castOp(int(v00[k]*w0[0] + v10[k]*w0[1] + v20[k]*w0[2] + v30[k]*w0[3]));
	      uint8_t v1 = castOp(int(v01[k]*w1[0] + v11[k]*w1[1] + v21[k]*w1[2] + v31[k]*w1[3]));
	      dstp[k] = uint8_t(v0 * fadep[0] + v1 * (1 - fadep[0]));
	    }
	  }
	}
      } else if (format == OCCAM_SHORT1) {
	for (int j=0;j<length;++j,dstp+=2,ixyp+=4,fxyp+=2,++fadep) {
	  int sx0 = ixyp[0];
	  int sy0 = ixyp[1];
	  int sx1 = ixyp[2];
	  int sy1 = ixyp[3];
	  if ((sx0 >= src_width0 || sx0+1 < 0 ||
	       sy0 >= src_height0 || sy0+1 < 0) ||
	      (sx1 >= src_width1 || sx1+1 < 0 ||
	       sy1 >= src_height1 || sy1+1 < 0)) {
	    *((short*)dstp) = 0;
	  } else {
	    const float* w0 = tab + fxyp[0]*4;
	    const float* w1 = tab + fxyp[1]*4;
	    int sx00 = clip(sx0, 0, src_width0);
	    int sx10 = clip(sx0+1, 0, src_width0);
	    int sy00 = clip(sy0, 0, src_height0);
	    int sy10 = clip(sy0+1, 0, src_height0);
	    const short* v00 = ((short*)(srcp0 + sy00*src_step0)) + sx00;
	    const short* v10 = (short*)(srcp0 + sy00*src_step0) + sx10;
	    const short* v20 = (short*)(srcp0 + sy10*src_step0) + sx00;
	    const short* v30 = (short*)(srcp0 + sy10*src_step0) + sx10;
	    int sx01 = clip(sx1, 0, src_width1);
	    int sx11 = clip(sx1+1, 0, src_width1);
	    int sy01 = clip(sy1, 0, src_height1);
	    int sy11 = clip(sy1+1, 0, src_height1);
	    const short* v01 = (short*)(srcp1 + sy01*src_step1) + sx01;
	    const short* v11 = (short*)(srcp1 + sy01*src_step1) + sx11;
	    const short* v21 = (short*)(srcp1 + sy11*src_step1) + sx01;
	    const short* v31 = (short*)(srcp1 + sy11*src_step1) + sx11;
	    short v0 = -1;
	    short v1 = -1;
	    if (v00[0]>0&&v10[0]>0&&v20[0]>0&&v30[0]>0)
	      v0 = short(v00[0]*w0[0] + v10[0]*w0[1] + v20[0]*w0[2] + v30[0]*w0[3]);
	    if (v01[0]>0&&v11[0]>0&&v21[0]>0&&v31[0]>0)
	      v1 = short(v01[0]*w1[0] + v11[0]*w1[1] + v21[0]*w1[2] + v31[0]*w1[3]);
	    if (v0 >= 0 && v1 >= 0)
	      *((short*)dstp) = short(v0 * fadep[0] + v1 * (1 - fadep[0]));
	    else if (v0 >= 0)
	      *((short*)dstp) = v0;
	    else if (v1 >= 0)
	      *((short*)dstp) = v1;
	    else
	      *((short*)dstp) = -999;
	  }
	}
      }
    }
  }

  return OCCAM_API_SUCCESS;
}

int ImageRemap::operator() (const OccamImage* const* img0, OccamImage** img1out) {
  for (int j=0;j<images.size();++j) {
    const OccamImage* img0j = img0[j];
    if (img0j->backend != OCCAM_CPU)
      return OCCAM_API_INVALID_PARAMETER;
    if (img0j->format != OCCAM_GRAY8 &&
	img0j->format != OCCAM_RGB24 &&
	img0j->format != OCCAM_SHORT1)
      return OCCAM_API_INVALID_FORMAT;
    if (img0j->width != images[j].width ||
	img0j->height != images[j].height)
      return OCCAM_API_INVALID_PARAMETER;
  }

  OccamImage* img1 = new OccamImage;
  *img1out = img1;
  memset(img1,0,sizeof(OccamImage));
  img1->cid = strdup(img0[0]->cid);
  memcpy(img1->timescale,img0[0]->timescale,sizeof(img1->timescale));
  img1->time_ns = img0[0]->time_ns;
  img1->index = img0[0]->index;
  img1->refcnt = 1;
  img1->backend = OCCAM_CPU;
  img1->format = img0[0]->format;
  img1->width = map_width;
  img1->height = map_height;
  int bpp = 1;
  occamImageFormatBytesPerPixel(img0[0]->format, &bpp);
  img1->step[0] = (img1->width*bpp+15)&~15;
  img1->data[0] = new uint8_t[img1->step[0]*img1->height];
  memset(img1->data[0],0,img1->step[0]*img1->height);

  const uint8_t** srcp = (const uint8_t**)alloca(sizeof(const uint8_t*)*images.size());
  int* src_stepp = (int*)alloca(sizeof(int)*images.size());
  for (int j=0;j<images.size();++j) {
    srcp[j] = img0[j]->data[0];
    src_stepp[j] = img0[j]->step[0];
  }

  int r = operator()(img1->format,
   		     srcp,src_stepp,
   		     img1->data[0],img1->step[0]);
  if (r != OCCAM_API_SUCCESS) {
    occamFreeImage(img1);
    *img1out = 0;
  }
  return r;
}

int ImageRemap::operator() (const OccamImage* img0, OccamImage** img1) {
  return operator()(&img0,img1);
}

// #ifdef OCCAM_OPENGL_SUPPORT

// class GLBlendRemapper {
//   BlendRemapper cpu_remap;
//   std::vector<float> array;
//   std::vector<GLuint> indices;
//   std::shared_ptr<GLProgramBind> prog;
//   GLuint vbo, ibo;
//   GLuint attrib_loc;
//   void init(const BlendRemapArgs& args, int scale) {
//     array.clear();
//     indices.clear();
//     int dst_width = args.dst_width*scale;
//     int dst_height = args.dst_height*scale;
//     int src_width = args.src_width;
//     int src_height = args.src_height;
//     int sensor_count = args.sensor_count;
//     int dstx_step = scale, dsty_step = scale;
//     int top_y = cpu_remap.cropTop() * scale;
//     int bottom_y = cpu_remap.cropBottom() * scale;
//     int cropped_height = cpu_remap.croppedHeight() * scale;

//     for (int Si=0;Si<sensor_count;++Si) {
//       for (int dsty=top_y;dsty<bottom_y;dsty+=dsty_step) {
// 	for (int dstx=0;dstx<dst_width;dstx+=dstx_step) {
// 	  int dstxi[] = {dstx,dstx+dstx_step,dstx+dstx_step,dstx};
// 	  int dstyi[] = {dsty,dsty,dsty+dsty_step,dsty+dsty_step};
// 	  float srcx[4], srcy[4], fade[4];
// 	  bool en[] = {
// 	    cpu_remap.map(Si,dstxi[0]/scale,dstyi[0]/scale,srcx[0],srcy[0],fade[0]),
// 	    cpu_remap.map(Si,dstxi[1]/scale,dstyi[1]/scale,srcx[1],srcy[1],fade[1]),
// 	    cpu_remap.map(Si,dstxi[2]/scale,dstyi[2]/scale,srcx[2],srcy[2],fade[2]),
// 	    cpu_remap.map(Si,dstxi[3]/scale,dstyi[3]/scale,srcx[3],srcy[3],fade[3]),
// 	  };
// 	  auto push = [&](int i0,int i1,int i2) {
// 	    if (!en[i0]||!en[i1]||!en[i2])
// 	      return;
// 	    int in[] = {i0,i1,i2};
// 	    for (int ii=0;ii<3;++ii) {
// 	      int i = in[ii];
// 	      indices.push_back(indices.size());
// 	      array.push_back((1-(dstxi[i]/float(dst_width)))*2-1);
// 	      array.push_back((1-((dstyi[i]-top_y)/float(cropped_height)))*2-1);
// 	      array.push_back(srcx[i]/float(src_width));
// 	      array.push_back(srcy[i]/float(src_height));
// 	      array.push_back(fade[i]);
// 	    }
// 	  };
// 	  push(0,3,1);
// 	  push(2,3,1);
// 	}
//       }
//     }

//     if (vbo)
//       GL_CHECK(glDeleteBuffers(1, &vbo));
//     if (ibo)
//       GL_CHECK(glDeleteBuffers(1, &ibo));

//     GL_CHECK(glGenBuffers(1, &vbo));
//     GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo));
//     GL_CHECK(glBufferData(GL_ARRAY_BUFFER, array.size() * sizeof(float),
// 			  &array[0], GL_STATIC_DRAW));
//     GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));

//     GL_CHECK(glGenBuffers(1, &ibo));
//     GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo));
//     GL_CHECK(glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
// 			  &indices[0], GL_STATIC_DRAW));
//     GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

//     auto vshader = R"EOS(
// #version 330 compatibility
// varying float a;
// attribute float b;
// void main(void) {
//   a = b;
//   gl_TexCoord[0] = gl_MultiTexCoord0;
//   gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
// }
// )EOS";
//     auto fshader = R"EOS(
// uniform sampler2D sampler0;
// varying float a;
// void main(void) {
//   vec4 t0 = texture2D(sampler0, vec2(gl_TexCoord[0]));
//   gl_FragColor.xyz = t0.xyz;
//   gl_FragColor.w = a;
// }
// )EOS";
//     if (!bool(prog)) {
//       prog = std::make_shared<GLProgramBind>(vshader,fshader);
//       attrib_loc = glGetAttribLocation(prog->program(), "b");
//     }
//   }
// public:
//   GLBlendRemapper()
//     : vbo(0), ibo(0) {
//   }
//   ~GLBlendRemapper() {
//     if (vbo)
//       GL_CHECK(glDeleteBuffers(1, &vbo));
//     if (ibo)
//       GL_CHECK(glDeleteBuffers(1, &ibo));
//   }
//   GLBlendRemapper(const GLBlendRemapper& x) = delete;
//   GLBlendRemapper& operator= (const GLBlendRemapper& rhs) = delete;
//   int croppedHeight() const {
//     return cpu_remap.croppedHeight();
//   }
//   void operator() (const BlendRemapArgs& args, int scale) {
//     if (cpu_remap(args))
//       init(args, scale);
//   }
//   void operator() (const BlendRemapArgs& args, int scale, GLuint srctex) {
//     if (cpu_remap(args))
//       init(args, scale);

//     GL_CHECK(glClearColor(0,0,0,0));
//     GL_CHECK(glClear(GL_COLOR_BUFFER_BIT));

//     GL_CHECK(glEnable(GL_BLEND));
//     GL_CHECK(glBlendFunc(GL_SRC_ALPHA,GL_DST_ALPHA));
//     GL_CHECK(glBlendEquation(GL_FUNC_ADD));

//     GL_CHECK(glBindTexture(GL_TEXTURE_2D, srctex));
//     GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
//     GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
//     GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT));
//     GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT));

//     GL_CHECK(glEnableClientState(GL_VERTEX_ARRAY));
//     GL_CHECK(glEnableClientState(GL_TEXTURE_COORD_ARRAY));
//     GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo));
//     GL_CHECK(glTexCoordPointer(2, GL_FLOAT, 5*sizeof(float), (GLvoid*)(2*sizeof(float))));
//     GL_CHECK(glVertexPointer(2, GL_FLOAT, 5*sizeof(float), (GLvoid*)(0)));
//     GL_CHECK(glVertexAttribPointer(attrib_loc, 1, GL_FLOAT, GL_FALSE, 5*sizeof(float), (GLvoid*)(4*sizeof(float))));
//     GL_CHECK(glEnableVertexAttribArray(attrib_loc));

//     GL_CHECK(glUseProgram(prog->program()));
//     GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo));
//     GL_CHECK(glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0));
//     GL_CHECK(glUseProgram(0));

//     GL_CHECK(glDisableVertexAttribArray(0));
//     GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
//     GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));
//     GL_CHECK(glDisableClientState(GL_TEXTURE_COORD_ARRAY));
//     GL_CHECK(glDisableClientState(GL_VERTEX_ARRAY));

//     GL_CHECK(glDisable(GL_BLEND));
//   }
// };

// #endif // OCCAM_OPENGL_SUPPORT
