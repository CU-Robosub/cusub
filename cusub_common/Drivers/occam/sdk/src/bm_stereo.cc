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

// CPU block matching derived from OpenCV 1bdd86edeba4babff7a78586beba20841bb87fa9
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

#include "module_utils.h"
#include "system.h"
#include <algorithm>
#include <sstream>
#include <string.h>

static const int DISPARITY_SHIFT = 4;

enum BMPrefilterTypes {
  OCCAM_PREFILTER_NONE = 0,
  OCCAM_PREFILTER_XSOBEL = 1,
  OCCAM_PREFILTER_NORMALIZED_RESPONSE = 2
};

static void prefilterNorm(int width, int height,
			  const uint8_t* img0p, int img0_step,
			  uint8_t* img1p, int img1_step,
			  int winsize,
			  int ftzero,
			  uint8_t* buf) {
  int x;
  int y;
  int wsz2 = winsize/2;
  int* vsum = (int*)occamAlignPtr(buf + (wsz2 + 1)*sizeof(vsum[0]), 32);
  int scale_g = winsize*winsize/8;
  int scale_s = (1024 + scale_g)/(scale_g*2);
  const int OFS = 256*5;
  const int TABSZ = OFS*2 + 256;
  uint8_t tab[TABSZ];
  const uint8_t* sptr = img0p;
  int srcstep = (int)img0_step;

  scale_g *= scale_s;

  for (x = 0; x < TABSZ; x++)
    tab[x] = (uint8_t)(x - OFS < -ftzero ? 0 : x - OFS > ftzero ? ftzero*2 : x - OFS + ftzero);

  for (x = 0; x < width; x++)
    vsum[x] = (unsigned short)(sptr[x]*(wsz2 + 2));

  for (y = 1; y < wsz2; y++)
    for (x = 0; x < width; x++)
      vsum[x] = (unsigned short)(vsum[x] + sptr[srcstep*y + x]);

  for (y = 0; y < height; y++) {
    const uint8_t* top = sptr + srcstep*std::max(y-wsz2-1,0);
    const uint8_t* bottom = sptr + srcstep*std::min(y+wsz2,height-1);
    const uint8_t* prev = sptr + srcstep*std::max(y-1,0);
    const uint8_t* curr = sptr + srcstep*y;
    const uint8_t* next = sptr + srcstep*std::min(y+1,height-1);
    uint8_t* dptr = img1p + img1_step*y;

    for (x = 0; x < width; x++)
      vsum[x] = (unsigned short)(vsum[x] + bottom[x] - top[x]);

    for (x = 0; x <= wsz2; x++) {
      vsum[-x-1] = vsum[0];
      vsum[width+x] = vsum[width-1];
    }

    int sum = vsum[0]*(wsz2 + 1);
    for (x = 1; x <= wsz2; x++)
      sum += vsum[x];

    int val = ((curr[0]*5 + curr[1] + prev[0] + next[0])*scale_g - sum*scale_s) >> 10;
    dptr[0] = tab[val + OFS];

    for (x = 1; x < width-1; x++) {
      sum += vsum[x+wsz2] - vsum[x-wsz2-1];
      val = ((curr[x]*4 + curr[x-1] + curr[x+1] + prev[x] + next[x])*scale_g - sum*scale_s) >> 10;
      dptr[x] = tab[val + OFS];
    }

    sum += vsum[x+wsz2] - vsum[x-wsz2-1];
    val = ((curr[x]*5 + curr[x-1] + prev[x] + next[x])*scale_g - sum*scale_s) >> 10;
    dptr[x] = tab[val + OFS];
  }
}

static void prefilterXSobel(int width, int height,
			    const uint8_t* img0p, int img0_step,
			    uint8_t* img1p, int img1_step,
			    int ftzero) {
  int x, y;
  const int OFS = 256*4;
  const int TABSZ = OFS*2 + 256;
  uint8_t tab[TABSZ];

  for (x = 0; x < TABSZ; x++)
    tab[x] = (uint8_t)(x - OFS < -ftzero ? 0 : x - OFS > ftzero ? ftzero*2 : x - OFS + ftzero);
  uint8_t val0 = tab[0 + OFS];

#if OCCAM_SSE2
  bool useSIMD = occamHardwareSupport(OCCAM_CPU_SSE2);
#endif

  for (y = 0; y < height-1; y += 2) {
    const uint8_t* srow1 = img0p + img0_step*y;;
    const uint8_t* srow0 = y > 0 ? srow1 - img0_step : height > 1 ? srow1 + img0_step : srow1;
    const uint8_t* srow2 = y < height-1 ? srow1 + img0_step : height > 1 ? srow1 - img0_step : srow1;
    const uint8_t* srow3 = y < height-2 ? srow1 + img0_step*2 : srow1;
    uint8_t* dptr0 = img1p + img1_step*y;
    uint8_t* dptr1 = dptr0 + img1_step;

    dptr0[0] = dptr0[width-1] = dptr1[0] = dptr1[width-1] = val0;
    x = 1;

#if OCCAM_SSE2
    if (useSIMD) {
      __m128i z = _mm_setzero_si128();
      __m128i ftz = _mm_set1_epi16((short)ftzero);
      int ftzero_2 = std::min(255,std::max(0,ftzero*2));
      __m128i ftz2 = _mm_set1_epi8(ftzero_2);
      for (; x <= width-9; x += 8) {
	__m128i c0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow0 + x - 1)), z);
	__m128i c1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow1 + x - 1)), z);
	__m128i d0 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow0 + x + 1)), z);
	__m128i d1 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow1 + x + 1)), z);

	d0 = _mm_sub_epi16(d0, c0);
	d1 = _mm_sub_epi16(d1, c1);

	__m128i c2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow2 + x - 1)), z);
	__m128i c3 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow3 + x - 1)), z);
	__m128i d2 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow2 + x + 1)), z);
	__m128i d3 = _mm_unpacklo_epi8(_mm_loadl_epi64((__m128i*)(srow3 + x + 1)), z);

	d2 = _mm_sub_epi16(d2, c2);
	d3 = _mm_sub_epi16(d3, c3);

	__m128i v0 = _mm_add_epi16(d0, _mm_add_epi16(d2, _mm_add_epi16(d1, d1)));
	__m128i v1 = _mm_add_epi16(d1, _mm_add_epi16(d3, _mm_add_epi16(d2, d2)));
	v0 = _mm_packus_epi16(_mm_add_epi16(v0, ftz), _mm_add_epi16(v1, ftz));
	v0 = _mm_min_epu8(v0, ftz2);

	_mm_storel_epi64((__m128i*)(dptr0 + x), v0);
	_mm_storel_epi64((__m128i*)(dptr1 + x), _mm_unpackhi_epi64(v0, v0));
      }
    }
#endif

    for (; x < width-1; x++) {
      int d0 = srow0[x+1] - srow0[x-1];
      int d1 = srow1[x+1] - srow1[x-1];
      int d2 = srow2[x+1] - srow2[x-1];
      int d3 = srow3[x+1] - srow3[x-1];
      int v0 = tab[d0 + d1*2 + d2 + OFS];
      int v1 = tab[d1 + d2*2 + d3 + OFS];
      dptr0[x] = (uint8_t)v0;
      dptr1[x] = (uint8_t)v1;
    }
  }

  for (; y < height; y++) {
    uint8_t* dptr = img1p + img1_step*y;
    for (x = 0; x < width; x++)
      dptr[x] = val0;
  }
}

#if OCCAM_SSE2
static void findStereoCorrespondenceBM_SSE2(int width, int height,
					    const uint8_t* img0p, int img0_step,
					    const uint8_t* img1p, int img1_step,
					    uint8_t* dispp, int disp_step,
					    uint8_t* costp, int cost_step,
					    int sad_window_size,
					    int num_disparities,
					    int min_disparity,
					    int prefilter_cap,
					    int texture_threshold,
					    int uniqueness_ratio,
					    uint8_t* buf,
					    int _dy0,
					    int _dy1) {
  const int ALIGN = 16;
  int x, y, d;
  int wsz = sad_window_size;
  int wsz2 = wsz/2;
  int dy0 = std::min(_dy0, wsz2+1);
  int dy1 = std::min(_dy1, wsz2+1);
  int ndisp = num_disparities;
  int mindisp = min_disparity;
  int lofs = std::max(ndisp - 1 + mindisp, 0);
  int rofs = -std::min(ndisp - 1 + mindisp, 0);
  int width1 = width - rofs - ndisp + 1;
  int ftzero = prefilter_cap;
  short FILTERED = (short)((mindisp - 1) << DISPARITY_SHIFT);

  unsigned short* sad;
  unsigned short* hsad0;
  unsigned short* hsad;
  unsigned short* hsad_sub;
  int *htext;
  uint8_t *cbuf0, *cbuf;
  const uint8_t* lptr0 = img0p + lofs;
  const uint8_t* rptr0 = img1p + rofs;
  const uint8_t *lptr, *lptr_sub, *rptr;
  short* dptr = (short*)dispp;
  int sstep = (int)img0_step;
  int dstep = (int)(disp_step/sizeof(dptr[0]));
  int cstep = (height + dy0 + dy1)*ndisp;
  short costbuf = 0;
  int coststep = costp ? (int)(cost_step/sizeof(costbuf)) : 0;
  const int TABSZ = 256;
  uint8_t tab[TABSZ];
  const __m128i d0_8 = _mm_setr_epi16(0,1,2,3,4,5,6,7);
  const __m128i dd_8 = _mm_set1_epi16(8);

  sad = (unsigned short*)occamAlignPtr(buf + sizeof(sad[0]), ALIGN);
  hsad0 = (unsigned short*)occamAlignPtr(sad + ndisp + 1 + dy0*ndisp, ALIGN);
  htext = (int*)occamAlignPtr((int*)(hsad0 + (height+dy1)*ndisp) + wsz2 + 2, ALIGN);
  cbuf0 = (uint8_t*)occamAlignPtr((uint8_t*)(htext + height + wsz2 + 2) + dy0*ndisp, ALIGN);

  for (x = 0; x < TABSZ; x++)
    tab[x] = (uint8_t)std::abs(x - ftzero);

  memset(hsad0 - dy0*ndisp, 0, (height + dy0 + dy1)*ndisp*sizeof(hsad0[0]));
  memset(htext - wsz2 - 1, 0, (height + wsz + 1)*sizeof(htext[0]));

  for (x = -wsz2-1; x < wsz2; x++) {
    hsad = hsad0 - dy0*ndisp;
    cbuf = cbuf0 + (x + wsz2 + 1)*cstep - dy0*ndisp;
    lptr = lptr0 + std::min(std::max(x, -lofs), width-lofs-1) - dy0*sstep;
    rptr = rptr0 + std::min(std::max(x, -rofs), width-rofs-1) - dy0*sstep;

    for (y = -dy0; y < height + dy1; y++, hsad += ndisp, cbuf += ndisp, lptr += sstep, rptr += sstep) {
      int lval = lptr[0];
      __m128i lv = _mm_set1_epi8((char)lval);
      __m128i z = _mm_setzero_si128();
      for (d = 0; d < ndisp; d += 16) {
	__m128i rv = _mm_loadu_si128((const __m128i*)(rptr + d));
	__m128i hsad_l = _mm_load_si128((__m128i*)(hsad + d));
	__m128i hsad_h = _mm_load_si128((__m128i*)(hsad + d + 8));
	__m128i diff = _mm_adds_epu8(_mm_subs_epu8(lv, rv), _mm_subs_epu8(rv, lv));
	_mm_store_si128((__m128i*)(cbuf + d), diff);
	hsad_l = _mm_add_epi16(hsad_l, _mm_unpacklo_epi8(diff,z));
	hsad_h = _mm_add_epi16(hsad_h, _mm_unpackhi_epi8(diff,z));
	_mm_store_si128((__m128i*)(hsad + d), hsad_l);
	_mm_store_si128((__m128i*)(hsad + d + 8), hsad_h);
      }
      htext[y] += tab[lval];
    }
  }

  for (y = 0; y < height; y++) {
    for (x = 0; x < lofs; x++)
      dptr[y*dstep + x] = FILTERED;
    for (x = lofs + width1; x < width; x++)
      dptr[y*dstep + x] = FILTERED;
  }
  dptr += lofs;

  for (x = 0; x < width1; x++, dptr++) {
    short* costptr = costp ? ((short*)costp) + lofs + x : &costbuf;
    int x0 = x - wsz2 - 1;
    int x1 = x + wsz2;
    const uint8_t* cbuf_sub = cbuf0 + ((x0 + wsz2 + 1) % (wsz + 1))*cstep - dy0*ndisp;
    cbuf = cbuf0 + ((x1 + wsz2 + 1) % (wsz + 1))*cstep - dy0*ndisp;
    hsad = hsad0 - dy0*ndisp;
    lptr_sub = lptr0 + std::min(std::max(x0, -lofs), width-1-lofs) - dy0*sstep;
    lptr = lptr0 + std::min(std::max(x1, -lofs), width-1-lofs) - dy0*sstep;
    rptr = rptr0 + std::min(std::max(x1, -rofs), width-1-rofs) - dy0*sstep;

    for (y = -dy0; y < height + dy1; y++, cbuf += ndisp, cbuf_sub += ndisp,
	   hsad += ndisp, lptr += sstep, lptr_sub += sstep, rptr += sstep) {
      int lval = lptr[0];
      __m128i lv = _mm_set1_epi8((char)lval);
      __m128i z = _mm_setzero_si128();
      for (d = 0; d < ndisp; d += 16) {
	__m128i rv = _mm_loadu_si128((const __m128i*)(rptr + d));
	__m128i hsad_l = _mm_load_si128((__m128i*)(hsad + d));
	__m128i hsad_h = _mm_load_si128((__m128i*)(hsad + d + 8));
	__m128i cbs = _mm_load_si128((const __m128i*)(cbuf_sub + d));
	__m128i diff = _mm_adds_epu8(_mm_subs_epu8(lv, rv), _mm_subs_epu8(rv, lv));
	__m128i diff_h = _mm_sub_epi16(_mm_unpackhi_epi8(diff, z), _mm_unpackhi_epi8(cbs, z));
	_mm_store_si128((__m128i*)(cbuf + d), diff);
	diff = _mm_sub_epi16(_mm_unpacklo_epi8(diff, z), _mm_unpacklo_epi8(cbs, z));
	hsad_h = _mm_add_epi16(hsad_h, diff_h);
	hsad_l = _mm_add_epi16(hsad_l, diff);
	_mm_store_si128((__m128i*)(hsad + d), hsad_l);
	_mm_store_si128((__m128i*)(hsad + d + 8), hsad_h);
      }
      htext[y] += tab[lval] - tab[lptr_sub[0]];
    }

    for (y = dy1; y <= wsz2; y++)
      htext[height+y] = htext[height+dy1-1];
    for (y = -wsz2-1; y < -dy0; y++)
      htext[y] = htext[-dy0];

    for (d = 0; d < ndisp; d++)
      sad[d] = (unsigned short)(hsad0[d-ndisp*dy0]*(wsz2 + 2 - dy0));

    hsad = hsad0 + (1 - dy0)*ndisp;
    for (y = 1 - dy0; y < wsz2; y++, hsad += ndisp)
      for (d = 0; d < ndisp; d += 16) {
	__m128i s0 = _mm_load_si128((__m128i*)(sad + d));
	__m128i s1 = _mm_load_si128((__m128i*)(sad + d + 8));
	__m128i t0 = _mm_load_si128((__m128i*)(hsad + d));
	__m128i t1 = _mm_load_si128((__m128i*)(hsad + d + 8));
	s0 = _mm_add_epi16(s0, t0);
	s1 = _mm_add_epi16(s1, t1);
	_mm_store_si128((__m128i*)(sad + d), s0);
	_mm_store_si128((__m128i*)(sad + d + 8), s1);
      }
    int tsum = 0;
    for (y = -wsz2-1; y < wsz2; y++)
      tsum += htext[y];

    for (y = 0; y < height; y++) {
      int minsad = std::numeric_limits<int>::max(), mind = -1;
      hsad = hsad0 + std::min(y + wsz2, height+dy1-1)*ndisp;
      hsad_sub = hsad0 + std::max(y - wsz2 - 1, -dy0)*ndisp;
      __m128i minsad8 = _mm_set1_epi16(std::numeric_limits<short>::max());
      __m128i mind8 = _mm_set1_epi16(0), d8 = d0_8, mask;

      for (d = 0; d < ndisp; d += 16) {
	__m128i u0 = _mm_load_si128((__m128i*)(hsad_sub + d));
	__m128i u1 = _mm_load_si128((__m128i*)(hsad + d));

	__m128i v0 = _mm_load_si128((__m128i*)(hsad_sub + d + 8));
	__m128i v1 = _mm_load_si128((__m128i*)(hsad + d + 8));

	__m128i usad8 = _mm_load_si128((__m128i*)(sad + d));
	__m128i vsad8 = _mm_load_si128((__m128i*)(sad + d + 8));

	u1 = _mm_sub_epi16(u1, u0);
	v1 = _mm_sub_epi16(v1, v0);
	usad8 = _mm_add_epi16(usad8, u1);
	vsad8 = _mm_add_epi16(vsad8, v1);

	mask = _mm_cmpgt_epi16(minsad8, usad8);
	minsad8 = _mm_min_epi16(minsad8, usad8);
	mind8 = _mm_max_epi16(mind8, _mm_and_si128(mask, d8));

	_mm_store_si128((__m128i*)(sad + d), usad8);
	_mm_store_si128((__m128i*)(sad + d + 8), vsad8);

	mask = _mm_cmpgt_epi16(minsad8, vsad8);
	minsad8 = _mm_min_epi16(minsad8, vsad8);

	d8 = _mm_add_epi16(d8, dd_8);
	mind8 = _mm_max_epi16(mind8, _mm_and_si128(mask, d8));
	d8 = _mm_add_epi16(d8, dd_8);
      }

      tsum += htext[y + wsz2] - htext[y - wsz2 - 1];
      if (tsum < texture_threshold) {
	dptr[y*dstep] = FILTERED;
	continue;
      }

      unsigned short OCCAM_DECL_ALIGNED(16) minsad_buf[8];
      unsigned short OCCAM_DECL_ALIGNED(16) mind_buf[8];
      _mm_store_si128((__m128i*)minsad_buf, minsad8);
      _mm_store_si128((__m128i*)mind_buf, mind8);
      for (d = 0; d < 8; d++)
	if (minsad > (int)minsad_buf[d] || (minsad == (int)minsad_buf[d] && mind > mind_buf[d])) {
	  minsad = minsad_buf[d];
	  mind = mind_buf[d];
	}

      if (uniqueness_ratio > 0) {
	int thresh = minsad + (minsad * uniqueness_ratio/100);
	__m128i thresh8 = _mm_set1_epi16((short)(thresh + 1));
	__m128i d1 = _mm_set1_epi16((short)(mind-1));
	__m128i d2 = _mm_set1_epi16((short)(mind+1));
	__m128i dd_16 = _mm_add_epi16(dd_8, dd_8);
	d8 = _mm_sub_epi16(d0_8, dd_16);

	for (d = 0; d < ndisp; d += 16) {
	  __m128i usad8 = _mm_load_si128((__m128i*)(sad + d));
	  __m128i vsad8 = _mm_load_si128((__m128i*)(sad + d + 8));
	  mask = _mm_cmpgt_epi16( thresh8, _mm_min_epi16(usad8,vsad8));
	  d8 = _mm_add_epi16(d8, dd_16);
	  if (!_mm_movemask_epi8(mask))
	    continue;
	  mask = _mm_cmpgt_epi16( thresh8, usad8);
	  mask = _mm_and_si128(mask, _mm_or_si128(_mm_cmpgt_epi16(d1,d8), _mm_cmpgt_epi16(d8,d2)));
	  if (_mm_movemask_epi8(mask))
	    break;
	  __m128i t8 = _mm_add_epi16(d8, dd_8);
	  mask = _mm_cmpgt_epi16(thresh8, vsad8);
	  mask = _mm_and_si128(mask, _mm_or_si128(_mm_cmpgt_epi16(d1,t8), _mm_cmpgt_epi16(t8,d2)));
	  if (_mm_movemask_epi8(mask))
	    break;
	}
	if (d < ndisp) {
	  dptr[y*dstep] = FILTERED;
	  continue;
	}
      }

      if (0 < mind && mind < ndisp - 1) {
	int p = sad[mind+1], n = sad[mind-1];
	d = p + n - 2*sad[mind] + std::abs(p - n);
	dptr[y*dstep] = (short)(((ndisp - mind - 1 + mindisp)*256 + (d != 0 ? (p-n)*256/d : 0) + 15) >> 4);
      }
      else
	dptr[y*dstep] = (short)((ndisp - mind - 1 + mindisp)*16);
      costptr[y*coststep] = sad[mind];
    }
  }
}
#endif

static void findStereoCorrespondenceBM(int width, int height,
				       const uint8_t* img0p, int img0_step,
				       const uint8_t* img1p, int img1_step,
				       uint8_t* dispp, int disp_step,
				       uint8_t* costp, int cost_step,
				       int sad_window_size,
				       int num_disparities,
				       int min_disparity,
				       int prefilter_cap,
				       int texture_threshold,
				       int uniqueness_ratio,
				       uint8_t* buf,
				       int _dy0,
				       int _dy1) {
#if OCCAM_SSE2
  if (occamHardwareSupport(OCCAM_CPU_SSE2)) {
    findStereoCorrespondenceBM_SSE2
      (width,height,img0p,img0_step,img1p,img1_step,
       dispp,disp_step,costp,cost_step,sad_window_size,
       num_disparities,min_disparity,prefilter_cap,texture_threshold,
       uniqueness_ratio,buf,_dy0,_dy1);
    return;
  }
#endif

  const int ALIGN = 16;
  int x, y, d;
  int wsz = sad_window_size;
  int wsz2 = wsz/2;
  int dy0 = std::min(_dy0, wsz2+1);
  int dy1 = std::min(_dy1, wsz2+1);
  int ndisp = num_disparities;
  int mindisp = min_disparity;
  int lofs = std::max(ndisp - 1 + mindisp, 0);
  int rofs = -std::min(ndisp - 1 + mindisp, 0);
  int width1 = width - rofs - ndisp + 1;
  int ftzero = prefilter_cap;
  short FILTERED = (short)((mindisp - 1) << DISPARITY_SHIFT);

  int* sad;
  int* hsad0;
  int* hsad;
  int* hsad_sub;
  int* htext;
  uint8_t *cbuf0, *cbuf;
  const uint8_t* lptr0 = img0p + lofs;
  const uint8_t* rptr0 = img1p + rofs;
  const uint8_t *lptr, *lptr_sub, *rptr;
  short* dptr = (short*)dispp;
  int sstep = (int)img0_step;
  int dstep = (int)(disp_step/sizeof(dptr[0]));
  int cstep = (height+dy0+dy1)*ndisp;
  int costbuf = 0;
  int coststep = costp ? (int)(cost_step/sizeof(costbuf)) : 0;
  const int TABSZ = 256;
  uint8_t tab[TABSZ];

  sad = (int*)occamAlignPtr(buf + sizeof(sad[0]), ALIGN);
  hsad0 = (int*)occamAlignPtr(sad + ndisp + 1 + dy0*ndisp, ALIGN);
  htext = (int*)occamAlignPtr((int*)(hsad0 + (height+dy1)*ndisp) + wsz2 + 2, ALIGN);
  cbuf0 = (uint8_t*)occamAlignPtr((uint8_t*)(htext + height + wsz2 + 2) + dy0*ndisp, ALIGN);

  for (x = 0; x < TABSZ; x++)
    tab[x] = (uint8_t)std::abs(x - ftzero);

  memset(hsad0 - dy0*ndisp, 0, (height + dy0 + dy1)*ndisp*sizeof(hsad0[0]));
  memset(htext - wsz2 - 1, 0, (height + wsz + 1)*sizeof(htext[0]));

  for (x = -wsz2-1; x < wsz2; x++) {
    hsad = hsad0 - dy0*ndisp; cbuf = cbuf0 + (x + wsz2 + 1)*cstep - dy0*ndisp;
    lptr = lptr0 + std::min(std::max(x, -lofs), width-lofs-1) - dy0*sstep;
    rptr = rptr0 + std::min(std::max(x, -rofs), width-rofs-1) - dy0*sstep;
    for (y = -dy0; y < height + dy1; y++, hsad += ndisp, cbuf += ndisp, lptr += sstep, rptr += sstep) {
      int lval = lptr[0];
      for (d = 0; d < ndisp; d++) {
	int diff = std::abs(lval - rptr[d]);
	cbuf[d] = (uint8_t)diff;
	hsad[d] = (int)(hsad[d] + diff);
      }
      htext[y] += tab[lval];
    }
  }

  for (y = 0; y < height; y++) {
    for (x = 0; x < lofs; x++)
      dptr[y*dstep + x] = FILTERED;
    for (x = lofs + width1; x < width; x++)
      dptr[y*dstep + x] = FILTERED;
  }
  dptr += lofs;

  for (x = 0; x < width1; x++, dptr++) {
    int* costptr = costp ? ((int*)costp)+ lofs + x : &costbuf;
    int x0 = x - wsz2 - 1, x1 = x + wsz2;
    const uint8_t* cbuf_sub = cbuf0 + ((x0 + wsz2 + 1) % (wsz + 1))*cstep - dy0*ndisp;
    cbuf = cbuf0 + ((x1 + wsz2 + 1) % (wsz + 1))*cstep - dy0*ndisp;
    hsad = hsad0 - dy0*ndisp;
    lptr_sub = lptr0 + std::min(std::max(x0, -lofs), width-1-lofs) - dy0*sstep;
    lptr = lptr0 + std::min(std::max(x1, -lofs), width-1-lofs) - dy0*sstep;
    rptr = rptr0 + std::min(std::max(x1, -rofs), width-1-rofs) - dy0*sstep;

    for (y = -dy0; y < height + dy1; y++, cbuf += ndisp, cbuf_sub += ndisp,
	   hsad += ndisp, lptr += sstep, lptr_sub += sstep, rptr += sstep) {
      int lval = lptr[0];
      for (d = 0; d < ndisp; d++) {
	int diff = std::abs(lval - rptr[d]);
	cbuf[d] = (uint8_t)diff;
	hsad[d] = hsad[d] + diff - cbuf_sub[d];
      }
      htext[y] += tab[lval] - tab[lptr_sub[0]];
    }

    for (y = dy1; y <= wsz2; y++)
      htext[height+y] = htext[height+dy1-1];
    for (y = -wsz2-1; y < -dy0; y++)
      htext[y] = htext[-dy0];

    for (d = 0; d < ndisp; d++)
      sad[d] = (int)(hsad0[d-ndisp*dy0]*(wsz2 + 2 - dy0));

    hsad = hsad0 + (1 - dy0)*ndisp;
    for (y = 1 - dy0; y < wsz2; y++, hsad += ndisp)
      for (d = 0; d < ndisp; d++)
	sad[d] = (int)(sad[d] + hsad[d]);
    int tsum = 0;
    for (y = -wsz2-1; y < wsz2; y++)
      tsum += htext[y];

    for (y = 0; y < height; y++) {
      int minsad = std::numeric_limits<int>::max();
      int mind = -1;
      hsad = hsad0 + std::min(y + wsz2, height+dy1-1)*ndisp;
      hsad_sub = hsad0 + std::max(y - wsz2 - 1, -dy0)*ndisp;

      for (d = 0; d < ndisp; d++) {
	int currsad = sad[d] + hsad[d] - hsad_sub[d];
	sad[d] = currsad;
	if (currsad < minsad) {
	  minsad = currsad;
	  mind = d;
	}
      }

      tsum += htext[y + wsz2] - htext[y - wsz2 - 1];
      if (tsum < texture_threshold) {
	dptr[y*dstep] = FILTERED;
	continue;
      }

      if (uniqueness_ratio > 0) {
	int thresh = minsad + (minsad * uniqueness_ratio/100);
	for (d = 0; d < ndisp; d++) {
	  if (sad[d] <= thresh && (d < mind-1 || d > mind+1))
	    break;
	}
	if (d < ndisp) {
	  dptr[y*dstep] = FILTERED;
	  continue;
	}
      }

      {
	sad[-1] = sad[1];
	sad[ndisp] = sad[ndisp-2];
	int p = sad[mind+1], n = sad[mind-1];
	d = p + n - 2*sad[mind] + std::abs(p - n);
	dptr[y*dstep] = (short)(((ndisp - mind - 1 + mindisp)*256 + (d != 0 ? (p-n)*256/d : 0) + 15) >> 4);
	costptr[y*coststep] = sad[mind];
      }
    }
  }
}

static void filterSpeckles(int width, int height,
			   uint8_t* img0p, int img0_step,
			   int newVal,
			   int maxSpeckleSize,
			   int maxDiff,
			   std::vector<uint8_t>& _buf) {
  int npixels = width*height;
  size_t bufSize = npixels*(int)(sizeof(short)*2 + sizeof(int) + sizeof(uint8_t));
  if (_buf.size() < bufSize)
    _buf.resize(bufSize);
  uint8_t* buf = &_buf[0];
  int i;
  int j;
  int dstep = (int)(img0_step/sizeof(short));
  int* labels = (int*)buf;
  buf += npixels*sizeof(labels[0]);
  short* wbuf = (short*)buf;
  buf += npixels*sizeof(short)*2;
  uint8_t* rtype = (uint8_t*)buf;
  int curlabel = 0;

  memset(labels, 0, npixels*sizeof(labels[0]));

  for (i = 0; i < height; i++) {
    short* ds = (short*)(img0p+img0_step*i);
    int* ls = labels + width*i;

    for (j = 0; j < width; j++) {
      if (ds[j] != newVal) {
	if (ls[j]) {
	  if (rtype[ls[j]])
	    ds[j] = (short)newVal;
	} else {
	  short* ws = wbuf;
	  short p_x = (short)j;
	  short p_y = (short)i;
	  curlabel++;
	  int count = 0;
	  ls[j] = curlabel;

	  while (ws >= wbuf) {
	    count++;
	    short* dpp = ((short*)(img0p+p_y*img0_step))+p_x;
	    short dp = *dpp;
	    int* lpp = labels + width*p_y + p_x;

	    if (p_y < height-1 && !lpp[+width] && dpp[+dstep] != newVal && std::abs(dp - dpp[+dstep]) <= maxDiff) {
	      lpp[+width] = curlabel;
	      *ws++ = p_x;
	      *ws++ = p_y+1;
	    }

	    if (p_y > 0 && !lpp[-width] && dpp[-dstep] != newVal && std::abs(dp - dpp[-dstep]) <= maxDiff) {
	      lpp[-width] = curlabel;
	      *ws++ = p_x;
	      *ws++ = p_y-1;
	    }

	    if (p_x < width-1 && !lpp[+1] && dpp[+1] != newVal && std::abs(dp - dpp[+1]) <= maxDiff) {
	      lpp[+1] = curlabel;
	      *ws++ = p_x+1;
	      *ws++ = p_y;
	    }

	    if (p_x > 0 && !lpp[-1] && dpp[-1] != newVal && std::abs(dp - dpp[-1]) <= maxDiff) {
	      lpp[-1] = curlabel;
	      *ws++ = p_x-1;
	      *ws++ = p_y;
	    }

	    ws-=2;
	    p_x = ws[0];
	    p_y = ws[1];
	  }

	  if (count <= maxSpeckleSize) {
	    rtype[ls[j]] = 1;
	    ds[j] = (short)newVal;
	  } else
	    rtype[ls[j]] = 0;
	}
      }
    }
  }
}


class BMStereoImpl : public OccamStereo, public OccamParameters {
  int prefilter_type;
  int prefilter_size;
  int prefilter_cap;
  int sad_window_size;
  int min_disparity;
  int num_disparities;
  int texture_threshold;
  int uniqueness_ratio;
  int speckle_range;
  int speckle_window_size;

  int get_prefilter_type() {
    return prefilter_type;
  }
  void set_prefilter_type(int value) {
    prefilter_type = value;
  }
  int get_prefilter_size() {
    return prefilter_size;
  }
  void set_prefilter_size(int value) {
    prefilter_size = value;
  }
  int get_prefilter_cap() {
    return prefilter_cap;
  }
  void set_prefilter_cap(int value) {
    prefilter_cap = value;
  }
  int get_sad_window_size() {
    return sad_window_size;
  }
  void set_sad_window_size(int value) {
    sad_window_size = value;
  }
  int get_min_disparity() {
    return min_disparity;
  }
  void set_min_disparity(int value) {
    min_disparity = value;
  }
  int get_num_disparities() {
    return num_disparities;
  }
  void set_num_disparities(int value) {
    num_disparities = value;
  }
  int get_texture_threshold() {
    return texture_threshold;
  }
  void set_texture_threshold(int value) {
    texture_threshold = value;
  }
  int get_uniqueness_ratio() {
    return uniqueness_ratio;
  }
  void set_uniqueness_ratio(int value) {
    uniqueness_ratio = value;
  }
  int get_speckle_range() {
    return speckle_range;
  }
  void set_speckle_range(int value) {
    speckle_range = value;
  }
  int get_speckle_window_size() {
    return speckle_window_size;
  }
  void set_speckle_window_size(int value) {
    speckle_window_size = value;
  }

public:
  BMStereoImpl()
    : prefilter_type(OCCAM_PREFILTER_XSOBEL),
      prefilter_size(9),
      prefilter_cap(31),
      sad_window_size(15),
      min_disparity(0),
      num_disparities(64),
      texture_threshold(10),
      uniqueness_ratio(15),
      speckle_range(120),
      speckle_window_size(400) {
    using namespace std::placeholders;
    registerParami(OCCAM_BM_PREFILTER_TYPE,"prefilter_type",OCCAM_SETTINGS,0,0,
		   std::bind(&BMStereoImpl::get_prefilter_type,this),
		   std::bind(&BMStereoImpl::set_prefilter_type,this,_1));
    {
      std::vector<std::pair<std::string,int> > values;
      values.push_back(std::make_pair("None", 0));
      values.push_back(std::make_pair("XSobel", 1));
      values.push_back(std::make_pair("Normalized", 2));
      setAllowedValues(OCCAM_BM_PREFILTER_TYPE, values);
    }
    registerParami(OCCAM_BM_PREFILTER_SIZE,"prefilter_size",OCCAM_SETTINGS,0,127,
		   std::bind(&BMStereoImpl::get_prefilter_size,this),
		   std::bind(&BMStereoImpl::set_prefilter_size,this,_1));
    registerParami(OCCAM_BM_PREFILTER_CAP,"prefilter_cap",OCCAM_SETTINGS,0,127,
		   std::bind(&BMStereoImpl::get_prefilter_cap,this),
		   std::bind(&BMStereoImpl::set_prefilter_cap,this,_1));
    registerParami(OCCAM_BM_SAD_WINDOW_SIZE,"sad_window_size",OCCAM_SETTINGS,0,0,
		   std::bind(&BMStereoImpl::get_sad_window_size,this),
		   std::bind(&BMStereoImpl::set_sad_window_size,this,_1));
    {
      std::vector<std::pair<std::string,int> > values;
      for (int j=3;j<64;j+=2) {
	std::stringstream sout;
	sout<<j;
	values.push_back(std::make_pair(sout.str(), j));
      }
      setAllowedValues(OCCAM_BM_SAD_WINDOW_SIZE, values);
    }
    registerParami(OCCAM_BM_MIN_DISPARITY,"min_disparity",OCCAM_SETTINGS,0,0,
		   std::bind(&BMStereoImpl::get_min_disparity,this),
		   std::bind(&BMStereoImpl::set_min_disparity,this,_1));
    registerParami(OCCAM_BM_NUM_DISPARITIES,"num_disparities",OCCAM_SETTINGS,0,0,
		   std::bind(&BMStereoImpl::get_num_disparities,this),
		   std::bind(&BMStereoImpl::set_num_disparities,this,_1));
    {
      std::vector<std::pair<std::string,int> > values;
      for (int j=16;j<256;j+=16) {
	std::stringstream sout;
	sout<<j;
	values.push_back(std::make_pair(sout.str(), j));
      }
      setAllowedValues(OCCAM_BM_NUM_DISPARITIES, values);
    }
    registerParami(OCCAM_BM_TEXTURE_THRESHOLD,"texture_threshold",OCCAM_SETTINGS,0,255,
		   std::bind(&BMStereoImpl::get_texture_threshold,this),
		   std::bind(&BMStereoImpl::set_texture_threshold,this,_1));
    registerParami(OCCAM_BM_UNIQUENESS_RATIO,"uniqueness_ratio",OCCAM_SETTINGS,0,255,
		   std::bind(&BMStereoImpl::get_uniqueness_ratio,this),
		   std::bind(&BMStereoImpl::set_uniqueness_ratio,this,_1));
    registerParami(OCCAM_BM_SPECKLE_RANGE,"speckle_range",OCCAM_SETTINGS,0,480,
		   std::bind(&BMStereoImpl::get_speckle_range,this),
		   std::bind(&BMStereoImpl::set_speckle_range,this,_1));
    registerParami(OCCAM_BM_SPECKLE_WINDOW_SIZE,"speckle_window_size",OCCAM_SETTINGS,0,1024,
		   std::bind(&BMStereoImpl::get_speckle_window_size,this),
		   std::bind(&BMStereoImpl::set_speckle_window_size,this,_1));

    setDefaultValuei(OCCAM_BM_PREFILTER_TYPE,OCCAM_PREFILTER_XSOBEL);
    setDefaultValuei(OCCAM_BM_PREFILTER_SIZE,9);
    setDefaultValuei(OCCAM_BM_PREFILTER_CAP,31);
    setDefaultValuei(OCCAM_BM_SAD_WINDOW_SIZE,15);
    setDefaultValuei(OCCAM_BM_MIN_DISPARITY,0);
    setDefaultValuei(OCCAM_BM_NUM_DISPARITIES,64);
    setDefaultValuei(OCCAM_BM_TEXTURE_THRESHOLD,10);
    setDefaultValuei(OCCAM_BM_UNIQUENESS_RATIO,12);
    setDefaultValuei(OCCAM_BM_SPECKLE_RANGE,120);
    setDefaultValuei(OCCAM_BM_SPECKLE_WINDOW_SIZE,400);
  }

  virtual int configure(int N,int width,int height,
			const double* const* D,const double* const* K,
			const double* const* R,const double* const* T) {
    return OCCAM_API_SUCCESS;
  }

  virtual int compute(int index,const OccamImage* img0,const OccamImage* img1,
		      OccamImage** dispp) {
    if (img0->backend != OCCAM_CPU ||
	img0->format != OCCAM_GRAY8 ||
	img0->width != img1->width ||
	img0->height != img1->height ||
	img0->format != img1->format ||
	img0->backend != img1->backend ||
	img0->step[0] != img1->step[0])
      return OCCAM_API_INVALID_PARAMETER;

    OccamImage* img0f = 0;
    OccamImage* img1f = 0;
    int bpp = 1;
    occamImageFormatBytesPerPixel(img0->format, &bpp);

    std::vector<uint8_t> slidingSumBuf;
    std::vector<uint8_t> costbuf;

    int width = img0->width;
    int height = img0->height;
    int ndisp = num_disparities;
    int mindisp = min_disparity;
    int wsz = sad_window_size;
    int bufSize0 = (int)((ndisp + 2)*sizeof(int));
    bufSize0 += (int)((height+wsz+2)*ndisp*sizeof(int));
    bufSize0 += (int)((height + wsz + 2)*sizeof(int));
    bufSize0 += (int)((height+wsz+2)*ndisp*(wsz+2)*sizeof(uint8_t) + 256);
    int bufSize1 = (int)((width + prefilter_size + 2) * sizeof(int) + 256);
    int bufSize2 = 0;
    if (speckle_range >= 0 && speckle_window_size > 0)
      bufSize2 = width*height*(sizeof(short)*2 + sizeof(int) + sizeof(uint8_t));
    int bufSize = std::max(bufSize0, std::max(bufSize1 * 2, bufSize2));
    if (slidingSumBuf.size()<bufSize)
      slidingSumBuf.resize(bufSize);
    short FILTERED = (short)((mindisp - 1) << DISPARITY_SHIFT);

    if (prefilter_type == OCCAM_PREFILTER_NONE) {
      occamCopyImage(img0, &img0f, 0);
      occamCopyImage(img1, &img1f, 0);

    } else if (prefilter_type == OCCAM_PREFILTER_XSOBEL) {
      img0f = new OccamImage;
      memset(img0f,0,sizeof(OccamImage));
      img0f->refcnt = 1;
      img0f->backend = img0->backend;
      img0f->format = img0->format;
      img0f->width = width;
      img0f->height = height;
      img0f->step[0] = (width*bpp+15)&~15;
      img0f->data[0] = new uint8_t[img0f->step[0]*height];

      img1f = new OccamImage;
      memset(img1f,0,sizeof(OccamImage));
      img1f->refcnt = 1;
      img1f->backend = img1->backend;
      img1f->format = img1->format;
      img1f->width = width;
      img1f->height = height;
      img1f->step[0] = (width*bpp+15)&~15;
      img1f->data[0] = new uint8_t[img1f->step[0]*height];

      prefilterXSobel(width, height,
    		      img0->data[0], img0->step[0],
    		      img0f->data[0], img0f->step[0],
    		      prefilter_cap);
      prefilterXSobel(width, height,
    		      img1->data[0], img1->step[0],
    		      img1f->data[0], img1f->step[0],
    		      prefilter_cap);

    } else if (prefilter_type == OCCAM_PREFILTER_NORMALIZED_RESPONSE) {
      img0f = new OccamImage;
      memset(img0f,0,sizeof(OccamImage));
      img0f->refcnt = 1;
      img0f->backend = img0->backend;
      img0f->format = img0->format;
      img0f->width = width;
      img0f->height = height;
      img0f->step[0] = (width*bpp+15)&~15;
      img0f->data[0] = new uint8_t[img0f->step[0]*height];

      img1f = new OccamImage;
      memset(img1f,0,sizeof(OccamImage));
      img1f->refcnt = 1;
      img1f->backend = img1->backend;
      img1f->format = img1->format;
      img1f->width = width;
      img1f->height = height;
      img1f->step[0] = (width*bpp+15)&~15;
      img1f->data[0] = new uint8_t[img1f->step[0]*height];

      prefilterNorm(width, height,
    		    img0->data[0], img0->step[0],
    		    img0f->data[0], img0f->step[0],
    		    prefilter_size,
    		    prefilter_cap,
    		    &slidingSumBuf[0]);

      prefilterNorm(width, height,
    		    img1->data[0], img1->step[0],
    		    img1f->data[0], img1f->step[0],
    		    prefilter_size,
    		    prefilter_cap,
    		    &slidingSumBuf[0]);
    }

    OccamImage* disp = new OccamImage;
    *dispp = disp;
    memset(disp,0,sizeof(OccamImage));
    disp->cid = strdup(img0->cid);
    memcpy(disp->timescale,img0->timescale,sizeof(disp->timescale));
    disp->time_ns = img0->time_ns;
    disp->index = img0->index;
    disp->refcnt = 1;
    disp->backend = img1->backend;
    disp->format = OCCAM_SHORT1;
    disp->width = width;
    disp->height = height;
    disp->step[0] = (width*2+15)&~15;
    disp->data[0] = new uint8_t[disp->step[0]*height];

    int costbuf_step = (width*sizeof(short)+15)&~15;
    int costbuf_size = height*costbuf_step;
    if (costbuf.size() < costbuf_size)
      costbuf.resize(costbuf_size);

    int SW2 = sad_window_size/2;

    findStereoCorrespondenceBM(width, height-sad_window_size,
    			       img0f->data[0]+img0f->step[0]*SW2, img0f->step[0],
    			       img1f->data[0]+img1f->step[0]*SW2, img1f->step[0],
    			       disp->data[0]+disp->step[0]*SW2, disp->step[0],
    			       &costbuf[0]+costbuf_step*SW2, costbuf_step,
    			       sad_window_size,
    			       num_disparities,
    			       min_disparity,
    			       prefilter_cap,
    			       texture_threshold,
    			       uniqueness_ratio,
    			       &slidingSumBuf[0],
    			       0,
    			       height-sad_window_size);

    if (speckle_range >= 0 && speckle_window_size > 0) {
      filterSpeckles(width, height,
    		     disp->data[0], disp->step[0],
    		     FILTERED,
    		     speckle_window_size,
    		     speckle_range,
    		     slidingSumBuf);
    }

    occamFreeImage(img0f);
    occamFreeImage(img1f);
    
    return OCCAM_API_SUCCESS;
  }
};

static OccamModuleFactory<BMStereoImpl> __module_factory
("bmcpu","Block Matching (CPU)",OCCAM_MODULE_STEREO,0,0);
extern void init_bm_stereo() {
  __module_factory.registerModule();
}

