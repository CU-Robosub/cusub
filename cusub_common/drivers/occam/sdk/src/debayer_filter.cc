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

#include "gl_utils.h"
#include "system.h"
#include "module_utils.h"
#include <string.h>
#include <vector>
#include <memory>
#include <iostream>

// CPU demosaicing derived from OpenCV b5cdc03b8143c9a1645e4b99026f073c1c490f81
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

// GLSL demosaicing derived from http://graphics.cs.williams.edu/papers/BayerJGT09/
/*
From http://jgt.akpeters.com/papers/McGuire08/

Efficient, High-Quality Bayer Demosaic Filtering on GPUs

Morgan McGuire

This paper appears in issue Volume 13, Number 4.
---------------------------------------------------------
Copyright (c) 2008, Morgan McGuire. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



#if OCCAM_SSE2
static int bayer2RGB_SIMD(bool use_simd,
			  const uint8_t* bayer, int bayer_step,
			  uint8_t* dst, int width, int blue) {
  if (!use_simd)
    return 0;
  /*
    B G B G | B G B G | B G B G | B G B G
    G R G R | G R G R | G R G R | G R G R
    B G B G | B G B G | B G B G | B G B G
  */

  __m128i delta1 = _mm_set1_epi16(1), delta2 = _mm_set1_epi16(2);
  __m128i mask = _mm_set1_epi16(blue < 0 ? -1 : 0), z = _mm_setzero_si128();
  __m128i masklo = _mm_set1_epi16(0x00ff);
  const uint8_t* bayer_end = bayer + width;

  for( ; bayer <= bayer_end - 18; bayer += 14, dst += 42 ) {
    __m128i r0 = _mm_loadu_si128((const __m128i*)bayer);
    __m128i r1 = _mm_loadu_si128((const __m128i*)(bayer+bayer_step));
    __m128i r2 = _mm_loadu_si128((const __m128i*)(bayer+bayer_step*2));

    __m128i b1 = _mm_add_epi16(_mm_and_si128(r0, masklo), _mm_and_si128(r2, masklo));
    __m128i nextb1 = _mm_srli_si128(b1, 2);
    __m128i b0 = _mm_add_epi16(b1, nextb1);
    b1 = _mm_srli_epi16(_mm_add_epi16(nextb1, delta1), 1);
    b0 = _mm_srli_epi16(_mm_add_epi16(b0, delta2), 2);
    // b0 b2 ... b14 b1 b3 ... b15
    b0 = _mm_packus_epi16(b0, b1);

    __m128i g0 = _mm_add_epi16(_mm_srli_epi16(r0, 8), _mm_srli_epi16(r2, 8));
    __m128i g1 = _mm_and_si128(r1, masklo);
    g0 = _mm_add_epi16(g0, _mm_add_epi16(g1, _mm_srli_si128(g1, 2)));
    g1 = _mm_srli_si128(g1, 2);
    g0 = _mm_srli_epi16(_mm_add_epi16(g0, delta2), 2);
    // g0 g2 ... g14 g1 g3 ... g15
    g0 = _mm_packus_epi16(g0, g1);

    r0 = _mm_srli_epi16(r1, 8);
    r1 = _mm_add_epi16(r0, _mm_srli_si128(r0, 2));
    r1 = _mm_srli_epi16(_mm_add_epi16(r1, delta1), 1);
    // r0 r2 ... r14 r1 r3 ... r15
    r0 = _mm_packus_epi16(r0, r1);

    b1 = _mm_and_si128(_mm_xor_si128(b0, r0), mask);
    b0 = _mm_xor_si128(b0, b1);
    r0 = _mm_xor_si128(r0, b1);

    // b1 g1 b3 g3 b5 g5...
    b1 = _mm_unpackhi_epi8(b0, g0);
    // b0 g0 b2 g2 b4 g4 ....
    b0 = _mm_unpacklo_epi8(b0, g0);

    // r1 0 r3 0 r5 0 ...
    r1 = _mm_unpackhi_epi8(r0, z);
    // r0 0 r2 0 r4 0 ...
    r0 = _mm_unpacklo_epi8(r0, z);

    // 0 b0 g0 r0 0 b2 g2 r2 ...
    g0 = _mm_slli_si128(_mm_unpacklo_epi16(b0, r0), 1);
    // 0 b8 g8 r8 0 b10 g10 r10 ...
    g1 = _mm_slli_si128(_mm_unpackhi_epi16(b0, r0), 1);

    // b1 g1 r1 0 b3 g3 r3 0 ...
    r0 = _mm_unpacklo_epi16(b1, r1);
    // b9 g9 r9 0 b11 g11 r11 0 ...
    r1 = _mm_unpackhi_epi16(b1, r1);

    // 0 b0 g0 r0 b1 g1 r1 0 ...
    b0 = _mm_srli_si128(_mm_unpacklo_epi32(g0, r0), 1);
    // 0 b4 g4 r4 b5 g5 r5 0 ...
    b1 = _mm_srli_si128(_mm_unpackhi_epi32(g0, r0), 1);

    _mm_storel_epi64((__m128i*)(dst-1+0), b0);
    _mm_storel_epi64((__m128i*)(dst-1+6*1), _mm_srli_si128(b0, 8));
    _mm_storel_epi64((__m128i*)(dst-1+6*2), b1);
    _mm_storel_epi64((__m128i*)(dst-1+6*3), _mm_srli_si128(b1, 8));

    // 0 b8 g8 r8 b9 g9 r9 0 ...
    g0 = _mm_srli_si128(_mm_unpacklo_epi32(g1, r1), 1);
    // 0 b12 g12 r12 b13 g13 r13 0 ...
    g1 = _mm_srli_si128(_mm_unpackhi_epi32(g1, r1), 1);

    _mm_storel_epi64((__m128i*)(dst-1+6*4), g0);
    _mm_storel_epi64((__m128i*)(dst-1+6*5), _mm_srli_si128(g0, 8));

    _mm_storel_epi64((__m128i*)(dst-1+6*6), g1);
  }

  return (int)(bayer - (bayer_end - width));
}
#else // OCCAM_SSE2
static int bayer2RGB_SIMD(bool use_simd, const uint8_t* bayer, int bayer_step, uint8_t* dst, int width, int blue) {
  return 0;
}
#endif // OCCAM_SSE2

static void bayer2RGB(bool use_simd,
		      const uint8_t* srcp, uint8_t* dstp,
		      int src_step, int dst_step,
		      int start_with_green, int blue,
		      int width, int height,
		      int first_row, int last_row) {
  int dcn = 3;
  int dcn2 = dcn << 1;

  const uint8_t* bayer0 = srcp + src_step * first_row;

  uint8_t* dst0 = dstp + (first_row + 1) * dst_step + dcn + 1;

  if (first_row % 2) {
    blue = -blue;
    start_with_green = !start_with_green;
  }

  for (int i = first_row; i < last_row; bayer0 += src_step, dst0 += dst_step, ++i ) {
    int t0, t1;
    const uint8_t* bayer = bayer0;
    uint8_t* dst = dst0;
    const uint8_t* bayer_end = bayer + width;

    // in case of when size.width <= 2
    if( width <= 0 ) {
      dst[-4] = dst[-3] = dst[-2] = dst[width*dcn-1] =
	dst[width*dcn] = dst[width*dcn+1] = 0;
      continue;
    }

    if( start_with_green ) {
      t0 = (bayer[1] + bayer[src_step*2+1] + 1) >> 1;
      t1 = (bayer[src_step] + bayer[src_step+2] + 1) >> 1;

      dst[-blue] = (uint8_t)t0;
      dst[0] = bayer[src_step+1];
      dst[blue] = (uint8_t)t1;

      bayer++;
      dst += dcn;
    }

    // simd optimization only for dcn == 3
    int delta = dcn == 4 ? 0 : bayer2RGB_SIMD(use_simd, bayer, src_step, dst, width, blue);
    bayer += delta;
    dst += delta*dcn;

    if( blue > 0 ) {
      for( ; bayer <= bayer_end - 2; bayer += 2, dst += dcn2 ) {
	t0 = (bayer[0] + bayer[2] + bayer[src_step*2] +	
      bayer[src_step*2+2] + 2) >> 2;
	t1 = (bayer[1] + bayer[src_step] +
	      bayer[src_step+2] + bayer[src_step*2+1]+2) >> 2;
	dst[-1] = (uint8_t)t0;
	dst[0] = (uint8_t)t1;
	dst[1] = bayer[src_step+1];

	t0 = (bayer[2] + bayer[src_step*2+2] + 1) >> 1;
	t1 = (bayer[src_step+1] + bayer[src_step+3] + 1) >> 1;
	dst[2] = (uint8_t)t0;
	dst[3] = bayer[src_step+2];
	dst[4] = (uint8_t)t1;
      }
    } else {
      for( ; bayer <= bayer_end - 2; bayer += 2, dst += dcn2 ) {
	t0 = (bayer[0] + bayer[2] + bayer[src_step*2] +
	      bayer[src_step*2+2] + 2) >> 2;
	t1 = (bayer[1] + bayer[src_step] +
	      bayer[src_step+2] + bayer[src_step*2+1]+2) >> 2;
	dst[1] = (uint8_t)t0;
	dst[0] = (uint8_t)t1;
	dst[-1] = bayer[src_step+1];

	t0 = (bayer[2] + bayer[src_step*2+2] + 1) >> 1;
	t1 = (bayer[src_step+1] + bayer[src_step+3] + 1) >> 1;
	dst[4] = (uint8_t)t0;
	dst[3] = bayer[src_step+2];
	dst[2] = (uint8_t)t1;
      }
    }

    // if skip one pixel at the end of row
    if( bayer < bayer_end ) {
      t0 = (bayer[0] + bayer[2] + bayer[src_step*2] +
	    bayer[src_step*2+2] + 2) >> 2;
      t1 = (bayer[1] + bayer[src_step] +
	    bayer[src_step+2] + bayer[src_step*2+1]+2) >> 2;
      dst[-blue] = (uint8_t)t0;
      dst[0] = (uint8_t)t1;
      dst[blue] = bayer[src_step+1];
      bayer++;
      dst += dcn;
    }

    // fill the last and the first pixels of row accordingly
    dst0[-4] = dst0[-1];
    dst0[-3] = dst0[0];
    dst0[-2] = dst0[1];
    dst0[width*dcn-1] = dst0[width*dcn-4];
    dst0[width*dcn] = dst0[width*dcn-3];
    dst0[width*dcn+1] = dst0[width*dcn-2];

    blue = -blue;
    start_with_green = !start_with_green;
  }
}

#ifdef OCCAM_OPENGL_SUPPORT

class GLDebayerFilter {
  std::vector<float> array;
  std::vector<GLuint> indices;
  std::shared_ptr<GLProgramBind> prog;
  GLuint sourceSize;
  GLuint firstRed;
  GLuint vbo, ibo;
  void init(int width, int height) {
    GLBlitRect(array, indices, 4,
	       0,0,width,height,
	       0,0,width,height,
	       width,height);

    GL_CHECK(glGenBuffers(1, &vbo));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo));
    GL_CHECK(glBufferData(GL_ARRAY_BUFFER, array.size() * sizeof(float),
			  &array[0], GL_STATIC_DRAW));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));

    GL_CHECK(glGenBuffers(1, &ibo));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo));
    GL_CHECK(glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint),
			  &indices[0], GL_STATIC_DRAW));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

    auto vshader = R"EOS(
uniform vec4 sourceSize;
uniform vec2 firstRed;
varying vec4 center;
varying vec4 xCoord;
varying vec4 yCoord;

void main(void) {
  center.xy = gl_MultiTexCoord0.xy;
  center.zw = gl_MultiTexCoord0.xy * sourceSize.xy + firstRed;
    
  vec2 invSize = sourceSize.zw;
  xCoord = center.x + vec4(-2.0 * invSize.x, -invSize.x, invSize.x, 2.0 * invSize.x);
  yCoord = center.y + vec4(-2.0 * invSize.y, -invSize.y, invSize.y, 2.0 * invSize.y);
    
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
)EOS";
    auto fshader = R"EOS(
uniform sampler2D source;

varying vec4 center;
varying vec4 yCoord;
varying vec4 xCoord;

void main(void) {
#define fetch(x, y) texture2D(source, vec2(x, y)).r

  float C = texture2D(source, center.xy).r; // ( 0, 0)
  const vec4 kC = vec4( 4.0,  6.0,  5.0,  5.0) / 8.0;

  vec2 alternate = mod(floor(center.zw), 2.0);

  vec4 Dvec = vec4(fetch(xCoord[1], yCoord[1]),  // (-1,-1)
		   fetch(xCoord[1], yCoord[2]),  // (-1, 1)
		   fetch(xCoord[2], yCoord[1]),  // ( 1,-1)
		   fetch(xCoord[2], yCoord[2])); // ( 1, 1)

  vec4 PATTERN = (kC.xyz * C).xyzz;

  Dvec.xy += Dvec.zw;
  Dvec.x  += Dvec.y;

  vec4 value = vec4(fetch(center.x, yCoord[0]),   // ( 0,-2)
		    fetch(center.x, yCoord[1]),   // ( 0,-1)
		    fetch(xCoord[0], center.y),   // (-1, 0)
		    fetch(xCoord[1], center.y));  // (-2, 0)

  vec4 temp = vec4(fetch(center.x, yCoord[3]),   // ( 0, 2)
		   fetch(center.x, yCoord[2]),   // ( 0, 1)
		   fetch(xCoord[3], center.y),   // ( 2, 0)
		   fetch(xCoord[2], center.y));  // ( 1, 0)

  const vec4 kA = vec4(-1.0, -1.5,  0.5, -1.0) / 8.0;
  const vec4 kB = vec4( 2.0,  0.0,  0.0,  4.0) / 8.0;
  const vec4 kD = vec4( 0.0,  2.0, -1.0, -1.0) / 8.0;

#define kE (kA.xywz)
#define kF (kB.xywz)

  value += temp;

#define A (value[0])
#define B (value[1])
#define D (Dvec.x)
#define E (value[2])
#define F (value[3])

  PATTERN.yzw += (kD.yz * D).xyy;

  PATTERN += (kA.xyz * A).xyzx + (kE.xyw * E).xyxz;
  PATTERN.xw += kB.xw * B;
  PATTERN.xz += kF.xz * F;

  vec3 rgb = (alternate.y == 0.0) ?
    ((alternate.x == 0.0) ?
     vec3(C, PATTERN.xy) :
     vec3(PATTERN.z, C, PATTERN.w)) :
    ((alternate.x == 0.0) ?
     vec3(PATTERN.w, C, PATTERN.z) :
     vec3(PATTERN.yx, C));

  gl_FragColor.rgb = rgb;
}
)EOS";
    if (!bool(prog)) {
      prog = std::make_shared<GLProgramBind>(vshader,fshader);
      sourceSize = glGetUniformLocation(prog->program(),"sourceSize");
      firstRed = glGetUniformLocation(prog->program(),"firstRed");
    }
  }
public:
  GLDebayerFilter()
    : vbo(0), ibo(0) {
  }
  ~GLDebayerFilter() {
    if (vbo)
      GL_CHECK(glDeleteBuffers(1, &vbo));
    if (ibo)
      GL_CHECK(glDeleteBuffers(1, &ibo));
  }
  GLDebayerFilter(const GLDebayerFilter& x) = delete;
  GLDebayerFilter& operator= (const GLDebayerFilter& rhs) = delete;
  void operator() (GLuint srctex, int width, int height) {
    if (!vbo)
      init(width, height);

    GL_CHECK(glEnableClientState(GL_VERTEX_ARRAY));
    GL_CHECK(glEnableClientState(GL_TEXTURE_COORD_ARRAY));

    GL_CHECK(glBindTexture(GL_TEXTURE_2D, srctex));
    GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT));
    GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo));
    GL_CHECK(glTexCoordPointer(2, GL_FLOAT, 4*sizeof(float), (GLvoid*)(2*sizeof(float))));
    GL_CHECK(glVertexPointer(2, GL_FLOAT, 4*sizeof(float), (GLvoid*)(0)));

    GL_CHECK(glUseProgram(prog->program()));
    GL_CHECK(glUniform4f(sourceSize, float(width), float(height),1.f/width, 1.f/height));
    GL_CHECK(glUniform2f(firstRed, 1.f, 1.f));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo));
    GL_CHECK(glDrawElements(GL_QUADS, indices.size(), GL_UNSIGNED_INT, 0));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    GL_CHECK(glUseProgram(0));

    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));
    GL_CHECK(glBindTexture(GL_TEXTURE_2D, 0));
    GL_CHECK(glDisableClientState(GL_TEXTURE_COORD_ARRAY));
    GL_CHECK(glDisableClientState(GL_VERTEX_ARRAY));
  }
};

#endif // OCCAM_OPENGL_SUPPORT

class OccamDebayerFilterImpl : public OccamImageFilter {
  bool use_simd;
#ifdef OCCAM_OPENGL_SUPPORT
  GLDebayerFilter gldebayer;
#endif // OCCAM_OPENGL_SUPPORT
public:
  OccamDebayerFilterImpl() {
    use_simd = occamHardwareSupport(OCCAM_CPU_SSE2);
  }

  virtual int compute(const OccamImage* img0,OccamImage** img1out) {
    OccamImage* img1 = 0;
    if (img0->format != OCCAM_GRAY8)
      return 0;
    if (img0->backend == OCCAM_OPENGL) {
#ifdef OCCAM_OPENGL_SUPPORT
      img1 = new OccamImage;
      memset(img1,0,sizeof(OccamImage));
      img1->cid = strdup(img0->cid);
      memcpy(img1->timescale,img0->timescale,sizeof(img1->timescale));
      img1->time_ns = img0->time_ns;
      img1->index = img0->index;
      img1->refcnt = 1;
      img1->backend = OCCAM_OPENGL;
      img1->format = OCCAM_RGB24;
      img1->width = img0->width;
      img1->height = img0->height;
      img1->texture[0] = createGLTexture(img1->width,img1->height,OCCAM_RGB24);
      GLFBO b(img1->texture[0], img1->width, img1->height);
      gldebayer(img0->texture[0],img0->width,img0->height);
#endif // OCCAM_OPENGL_SUPPORT
    } else if (img0->backend == OCCAM_CPU) {
      img1 = new OccamImage;
      memset(img1,0,sizeof(OccamImage));
      img1->cid = strdup(img0->cid);
      memcpy(img1->timescale,img0->timescale,sizeof(img1->timescale));
      img1->time_ns = img0->time_ns;
      img1->index = img0->index;
      img1->refcnt = 1;
      img1->backend = OCCAM_CPU;
      img1->format = OCCAM_RGB24;
      img1->width = img0->width;
      img1->height = img0->height;
      memset(img1->step,0,sizeof(img1->step));
      memset(img1->data,0,sizeof(img1->data));
      img1->step[0] = ((img0->width*3)+15)&~15;
      img1->data[0] = new uint8_t[img1->height*img1->step[0]];
      int start_with_green = 0;
      int blue = -1;
      bayer2RGB(use_simd,
		img0->data[0], img1->data[0],
		img0->step[0], img1->step[0],
		start_with_green, blue,
		img0->width-2, img0->height,
		0, img0->height-2);
    }
    *img1out = img1;
    return OCCAM_API_SUCCESS;
  }
};

static OccamModuleFactory<OccamDebayerFilterImpl> __module_factory
("dbf","Debayer",OCCAM_MODULE_DEBAYER_FILTER,0,0);
void init_debayer_filter() {
  __module_factory.registerModule();
}
