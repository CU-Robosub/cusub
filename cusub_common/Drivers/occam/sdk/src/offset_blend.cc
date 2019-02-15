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

#include "indigo.h"
#include "gl_utils.h"
#include "module_utils.h"
#include <string.h>
#include <assert.h>
#include <algorithm>
#include <vector>
#include <memory>
#undef min
#undef max

class CPUOffsetBlender {
  std::vector<int> si_x;
  std::vector<int> si_y;
  std::vector<int> si_width;
  std::vector<int> si_height;
  std::vector<int> offsetx;
  std::vector<int> offsety;

public:
  void configure(int N,
		 const int* _si_x,
		 const int* _si_y,
		 const int* _si_width,
		 const int* _si_height,
		 const int* _offsetx,
		 const int* _offsety) {
    si_x.assign(_si_x,_si_x+N);
    si_y.assign(_si_y,_si_y+N);
    si_width.assign(_si_width,_si_width+N);
    si_height.assign(_si_height,_si_height+N);
    offsetx.assign(_offsetx,_offsetx+N);
    offsety.assign(_offsety,_offsety+N);
  }

  OccamImage* operator() (const OccamImage* const* img0) {
    int N = si_x.size();
    assert(img0[0]->backend == OCCAM_CPU);
    assert(img0[0]->format == OCCAM_GRAY8 || img0[0]->format == OCCAM_RGB24);

    OccamImage* img1 = new OccamImage;
    memset(img1,0,sizeof(OccamImage));
    img1->cid = strdup(img0[0]->cid);
    memcpy(img1->timescale,img0[0]->timescale,sizeof(img1->timescale));
    img1->time_ns = img0[0]->time_ns;
    img1->index = img0[0]->index;
    img1->refcnt = 1;
    img1->backend = img0[0]->backend;
    img1->format = img0[0]->format;
    int channels = 1;
    switch (img1->format) {
    case OCCAM_GRAY8: channels = 1; break;
    case OCCAM_RGB24: channels = 3; break;
    default:
      assert(0);
      return 0;
    }

    ////////////////////////////////////////////////////////////////////

    int width = 0;
    int height = 0;
    for (int si_j=0;si_j<N;++si_j) {
      width += si_width[si_j];
      height = std::max(height,si_height[si_j]);
    }
    img1->width = width;
    img1->height = height;
    img1->step[0] = ((width*channels)+15)&~15;
    img1->data[0] = new uint8_t[img1->height*img1->step[0]];

    int left_x = width;
    int right_x = 0;
    int top_y = 0;
    int bottom_y = height;

    std::vector<int> dstx0;
    dstx0.reserve(N);
    int dstx = 0;
    int n = 0;
    for (int si_j=0;si_j<N;++si_j) {
      int linebytes = si_width[si_j]*channels;
      int dst_step = img1->step[0];
      int src_step = img0[si_j]->step[0];
      uint8_t* dstp = img1->data[0]+dstx*channels;
      const uint8_t* srcp = img0[si_j]->data[0]+src_step*si_y[si_j]+si_x[si_j]*channels;

      int offy = std::max(-si_height[si_j],std::min(si_height[si_j],n>=N?0:offsety[n]));
      int si_heightj = si_height[si_j];
      if (offy<0) {
	si_heightj += offy;
	srcp += src_step*-offy;
      } else {
	si_heightj -= offy;
	dstp += dst_step*offy;
      }

      for (int y=0;y<si_heightj;++y,srcp+=src_step,dstp+=dst_step)
	memcpy(dstp,srcp,linebytes);

      left_x = std::max(0,std::min(left_x,dstx));
      right_x = std::min(width,std::max(right_x,dstx+si_width[si_j]));
      top_y = std::max(0,std::max(top_y,offy));
      bottom_y = std::min(std::min(bottom_y,si_height[si_j]+offy),height);

      dstx0.push_back(dstx);
      dstx += si_width[si_j];
      dstx -= std::min(si_width[si_j],n>=N?0:offsetx[n]);
      ++n;
    }

    ////////////////////////////////////////////////////////////////////

    for (int j=0;j<N-1;++j) {
      int si0 = j, si1 = j+1;
      int img0_dst_x = dstx0[j];
      int img1_dst_x = dstx0[j+1];
      int overlap_width = img0_dst_x + si_width[si0] - img1_dst_x;
      if (!overlap_width)
	continue;

      int offy0 = j>=N?0:std::max(-si_height[si0],std::min(si_height[si0],offsety[j]));
      int offy1 = j+1>=N?0:std::max(-si_height[si1],std::min(si_height[si1],offsety[j+1]));

      int dst_step = img1->step[0];
      int src_step0 = img0[j]->step[0];
      int src_step1 = img0[j+1]->step[0];
      uint8_t* dstp = img1->data[0]+img1_dst_x*channels;
      int fade_step = (2<<16) / overlap_width;
      for (int y=0;y<height;++y,dstp+=dst_step) {
	int y0 = y-offy0;
	if (y0 < 0 || y0 >= si_height[si0])
	  continue;
	int y1 = y-offy1;
	if (y1 < 0 || y1 >= si_height[si1])
	  continue;
	const uint8_t* srcp0 = img0[j]->data[0]+src_step0*(si_y[si0]+y0)+(si_x[si0]+si_width[si0]-overlap_width)*channels;
	const uint8_t* srcp1 = img0[j+1]->data[0]+src_step0*(si_y[si1]+y1)+si_x[si1]*channels;

	int fade0 = (2<<16), fade1 = 0;
	for (int x=0,xx=overlap_width*channels;x<xx;x+=channels) {
	  for (int c=0;c<channels;++c) {
	    int v0 = int(srcp0[x+c]);
	    int v1 = int(srcp1[x+c]);
	    int v = ((v0*fade0)>>17) + ((v1*fade1)>>17);
	    dstp[x+c] = v;
	  }
	  fade0 -= fade_step;
	  fade1 += fade_step;
	}
      }
    }

    ////////////////////////////////////////////////////////////////////

    OccamImage* img2 = new OccamImage;
    memset(img2,0,sizeof(OccamImage));
    img2->cid = strdup(img0[0]->cid);
    memcpy(img2->timescale,img0[0]->timescale,sizeof(img1->timescale));
    img2->time_ns = img0[0]->time_ns;
    img2->index = img0[0]->index;
    img2->refcnt = 1;
    img2->backend = img0[0]->backend;
    img2->format = img0[0]->format;
    img2->width = right_x-left_x;
    img2->height = bottom_y-top_y;
    memset(img2->step,0,sizeof(img2->step));
    memset(img2->data,0,sizeof(img2->data));
    img2->step[0] = ((img2->width*channels)+15)&~15;
    img2->data[0] = new uint8_t[img2->height*img2->step[0]];
    int linebytes = img2->width*channels;
    int src_step = img1->step[0];
    int dst_step = img2->step[0];
    const uint8_t* srcp = img1->data[0]+top_y*src_step+left_x*channels;
    uint8_t* dstp = img2->data[0];
    for (int y=0,yy=img2->height;y<yy;++y,srcp+=src_step,dstp+=dst_step)
      memcpy(dstp,srcp,linebytes);

    occamFreeImage(img1);
    return img2;
  }
};

#ifdef OCCAM_OPENGL_SUPPORT
class GLOffsetBlender {
  bool init_required = true;
  int N = 0;
  std::vector<int> si_x;
  std::vector<int> si_y;
  std::vector<int> si_width;
  std::vector<int> si_height;
  std::vector<int> offsetx;
  std::vector<int> offsety;
  int N0;
  std::vector<int> si_x0;
  std::vector<int> si_y0;
  std::vector<int> si_width0;
  std::vector<int> si_height0;
  std::vector<int> offsetx0;
  std::vector<int> offsety0;

  int width1, height1;
  int width2, height2;

  std::vector<float> array1;
  std::vector<GLuint> indices1;
  GLuint vbo1, ibo1;

  std::shared_ptr<GLProgramBind> prog;
  GLuint sampler0, sampler1;
  std::vector<float> array2;
  std::vector<GLuint> indices2;
  GLuint vbo2, ibo2;

  std::vector<float> array3;
  std::vector<GLuint> indices3;
  GLuint vbo3, ibo3;

  void init(int src_width, int src_height) {
    if (!init_required)
      return;
    init_required = false;

    ////////////////////////////////////////////////////////////////////

    int width = 0;
    int height = 0;
    for (int si_j=0;si_j<N;++si_j) {
      width += si_width[si_j];
      height = std::max(height,si_height[si_j]);
    }
    width1 = width;
    height1 = height;

    int left_x = width;
    int right_x = 0;
    int top_y = 0;
    int bottom_y = height;

    array1.clear();
    indices1.clear();

    std::vector<int> dstx0;
    dstx0.reserve(N);
    int dstx = 0;
    int n = 0;
    for (int si_j=0;si_j<N;++si_j) {
      int dx0 = dstx;
      int dy0 = 0;
      int sx0 = si_x[si_j];
      int sy0 = si_y[si_j];

      int offy = std::max(-si_height[si_j],std::min(si_height[si_j],n>=N?0:offsety[n]));
      int si_heightj = si_height[si_j];
      if (offy<0) {
	si_heightj += offy;
	sy0 += -offy;
      } else {
	si_heightj -= offy;
	dy0 += offy;
      }

      GLBlitRect(array1,indices1,4,
		 sx0,sy0,src_width,src_height,
		 dx0,dy0,width1,height1,
		 si_width[si_j], si_heightj);

      left_x = std::max(0,std::min(left_x,dstx));
      right_x = std::min(width,std::max(right_x,dstx+si_width[si_j]));
      top_y = std::max(0,std::max(top_y,offy));
      bottom_y = std::min(std::min(bottom_y,si_height[si_j]+offy),height);

      dstx0.push_back(dstx);
      dstx += si_width[si_j];
      dstx -= std::min(si_width[si_j],n>=N?0:offsetx[n]);
      ++n;
    }

    if (vbo1)
      GL_CHECK(glDeleteBuffers(1,&vbo1));
    if (ibo1)
      GL_CHECK(glDeleteBuffers(1,&ibo1));

    GL_CHECK(glGenBuffers(1, &vbo1));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo1));
    GL_CHECK(glBufferData(GL_ARRAY_BUFFER, array1.size() * sizeof(float),
			  &array1[0], GL_STATIC_DRAW));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));

    GL_CHECK(glGenBuffers(1, &ibo1));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo1));
    GL_CHECK(glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices1.size() * sizeof(GLuint),
			  &indices1[0], GL_STATIC_DRAW));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

    ////////////////////////////////////////////////////////////////////

    array2.clear();
    indices2.clear();

    for (int j=0;j<N-1;++j) {
      int si0 = j, si1 = j+1;
      int img0_dst_x = dstx0[j];
      int img1_dst_x = dstx0[j+1];
      int overlap_width = img0_dst_x + si_width[si0] - img1_dst_x;
      if (!overlap_width)
	continue;
      int offy0 = j>=N?0:std::max(-si_height[si0],std::min(si_height[si0],offsety[j]));
      int offy1 = j+1>=N?0:std::max(-si_height[si1],std::min(si_height[si1],offsety[j+1]));
      int dx0 = img1_dst_x;
      int dy0 = 0;
      int y0 = -offy0;
      int y1 = -offy1;
      int sx0 = (si_x[si0]+si_width[si0]-overlap_width);
      int sy0 = (si_y[si0]+y0);
      int sx1 = si_x[si1];
      int sy1 = (si_y[si1]+y1);
      GLBlitRect2(array2,
		  indices2,
		  6,
		  sx0,sy0,src_width,src_height,
		  sx1,sy1,src_width,src_height,
		  dx0,dy0,width1,height1,
		  overlap_width, height);
    }

    if (array2.empty())
      return;

    if (!bool(prog)) {
      auto fshader = R"EOS(
uniform sampler2D sampler0;
uniform sampler2D sampler1;
varying float a;

void main(void) {
  vec4 t0 = texture2D(sampler0, vec2(gl_TexCoord[0]));
  vec4 t1 = texture2D(sampler1, vec2(gl_TexCoord[1]));
  float ac = 1 - a;
  gl_FragColor.xyz = t0.xyz*a + t1.xyz*ac;
  gl_FragColor.w = 1;
}
)EOS";
      auto vshader = R"EOS(
#version 330 compatibility

varying float a;

void main(void) {
  a = (gl_VertexID&3)==0||(gl_VertexID&3)==3 ? 1 : 0;
  gl_TexCoord[0] = gl_MultiTexCoord0;
  gl_TexCoord[1] = gl_MultiTexCoord1;
  gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
}
)EOS";
      prog = std::make_shared<GLProgramBind>(vshader, fshader);
      sampler0 = glGetUniformLocation(prog->program(),"sampler0");
      sampler1 = glGetUniformLocation(prog->program(),"sampler1");
    }
    
    if (vbo2)
      GL_CHECK(glDeleteBuffers(1,&vbo2));
    if (ibo2)
      GL_CHECK(glDeleteBuffers(1,&ibo2));

    GL_CHECK(glGenBuffers(1, &vbo2));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo2));
    GL_CHECK(glBufferData(GL_ARRAY_BUFFER, array2.size() * sizeof(float),
			  &array2[0], GL_STATIC_DRAW));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));

    GL_CHECK(glGenBuffers(1, &ibo2));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo2));
    GL_CHECK(glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices2.size() * sizeof(GLuint),
			  &indices2[0], GL_STATIC_DRAW));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

    ////////////////////////////////////////////////////////////////////

    width2 = right_x-left_x;
    height2 = bottom_y-top_y;

    array3.clear();
    indices3.clear();
    GLBlitRect(array3,indices3,4,
	       left_x,top_y,width1,height1,
	       0,0,width2,height2,
	       width2,height2);

    if (vbo3)
      GL_CHECK(glDeleteBuffers(1,&vbo3));
    if (ibo3)
      GL_CHECK(glDeleteBuffers(1,&ibo3));

    GL_CHECK(glGenBuffers(1, &vbo3));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo3));
    GL_CHECK(glBufferData(GL_ARRAY_BUFFER, array3.size() * sizeof(float),
			  &array3[0], GL_STATIC_DRAW));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));

    GL_CHECK(glGenBuffers(1, &ibo3));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo3));
    GL_CHECK(glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices3.size() * sizeof(GLuint),
			  &indices3[0], GL_STATIC_DRAW));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
  }

public:
  GLOffsetBlender()
    : N0(0), vbo1(0), ibo1(0), vbo2(0), ibo2(0), vbo3(0), ibo3(0) {
  }
  GLOffsetBlender(const GLOffsetBlender& x) = delete;
  GLOffsetBlender& operator= (const GLOffsetBlender& rhs) = delete;
  ~GLOffsetBlender() {
    if (vbo1)
      GL_CHECK(glDeleteBuffers(1,&vbo1));
    if (ibo1)
      GL_CHECK(glDeleteBuffers(1,&ibo1));
    if (vbo2)
      GL_CHECK(glDeleteBuffers(1,&vbo2));
    if (ibo2)
      GL_CHECK(glDeleteBuffers(1,&ibo2));
    if (vbo3)
      GL_CHECK(glDeleteBuffers(1,&vbo3));
    if (ibo3)
      GL_CHECK(glDeleteBuffers(1,&ibo3));
  }

  void configure(int N0,
		 const int* si_x0,
		 const int* si_y0,
		 const int* si_width0,
		 const int* si_height0,
		 const int* offsetx0,
		 const int* offsety0) {
    if (N0 == N &&
	std::equal(si_x.begin(),si_x.end(),si_x0) &&
	std::equal(si_y.begin(),si_y.end(),si_y0) &&
	std::equal(si_width.begin(),si_width.end(),si_width0) &&
	std::equal(si_height.begin(),si_height.end(),si_height0) &&
	std::equal(offsetx.begin(),offsetx.end(),offsetx0) &&
	std::equal(offsety.begin(),offsety.end(),offsety0))
      return;
    N = N0;
    si_x.assign(si_x0,si_x0+N);
    si_y.assign(si_y0,si_y0+N);
    si_width.assign(si_width0,si_width0+N);
    si_height.assign(si_height0,si_height0+N);
    offsetx.assign(offsetx0,offsetx0+N);
    offsety.assign(offsety0,offsety0+N);
    init_required = true;
  }

  OccamImage* operator() (const OccamImage* img0) {
  assert(img0->backend == OCCAM_OPENGL);
  assert(img0->format == OCCAM_GRAY8 || img0->format == OCCAM_RGB24);
  init(img0->width,img0->height);

  OccamImage* img1 = new OccamImage;
  memset(img1,0,sizeof(OccamImage));
  img1->cid = strdup(img0->cid);
  memcpy(img1->timescale,img0->timescale,sizeof(img1->timescale));
  img1->time_ns = img0->time_ns;
  img1->index = img0->index;
  img1->refcnt = 1;
  img1->backend = OCCAM_OPENGL;
  img1->format = img0->format;

  GLuint tex1 = createGLTexture(width1, height1, img0->format);

  {
    GLFBO b(tex1, width1, height1);
    GL_CHECK(glEnableClientState(GL_VERTEX_ARRAY));
    GL_CHECK(glEnableClientState(GL_TEXTURE_COORD_ARRAY));
    GL_CHECK(glEnable(GL_TEXTURE_2D));
    GL_CHECK(glBindTexture(GL_TEXTURE_2D, img0->texture[0]));
    GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT));
    GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo1));
    GL_CHECK(glTexCoordPointer(2, GL_FLOAT, 4*sizeof(float), (GLvoid*)(2*sizeof(float))));
    GL_CHECK(glVertexPointer(2, GL_FLOAT, 4*sizeof(float), (GLvoid*)(0)));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo1));
    GL_CHECK(glDrawElements(GL_QUADS, indices1.size(), GL_UNSIGNED_INT, 0));
    GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));
    GL_CHECK(glBindTexture(GL_TEXTURE_2D, 0));
    GL_CHECK(glDisable(GL_TEXTURE_2D));
    GL_CHECK(glDisableClientState(GL_TEXTURE_COORD_ARRAY));
    GL_CHECK(glDisableClientState(GL_VERTEX_ARRAY));
  }

  if (array2.size()) {
    {
      GLFBO b(tex1, width1, height1);
      GL_CHECK(glEnableClientState(GL_VERTEX_ARRAY));

      GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo2));
      GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo2));

      GL_CHECK(glVertexPointer(2, GL_FLOAT, 6*sizeof(float), (GLvoid*)(0)));

      GL_CHECK(glActiveTexture(GL_TEXTURE0));
      GL_CHECK(glBindTexture(GL_TEXTURE_2D, img0->texture[0]));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT));
      GL_CHECK(glEnableClientState(GL_TEXTURE_COORD_ARRAY));
      GL_CHECK(glClientActiveTexture(GL_TEXTURE0));
      GL_CHECK(glEnableClientState(GL_TEXTURE_COORD_ARRAY));
      GL_CHECK(glTexCoordPointer(2, GL_FLOAT, 6*sizeof(float),(GLvoid*)(2*sizeof(float))));

      GL_CHECK(glActiveTexture(GL_TEXTURE1));
      GL_CHECK(glBindTexture(GL_TEXTURE_2D, img0->texture[0]));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT));
      GL_CHECK(glClientActiveTexture(GL_TEXTURE1));
      GL_CHECK(glEnableClientState(GL_TEXTURE_COORD_ARRAY));
      GL_CHECK(glTexCoordPointer(2, GL_FLOAT, 6*sizeof(float),(GLvoid*)(4*sizeof(float))));

      GL_CHECK(glUseProgram(prog->program()));
      GL_CHECK(glUniform1i(sampler0, 0));
      GL_CHECK(glUniform1i(sampler1, 1));
      GL_CHECK(glDrawElements(GL_QUADS, indices2.size(), GL_UNSIGNED_INT, 0));
      GL_CHECK(glUseProgram(0));

      GL_CHECK(glActiveTexture(GL_TEXTURE1));
      GL_CHECK(glBindTexture(GL_TEXTURE_2D, 0));
      GL_CHECK(glActiveTexture(GL_TEXTURE0));
      GL_CHECK(glBindTexture(GL_TEXTURE_2D, 0));
      GL_CHECK(glClientActiveTexture(GL_TEXTURE0));

      GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
      GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));

      GL_CHECK(glDisableClientState(GL_TEXTURE_COORD_ARRAY));
      GL_CHECK(glDisableClientState(GL_VERTEX_ARRAY));
    }

    GLuint tex2 = createGLTexture(width2, height2, img0->format);

    {
      GLFBO b(tex2, width2, height2);
      GL_CHECK(glEnableClientState(GL_VERTEX_ARRAY));
      GL_CHECK(glEnableClientState(GL_TEXTURE_COORD_ARRAY));
      GL_CHECK(glEnable(GL_TEXTURE_2D));
      GL_CHECK(glBindTexture(GL_TEXTURE_2D, tex1));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT));
      GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT));
      GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, vbo3));
      GL_CHECK(glTexCoordPointer(2, GL_FLOAT, 4*sizeof(float), (GLvoid*)(2*sizeof(float))));
      GL_CHECK(glVertexPointer(2, GL_FLOAT, 4*sizeof(float), (GLvoid*)(0)));
      GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo3));
      GL_CHECK(glDrawElements(GL_QUADS, indices1.size(), GL_UNSIGNED_INT, 0));
      GL_CHECK(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
      GL_CHECK(glBindBuffer(GL_ARRAY_BUFFER, 0));
      GL_CHECK(glBindTexture(GL_TEXTURE_2D, 0));
      GL_CHECK(glDisable(GL_TEXTURE_2D));
      GL_CHECK(glDisableClientState(GL_TEXTURE_COORD_ARRAY));
      GL_CHECK(glDisableClientState(GL_VERTEX_ARRAY));
    }

    destroyGLTexture(tex1, width1, height1, img0->format);

    img1->width = width2;
    img1->height = height2;
    img1->texture[0] = tex2;
  } else {
    img1->width = width1;
    img1->height = height1;
    img1->texture[0] = tex1;
  }

  return img1;
  }
};
#endif // OCCAM_OPENGL_SUPPORT

class OccamOffsetBlendFilterImpl : public OccamBlendFilter,
				   public OccamParameters {
  CPUOffsetBlender cpu_blender;
#ifdef OCCAM_OPENGL_SUPPORT
  GLOffsetBlender gl_blender;
#endif // OCCAM_OPENGL_SUPPORT

  int offset_preset;
  int offset_x[3][4];
  int offset_y[3][4];

  int get_offset_preset() {
    return offset_preset;
  }
  int get_offset_preset_count() {
    return 3;
  }
  void set_offset_preset(int value) {
    offset_preset = std::min(2,std::max(0,value));
  }
  void get_offset_x(int* offsetx0) {
    std::copy(offset_x[offset_preset],offset_x[offset_preset]+4,offsetx0);
  }
  void set_offset_x(const int* offsetx0) {
    std::copy(offsetx0,offsetx0+4,offset_x[offset_preset]);
  }
  void get_offset_y(int* offsety0) {
    std::copy(offset_y[offset_preset],offset_y[offset_preset]+4,offsety0);
  }
  void set_offset_y(const int* offsety0) {
    std::copy(offsety0,offsety0+4,offset_y[offset_preset]);
  }

public:
  OccamOffsetBlendFilterImpl() {
    offset_preset = 0;
    memset(offset_x,0,sizeof(offset_x));
    memset(offset_y,0,sizeof(offset_y));

    using namespace std::placeholders;
    registerParami(OCCAM_STITCHING_OFFSET_PRESET,"offset_preset",OCCAM_SETTINGS,0,1,
		   std::bind(&OccamOffsetBlendFilterImpl::get_offset_preset,this),
		   std::bind(&OccamOffsetBlendFilterImpl::set_offset_preset,this,_1));
    registerParami(OCCAM_STITCHING_OFFSET_PRESET_COUNT,"offset_preset_count",OCCAM_SETTINGS,0,0,
		   std::bind(&OccamOffsetBlendFilterImpl::get_offset_preset_count,this));
    registerParamiv(OCCAM_STITCHING_OFFSET_X,
		    "offset_x", OCCAM_SETTINGS, 0, 0, 4,
		    std::bind(&OccamOffsetBlendFilterImpl::get_offset_x,this,_1),
		    std::bind(&OccamOffsetBlendFilterImpl::set_offset_x,this,_1));
    registerParamiv(OCCAM_STITCHING_OFFSET_Y,
		    "offset_y", OCCAM_SETTINGS, 0, 0, 4,
		    std::bind(&OccamOffsetBlendFilterImpl::get_offset_y,this,_1),
		    std::bind(&OccamOffsetBlendFilterImpl::set_offset_y,this,_1));
  }

  virtual int configure(int N,
			const int* sensor_width,
			const int* sensor_height,
			const double* const* D,
			const double* const* K,
			const double* const* R,
			const double* const* T) {
    return OCCAM_API_SUCCESS;
  }

  virtual int compute(const OccamImage* const* img0,OccamImage** img1out) {
    OccamImage* img1 = 0;
    if (img0[0]->backend == OCCAM_CPU)
      img1 = cpu_blender(img0);
#ifdef OCCAM_OPENGL_SUPPORT
    // else if (img0->backend == OCCAM_OPENGL)
    //   img1 = gl_blender(img0);
#endif // OCCAM_OPENGL_SUPPORT
    *img1out = img1;
    return OCCAM_API_SUCCESS;
  }
};

static OccamModuleFactory<OccamOffsetBlendFilterImpl> __module_factory
("offset_blend","Offset",OCCAM_MODULE_BLEND_FILTER,0,0);
void init_offset_blend_filter() {
  //  __module_factory.registerModule();
}
