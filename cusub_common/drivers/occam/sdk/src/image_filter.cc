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
#include "module_utils.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <string.h>
#include <math.h>
#undef min
#undef max

static void cpuImageFilter(const uint8_t* srcp, uint8_t* dstp,
			   int src_step, int dst_step,
			   int width, int height,
			   int channels,
			   int brightness1k,
			   int gamma1k,
			   int black_level1k,
			   int white_balance_red1k,
			   int white_balance_green1k,
			   int white_balance_blue1k) {
  float black_level = black_level1k/1000.f;
  float brightness = brightness1k/1000.f;
  float gamma = gamma1k/1000.f;
  float gammas = float(pow(256,gamma-1));
  float sr = white_balance_red1k/1000.f;
  float sg = white_balance_green1k/1000.f;
  float sb = white_balance_blue1k/1000.f;

  uint8_t lut[3][256];

  for (int i=0;i<256;++i) {
    float v = i/256.f;

    float r = std::max(0.f,v-black_level);
    float g = std::max(0.f,v-black_level);
    float b = std::max(0.f,v-black_level);

    r = gammas*pow(r,gamma);
    g = gammas*pow(g,gamma);
    b = gammas*pow(b,gamma);
    
    r *= brightness;
    g *= brightness;
    b *= brightness;

    r *= sr;
    g *= sg;
    b *= sb;

    int ri = std::min(255,std::max(0,int(r*255)));
    int gi = std::min(255,std::max(0,int(g*255)));
    int bi = std::min(255,std::max(0,int(b*255)));

    lut[0][i] = ri;
    lut[1][i] = gi;
    lut[2][i] = bi;
  }

  for (int y=0;y<height;++y,srcp+=src_step,dstp+=dst_step)
    for (int x=0,xx=0;x<width;++x,xx+=channels)
      for (int c=0;c<channels;++c)
	dstp[xx+c] = lut[c][srcp[xx+c]];
}

#ifdef OCCAM_OPENGL_SUPPORT

class GLImageFilter {
  std::vector<float> array;
  std::vector<GLuint> indices;
  std::shared_ptr<GLProgramBind> prog;
  GLuint black_level;
  GLuint gamma;
  GLuint gammas;
  GLuint brightness;
  GLuint balance;
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

    auto fshader = R"EOS(
uniform sampler2D source;

uniform float black_level;
uniform float gamma;
uniform float gammas;
uniform float brightness;
uniform vec3 balance;

void main(void) {
  vec3 rgb = texture2D(source, vec2(gl_TexCoord[0])).rgb;
  rgb = max(vec3(0,0,0),rgb-black_level);
  rgb.r = gammas*pow(rgb.r,gamma);
  rgb.g = gammas*pow(rgb.g,gamma);
  rgb.b = gammas*pow(rgb.b,gamma);
  rgb *= brightness;
  rgb.r *= balance.r;
  rgb.g *= balance.g;
  rgb.b *= balance.b;
  gl_FragColor.rgb = rgb;
  gl_FragColor.a = 1;
}
)EOS";
    if (!bool(prog)) {
      prog = std::make_shared<GLProgramBind>(std::string(),fshader);
      black_level = glGetUniformLocation(prog->program(),"black_level");
      gamma = glGetUniformLocation(prog->program(),"gamma");
      gammas = glGetUniformLocation(prog->program(),"gammas");
      brightness = glGetUniformLocation(prog->program(),"brightness");
      balance = glGetUniformLocation(prog->program(),"balance");
    }
  }
public:
  GLImageFilter()
    : vbo(0), ibo(0) {
  }
  ~GLImageFilter() {
    if (vbo)
      GL_CHECK(glDeleteBuffers(1, &vbo));
    if (ibo)
      GL_CHECK(glDeleteBuffers(1, &ibo));
  }
  GLImageFilter(const GLImageFilter& x) = delete;
  GLImageFilter& operator= (const GLImageFilter& rhs) = delete;
  void operator() (GLuint srctex,
		   int width,
		   int height,
		   int brightness1k,
		   int gamma1k,
		   int black_level1k,
		   int white_balance_red1k,
		   int white_balance_green1k,
		   int white_balance_blue1k) {
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
    float w1k = 1.f/1000;
    GL_CHECK(glUniform1f(black_level, w1k*black_level1k));
    GL_CHECK(glUniform1f(gamma, w1k*gamma1k));
    GL_CHECK(glUniform1f(gammas,float(pow(256,(w1k * gamma1k)-1))));
    GL_CHECK(glUniform1f(brightness, w1k*brightness1k));
    GL_CHECK(glUniform3f(balance,w1k*white_balance_red1k,w1k*white_balance_green1k,w1k*white_balance_blue1k));

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

class OccamImageFilterImpl : public OccamImageFilter,
			     public OccamParameters {
#ifdef OCCAM_OPENGL_SUPPORT
  GLImageFilter glimage;
#endif // OCCAM_OPENGL_SUPPORT

  bool is_color;
  bool enabled;
  int brightness1k;
  int gamma1k;
  int black_level1k;
  int white_balance_red1k;
  int white_balance_green1k;
  int white_balance_blue1k;

  bool get_color() {
    return is_color;
  }
  void set_color(bool value) {
    if (is_color == value)
      return;
    is_color = value;
    unregisterParam(OCCAM_WHITE_BALANCE_RED);
    unregisterParam(OCCAM_WHITE_BALANCE_GREEN);
    unregisterParam(OCCAM_WHITE_BALANCE_BLUE);
    if (is_color) {
      using namespace std::placeholders;
      registerParami(OCCAM_WHITE_BALANCE_RED,"white_balance_red",OCCAM_SETTINGS,1,2000,
		     std::bind(&OccamImageFilterImpl::get_white_balance_red,this),
		     std::bind(&OccamImageFilterImpl::set_white_balance_red,this,_1));
      registerParami(OCCAM_WHITE_BALANCE_GREEN,"white_balance_green",OCCAM_SETTINGS,1,2000,
		     std::bind(&OccamImageFilterImpl::get_white_balance_green,this),
		     std::bind(&OccamImageFilterImpl::set_white_balance_green,this,_1));
      registerParami(OCCAM_WHITE_BALANCE_BLUE,"white_balance_blue",OCCAM_SETTINGS,1,2000,
		     std::bind(&OccamImageFilterImpl::get_white_balance_blue,this),
		     std::bind(&OccamImageFilterImpl::set_white_balance_blue,this,_1));
    } else {
      white_balance_red1k = 1000;
      white_balance_green1k = 1000;
      white_balance_blue1k = 1000;
    }
  }
  bool get_enabled() {
    return enabled;
  }
  void set_enabled(bool value) {
    enabled = value;
  }
  int get_brightness() {
    return brightness1k;
  }
  void set_brightness(int value) {
    brightness1k = value;
  }
  int get_gamma() {
    return gamma1k;
  }
  void set_gamma(int value) {
    gamma1k = value;
  }
  int get_black_level() {
    return black_level1k;
  }
  void set_black_level(int value) {
    black_level1k = value;
  }
  int get_white_balance_red() {
    return white_balance_red1k;
  }
  void set_white_balance_red(int value) {
    white_balance_red1k = value;
  }
  int get_white_balance_green() {
    return white_balance_green1k;
  }
  void set_white_balance_green(int value) {
    white_balance_green1k = value;
  }
  int get_white_balance_blue() {
    return white_balance_blue1k;
  }
  void set_white_balance_blue(int value) {
    white_balance_blue1k = value;
  }

public:
  OccamImageFilterImpl()
    : is_color(false),
      enabled(true),
      brightness1k(1000),
      gamma1k(1000),
      black_level1k(0),
      white_balance_red1k(1000),
      white_balance_green1k(1000),
      white_balance_blue1k(1000) {
    using namespace std::placeholders;
    registerParamb(OCCAM_COLOR,"color",OCCAM_NOT_STORED,
		   std::bind(&OccamImageFilterImpl::get_color,this),
		   std::bind(&OccamImageFilterImpl::set_color,this,_1));
    registerParamb(OCCAM_IMAGE_PROCESSING_ENABLED,"image_processing_enabled",OCCAM_SETTINGS,
		   std::bind(&OccamImageFilterImpl::get_enabled,this),
		   std::bind(&OccamImageFilterImpl::set_enabled,this,_1));
    registerParami(OCCAM_BLACK_LEVEL,"blacklevel",OCCAM_SETTINGS,0,1000,
		   std::bind(&OccamImageFilterImpl::get_black_level,this),
		   std::bind(&OccamImageFilterImpl::set_black_level,this,_1));
    registerParami(OCCAM_BRIGHTNESS,"brightness",OCCAM_SETTINGS,1,2000,
		   std::bind(&OccamImageFilterImpl::get_brightness,this),
		   std::bind(&OccamImageFilterImpl::set_brightness,this,_1));
    registerParami(OCCAM_GAMMA,"gamma",OCCAM_SETTINGS,1,2000,
		   std::bind(&OccamImageFilterImpl::get_gamma,this),
		   std::bind(&OccamImageFilterImpl::set_gamma,this,_1));
  }

  virtual int compute(const OccamImage* img0,OccamImage** img1out) {
    if (!enabled)
      return occamCopyImage(img0,img1out,false);

    OccamImage* img1 = 0;
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
      img1->format = img0->format;
      img1->width = img0->width;
      img1->height = img0->height;
      memset(img1->texture,0,sizeof(img1->texture));
      img1->texture[0] = createGLTexture(img1->width,img1->height,img0->format);
      GLFBO b(img1->texture[0], img1->width, img1->height);
      glimage(img0->texture[0], img1->width, img1->height,
	      brightness1k, gamma1k, black_level1k,
	      white_balance_red1k, white_balance_green1k,white_balance_blue1k);
#endif // OCCAM_OPENGL_SUPPORT
    } else if (img0->backend == OCCAM_CPU) {
      int channels = 0;
      switch (img0->format) {
      case OCCAM_GRAY8: channels = 1; break;
      case OCCAM_RGB24: channels = 3; break;
      default: return 0;
      }
      img1 = new OccamImage;
      memset(img1,0,sizeof(OccamImage));
      img1->cid = strdup(img0->cid);
      memcpy(img1->timescale,img0->timescale,sizeof(img1->timescale));
      img1->time_ns = img0->time_ns;
      img1->index = img0->index;
      img1->refcnt = 1;
      img1->backend = OCCAM_CPU;
      img1->format = img0->format;
      img1->width = img0->width;
      img1->height = img0->height;
      memset(img1->step,0,sizeof(img1->step));
      memset(img1->data,0,sizeof(img1->data));
      img1->step[0] = ((img0->width*channels)+15)&~15;
      img1->data[0] = new uint8_t[img1->height*img1->step[0]];
      cpuImageFilter(img0->data[0], img1->data[0], img0->step[0], img1->step[0],
		     img0->width, img0->height, channels, brightness1k, gamma1k,
		     black_level1k, white_balance_red1k,white_balance_green1k,
		     white_balance_blue1k);
    }
    *img1out = img1;
    return OCCAM_API_SUCCESS;
  }
};

static OccamModuleFactory<OccamImageFilterImpl> __module_factory
("imf","Image Processing",OCCAM_MODULE_IMAGE_FILTER,0,0);
void init_image_filter() {
  __module_factory.registerModule();
}
