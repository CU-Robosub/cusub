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

#ifdef OCCAM_OPENGL_SUPPORT

#include "gl_utils.h"
#include <assert.h>
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <list>
#include <map>
#include <GLFW/glfw3.h>
#include <string.h>

//////////////////////////////////////////////////////////////////////////////////
//

struct TextureKey {
  int width;
  int height;
  uint32_t format0;
  uint32_t format1;
  uint32_t type;
  TextureKey(int _width,
	     int _height,
	     uint32_t _format0,
	     uint32_t _format1,
	     uint32_t _type)
    : width(_width),
      height(_height),
      format0(_format0),
      format1(_format1),
      type(_type) {
  }
  bool operator< (const TextureKey& rhs) const {
    if (width != rhs.width)
      return width < rhs.width;
    if (height != rhs.height)
      return height < rhs.height;
    if (format0 != rhs.format0)
      return format0 < rhs.format0;
    if (format1 != rhs.format1)
      return format1 < rhs.format1;
    return type < rhs.type;
  }
};
static std::mutex* texcache_lock = 0;
static std::multimap<TextureKey, uint32_t>* texcache = 0;
static uint64_t texcache_size = 0;

//////////////////////////////////////////////////////////////////////////////////
// GLDownloadThread

class GLDownloadThread {
  void* gl0;
  std::mutex lock;
  std::condition_variable cond;
  std::thread th;
  bool shutdown;
  std::list<std::pair<OccamImage*,std::function<void(OccamImage*)> > > pending;
  void threadproc() {
#ifdef _WIN32
    SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_BELOW_NORMAL);
#endif // _WIN32

    glfwMakeContextCurrent((GLFWwindow*)gl0);

    for (;;) {
      OccamImage* img0;
      std::function<void(OccamImage*)> done_cb;
      {
	std::unique_lock<std::mutex> g(lock);
	if (shutdown)
	  break;
	if (pending.empty()) {
	  cond.wait(g);
	  continue;
	}
	assert(!pending.empty());
	img0 = pending.begin()->first;
	done_cb = pending.begin()->second;
	pending.pop_front();
      }

      if (img0->sync) {
	glWaitSync((GLsync)img0->sync,0,GL_TIMEOUT_IGNORED);
	glDeleteSync((GLsync)img0->sync);
	img0->sync = 0;
      }

      GLenum internal_format;
      GLenum format;
      GLenum type;
      if (!createGLTextureFormatInfo(img0->format, internal_format, format, type)) {
	occamFreeImage(img0);
	done_cb(0);
	continue;
      }

      int channels = 1;
      switch (img0->format) {
      case OCCAM_GRAY8: channels = 1; break;
      case OCCAM_RGB24: channels = 3; break;
      default:
	assert(0);
	break;
      }

      OccamImage* img1 = new OccamImage;
      memset(img1,0,sizeof(OccamImage));
      img1->cid = strdup(img0->cid);
      memcpy(img1->timescale,img0->timescale,sizeof(img1->timescale));
      img1->time_ns = img0->time_ns;
      img1->index = img0->index;
      img1->refcnt = 1;
      img1->backend = OCCAM_CPU;
      img1->format = img0->format;

      GL_CHECK(glBindTexture(GL_TEXTURE_2D,img0->texture[0]));
      GLint pack_alignment;
      GLint width;
      GLint height;
      GL_CHECK(glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width));
      GL_CHECK(glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height));
      GL_CHECK(glGetIntegerv(GL_PACK_ALIGNMENT, &pack_alignment));
      img1->width = width;
      img1->height = height;
      img1->step[0] = ((width*channels)+pack_alignment-1)&~(pack_alignment-1);
      img1->data[0] = new uint8_t[img1->height * img1->step[0]];
      GL_CHECK(glGetTexImage(GL_TEXTURE_2D,0,format,type,img1->data[0]));
      GL_CHECK(glBindTexture(GL_TEXTURE_2D,0));

      done_cb(img1);
      occamFreeImage(img0);
    }
  }
public:
  GLDownloadThread()
    : shutdown(false) {
    occamGLCreateContext(&gl0);
    if (!gl0)
      return;
    glfwMakeContextCurrent(0);
    th = std::thread([this](){
	this->threadproc();
      });
  }
  ~GLDownloadThread() {
    {
      std::unique_lock<std::mutex> g(lock);
      shutdown = true;
      cond.notify_all();
    }
    th.join();
    occamGLFreeContext(gl0);
  }
  GLDownloadThread(const GLDownloadThread& x) = delete;
  GLDownloadThread& operator= (const GLDownloadThread& rhs) = delete;
  void push(OccamImage* img, std::function<void(OccamImage*)> done_cb) {
    std::unique_lock<std::mutex> g(lock);
    if (!gl0) {
      done_cb(0);
      return;
    }
    pending.push_back(std::make_pair(img,done_cb));
    cond.notify_one();
  }
};

static GLDownloadThread* gdl = 0;

//////////////////////////////////////////////////////////////////////////////////
// GLProgramBind

GLProgramBind::GLProgramBind()
  : prog(-1),
    shader_count(0) {
  prog = glCreateProgram();
  assert(prog>0);
}

GLProgramBind::GLProgramBind(const std::string& vertex_shader,
			     const std::string& fragment_shader)
  : prog(-1),
    shader_count(0) {
  prog = glCreateProgram();
  assert(prog>0);

  if (!vertex_shader.empty())
    add(GL_VERTEX_SHADER, vertex_shader);
  if (!fragment_shader.empty())
    add(GL_FRAGMENT_SHADER, fragment_shader);
  link();
}

GLProgramBind::~GLProgramBind() {
  for (int j=0;j<shader_count;++j) {
    glDetachShader(prog, shaders[j]);
    glDeleteShader(shaders[j]);
  }
  if (prog != -1)
    glDeleteProgram(prog);
}

GLuint GLProgramBind::add(GLenum shader_type, const std::string& shader_source) {
  if (shader_source.empty())
    return -1;

  GLuint shader = glCreateShader(shader_type);
  assert(shader>0);
  int lenOfStrings[] = {int(shader_source.size())};
  const char* shader_code_cstr = shader_source.c_str();
  GL_CHECK(glShaderSource(shader, 1, &shader_code_cstr, lenOfStrings));
  GL_CHECK(glCompileShader(shader));

  GLint isCompiled = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &isCompiled);
  if (isCompiled == GL_FALSE) {
    GLint maxLength = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);
    std::vector<GLchar> infoLog(maxLength);
    glGetShaderInfoLog(shader, maxLength, &maxLength, &infoLog[0]);
    std::cerr<<"Shader compile failure: "<<std::endl;
    if (!infoLog.empty())
      std::cerr<<(const char*)&infoLog[0]<<std::endl;
    return -1;
  }

  assert(shader_count<sizeof(shaders)/sizeof(shaders[0]));
  shaders[shader_count++] = shader;

  return shader;
}

void GLProgramBind::link() {
  for (int j=0;j<shader_count;++j)
    GL_CHECK(glAttachShader(prog, shaders[j]));
  GL_CHECK(glLinkProgram(prog));
  GLint isLinked = 0;
  glGetProgramiv(prog, GL_LINK_STATUS, (int *)&isLinked);
  if (isLinked == GL_FALSE) {
    GLint maxLength = 0;
    glGetProgramiv(prog, GL_INFO_LOG_LENGTH, &maxLength);
    std::vector<GLchar> infoLog(maxLength);
    glGetProgramInfoLog(prog, maxLength, &maxLength, &infoLog[0]);
    std::cerr<<"Shader link failure:"<<std::endl;
    if (!infoLog.empty())
      std::cerr<<(const char*)&infoLog[0]<<std::endl;
  }
}

GLuint GLProgramBind::program() const {
  return prog;
}

//////////////////////////////////////////////////////////////////////////////////
// GLFBO

GLFBO::GLFBO(GLuint tex, int width, int height) {
  GL_CHECK(glGenFramebuffersEXT(1, &fbo));
  GL_CHECK(glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo));
  GL_CHECK(glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, tex, 0));
  GL_CHECK(glViewport(0,0,width,height));
  GL_CHECK(glMatrixMode(GL_PROJECTION));
  GL_CHECK(glLoadIdentity());
  GL_CHECK(glMatrixMode(GL_MODELVIEW));
  GL_CHECK(glLoadIdentity());
  GL_CHECK(glDisable(GL_DEPTH_TEST));
  GL_CHECK(glScaled(1,-1,1));
}

GLFBO::~GLFBO() {
  GL_CHECK(glFramebufferTexture2DEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, 0, 0));
  GL_CHECK(glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0));
  GL_CHECK(glDeleteFramebuffers(1,&fbo));
}

//////////////////////////////////////////////////////////////////////////////////
// 

static void* primary_context = 0;

void occamInitGL() {
  texcache_lock = new std::mutex;
  texcache = new std::multimap<TextureKey, uint32_t>;
  if (!glfwInit())
    exit(EXIT_FAILURE);
  occamGLCreateContext(&primary_context);
  glewInit();
  gdl = new GLDownloadThread;
}

void occamShutdownGL() {
  delete gdl;
  gdl = 0;
  occamGLFreeContext(primary_context);
  glfwTerminate();
  delete texcache_lock;
  texcache_lock = 0;
  delete texcache;
  texcache = 0;
}

int occamGLCreateContext(void** context) {
  glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
  GLFWwindow* share = (GLFWwindow*)primary_context;
  GLFWwindow* window = glfwCreateWindow(120, 120, "indigosdk", 0, share);
  if (!window)
    return OCCAM_API_GENERIC_ERROR;
  glfwMakeContextCurrent(window);
  *context = window;
  return OCCAM_API_SUCCESS;
}

int occamGLFreeContext(void* context) {
  GLFWwindow* window = (GLFWwindow*)context;
  glfwDestroyWindow(window);
  return OCCAM_API_SUCCESS;
}

int occamGLMakeCurrent(void* context) {
  GLFWwindow* window = (GLFWwindow*)context;
  glfwMakeContextCurrent(0);
  glfwMakeContextCurrent(window);
  return OCCAM_API_SUCCESS;
}

int occamGLShareContext() {
#ifdef _WIN32
  HDC dc0 = wglGetCurrentDC();
  HGLRC glrc0 = wglGetCurrentContext();
  glfwMakeContextCurrent(0);
  glfwMakeContextCurrent((GLFWwindow*)primary_context);
  HGLRC glrc1 = wglGetCurrentContext();
  wglMakeCurrent(dc0,glrc0);
  if (!wglShareLists(glrc1, glrc0))
    return OCCAM_API_GENERIC_ERROR;
  return OCCAM_API_SUCCESS;
#endif // _WIN32

  return OCCAM_API_GENERIC_ERROR;
}

void resetGLTextureCache() {
  //  std::cerr<<"reset gl texture cache"<<std::endl;
  for (auto it=texcache->begin();it!=texcache->end();++it)
    GL_CHECK(glDeleteTextures(1, &it->second));    
  texcache->clear();
  texcache_size = 0;
}

GLuint createGLTexture(int width, int height,
		       GLenum format0, GLenum format1, GLenum type) {
  {
    std::unique_lock<std::mutex> g(*texcache_lock);
    auto it = texcache->find(TextureKey(width,height,format0,format1,type));
    if (it != texcache->end()) {
      GLuint texid = it->second;
      //      std::cerr<<"re-use texture "<<texid<<" ("<<width<<" "<<height<<" "<<format0<<" "<<format1<<" "<<type<<")"<<std::endl;
      texcache_size -= it->first.width * it->first.height;
      texcache->erase(it);
      return texid;
    }
  }

  GLuint texid;
  GL_CHECK(glGenTextures(1, &texid));
  GL_CHECK(glBindTexture(GL_TEXTURE_2D,texid));
  GL_CHECK(glTexImage2D(GL_TEXTURE_2D,
			0,
			format0,
			width,
			height,
			0,
			format1,
			type,
			0));
  GL_CHECK(glBindTexture(GL_TEXTURE_2D,0));

  //  std::cerr<<"allocate texture "<<texid<<" ("<<width<<" "<<height<<" "<<format0<<" "<<format1<<" "<<type<<")"<<std::endl;

  return texid;
}

bool createGLTextureFormatInfo(OccamImageFormat format0,
			       GLenum& internal_format, GLenum& format, GLenum& type) {
  
  switch (format0) {
  case OCCAM_GRAY8: {
    internal_format = GL_RED;
    format = GL_RED;
    type = GL_UNSIGNED_BYTE;
    return true;
  }
  case OCCAM_RGB24: {
    internal_format = GL_RGB;
    format = GL_RGB;
    type = GL_UNSIGNED_BYTE;
    return true;
  }
  }
  return false;
}

GLuint createGLTexture(int width, int height, OccamImageFormat format0) {
  GLenum internal_format, format, type;
  if (!createGLTextureFormatInfo(format0, internal_format, format, type))
    return -1;
  return createGLTexture(width, height, internal_format, format, type);
}

int destroyGLTexture(GLuint texid, int width, int height, OccamImageFormat format0) {
  GLenum internal_format, format, type;
  if (!createGLTextureFormatInfo(format0, internal_format, format, type))
    OCCAM_API_GENERIC_ERROR;
  if (OCCAM_API_SUCCESS != occamGLFreeTexture
      (texid, width, height, internal_format, format, type))
    GL_CHECK(glDeleteTextures(1, &texid));
  return OCCAM_API_SUCCESS;
}

int occamGLCreateTexture(int width, int height,
			 uint32_t format0, uint32_t format1, uint32_t type,
			 uint32_t* texid) {
  *texid = createGLTexture(width, height, format0, format1, type);
  return OCCAM_API_SUCCESS;
}

int occamGLFreeTexture(uint32_t texid, int width, int height,
		       uint32_t format0, uint32_t format1, uint32_t type) {
  //  std::cerr<<"recycle texture "<<texid<<" ("<<width<<" "<<height<<" "<<format0<<" "<<format1<<" "<<type<<")"<<std::endl;
  std::unique_lock<std::mutex> g(*texcache_lock);
  texcache->insert(std::make_pair(TextureKey(width,height,format0,format1,type), texid));
  texcache_size += width * height;
  const int texcache_reset_threshold = 100*1024*1024;
  if (texcache_size > texcache_reset_threshold)
    resetGLTextureCache();
  //  std::cerr<<"texcache_size = "<<texcache_size<<std::endl;
  return OCCAM_API_SUCCESS;
}

void GLBlitRect(std::vector<float>& array,
		std::vector<GLuint>& indices,
		int array_step,
		int src_x,int src_y,int src_width,int src_height,
		int dst_x,int dst_y,int dst_width,int dst_height,
		int blit_width, int blit_height) {
  int base_index = array.size()/array_step;
  int x0_y0_index = base_index+0;
  int x1_y0_index = base_index+1;
  int x1_y1_index = base_index+2;
  int x0_y1_index = base_index+3;
  indices.push_back(x0_y0_index);
  indices.push_back(x1_y0_index);
  indices.push_back(x1_y1_index);
  indices.push_back(x0_y1_index);

  auto push = [&](int sx0,int sy0,int dx0,int dy0) {
    float sx1 = float(sx0)/src_width;
    float sy1 = float(sy0)/src_height;
    float dx1 = (float(dx0)/dst_width)*2-1;
    float dy1 = (1-(float(dy0)/dst_height))*2-1;
    array.push_back(dx1);
    array.push_back(dy1);
    array.push_back(sx1);
    array.push_back(sy1);
  };
  push(src_x,src_y,dst_x,dst_y);
  push(src_x+blit_width,src_y,dst_x+blit_width,dst_y);
  push(src_x+blit_width,src_y+blit_height,dst_x+blit_width,dst_y+blit_height);
  push(src_x,src_y+blit_height,dst_x,dst_y+blit_height);
}

void GLBlitRect2(std::vector<float>& array,
		 std::vector<GLuint>& indices,
		 int array_step,
		 int src0_x,int src0_y,int src0_width,int src0_height,
		 int src1_x,int src1_y,int src1_width,int src1_height,
		 int dst_x,int dst_y,int dst_width,int dst_height,
		 int blit_width, int blit_height) {
  int base_index = array.size()/array_step;
  int x0_y0_index = base_index+0;
  int x1_y0_index = base_index+1;
  int x1_y1_index = base_index+2;
  int x0_y1_index = base_index+3;
  indices.push_back(x0_y0_index);
  indices.push_back(x1_y0_index);
  indices.push_back(x1_y1_index);
  indices.push_back(x0_y1_index);

  auto push = [&](int sx0,int sy0,int sx1,int sy1,int dx0,int dy0) {
    float sx2 = float(sx0)/src0_width;
    float sy2 = float(sy0)/src0_height;
    float sx3 = float(sx1)/src1_width;
    float sy3 = float(sy1)/src1_height;
    float dx1 = (float(dx0)/dst_width)*2-1;
    float dy1 = (1-(float(dy0)/dst_height))*2-1;
    array.push_back(dx1);
    array.push_back(dy1);
    array.push_back(sx2);
    array.push_back(sy2);
    array.push_back(sx3);
    array.push_back(sy3);
  };
  push(src0_x,src0_y,src1_x,src1_y,dst_x,dst_y);
  push(src0_x+blit_width,src0_y,src1_x+blit_width,src1_y,dst_x+blit_width,dst_y);
  push(src0_x+blit_width,src0_y+blit_height,src1_x+blit_width,src1_y+blit_height,
       dst_x+blit_width,dst_y+blit_height);
  push(src0_x,src0_y+blit_height,src1_x,src1_y+blit_height,dst_x,dst_y+blit_height);
}

OccamImage* localToGL(OccamImage* img0) {
  if (img0->backend != OCCAM_CPU) {
    OccamImage* img1;
    if (OCCAM_API_SUCCESS != occamCopyImage(img0, &img1, 0))
      return 0;
    return img1;
  }
  
  OccamImage* img1 = new OccamImage;
  img1->cid = strdup(img0->cid);
  memcpy(img1->timescale,img0->timescale,sizeof(img1->timescale));
  img1->time_ns = img0->time_ns;
  img1->index = img0->index;
  img1->refcnt = 1;
  img1->backend = OCCAM_OPENGL;
  img1->format = img0->format;
  img1->width = img0->width;
  img1->height = img0->height;
  img1->subimage_count = img0->subimage_count;
  memcpy(img1->si_x,img0->si_x,sizeof(img1->si_x));
  memcpy(img1->si_y,img0->si_y,sizeof(img1->si_y));
  memcpy(img1->si_width,img0->si_width,sizeof(img1->si_width));
  memcpy(img1->si_height,img0->si_height,sizeof(img1->si_height));
  memset(img1->step,0,sizeof(img1->step));
  memset(img1->data,0,sizeof(img1->data));
  memset(img1->texture,0,sizeof(img1->texture));
  img1->sync = 0;

  int plane_count = 0;
  int plane_width[OCCAM_IMAGE_MAX_PLANES];
  int plane_height[OCCAM_IMAGE_MAX_PLANES];
  int channels;
  switch (img1->format) {
  case OCCAM_GRAY8: {
    plane_count = 1;
    plane_width[0] = img0->width;
    plane_height[0] = img0->height;
    channels = 1;
    img1->texture[0] = createGLTexture(img1->width, img1->height, OCCAM_GRAY8);
    break;
  }
  case OCCAM_RGB24: {
    plane_count = 1;
    channels = 3;
    plane_width[0] = img0->width;
    plane_height[0] = img0->height;
    img1->texture[0] = createGLTexture(img1->width, img1->height, OCCAM_RGB24);
    break;
  }
  }

  GLenum internal_format, format, type;
  if (!createGLTextureFormatInfo(img1->format, internal_format, format, type))
    return 0;

  for (int j=0;j<plane_count;++j) {
    GL_CHECK(glBindTexture(GL_TEXTURE_2D,img1->texture[j]));
    glPixelStorei(GL_UNPACK_ROW_LENGTH,img0->step[j]/channels);
    GL_CHECK(glTexSubImage2D(GL_TEXTURE_2D,
			     0,
			     0,
			     0,
			     plane_width[j],
			     plane_height[j],
			     format,
			     type,
			     img0->data[j]));
    GL_CHECK(glBindTexture(GL_TEXTURE_2D,0));
  }

  return img1;
}

OccamImage* GLToLocal(OccamImage* img) {
  std::mutex lock;
  std::condition_variable cond;
  bool cond_signaled = false;
  OccamImage* img1 = 0;
  GLToLocal(img, [&lock,&cond,&cond_signaled,&img1](OccamImage* _img1){
      std::unique_lock<std::mutex> g(lock);
      cond.notify_one();
      cond_signaled = true;
      img1 = _img1;
    });
  std::unique_lock<std::mutex> g(lock);
  if (cond_signaled)
    return img1;
  cond.wait(g);
  return img1;
}

void GLToLocal(OccamImage* img, std::function<void(OccamImage*)> done_cb) {
  gdl->push(img, done_cb);
}

int occamGLDownload(OccamImage* img0, GLDownloadCallback cb, void* cb_data) {
  GLToLocal(img0, [cb,cb_data](OccamImage* img1){
      cb(img1, cb_data);
    });
  return OCCAM_API_SUCCESS;
}

int occamGLUpload(OccamImage* img0, GLUploadCallback cb, void* cb_data) {
  OccamImage* img1 =localToGL(img0);
  cb(img1, cb_data);
  return OCCAM_API_SUCCESS;
}

#endif // OCCAM_OPENGL_SUPPORT

