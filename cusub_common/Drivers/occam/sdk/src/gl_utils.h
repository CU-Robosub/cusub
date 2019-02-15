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

#pragma once

#ifdef OCCAM_OPENGL_SUPPORT

#ifdef _WIN32
#include <windows.h>
#endif // _WIN32
#include <glew.h>
#include <glfw3.h>
#include <GL/gl.h>
#include <string>
#include <vector>
#include <functional>

class GLProgramBind {
  GLuint prog, shaders[2];
  int shader_count;
public:
  GLProgramBind();
  GLProgramBind(const std::string& vertex_shader,
		const std::string& fragment_shader);
  ~GLProgramBind();
  GLProgramBind(const GLProgramBind& x) = delete;
  GLProgramBind& operator= (const GLProgramBind& rhs) = delete;
  GLuint add(GLenum shader_type, const std::string& shader_source);
  void link();
  GLuint program() const;
};

class GLFBO {
  GLuint fbo;
public:
  GLFBO(GLuint tex, int width, int height);
  ~GLFBO();
  GLFBO(const GLFBO& x) = delete;
  GLFBO& operator= (const GLFBO& rhs) = delete;
};

void occamInitGL();
void occamShutdownGL();

GLuint createGLTexture(int width, int height, GLenum format, GLenum type);
GLuint createGLTexture(int width, int height, OccamImageFormat format);
bool createGLTextureFormatInfo(OccamImageFormat format0,
			       GLenum& internal_format, GLenum& format, GLenum& type);
int destroyGLTexture(GLuint tex, int width, int height, OccamImageFormat format);

void GLBlitRect(std::vector<float>& array,
		std::vector<GLuint>& indices,
		int array_step,
		int src_x,int src_y,int src_width,int src_height,
		int dst_x,int dst_y,int dst_width,int dst_height,
		int blit_width, int blit_height);
void GLBlitRect2(std::vector<float>& array,
		 std::vector<GLuint>& indices,
		 int array_step,
		 int src0_x,int src0_y,int src0_width,int src0_height,
		 int src1_x,int src1_y,int src1_width,int src1_height,
		 int dst_x,int dst_y,int dst_width,int dst_height,
		 int blit_width, int blit_height);

OccamImage* localToGL(OccamImage* img);
OccamImage* GLToLocal(OccamImage* img);
void GLToLocal(OccamImage* img, std::function<void(OccamImage*)> done_cb);

static void CheckOpenGLError(const char* stmt, const char* fname, int line) {
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    printf("OpenGL error %08x, at %s:%i - for %s\n", err, fname, line, stmt);
    abort();
  }
}

#ifdef _DEBUG
#define GL_CHECK(stmt) do {				\
    stmt;						\
    CheckOpenGLError(#stmt, __FILE__, __LINE__);	\
  } while (0)
#else
#define GL_CHECK(stmt)				\
  {						\
    stmt;					\
  }
#endif

#endif // OCCAM_OPENGL_SUPPORT

// Local Variables:
// mode: c++
// End:
