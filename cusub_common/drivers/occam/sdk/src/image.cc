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

#ifdef _WIN32
#include <windows.h>
#endif // _WIN32
#include "indigo.h"
#include "gl_utils.h"
#include "system.h"
#include <stdlib.h>
#include <string.h>

int occamFreeImage(OccamImage* image) {
  if (!image)
    return OCCAM_API_SUCCESS;

  int refcnt1 = OCCAM_XADD(&image->refcnt, -1);

  if (refcnt1 == 1) {
    if (image->owner) {
      occamFreeImage(image->owner);
    }

    else if (image->backend == OCCAM_CPU) {
      delete [] image->data[0];
      delete [] image->data[1];
      delete [] image->data[2];
    }

    else if (image->backend == OCCAM_OPENGL) {
#ifdef OCCAM_OPENGL_SUPPORT
      if (image->texture[0])
	destroyGLTexture(image->texture[0],image->width,image->height,image->format);
#endif // OCCAM_OPENGL_SUPPORT
    }

    if (image->cid)
      free(image->cid);

    delete image;
  }
  return OCCAM_API_SUCCESS;
}

int occamSubImage(OccamImage* image, OccamImage** new_image, int x, int y, int width, int height) {
  int planes = 0;
  occamImageFormatPlanes(image->format, &planes);
  if (image->backend != OCCAM_CPU ||
      planes != 1)
    return OCCAM_API_NOT_SUPPORTED;
  if (x<0||y<0||x>=image->width||y>=image->height)
    return OCCAM_API_INVALID_PARAMETER;
  *new_image = new OccamImage(*image);
  if (image->cid)
    (*new_image)->cid = strdup(image->cid);
  (*new_image)->refcnt = 1;  
  OCCAM_XADD(&image->refcnt, 1);
  (*new_image)->owner = image;
  int bpp = 0;
  occamImageFormatBytesPerPixel(image->format, &bpp);
  (*new_image)->data[0] = image->data[0]+y*image->step[0]+x*bpp;
  (*new_image)->width = width;
  (*new_image)->height = height;
  (*new_image)->subimage_count = 0;
  memset((*new_image)->si_x,0,sizeof((*new_image)->si_x));
  memset((*new_image)->si_y,0,sizeof((*new_image)->si_y));
  memset((*new_image)->si_width,0,sizeof((*new_image)->si_width));
  memset((*new_image)->si_height,0,sizeof((*new_image)->si_height));
  return OCCAM_API_SUCCESS;
}

int occamCopyImage(const OccamImage* image, OccamImage** new_image, int deep_copy) {
  if (deep_copy && image->backend != OCCAM_CPU)
    return OCCAM_API_INVALID_PARAMETER;

  if (deep_copy) {
    *new_image = new OccamImage(*image);
    (*new_image)->refcnt = 1;
    if (image->cid)
      (*new_image)->cid = strdup(image->cid);

    int c0_height = image->height;
    int c1_height = 0;
    int c2_height = 0;
    int c0_size = image->step[0] * c0_height;
    int c1_size = image->step[1] * c1_height;
    int c2_size = image->step[2] * c2_height;

    (*new_image)->data[0] = c0_size ? new uint8_t[c0_size] : 0;
    (*new_image)->data[1] = c1_size ? new uint8_t[c1_size] : 0;
    (*new_image)->data[2] = c2_size ? new uint8_t[c2_size] : 0;
    if ((*new_image)->data[0])
      memcpy((*new_image)->data[0],image->data[0],c0_size);
    if ((*new_image)->data[1])
      memcpy((*new_image)->data[1],image->data[1],c1_size);
    if ((*new_image)->data[2])
      memcpy((*new_image)->data[2],image->data[2],c2_size);
  } else {
    OCCAM_XADD(&((OccamImage*)image)->refcnt, 1);
    *new_image = (OccamImage*)image;
  }

  return OCCAM_API_SUCCESS;
}

int occamImageFormatPlanes(OccamImageFormat format, int* planes) {
  switch (format) {
  case OCCAM_R8:
    *planes = 1;
    break;
  case OCCAM_RG16:
    *planes = 1;
    break;
  case OCCAM_RGB24:
    *planes = 1;
    break;
  case OCCAM_RGBA32:
    *planes = 1;
    break;
  case OCCAM_SHORT1:
    *planes = 1;
    break;
  case OCCAM_FLOAT1:
    *planes = 1;
    break;
  case OCCAM_FLOAT2:
    *planes = 1;
    break;
  case OCCAM_FLOAT3:
    *planes = 1;
    break;
  case OCCAM_YUV422_PACKED:
    *planes = 1;
    break;
  case OCCAM_YUV420_PACKED:
    *planes = 1;
    break;
  case OCCAM_YUV422_PLANAR:
    *planes = 3;
    break;
  case OCCAM_YUV420_PLANAR:
    *planes = 3;
    break;
  default:
    return OCCAM_API_INVALID_FORMAT;
  }
  return OCCAM_API_SUCCESS;
}

int occamImageFormatBytesPerPixel(OccamImageFormat format, int* bpp) {
  switch (format) {
  case OCCAM_R8:
    *bpp = 1;
    break;
  case OCCAM_RG16:
    *bpp =2;
    break;
  case OCCAM_RGB24:
    *bpp = 3;
    break;
  case OCCAM_RGBA32:
    *bpp = 4;
    break;
  case OCCAM_SHORT1:
    *bpp = 2;
    break;
  case OCCAM_FLOAT1:
    *bpp = 4;
    break;
  case OCCAM_FLOAT2:
    *bpp = 8;
    break;
  case OCCAM_FLOAT3:
    *bpp = 12;
    break;
  case OCCAM_YUV422_PACKED:
    *bpp = 1;
    break;
  case OCCAM_YUV420_PACKED:
    *bpp = 1;
    break;
  case OCCAM_YUV422_PLANAR:
    *bpp = 1;
    break;
  case OCCAM_YUV420_PLANAR:
    *bpp = 1;
    break;
  default:
    return OCCAM_API_INVALID_FORMAT;
  }
  return OCCAM_API_SUCCESS;
}

