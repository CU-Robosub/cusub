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
#include "system.h"
#include <assert.h>
#include <string.h>

int occamFreePointCloud(OccamPointCloud* point_cloud) {
  if (!point_cloud)
    return OCCAM_API_SUCCESS;

  int refcnt1 = OCCAM_XADD(&point_cloud->refcnt, -1);

  if (refcnt1 == 1) {
    if (point_cloud->xyz)
      occamFree(point_cloud->xyz);
    if (point_cloud->rgb)
      occamFree(point_cloud->rgb);
    if (point_cloud->cid)
      occamFree(point_cloud->cid);
    occamFree(point_cloud);
  }
  return OCCAM_API_SUCCESS;
}

int occamCopyPointCloud(const OccamPointCloud* point_cloud, OccamPointCloud** new_point_cloud, int deep_copy) {
  if (deep_copy) {
    *new_point_cloud = (OccamPointCloud*)occamAlloc(sizeof(OccamPointCloud));
    memcpy(*new_point_cloud,point_cloud,sizeof(OccamPointCloud));
    (*new_point_cloud)->refcnt = 1;
    if (point_cloud->cid)
      (*new_point_cloud)->cid = strdup(point_cloud->cid);
    if (point_cloud->xyz) {
      (*new_point_cloud)->xyz = (float*)occamAlloc(sizeof(float)*3*point_cloud->point_count);
      memcpy((*new_point_cloud)->xyz,point_cloud->xyz,point_cloud->point_count*sizeof(float)*3);
    }
    if (point_cloud->rgb) {
      (*new_point_cloud)->rgb = (uint8_t*)occamAlloc(sizeof(uint8_t)*3*point_cloud->point_count);
      memcpy((*new_point_cloud)->rgb,point_cloud->rgb,point_cloud->point_count*sizeof(uint8_t)*3);
    }
  } else {
    OCCAM_XADD(&((OccamPointCloud*)point_cloud)->refcnt, 1);
    *new_point_cloud = (OccamPointCloud*)point_cloud;
  }

  return OCCAM_API_SUCCESS;
}
