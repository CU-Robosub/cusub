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
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

static void reportError(int error_code) {
  std::cerr<<"Occam API Error: "<<error_code<<std::endl;
  abort();
}

template <class T>
static std::string matstr(int m, int n, const T* M) {
  std::stringstream sout;
  sout<<"[";
  for (int i=0;i<m;++i) {
    sout<<(i?";":"")<<"[";
    for (int j=0;j<n;++j)
      sout<<(j?",":"")<<M[i*n+j];
    sout<<"]";
  }
  sout<<"]";
  return sout.str();
}

int main(int argc, const char** argv) {
  int r;
  int dev_index = argc>=2?atoi(argv[1]):0;
  OccamDeviceList* device_list;
  OccamDevice* device;

  if ((r = occamInitialize()) != OCCAM_API_SUCCESS)
    reportError(r);

  if ((r = occamEnumerateDeviceList(2000, &device_list)) != OCCAM_API_SUCCESS)
    reportError(r);
  if (dev_index<0 || dev_index >= device_list->entry_count) {
    std::cerr<<"No cameras found."<<std::endl;
    return 1;
  }

  if ((r = occamOpenDevice(device_list->entries[dev_index].cid, &device)) != OCCAM_API_SUCCESS)
    reportError(r);

  int sensor_count;
  int sensor_width;
  int sensor_height;
  if ((r = occamGetDeviceValuei(device, OCCAM_SENSOR_COUNT, &sensor_count)) != OCCAM_API_SUCCESS)
    reportError(r);
  if ((r = occamGetDeviceValuei(device, OCCAM_SENSOR_WIDTH, &sensor_width)) != OCCAM_API_SUCCESS)
    reportError(r);
  if ((r = occamGetDeviceValuei(device, OCCAM_SENSOR_HEIGHT, &sensor_height)) != OCCAM_API_SUCCESS)
    reportError(r);

  std::cout<<"camera: "<<device_list->entries[dev_index].cid<<std::endl;
  std::cout<<std::endl;

  //////////////////////////////////////////////////////////////////////////////////
  // geometric calibration parameters

  for (int j=0;j<sensor_count;++j) {
    double D[5], K[9], R[9], T[3];

    if ((r = occamGetDeviceValuerv
	 (device, OccamParam(OCCAM_SENSOR_DISTORTION_COEFS0+j), D, 5)) != OCCAM_API_SUCCESS)
      reportError(r);
    if ((r = occamGetDeviceValuerv
	 (device, OccamParam(OCCAM_SENSOR_INTRINSICS0+j), K, 9)) != OCCAM_API_SUCCESS)
      reportError(r);
    if ((r = occamGetDeviceValuerv
	 (device, OccamParam(OCCAM_SENSOR_ROTATION0+j), R, 9)) != OCCAM_API_SUCCESS)
      reportError(r);
    if ((r = occamGetDeviceValuerv
	 (device, OccamParam(OCCAM_SENSOR_TRANSLATION0+j), T, 3)) != OCCAM_API_SUCCESS)
      reportError(r);

    std::cout<<"sensor "<<j<<":"<<std::endl;
    std::cout<<"  width = "<<sensor_width<<std::endl;
    std::cout<<"  height = "<<sensor_height<<std::endl;
    std::cout<<"  D = "<<matstr(1,5,D)<<std::endl;
    std::cout<<"  K = "<<matstr(3,3,K)<<std::endl;
    std::cout<<"  R = "<<matstr(3,3,R)<<std::endl;
    std::cout<<"  T = "<<matstr(1,3,T)<<std::endl;
  }

  std::cout<<std::endl;

  //////////////////////////////////////////////////////////////////////////////////
  // cylindrical blending parameters

  int stitching_radius;
  int stitching_rotation;
  int stitching_scalewidth;
  int stitching_crop;
  if ((r = occamGetDeviceValuei(device, OCCAM_STITCHING_RADIUS, &stitching_radius)) != OCCAM_API_SUCCESS)
    reportError(r);
  if ((r = occamGetDeviceValuei(device, OCCAM_STITCHING_ROTATION, &stitching_rotation)) != OCCAM_API_SUCCESS)
    reportError(r);
  if ((r = occamGetDeviceValuei(device, OCCAM_STITCHING_SCALEWIDTH, &stitching_scalewidth)) != OCCAM_API_SUCCESS)
    reportError(r);
  if ((r = occamGetDeviceValuei(device, OCCAM_STITCHING_CROP, &stitching_crop)) != OCCAM_API_SUCCESS)
    reportError(r);
  std::cout<<"stitching_radius: "<<stitching_radius<<std::endl;
  std::cout<<"stitching_rotation: "<<stitching_rotation<<std::endl;
  std::cout<<"stitching_scalewidth: "<<stitching_scalewidth<<std::endl;
  std::cout<<"stitching_crop: "<<stitching_crop<<std::endl;

  occamCloseDevice(device);
  occamFreeDeviceList(device_list);
  occamShutdown();

  return 0;
}

