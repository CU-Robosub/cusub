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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <iostream>
#include <assert.h>

void reportError(int r) {
  char buf[256];
  occamGetErrorString((OccamError)r, buf, sizeof(buf));
  std::cerr<<"Fatal error: "<<buf<<" ("<<r<<")"<<std::endl;
  abort();
}
#define OCCAM_CHECK(call) {int r = (call);if (r != OCCAM_API_SUCCESS) reportError(r);}

// In order to perform geometric stitching, we require the full calibration of the sensors.
// This can be read from a file, made up yourself via your own calibration routine, or read
// from the camera using the read_calib example. In this example we just hard-code it.
// Note that the numbers below will not correspond to your camera and won't produce
// correct results (though probably not too far off for Omni 60 build). You must replace them.
struct SensorInfo {
  int width;   // Width of each sensor image
  int height;  // Height of each sensor image

  // Each sensor is represented by standard OpenCV/HZ model:
  // x = K*D([R;T]*X) for 3x1 homogeneous image point x, 4x1 homogeneous world point X.
  // D is 5-parameter polynomial radial/tangential model, same as used in OpenCV.
  double D[5];
  double K[9];
  double R[9];
  double T[3];
};
const int num_sensors = 5;
static SensorInfo sensors[num_sensors] = {
  {
    752,
    480,
    {-0.457145,0.241763,-0.00143307,-0.00129977,-0.0582808},
    {520.617,0,360.782,0,522.858,201.888,0,0,1},
    {0.999996,-0.00268348,-3.09546e-005,0.00268364,0.999968,0.00756186,1.06615e-005,-0.00756192,0.999971},
    {-2.15195,-0.042289,-18.3349}
  },

  {
    752,
    480,
    {-0.458368,0.236445,0.000825515,-0.000278213,-0.0581637},
    {524.451,0,373.553,0,526.513,189.37,0,0,1},
    {0.309898,-0.00317694,-0.950764,0.00200492,0.999994,-0.00268795,0.950768,-0.00107322,0.309903},
    {-4.71971,0.0675663,-25.2683}
  },

  {
    752,
    480,
    {-0.467717,0.285539,0.00077994,-0.000782278,-0.0977914},
    {520.177,0,395.738,0,522.758,213.538,0,0,1},
    {-0.808187,-0.00093463,-0.588925,0.00323188,0.999977,-0.00602211,0.588916,-0.00677033,-0.808166},
    {-0.437803,-1.11268,-26.5371}
  },

  {
    752,
    480,
    {-0.470109,0.282058,0.000151529,0.000146519,-0.0953601},
    {519.623,0,360.313,0,521.984,204.527,0,0,1},
    {-0.812514,-0.00129335,0.582941,-0.00309353,0.999993,-0.00209317,-0.582934,-0.00350408,-0.812512},
    {-8.12113,-1.78012,-21.5154}
  },

  {
    752,
    480,
    {-0.454517,0.239872,0.000626866,0.00019811,-0.0699283},
    {518.378,0,397.503,0,520.922,186.68,0,0,1},
    {0.312189,-0.00154451,0.950019,-0.00741615,0.999964,0.00406275,-0.949991,-0.00831382,0.312166},
    {0.508889,-1.60724,-27.7679}
  },
};

static void convertImage(cv::Mat& src, OccamImage* dst) {
  memset(dst,0,sizeof(*dst));
  dst->refcnt = 1;
  dst->backend = OCCAM_CPU;
  dst->cid = (char*)"";
  dst->width = src.cols;
  dst->height = src.rows;
  dst->step[0] = src.step;
  dst->data[0] = src.data;
  if (src.type() == CV_8UC1) {
    dst->format = OCCAM_GRAY8;
  } else if (src.type() == CV_8UC3) {
    dst->format = OCCAM_RGB24;
  } else {
    std::cerr<<"Unknown image type "<<src.type()<<std::endl;
    abort();
  }
}

static void convertImage(OccamImage* src, cv::Mat& dst) {
  if (src->backend == OCCAM_CPU && src->format == OCCAM_GRAY8) {
    dst = cv::Mat(src->height, src->width, CV_8UC1, src->data[0], src->step[0]);
  } else if (src->backend == OCCAM_CPU && src->format == OCCAM_RGB24) {
    dst = cv::Mat(src->height, src->width, CV_8UC3, src->data[0], src->step[0]);
  } else {
    std::cerr<<"Unknown image backend/format "<<src->backend<<"/"<<src->format<<std::endl;
    abort();
  }
}

int main(int argc, const char** argv) {
  // Argument checking
  if (argc != num_sensors+2) {
    std::cerr<<"Usage: "<<argv[0];
    for (int j=0;j<num_sensors;++j)
      std::cerr<<" <image"<<(j+1)<<">";
    std::cerr<<" <out_image>"<<std::endl;
    return 1;
  }

  // Read the images. Note that the order that the images are given in must correspond
  // to the calibration data above.
  cv::Mat img0[num_sensors];
  for (int j=0;j<num_sensors;++j) {
    cv::Mat imgj = cv::imread(argv[j+1]);
    if (imgj.empty()) {
      std::cerr<<"Failed to read image "<<argv[j+1]<<std::endl;
      return 2;
    }
    img0[j] = imgj;
  }

  // Convert images to OccamImage format
  OccamImage img1[num_sensors];
  OccamImage* img1p[num_sensors];
  for (int j=0;j<num_sensors;++j) {
    convertImage(img0[j],&img1[j]);
    img1p[j] = &img1[j];
  }

  // Construct the cylindrical stitcher/blender module
  void* blend_handle = 0;
  IOccamBlendFilter* blend_iface = 0;
  IOccamParameters* param_iface = 0;
  OCCAM_CHECK(occamInitialize());
  OCCAM_CHECK(occamConstructModule(OCCAM_MODULE_BLEND_FILTER, "cylb", &blend_handle));
  assert(blend_handle);
  OCCAM_CHECK(occamGetInterface(blend_handle, IOCCAMBLENDFILTER, (void**)&blend_iface));
  OCCAM_CHECK(occamGetInterface(blend_handle, IOCCAMPARAMETERS, (void**)&param_iface));
  assert(blend_iface);
  assert(param_iface);

  // Configure the stitcher
  OCCAM_CHECK(param_iface->setValuei(blend_handle,OCCAM_STITCHING_RADIUS,5000000));
  OCCAM_CHECK(param_iface->setValuei(blend_handle,OCCAM_STITCHING_ROTATION,0));
  OCCAM_CHECK(param_iface->setValuei(blend_handle,OCCAM_STITCHING_SCALEWIDTH,1000));
  OCCAM_CHECK(param_iface->setValuei(blend_handle,OCCAM_STITCHING_CROP,1));

  // Pass the calibration data to the stitcher
  int sensor_width[num_sensors];
  int sensor_height[num_sensors];
  double* Dp[num_sensors];
  double* Kp[num_sensors];
  double* Rp[num_sensors];
  double* Tp[num_sensors];
  for (int j=0;j<num_sensors;++j) {
    sensor_width[j] = sensors[j].width;
    sensor_height[j] = sensors[j].height;
    Dp[j] = sensors[j].D;
    Kp[j] = sensors[j].K;
    Rp[j] = sensors[j].R;
    Tp[j] = sensors[j].T;
  }
  // The configure step generates the remap table and is expensive. When converting multiple images,
  // the stitcher only needs to be configured once.
  OCCAM_CHECK(blend_iface->configure(blend_handle,num_sensors,sensor_width,sensor_height,Dp,Kp,Rp,Tp));

  // Stitch the images
  OccamImage* img2 = 0;
  OCCAM_CHECK(blend_iface->compute(blend_handle,img1p,&img2));

  // Write the output image file
  cv::Mat img3;
  convertImage(img2, img3);
  if (!cv::imwrite(argv[num_sensors+1], img3)) {
    std::cerr<<"Failed to write image "<<argv[num_sensors+1]<<std::endl;
    return 3;
  }

  // Free the image
  OCCAM_CHECK(occamFreeImage(img2));
  
  // Release the stitcher module
  occamReleaseModule(blend_handle);

  return 0;
}
