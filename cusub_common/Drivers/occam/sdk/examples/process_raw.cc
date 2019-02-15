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

// Reads raw images and device parameters from the current directory, as written by
// record_raw example, and generate color-processed and stitched images.

#include "indigo.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <memory>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static void reportError(int error_code) {
  char str[1024] = {0};
  occamGetErrorString((OccamError)error_code, str, sizeof(str));
  std::cerr<<"Occam API Error: "<<str<<" ("<<error_code<<")"<<std::endl;
  abort();
}

#define OCCAM_CHECK(call) {int r = (call);if (r != OCCAM_API_SUCCESS) reportError(r);}

typedef std::shared_ptr<OccamImage> OccamImagePtr;

class OfflineProcessor {
  std::string model;
  std::string serial;
  int is_color = 0;
  int sensor_count = 0;
  int sensor_width = 0;
  int sensor_height = 0;
  int image_processing_enabled = 0;
  int brightness = 0;
  int gamma = 0;
  int black_level = 0;
  int white_balance_red = 0;
  int white_balance_green = 0;
  int white_balance_blue = 0;
  int stitching_radius = 0;
  int stitching_rotation = 0;
  int stitching_scalewidth = 0;
  int stitching_crop = 0;

  std::shared_ptr<void> debayerf_handle;
  std::shared_ptr<void> imagef_handle;
  std::shared_ptr<void> blend_handle;

  int frame_count = 0;

  // load device parameters from file named "device_info" in current directory and configure the
  // debayer/image filter/blender modules.
  void loadDeviceInfo() {
    std::ifstream fin("device_info");
    if (!fin.is_open()) {
      std::cerr<<"failed to open device_info file"<<std::endl;
      abort();
    }

    fin>>model;
    fin>>serial;
    fin>>is_color;
    fin>>sensor_count;
    fin>>sensor_width;
    fin>>sensor_height;
    fin>>image_processing_enabled;
    fin>>brightness;
    fin>>gamma;
    fin>>black_level;
    fin>>white_balance_red;
    fin>>white_balance_green;
    fin>>white_balance_blue;
    fin>>stitching_radius;
    fin>>stitching_rotation;
    fin>>stitching_scalewidth;
    fin>>stitching_crop;

    if (model != "omni5u3mt9v022") {
      std::cerr<<"This example only works with omni5u3mt9v022"<<std::endl;
      abort();
    }

    std::vector<double> D(sensor_count*5);
    std::vector<double> K(sensor_count*9);
    std::vector<double> R(sensor_count*9);
    std::vector<double> T(sensor_count*3);
    std::vector<double*> Dp(sensor_count);
    std::vector<double*> Kp(sensor_count);
    std::vector<double*> Rp(sensor_count);
    std::vector<double*> Tp(sensor_count);
    std::vector<int> sensor_widthp(sensor_count);
    std::vector<int> sensor_heightp(sensor_count);
    for (int j=0;j<sensor_count;++j) {

      for (int k=0;k<5;++k)
	fin>>D[j*5+k];
      for (int k=0;k<9;++k)
	fin>>K[j*9+k];
      for (int k=0;k<9;++k)
	fin>>R[j*9+k];
      for (int k=0;k<3;++k)
	fin>>T[j*3+k];

      Dp[j] = &D[j*5];
      Kp[j] = &K[j*9];
      Rp[j] = &R[j*9];
      Tp[j] = &T[j*3];

      sensor_widthp[j] = sensor_width;
      sensor_heightp[j] = sensor_height;
    }

    // Configure the image filter
    {
      IOccamParameters* param_iface = 0;
      OCCAM_CHECK(occamGetInterface(imagef_handle.get(), IOCCAMPARAMETERS, (void**)&param_iface));

      OCCAM_CHECK(param_iface->setValuei(imagef_handle.get(),OCCAM_COLOR,is_color));
      OCCAM_CHECK(param_iface->setValuei(imagef_handle.get(),OCCAM_IMAGE_PROCESSING_ENABLED,image_processing_enabled));
      OCCAM_CHECK(param_iface->setValuei(imagef_handle.get(),OCCAM_BRIGHTNESS,brightness));
      OCCAM_CHECK(param_iface->setValuei(imagef_handle.get(),OCCAM_GAMMA,gamma));
      OCCAM_CHECK(param_iface->setValuei(imagef_handle.get(),OCCAM_BLACK_LEVEL,black_level));
      if (is_color) {
	OCCAM_CHECK(param_iface->setValuei(imagef_handle.get(),OCCAM_WHITE_BALANCE_RED,white_balance_red));
	OCCAM_CHECK(param_iface->setValuei(imagef_handle.get(),OCCAM_WHITE_BALANCE_GREEN,white_balance_green));
	OCCAM_CHECK(param_iface->setValuei(imagef_handle.get(),OCCAM_WHITE_BALANCE_BLUE,white_balance_blue));
      }
    }

    // Configure the stitcher
    {
      IOccamBlendFilter* blend_iface = 0;
      IOccamParameters* param_iface = 0;
      OCCAM_CHECK(occamGetInterface(blend_handle.get(), IOCCAMBLENDFILTER, (void**)&blend_iface));
      OCCAM_CHECK(occamGetInterface(blend_handle.get(), IOCCAMPARAMETERS, (void**)&param_iface));

      OCCAM_CHECK(param_iface->setValuei(blend_handle.get(),OCCAM_STITCHING_RADIUS,stitching_radius));
      OCCAM_CHECK(param_iface->setValuei(blend_handle.get(),OCCAM_STITCHING_ROTATION,stitching_rotation));
      OCCAM_CHECK(param_iface->setValuei(blend_handle.get(),OCCAM_STITCHING_SCALEWIDTH,stitching_scalewidth));
      OCCAM_CHECK(param_iface->setValuei(blend_handle.get(),OCCAM_STITCHING_CROP,stitching_crop));
      OCCAM_CHECK(blend_iface->configure(blend_handle.get(),sensor_count,&sensor_widthp[0],&sensor_heightp[0],
					 &Dp[0],&Kp[0],&Rp[0],&Tp[0]));
    }

  }

  static void convertImage(cv::Mat& src, OccamImagePtr& dst) {
    dst = OccamImagePtr((OccamImage*)occamAlloc(sizeof(OccamImage)),occamFree);
    memset(dst.get(),0,sizeof(*dst.get()));
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

  static void convertImage(OccamImagePtr src, cv::Mat& dst) {
    if (src->backend == OCCAM_CPU && src->format == OCCAM_GRAY8) {
      dst = cv::Mat(src->height, src->width, CV_8UC1, src->data[0], src->step[0]);
    } else if (src->backend == OCCAM_CPU && src->format == OCCAM_RGB24) {
      dst = cv::Mat(src->height, src->width, CV_8UC3, src->data[0], src->step[0]);
    } else {
      std::cerr<<"Unknown image backend/format "<<src->backend<<"/"<<src->format<<std::endl;
      abort();
    }
  }

  // Optionally run debayer module if this is a color device, and
  // then run the image processing module. If image processing is
  // configured as disabled in the device, the module itself acts as
  // passthrough.
  OccamImagePtr processImage(OccamImagePtr img0) {
    if (is_color) {
      IOccamImageFilter* imagef_iface;
      occamGetInterface(debayerf_handle.get(),IOCCAMIMAGEFILTER,(void**)&imagef_iface);
      OccamImage* img1 = 0;
      imagef_iface->compute(debayerf_handle.get(),img0.get(),&img1);
      img0 = OccamImagePtr(img1,occamFreeImage);
    }

    IOccamImageFilter* imagef_iface;
    occamGetInterface(imagef_handle.get(),IOCCAMIMAGEFILTER,(void**)&imagef_iface);
    OccamImage* img1 = 0;
    imagef_iface->compute(imagef_handle.get(),img0.get(),&img1);
    return OccamImagePtr(img1,occamFreeImage);
  }

  // Stitch the given images
  OccamImagePtr blendImages(const std::vector<OccamImagePtr>& srcimg) {
    OccamImage* img1 = 0;
    int N = srcimg.size();
    OccamImage** img0 = (OccamImage**)alloca(N*sizeof(OccamImage*));
    for (int j=0;j<N;++j)
      img0[j] = srcimg[j].get();
    IOccamBlendFilter* blend_iface = 0;
    occamGetInterface(blend_handle.get(),IOCCAMBLENDFILTER,(void**)&blend_iface);
    blend_iface->compute(blend_handle.get(),img0,&img1);
    return OccamImagePtr(img1,occamFreeImage);
  }

public:
  OfflineProcessor() {
    void* debayerf_handle0;
    void* imagef_handle0;
    void* blend_handle0;
    OCCAM_CHECK(occamConstructModule(OCCAM_MODULE_DEBAYER_FILTER, "dbf", &debayerf_handle0));
    OCCAM_CHECK(occamConstructModule(OCCAM_MODULE_IMAGE_FILTER, "imf", &imagef_handle0));
    OCCAM_CHECK(occamConstructModule(OCCAM_MODULE_BLEND_FILTER, "cylb", &blend_handle0));

    debayerf_handle = std::shared_ptr<void>(debayerf_handle0,occamReleaseModule);
    imagef_handle = std::shared_ptr<void>(imagef_handle0,occamReleaseModule);
    blend_handle = std::shared_ptr<void>(blend_handle0,occamReleaseModule);

    loadDeviceInfo();
  }

  // Called once per frame. If there is another frame to read, it reads the raw images for that frame, writes
  // color-processed images, and writes a stitched image. If there are no more images, it returns false.
  bool processFrame() {
    ++frame_count;

    std::cerr<<"Processing frame "<<frame_count<<std::endl;

    std::vector<cv::Mat> img0(sensor_count);
    std::vector<OccamImagePtr> img1(sensor_count);
    std::vector<OccamImagePtr> img2(sensor_count);
    for (int image_index=0;image_index<sensor_count;++image_index) {

      {
	std::stringstream sout;
	sout<<"Frame-"<<std::setfill('0')<<std::setw(6)<<frame_count<<"-RAW_IMAGE"<<image_index<<".bmp";	
	std::cerr<<"Reading "<<sout.str()<<std::endl;
	img0[image_index] = cv::imread(sout.str(),cv::IMREAD_GRAYSCALE);
	if (img0[image_index].empty()) {
	  return false;
	}
      }

      convertImage(img0[image_index], img1[image_index]);

      img2[image_index] = processImage(img1[image_index]);

      {
	std::stringstream sout;
	sout<<"Frame-"<<std::setfill('0')<<std::setw(6)<<frame_count<<"-IMAGE"<<image_index<<".bmp";
	std::cerr<<"Writing "<<sout.str()<<std::endl;
	cv::Mat dst;
	convertImage(img2[image_index], dst);
	if (is_color)
	  cv::cvtColor(dst, dst, cv::COLOR_RGB2BGR);
	cv::imwrite(sout.str(), dst);
      }
    }

    {
      std::stringstream sout;
      sout<<"Frame-"<<std::setfill('0')<<std::setw(6)<<frame_count<<"-STITCHED_IMAGE0.bmp";
      std::cerr<<"Writing "<<sout.str()<<std::endl;
      OccamImagePtr stitched = blendImages(img2);
      cv::Mat dst;
      convertImage(stitched, dst);
      cv::imwrite(sout.str(), dst);
    }

    return true;
  }
};


int main(int argc, const char** argv) {
  OCCAM_CHECK(occamInitialize());

  OfflineProcessor offline_proc;
  while (offline_proc.processFrame());

  occamShutdown();

  return 0;
}
