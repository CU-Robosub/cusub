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

#include "device_iface.h"
#include "omni_libusb.h"
#include "gl_utils.h"
#include <string.h>
#include <algorithm>
#include <iostream>
#undef min
#undef max

static DeferredImage subImage(const DeferredImage& img0,int x,int y,int width,int height) {
  auto gen_fn = [=](){
    OccamImage* img1 = 0;
    occamSubImage(img0->get(), &img1, x, y, width, height);
    return std::shared_ptr<OccamImage>(img1,occamFreeImage);
  };
  return DeferredImage(gen_fn,img0);
};

static DeferredImage processImage(std::shared_ptr<void> imagef_handle,
				  std::shared_ptr<void> debayerf_handle,
				  bool is_color,
				  DeferredImage img0) {
  if (is_color) {
    auto gen_fn = [=](){
      IOccamImageFilter* imagef_iface;
      occamGetInterface(debayerf_handle.get(),IOCCAMIMAGEFILTER,(void**)&imagef_iface);
      OccamImage* img1 = 0;
      imagef_iface->compute(debayerf_handle.get(),img0->get(),&img1);
      return std::shared_ptr<OccamImage>(img1,occamFreeImage);
    };
    img0 = DeferredImage(gen_fn,img0);
  }

  auto gen_fn = [=](){
    IOccamImageFilter* imagef_iface;
    occamGetInterface(imagef_handle.get(),IOCCAMIMAGEFILTER,(void**)&imagef_iface);
    OccamImage* img1 = 0;
    imagef_iface->compute(imagef_handle.get(),img0->get(),&img1);
    return std::shared_ptr<OccamImage>(img1,occamFreeImage);
  };
  return DeferredImage(gen_fn,img0);
}

static DeferredImage htile(const std::vector<DeferredImage>& img0) {
  auto gen_fn = [=](){
    const OccamImage* img0p = img0[0]->get();

    OccamImage* img1 = new OccamImage;
    memset(img1,0,sizeof(OccamImage));
    img1->cid = strdup(img0p->cid);
    memcpy(img1->timescale,img0p->timescale,sizeof(img1->timescale));
    img1->time_ns = img0p->time_ns;
    img1->index = img0p->index;
    img1->refcnt = 1;
    img1->backend = img0p->backend;
    img1->format = img0p->format;

    int width = 0;
    int height = 0;
    img1->subimage_count = img0.size();
    for (int j=0;j<img0.size();++j) {
      img1->si_x[j] = width;
      img1->si_y[j] = 0;
      img1->si_width[j] = img0[j]->get()->width;
      img1->si_height[j] = img0[j]->get()->height;
      width += img0[j]->get()->width;
      height = std::max(height,img0[j]->get()->height);
    }

    img1->width = width;
    img1->height = height;

    int bpp = 1;
    occamImageFormatBytesPerPixel(img1->format, &bpp);
    img1->step[0] = ((img1->width*bpp)+15)&~15;
    img1->data[0] = new uint8_t[img1->height * img1->step[0]];

    uint8_t* imgp0 = img1->data[0];
    for (int y=0;y<img1->height;++y,imgp0+=img1->step[0]) {
      uint8_t* imgp1 = imgp0;
      for (int j=0,x=0;j<img0.size();++j) {
	const OccamImage* imgjp = img0[j]->get();
	if (y<imgjp->height)
	  memcpy(imgp1,imgjp->data[0]+y*imgjp->step[0],imgjp->width*bpp);
	else
	  memset(imgp1,0,imgjp->width*bpp);
	x += imgjp->width*bpp;
	imgp1 += imgjp->width*bpp;
      }
    }

    return std::shared_ptr<OccamImage>(img1,occamFreeImage);
  };  
  const Deferred** deps = (const Deferred**)alloca(sizeof(Deferred*)*img0.size());
  for (int j=0;j<img0.size();++j)
    deps[j] = &img0[j];
  return DeferredImage(gen_fn,img0.size(),deps);
}

static DeferredImage blendImages(std::shared_ptr<void> blend_handle,
				 const std::vector<DeferredImage>& srcimg) {
  auto gen_fn = [=](){
    OccamImage* img1 = 0;
    int N = srcimg.size();
    OccamImage** img0 = (OccamImage**)alloca(N*sizeof(OccamImage*));
    for (int j=0;j<N;++j)
      img0[j] = srcimg[j]->get();
    IOccamBlendFilter* blend_iface = 0;
    occamGetInterface(blend_handle.get(),IOCCAMBLENDFILTER,(void**)&blend_iface);
    blend_iface->compute(blend_handle.get(),img0,&img1);
    return std::shared_ptr<OccamImage>(img1,occamFreeImage);
  };
  const Deferred** deps = (const Deferred**)alloca(sizeof(Deferred*)*srcimg.size());
  for (int j=0;j<srcimg.size();++j)
    deps[j] = &srcimg[j];
  return DeferredImage(gen_fn,srcimg.size(),deps);
}

class OccamDevice_omni5u3mt9v022 : public OmniDevice {
  OccamHDRMode hdr_mode;
  int hdr_shutter1;
  int hdr_shutter2;
  int hdr_t2_ratio;
  int hdr_t3_ratio;
  int hdr_vstep1;
  int hdr_vstep2;
  int hdr_vstep3;
  int hdr_vstep4;
  int binning_mode;
  int target_fps;
  double D[5][5];
  double K[5][9];
  double R[5][9];
  double T[5][3];

  // exposure/gain options
  int get_exposure() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0x0b")[0]);
  }
  void set_exposure(int value) {
    program("w0xcc02=1;w0xcc01=0xb8;w0x0b=?",value);
  }
  int get_exposure_microseconds() {
    return int(get_exposure()*32.538);
  }
  void set_exposure_microseconds(int value) {
    set_exposure(int(value/32.538));
  }
  bool get_auto_exposure() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0xaf")[0])&1?true:false;
  }
  void set_auto_exposure(bool value) {
    while (get_auto_exposure() != value)
      program("w0xcc02=1;w0xcc01=0xb8;W0xaf(1)=?",value?1:0);
  }
  int get_gain() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0x35")[0]);
  }
  void set_gain(int value) {
    value = std::min(std::max(1,value),64);
    program("w0xcc02=1;w0xcc01=0xb8;W0x35(127)=?",value);
  }
  bool get_auto_gain() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0xaf")[0]&2)?true:false;
  }
  void set_auto_gain(bool value) {
    while (get_auto_gain() != value)
      program("w0xcc02=1;w0xcc01=0xb8;W0xaf(2)=?",value?2:0);
  }

  int get_hdr_mode() {
    return int(hdr_mode);
  }
  void set_hdr_mode(int value) {
    hdr_mode = OccamHDRMode(value);
    program("w0xcc02=1;w0xcc01=0xb8;W0xf(64)=0;W0xa(511)=372;"
	    "w0x31=29;w0x32=18;w0x33=15;w0x34=4;");
    switch (hdr_mode) {
    case OCCAM_HDR_MODE_DISABLED:
      break;
    case OCCAM_HDR_MODE_AUTO_SHUTTER:
      program("W0xf(64)=64");
      break;
    case OCCAM_HDR_MODE_MANUAL:
      program("W0xf(64)=64;W0xa(256)=0;");
      break;
    }
    set_hdr_shutter1(hdr_shutter1);
    set_hdr_shutter2(hdr_shutter2);
    set_hdr_t2_ratio(hdr_t2_ratio);
    set_hdr_t3_ratio(hdr_t3_ratio);
    set_hdr_vstep1(hdr_vstep1);
    set_hdr_vstep2(hdr_vstep2);
    set_hdr_vstep3(hdr_vstep3);
    set_hdr_vstep4(hdr_vstep4);
  }
  int get_hdr_shutter1() {
    return hdr_shutter1;
  }
  void set_hdr_shutter1(int value) {
    hdr_shutter1 = value;
    if (hdr_mode == OCCAM_HDR_MODE_MANUAL)
      program("w0xcc02=1;w0xcc01=0xb8;w0x8=?",value);
  }
  int get_hdr_shutter2() {
    return hdr_shutter2;
  }
  void set_hdr_shutter2(int value) {
    hdr_shutter2 = value;
    if (hdr_mode == OCCAM_HDR_MODE_MANUAL)
    program("w0xcc02=1;w0xcc01=0xb8;w0x9=?",value);
  }
  int get_hdr_t2_ratio() {
    return hdr_t2_ratio;
  }
  void set_hdr_t2_ratio(int value) {
    hdr_t2_ratio = value;
    if (hdr_mode == OCCAM_HDR_MODE_AUTO_SHUTTER)
      program("w0xcc02=1;w0xcc01=0xb8;W0xa(0xf)=?",value&15);
  }
  int get_hdr_t3_ratio() {
    return hdr_t3_ratio;
  }
  void set_hdr_t3_ratio(int value) {
    hdr_t3_ratio = value;
    if (hdr_mode == OCCAM_HDR_MODE_AUTO_SHUTTER)
      program("w0xcc02=1;w0xcc01=0xb8;W0xa(0xf0)=?",(value&15)<<4);
  }
  int get_hdr_vstep1() {
    return hdr_vstep1;
  }
  void set_hdr_vstep1(int value) {
    hdr_vstep1 = value;
    if (hdr_mode != OCCAM_HDR_MODE_DISABLED)
      program("w0xcc02=1;w0xcc01=0xb8;w0x31=?",value);
  }
  int get_hdr_vstep2() {
    return hdr_vstep2;
  }
  void set_hdr_vstep2(int value) {
    hdr_vstep2 = value;
    if (hdr_mode != OCCAM_HDR_MODE_DISABLED)
      program("w0xcc02=1;w0xcc01=0xb8;w0x32=?",value);
  }
  int get_hdr_vstep3() {
    return hdr_vstep3;
  }
  void set_hdr_vstep3(int value) {
    hdr_vstep3 = value;
    if (hdr_mode != OCCAM_HDR_MODE_DISABLED)
      program("w0xcc02=1;w0xcc01=0xb8;w0x33=?",value);
  }
  int get_hdr_vstep4() {
    return hdr_vstep4;
  }
  void set_hdr_vstep4(int value) {
    hdr_vstep4 = value;
    if (hdr_mode != OCCAM_HDR_MODE_DISABLED)
      program("w0xcc02=1;w0xcc01=0xb8;w0x34=?",value);
  }

  int get_binning_mode() {
    return binning_mode;
  }
  void set_binning_mode(int value) {
    binning_mode = value;
    if (binning_mode == OCCAM_BINNING_DISABLED)
      program("w0xdf01=752;w0xdf03=480;w0xcc02=1;w0xcc01=0xb8;"
	      "W0xd(15)=0;w0xcc02=2;w0xcc01=0x94;w8110=0;w8005=1;");
    else if (binning_mode == OCCAM_BINNING_2x2)
      program("w0xdf01=376;w0xdf03=960;w0xcc02=1;w0xcc01=0xb8;"
	      "W0xd(15)=5;w0xcc02=2;w0xcc01=0x94;w8110=1;w8005=1;");
    else if (binning_mode == OCCAM_BINNING_4x4)
      program("w0xdf01=188;w0xdf03=1920;w0xcc02=1;w0xcc01=0xb8;"
	      "W0xd(15)=10;w0xcc02=2;w0xcc01=0x94;w8110=2;w8005=1;");
  }

  int get_adc_vref() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0x2c")[0]);
  }
  void set_adc_vref(int value) {
    program("w0xcc02=1;w0xcc01=0xb8;w0x2c=?",value);
  }

  bool get_adc_companding() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0x1c")[0])==3;
  }
  void set_adc_companding(bool value) {
    program("w0xcc02=1;w0xcc01=0xb8;w0x1c=?",value?3:2);
  }

  int get_target_fps() {
    return target_fps;
  }
  void set_target_fps(int value) {
    target_fps = value;
    if (target_fps == 60) {
      program("w0xdf05=60;w0xcc02=1;w0xcc01=0xb8;w0x6=45");
      setMinMaxValues(OCCAM_EXPOSURE,1,480);
      setMinMaxValues(OCCAM_EXPOSURE_MICROSECONDS,1,int(480*32.538));
      if (get_exposure()>480)
	set_exposure(480);
    } else if (target_fps == 45) {
      program("w0xdf05=45;w0xcc02=1;w0xcc01=0xb8;w0x6=200");
      setMinMaxValues(OCCAM_EXPOSURE,1,720);
      setMinMaxValues(OCCAM_EXPOSURE_MICROSECONDS,1,int(720*32.538));
      if (get_exposure()>720)
	set_exposure(720);
    } else if (target_fps == 30) {
      program("w0xdf05=30;w0xcc02=1;w0xcc01=0xb8;w0x6=525");
      setMinMaxValues(OCCAM_EXPOSURE,1,960);
      setMinMaxValues(OCCAM_EXPOSURE_MICROSECONDS,1,int(960*32.538));
      if (get_exposure()>960)
	set_exposure(960);
    } else if (target_fps == 15) {
      program("w0xdf05=15;w0xcc02=1;w0xcc01=0xb8;w0x6=1600");
      setMinMaxValues(OCCAM_EXPOSURE,1,1920);
      setMinMaxValues(OCCAM_EXPOSURE_MICROSECONDS,1,int(1920*32.538));
      if (get_exposure()>1920)
	set_exposure(1920);
    } else if (target_fps == 1) {
      program("w0xdf05=1;w0xcc02=1;w0xcc01=0xb8;w0x6=11000");
      setMinMaxValues(OCCAM_EXPOSURE,1,28800);
      setMinMaxValues(OCCAM_EXPOSURE_MICROSECONDS,1,int(28800*32.538));
      if (get_exposure()>28800)
	set_exposure(28800);
    }
  }

  int get_aec_agc_desired_bin() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0xa5")[0]);
  }
  void set_aec_agc_desired_bin(int value) {
    value = std::min(64,std::max(1,value));
    program("w0xcc02=1;w0xcc01=0xb8;w0xa5=?",value);
  }
  int get_aec_update_frequency() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0xa6")[0]);
  }
  void set_aec_update_frequency(int value) {
    value = std::min(15,std::max(0,value));
    program("w0xcc02=1;w0xcc01=0xb8;w0xa6=?",value);
  }
  int get_aec_low_pass_filter() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0xa8")[0]);
  }
  void set_aec_low_pass_filter(int value) {
    if (value != 0 && value != 1 && value != 2)
      value = 2;
    program("w0xcc02=1;w0xcc01=0xb8;w0xa8=?",value);
  }
  int get_agc_output_update_frequency() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0xa9")[0]);
  }
  void set_agc_output_update_frequency(int value) {
    value = std::min(15,std::max(0,value));
    program("w0xcc02=1;w0xcc01=0xb8;w0xa9=?",value);
  }
  int get_agc_low_pass_filter() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0xab")[0]);
  }
  void set_agc_low_pass_filter(int value) {
    if (value != 0 && value != 1 && value != 2)
      value = 2;
    program("w0xcc02=1;w0xcc01=0xb8;w0xab=?",value);
  }
  int get_agc_aec_pixel_count() {
    return int(program("w0xcc02=1;w0xcc01=0xb8;r0xb0")[0]);
  }
  void set_agc_aec_pixel_count(int value) {
    value = std::min(65535,std::max(0,value));
    program("w0xcc02=1;w0xcc01=0xb8;w0xb0=?",value);
  }

  // camera information
  int get_sensor_width() {
    return 752;
  }
  int get_sensor_height() {
    return 480;
  }
  int get_sensor_count() {
    return 5;
  }
  bool get_color() {
    char cn[] = {serial().end()[-1],0};
    return atoi(cn)&1;
  }
  void get_D(int index, double* D0) {
    std::copy(D[index],D[index]+5,D0);
  }
  void set_D(int index, const double* D0) {
    std::copy(D0,D0+5,D[index]);
  }
  void get_K(int index, double* K0) {
    std::copy(K[index],K[index]+9,K0);
  }
  void set_K(int index, const double* K0) {
    std::copy(K0,K0+9,K[index]);
  }
  void get_R(int index, double* R0) {
    std::copy(R[index],R[index]+9,R0);
  }
  void set_R(int index, const double* R0) {
    std::copy(R0,R0+9,R[index]);
  }
  void get_T(int index, double* T0) {
    std::copy(T[index],T[index]+3,T0);
  }
  void set_T(int index, const double* T0) {
    std::copy(T0,T0+3,T[index]);
  }

  // other parameters
  int get_trigger_mode() {
    //    return program("w2=2;w1=0x94;r8004")[0];
    return 0;
  }
  void set_trigger_mode(int value) {
    //    program("w2=2;w1=0x94;w8004=?",value);
  }

  void read_geometric_calib() {
#pragma pack(push,1)
    struct GeometricCalibData_generic {
      uint32_t magic;
      float D[5][5];
      float K[5][9];
      float R[5][9];
      float T[5][3];
    };
#pragma pack(pop)
    GeometricCalibData_generic data;
    if (!readDeviceData(0, 256, &data, sizeof(data)))
      return;
    static const uint32_t GEOMETRIC_CALIB_MAGIC = 0x0e2f4f11;
    if (data.magic != GEOMETRIC_CALIB_MAGIC)
      return;
    for (int j=0;j<5;++j) {
      for (int k=0;k<5;++k)
	D[j][k] = data.D[j][k];
      for (int k=0;k<9;++k)
	K[j][k] = data.K[j][k];
      for (int k=0;k<9;++k)
	R[j][k] = data.R[j][k];
      for (int k=0;k<3;++k)
	T[j][k] = data.T[j][k];
    }
  }

public:
  OccamDevice_omni5u3mt9v022(const std::string& cid)
    : OmniDevice(cid),
      hdr_mode(OCCAM_HDR_MODE_DISABLED),
      hdr_shutter1(443),
      hdr_shutter2(473),
      hdr_t2_ratio(4),
      hdr_t3_ratio(6),
      hdr_vstep1(29),
      hdr_vstep2(18),
      hdr_vstep3(15),
      hdr_vstep4(4),
      binning_mode(OCCAM_BINNING_DISABLED),
      target_fps(60) {
    using namespace std::placeholders;

    addConfigurableModule(OCCAM_DEBAYER_FILTER0,"debayer_filter0",
			  OCCAM_MODULE_DEBAYER_FILTER);
    addConfigurableModule(OCCAM_IMAGE_FILTER0,"image_filter0",
			  OCCAM_MODULE_IMAGE_FILTER);
    addConfigurableModule(OCCAM_UNDISTORT_FILTER0,"undistort_filter0",
			  OCCAM_MODULE_UNDISTORT_FILTER);
    addConfigurableModule(OCCAM_BLENDER0,"blender0",
			  OCCAM_MODULE_BLEND_FILTER);

    setDefaultDeviceValueb(OCCAM_COLOR,get_color());
    setDeviceValueb(OCCAM_IMAGE_PROCESSING_ENABLED,get_color());
    setDefaultDeviceValueb(OCCAM_IMAGE_PROCESSING_ENABLED,get_color());
    setDefaultDeviceValuei(OCCAM_BRIGHTNESS,1970);
    setDefaultDeviceValuei(OCCAM_GAMMA,1000);
    setDefaultDeviceValuei(OCCAM_BLACK_LEVEL,20);
    setDefaultDeviceValuei(OCCAM_WHITE_BALANCE_RED,1177);
    setDefaultDeviceValuei(OCCAM_WHITE_BALANCE_GREEN,1268);
    setDefaultDeviceValuei(OCCAM_WHITE_BALANCE_BLUE,1645);

    registerParami(OCCAM_EXPOSURE,"exposure",OCCAM_SETTINGS,1,480,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_exposure,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_exposure,this,_1));
    registerParami(OCCAM_EXPOSURE_MICROSECONDS,"exposure_microseconds",OCCAM_NOT_STORED,1,1000000,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_exposure_microseconds,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_exposure_microseconds,this,_1));
    registerParamb(OCCAM_AUTO_EXPOSURE,"auto_exposure",OCCAM_SETTINGS,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_auto_exposure,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_auto_exposure,this,_1));
    registerParami(OCCAM_GAIN,"gain",OCCAM_SETTINGS,17,64,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_gain,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_gain,this,_1));
    registerParamb(OCCAM_AUTO_GAIN,"auto_gain",OCCAM_SETTINGS,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_auto_gain,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_auto_gain,this,_1));
    setDefaultDeviceValuei(OCCAM_EXPOSURE,480);
    setDefaultDeviceValueb(OCCAM_AUTO_EXPOSURE,true);
    setDefaultDeviceValuei(OCCAM_GAIN,17);
    setDefaultDeviceValueb(OCCAM_AUTO_GAIN,true);

    registerParami(OCCAM_HDR_MODE,"hdr_mode",OCCAM_SETTINGS,0,0,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_hdr_mode,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_hdr_mode,this,_1));
    std::vector<std::pair<std::string,int> > hdr_mode_values;
    hdr_mode_values.push_back(std::make_pair("Disabled",OCCAM_HDR_MODE_DISABLED));
    hdr_mode_values.push_back(std::make_pair("Auto Shutter",OCCAM_HDR_MODE_AUTO_SHUTTER));
    hdr_mode_values.push_back(std::make_pair("Manual",OCCAM_HDR_MODE_MANUAL));
    setAllowedValues(OCCAM_HDR_MODE,hdr_mode_values);

    registerParami(OCCAM_HDR_SHUTTER1,"hdr_shutter1",OCCAM_SETTINGS,0,480,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_hdr_shutter1,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_hdr_shutter1,this,_1));
    registerParami(OCCAM_HDR_SHUTTER2,"hdr_shutter2",OCCAM_SETTINGS,0,480,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_hdr_shutter2,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_hdr_shutter2,this,_1));
    registerParami(OCCAM_HDR_T2_RATIO,"hdr_t2_ratio",OCCAM_SETTINGS,0,15,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_hdr_t2_ratio,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_hdr_t2_ratio,this,_1));
    registerParami(OCCAM_HDR_T3_RATIO,"hdr_t3_ratio",OCCAM_SETTINGS,0,15,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_hdr_t3_ratio,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_hdr_t3_ratio,this,_1));
    registerParami(OCCAM_HDR_VSTEP1,"hdr_vstep1",OCCAM_SETTINGS,0,31,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_hdr_vstep1,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_hdr_vstep1,this,_1));
    registerParami(OCCAM_HDR_VSTEP2,"hdr_vstep2",OCCAM_SETTINGS,0,31,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_hdr_vstep2,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_hdr_vstep2,this,_1));
    registerParami(OCCAM_HDR_VSTEP3,"hdr_vstep3",OCCAM_SETTINGS,0,31,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_hdr_vstep3,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_hdr_vstep3,this,_1));
    registerParami(OCCAM_HDR_VSTEP4,"hdr_vstep4",OCCAM_SETTINGS,0,31,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_hdr_vstep4,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_hdr_vstep4,this,_1));
    setDefaultDeviceValuei(OCCAM_HDR_MODE,OCCAM_HDR_MODE_DISABLED);
    setDefaultDeviceValuei(OCCAM_HDR_SHUTTER1,443);
    setDefaultDeviceValuei(OCCAM_HDR_SHUTTER2,473);
    setDefaultDeviceValuei(OCCAM_HDR_T2_RATIO,4);
    setDefaultDeviceValuei(OCCAM_HDR_T3_RATIO,6);
    setDefaultDeviceValuei(OCCAM_HDR_VSTEP1,29);
    setDefaultDeviceValuei(OCCAM_HDR_VSTEP2,18);
    setDefaultDeviceValuei(OCCAM_HDR_VSTEP3,15);
    setDefaultDeviceValuei(OCCAM_HDR_VSTEP4,4);

    if (minimumFirmware(1,4,6)) {
      registerParami(OCCAM_BINNING_MODE,"binning_mode",OCCAM_SETTINGS,0,0,
		     std::bind(&OccamDevice_omni5u3mt9v022::get_binning_mode,this),
		     std::bind(&OccamDevice_omni5u3mt9v022::set_binning_mode,this,_1));
      std::vector<std::pair<std::string,int> > binning_modes_values;
      binning_modes_values.push_back(std::make_pair("Disabled",OCCAM_BINNING_DISABLED));
      binning_modes_values.push_back(std::make_pair("2x2",OCCAM_BINNING_2x2));
      binning_modes_values.push_back(std::make_pair("4x4",OCCAM_BINNING_4x4));
      setAllowedValues(OCCAM_BINNING_MODE,binning_modes_values);
      setDefaultDeviceValuei(OCCAM_BINNING_MODE,OCCAM_BINNING_DISABLED);
    }

    registerParami(OCCAM_ADC_VREF,"adc_vref",OCCAM_SETTINGS,0,0,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_adc_vref,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_adc_vref,this,_1));
    std::vector<std::pair<std::string,int> > adc_vref_values;
    adc_vref_values.push_back(std::make_pair("1.0V",0));
    adc_vref_values.push_back(std::make_pair("1.1V",1));
    adc_vref_values.push_back(std::make_pair("1.2V",2));
    adc_vref_values.push_back(std::make_pair("1.3V",3));
    adc_vref_values.push_back(std::make_pair("1.4V",4));
    adc_vref_values.push_back(std::make_pair("1.5V",5));
    adc_vref_values.push_back(std::make_pair("1.6V",6));
    adc_vref_values.push_back(std::make_pair("2.1V",7));
    setAllowedValues(OCCAM_ADC_VREF,adc_vref_values);
    setDefaultDeviceValuei(OCCAM_ADC_VREF,4);

    registerParamb(OCCAM_ADC_COMPANDING,"adc_companding",OCCAM_SETTINGS,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_adc_companding,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_adc_companding,this,_1));

    registerParami(OCCAM_TARGET_FPS,"target_fps",OCCAM_SETTINGS,0,0,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_target_fps,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_target_fps,this,_1));
    std::vector<std::pair<std::string,int> > target_fps_values;
    target_fps_values.push_back(std::make_pair("60",60));
    target_fps_values.push_back(std::make_pair("45",45));
    target_fps_values.push_back(std::make_pair("30",30));
    target_fps_values.push_back(std::make_pair("15",15));
    target_fps_values.push_back(std::make_pair("1",1));
    setAllowedValues(OCCAM_TARGET_FPS,target_fps_values);
    setDefaultDeviceValuei(OCCAM_TARGET_FPS,60);

    registerParami(OCCAM_AEC_AGC_DESIRED_BIN,"aec_agc_desired_bin",OCCAM_SETTINGS,1,64,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_aec_agc_desired_bin,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_aec_agc_desired_bin,this,_1));
    setDefaultDeviceValuei(OCCAM_AEC_AGC_DESIRED_BIN,58);
    registerParami(OCCAM_AEC_UPDATE_FREQUENCY,"aec_update_frequency",OCCAM_SETTINGS,0,15,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_aec_update_frequency,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_aec_update_frequency,this,_1));
    setDefaultDeviceValuei(OCCAM_AEC_UPDATE_FREQUENCY,2);
    registerParami(OCCAM_AEC_LOW_PASS_FILTER,"aec_low_pass_filter",OCCAM_SETTINGS,0,2,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_aec_low_pass_filter,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_aec_low_pass_filter,this,_1));
    std::vector<std::pair<std::string,int> > aec_low_pass_filter_values;
    aec_low_pass_filter_values.push_back(std::make_pair("Mode 0",0));
    aec_low_pass_filter_values.push_back(std::make_pair("Mode 1",1));
    aec_low_pass_filter_values.push_back(std::make_pair("Mode 2",2));
    setAllowedValues(OCCAM_AEC_LOW_PASS_FILTER,aec_low_pass_filter_values);
    setDefaultDeviceValuei(OCCAM_AEC_LOW_PASS_FILTER,2);
    registerParami(OCCAM_AGC_OUTPUT_UPDATE_FREQUENCY,"agc_output_update_frequency",OCCAM_SETTINGS,0,15,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_agc_output_update_frequency,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_agc_output_update_frequency,this,_1));
    setDefaultDeviceValuei(OCCAM_AGC_OUTPUT_UPDATE_FREQUENCY,2);
    registerParami(OCCAM_AGC_LOW_PASS_FILTER,"agc_low_pass_filter",OCCAM_SETTINGS,0,2,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_agc_low_pass_filter,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_agc_low_pass_filter,this,_1));
    std::vector<std::pair<std::string,int> > agc_low_pass_filter_values;
    agc_low_pass_filter_values.push_back(std::make_pair("Mode 0",0));
    agc_low_pass_filter_values.push_back(std::make_pair("Mode 1",1));
    agc_low_pass_filter_values.push_back(std::make_pair("Mode 2",2));
    setAllowedValues(OCCAM_AGC_LOW_PASS_FILTER,agc_low_pass_filter_values);
    setDefaultDeviceValuei(OCCAM_AGC_LOW_PASS_FILTER,2);
    registerParami(OCCAM_AGC_AEC_PIXEL_COUNT,"agc_aec_pixel_count",OCCAM_SETTINGS,0,65535,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_agc_aec_pixel_count,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_agc_aec_pixel_count,this,_1));
    setDefaultDeviceValuei(OCCAM_AGC_AEC_PIXEL_COUNT,44000);

    registerParami(OCCAM_SENSOR_WIDTH,"sensor_width",OCCAM_NOT_STORED,0,0,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_sensor_width,this));
    registerParami(OCCAM_SENSOR_HEIGHT,"sensor_height",OCCAM_NOT_STORED,0,0,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_sensor_height,this));
    registerParami(OCCAM_SENSOR_COUNT,"sensor_count",OCCAM_NOT_STORED,0,0,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_sensor_count,this));
    registerParamb(OCCAM_COLOR,"color",OCCAM_NOT_STORED,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_color,this));
    registerParami(OCCAM_TRIGGER_MODE,"trigger_mode",OCCAM_SETTINGS,0,1,
		   std::bind(&OccamDevice_omni5u3mt9v022::get_trigger_mode,this),
		   std::bind(&OccamDevice_omni5u3mt9v022::set_trigger_mode,this,_1)); 

    registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS0,
		    "D[0]", OCCAM_CALIBRATION, 0, 0, 5,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_D,this,0,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_D,this,0,_1));
    registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS1,
		    "D[1]", OCCAM_CALIBRATION, 0, 0, 5,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_D,this,1,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_D,this,1,_1));
    registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS2,
		    "D[2]", OCCAM_CALIBRATION, 0, 0, 5,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_D,this,2,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_D,this,2,_1));
    registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS3,
		    "D[3]", OCCAM_CALIBRATION, 0, 0, 5,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_D,this,3,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_D,this,3,_1));
    registerParamrv(OCCAM_SENSOR_DISTORTION_COEFS4,
		    "D[4]", OCCAM_CALIBRATION, 0, 0, 5,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_D,this,4,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_D,this,4,_1));

    registerParamrv(OCCAM_SENSOR_INTRINSICS0,
		    "K[0]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_K,this,0,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_K,this,0,_1));
    registerParamrv(OCCAM_SENSOR_INTRINSICS1,
		    "K[1]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_K,this,1,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_K,this,1,_1));
    registerParamrv(OCCAM_SENSOR_INTRINSICS2,
		    "K[2]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_K,this,2,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_K,this,2,_1));
    registerParamrv(OCCAM_SENSOR_INTRINSICS3,
		    "K[3]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_K,this,3,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_K,this,3,_1));
    registerParamrv(OCCAM_SENSOR_INTRINSICS4,
		    "K[4]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_K,this,4,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_K,this,4,_1));

    registerParamrv(OCCAM_SENSOR_ROTATION0,
		    "R[0]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_R,this,0,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_R,this,0,_1));
    registerParamrv(OCCAM_SENSOR_ROTATION1,
		    "R[1]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_R,this,1,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_R,this,1,_1));
    registerParamrv(OCCAM_SENSOR_ROTATION2,
		    "R[2]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_R,this,2,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_R,this,2,_1));
    registerParamrv(OCCAM_SENSOR_ROTATION3,
		    "R[3]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_R,this,3,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_R,this,3,_1));
    registerParamrv(OCCAM_SENSOR_ROTATION4,
		    "R[4]", OCCAM_CALIBRATION, 0, 0, 9,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_R,this,4,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_R,this,4,_1));

    registerParamrv(OCCAM_SENSOR_TRANSLATION0,
		    "T[0]", OCCAM_CALIBRATION, 0, 0, 3,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_T,this,0,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_T,this,0,_1));
    registerParamrv(OCCAM_SENSOR_TRANSLATION1,
		    "T[1]", OCCAM_CALIBRATION, 0, 0, 3,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_T,this,1,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_T,this,1,_1));
    registerParamrv(OCCAM_SENSOR_TRANSLATION2,
		    "T[2]", OCCAM_CALIBRATION, 0, 0, 3,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_T,this,2,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_T,this,2,_1));
    registerParamrv(OCCAM_SENSOR_TRANSLATION3,
		    "T[3]", OCCAM_CALIBRATION, 0, 0, 3,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_T,this,3,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_T,this,3,_1));
    registerParamrv(OCCAM_SENSOR_TRANSLATION4,
		    "T[4]", OCCAM_CALIBRATION, 0, 0, 3,
		    std::bind(&OccamDevice_omni5u3mt9v022::get_T,this,4,_1),
		    std::bind(&OccamDevice_omni5u3mt9v022::set_T,this,4,_1));

    program("w0xdf06=0;w0xcc02=1;w0xcc01=0xb8;W0xd(0x30)=0x30");

    if (minimumFirmware(1,6,0))
      program("w0xdf06=1;w0xcc02=2;w0xcc01=0x94;w8140=1");

    for (int j=0;j<5;++j) {
      double* Dj = D[j];
      double* Kj = K[j];
      double* Rj = R[j];
      double* Tj = T[j];
      Dj[0]=Dj[1]=Dj[2]=Dj[3]=Dj[4]=0;
      Kj[0]=1,Kj[1]=0,Kj[2]=0,Kj[3]=0,Kj[4]=1,Kj[5]=0,Kj[6]=0,Kj[7]=0,Kj[8]=1;
      Rj[0]=1,Rj[1]=0,Rj[2]=0,Rj[3]=0,Rj[4]=1,Rj[5]=0,Rj[6]=0,Rj[7]=0,Rj[8]=1;
      Tj[0]=Tj[1]=Tj[2]=0;
    }

    read_geometric_calib();
    loadSettings();
  }

  virtual int readData(DeviceOutput& out) {
    OccamImage* read_image;
    int r = OmniDevice::readImage(&read_image, 0);
    if (r != OCCAM_API_SUCCESS)
      return r;

    DeferredImage img0(std::shared_ptr<OccamImage>(read_image,occamFreeImage));

    int sensor_width = 752;
    int sensor_height = 480;
    bool is_color = get_color();
    int subsensor_scale = 1;

    if (binning_mode == OCCAM_BINNING_2x2) {
      sensor_width>>=1;
      sensor_height>>=1;
      subsensor_scale = 4;
      is_color = false;
    } else if (binning_mode == OCCAM_BINNING_4x4) {
      sensor_width>>=2;
      sensor_height>>=2;
      subsensor_scale = 16;
      is_color = false;
    }

    if (img0->get()->width != sensor_width)
      return OCCAM_API_DATA_NOT_AVAILABLE;

    std::shared_ptr<void> debayerf_handle = module(OCCAM_DEBAYER_FILTER0);
    std::shared_ptr<void> imagef_handle = module(OCCAM_IMAGE_FILTER0);
    std::shared_ptr<void> blend_handle = module(OCCAM_BLENDER0);

    auto img0_raw0 = subImage(img0,0,0*sensor_height*subsensor_scale,sensor_width,sensor_height);
    auto img0_raw1 = subImage(img0,0,2*sensor_height*subsensor_scale,sensor_width,sensor_height);
    auto img0_raw2 = subImage(img0,0,4*sensor_height*subsensor_scale,sensor_width,sensor_height);
    auto img0_raw3 = subImage(img0,0,1*sensor_height*subsensor_scale,sensor_width,sensor_height);
    auto img0_raw4 = subImage(img0,0,3*sensor_height*subsensor_scale,sensor_width,sensor_height);

    auto img0_processed0 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw0);
    auto img0_processed1 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw1);
    auto img0_processed2 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw2);
    auto img0_processed3 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw3);
    auto img0_processed4 = processImage(imagef_handle,debayerf_handle,is_color,img0_raw4);

    auto img0_raw_tiles0 =
      htile({img0_raw0,img0_raw1,img0_raw2,img0_raw3,img0_raw4});
    auto img0_processed_tiles0 =
      htile({img0_processed0,img0_processed1,img0_processed2,img0_processed3,img0_processed4});

    {
      IOccamBlendFilter* blend_iface = 0;
      occamGetInterface(blend_handle.get(),IOCCAMBLENDFILTER,(void**)&blend_iface);
      int sensor_width[] = {752,752,752,752,752};
      int sensor_height[] = {480,480,480,480,480};
      double* Dp[] = {D[0],D[1],D[2],D[3],D[4]};
      double* Kp[] = {K[0],K[1],K[2],K[3],K[4]};
      double* Rp[] = {R[0],R[1],R[2],R[3],R[4]};
      double* Tp[] = {T[0],T[1],T[2],T[3],T[4]};
      blend_iface->configure(blend_handle.get(),5,sensor_width,sensor_height,Dp,Kp,Rp,Tp);
    }
    auto img0_processed_stitched0 =
      blendImages(blend_handle,{img0_processed0,img0_processed1,img0_processed2,img0_processed3,img0_processed4});

    out.set(OCCAM_IMAGE_TILES0,img0_processed_tiles0);
    out.set(OCCAM_STITCHED_IMAGE0,img0_processed_stitched0);
    out.set(OCCAM_RAW_IMAGE_TILES0,img0_raw_tiles0);

    out.set(OCCAM_IMAGE0,img0_processed0);
    out.set(OCCAM_IMAGE1,img0_processed1);
    out.set(OCCAM_IMAGE2,img0_processed2);
    out.set(OCCAM_IMAGE3,img0_processed3);
    out.set(OCCAM_IMAGE4,img0_processed4);
    out.set(OCCAM_RAW_IMAGE0,img0_raw0);
    out.set(OCCAM_RAW_IMAGE1,img0_raw1);
    out.set(OCCAM_RAW_IMAGE2,img0_raw2);
    out.set(OCCAM_RAW_IMAGE3,img0_raw3);
    out.set(OCCAM_RAW_IMAGE4,img0_raw4);

    return OCCAM_API_SUCCESS;
  }

  virtual void availableData(std::vector<std::pair<OccamDataName,OccamDataType> >& available_data) {
    available_data.push_back(std::make_pair(OCCAM_IMAGE_TILES0,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_STITCHED_IMAGE0,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_IMAGE0,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_IMAGE1,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_IMAGE2,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_IMAGE3,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_IMAGE4,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE_TILES0,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE0,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE1,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE2,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE3,OCCAM_IMAGE));
    available_data.push_back(std::make_pair(OCCAM_RAW_IMAGE4,OCCAM_IMAGE));
  }
};

void init_omni5u3mt9v022() {
  registerDevice<OccamDevice_omni5u3mt9v022>("omni5u3mt9v022");
}
