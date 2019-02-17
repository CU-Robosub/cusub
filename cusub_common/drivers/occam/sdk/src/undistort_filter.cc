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
#include "module_utils.h"
#include <string.h>
#include <algorithm>
#include <vector>
#include <assert.h>

struct UndistortSensorArgs {
  int x, y, width, height;
  double D[5];
  double K0[9];
  double K1[9];
  bool operator== (const UndistortSensorArgs& rhs) const {
    return x == rhs.x &&
      y == rhs.y &&
      width == rhs.width &&
      height == rhs.height &&
      std::equal(D,D+5,rhs.D) &&
      std::equal(K0,K0+9,rhs.K0) &&
      std::equal(K1,K1+9,rhs.K1);
  }
  bool operator!= (const UndistortSensorArgs& rhs) const {
    return !(*this == rhs);
  }
};

struct UndistortArgs {
  UndistortSensorArgs sensors[10];
  int sensor_count;
  int dst_step;
  int src_step;
  UndistortArgs()
    : sensor_count(0),
      dst_step(0),
      src_step(0) {
  }
  bool operator== (const UndistortArgs& rhs) const {
    if (sensor_count != rhs.sensor_count ||
	dst_step != rhs.dst_step ||
	src_step != rhs.src_step)
      return false;
    for (int j=0;j<sensor_count;++j)
      if (sensors[j] != rhs.sensors[j])
	return false;
    return true;
  }
  bool operator!= (const UndistortArgs& rhs) const {
    return !(*this == rhs);
  }
};

class OccamUndistortFilterImpl : public OccamUndistortFilter {
  UndistortArgs args0;
  UndistortArgs args;
  std::vector<int> px, py;

  void init(int Si) {
    int si_x = args.sensors[Si].x;
    int si_y = args.sensors[Si].y;
    int si_width = args.sensors[Si].width;
    int si_height = args.sensors[Si].height;
    int dst_step = args.dst_step;
    int src_step = args.src_step;

    const double* D = args.sensors[Si].D;
    const double* K0 = args.sensors[Si].K0;
    const double* K1 = args.sensors[Si].K1;

    for (int dst_y=0;dst_y<si_height;++dst_y) {
      for (int dst_x=0;dst_x<si_width;++dst_x) {

	double xh[2];
	{
	  double r2, r4, r6, a1, a2, a3, cdist, icdist2;
	  double xd, yd;
	  double k[] = {D[0],D[1],D[2],D[3],D[4],0,0,0,0,0,0,0,0,0,0,0,0};
	  double fx=K0[0],fy=K0[4],cx=K0[2],cy=K0[5];

	  double x = 2*double(dst_x)/si_width-1;
	  double y = 2*double(dst_y)/si_height-1;

	  r2 = x*x + y*y;
	  r4 = r2*r2;
	  r6 = r4*r2;
	  a1 = 2*x*y;
	  a2 = r2 + 2*x*x;
	  a3 = r2 + 2*y*y;
	  cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
	  icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
	  xd = x*cdist*icdist2 + k[2]*a1 + k[3]*a2 + k[8]*r2+k[9]*r4;
	  yd = y*cdist*icdist2 + k[2]*a3 + k[3]*a1 + k[10]*r2+k[11]*r4;

	  xh[0] = xd*fx + cx;
	  xh[1] = yd*fy + cy;
	}

	int src_x = int(xh[0]);
	int src_y = int(xh[1]);
	if (src_x < 0 || src_x >= si_width ||
	    src_y < 0 || src_y >= si_height)
	  continue;

	py.push_back((dst_x+si_x)+(dst_y+si_y)*dst_step);
	px.push_back((src_x+si_x)+(src_y+si_y)*src_step);
      }
    }
  }
  void init() {
    px.clear();
    py.clear();
    for (int Si=0;Si<args.sensor_count;++Si)
      init(Si);
  }

public:
  virtual int configure(int N,const int* si_x,const int* si_y,
			const int* si_width,const int* si_height,
			const double* const* D,const double* const* K0,const double* const* K1)  {
    UndistortArgs _args0;
    assert(sizeof(_args0.sensors)/sizeof(_args0.sensors[0])>=N);
    _args0.sensor_count = N;
    for (int j=0;j<N;++j) {
      _args0.sensors[j].x = si_x[j];
      _args0.sensors[j].y = si_y[j];
      _args0.sensors[j].width = si_width[j];
      _args0.sensors[j].height = si_height[j];
      std::copy(D[j],D[j]+5,_args0.sensors[j].D);
      std::copy(K0[j],K0[j]+9,_args0.sensors[j].K0);
      std::copy(K1[j],K1[j]+9,_args0.sensors[j].K1);
    }
    args0 = _args0;
    return OCCAM_API_SUCCESS;
  }

  virtual int compute(const OccamImage* img0,OccamImage** img1p) {
    int channels = 1;
    switch (img0->format) {
    case OCCAM_GRAY8: channels = 1; break;
    case OCCAM_RGB24: channels = 3; break;
    }

    OccamImage* img1 = new OccamImage;
    *img1p = img1;
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

    args0.dst_step = img1->step[0];
    args0.src_step = img0->step[0];

    if (args != args0) {
      args = args0;
      init();
    }

    uint8_t* dstp = img1->data[0];
    uint8_t* srcp = img0->data[0];
    assert(px.size() == py.size());
    for (int j=0;j<px.size();++j)
      dstp[py[j]] = srcp[px[j]];

    return OCCAM_API_SUCCESS;
  }

  virtual int undistortPoints(int N,const int* sensor_indices,
			      const float* x0p,const float* y0p,float* x1p,float* y1p) {
    for (int j=0;j<N;++j) {
      int Si = sensor_indices[j];
      if (Si<0||Si>=args0.sensor_count)
	continue;
      const double* D = args0.sensors[Si].D;
      const double* K0 = args0.sensors[Si].K0;
      const double* K1 = args0.sensors[Si].K1;
      double k[] = {D[0],D[1],D[2],D[3],D[4],0,0,0,0,0,0,0};

      double fx = K0[0];
      double fy = K0[4];
      double ifx = 1./fx;
      double ify = 1./fy;
      double cx = K0[2];
      double cy = K0[5];

      double x = double(x0p[j]);
      double y = double(y0p[j]);
      double x0 = x = (x - cx)*ifx;
      double y0 = y = (y - cy)*ify;

      for (int jj=0;jj<5;++jj) {
	double r2 = x*x + y*y;
	double icdist = (1 + ((k[7]*r2 + k[6])*r2 + k[5])*r2)/(1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2);
	double deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x)+ k[8]*r2+k[9]*r2*r2;
	double deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y+ k[10]*r2+k[11]*r2*r2;
	x = (x0 - deltaX)*icdist;
	y = (y0 - deltaY)*icdist;
      }

      double xx = K1[0]*x + K1[1]*y + K1[2];
      double yy = K1[3]*x + K1[4]*y + K1[5];
      double ww = 1./(K1[6]*x + K1[7]*y + K1[8]);
      x = xx*ww;
      y = yy*ww;

      x1p[j] = float(x);
      y1p[j] = float(y);
    }

    return OCCAM_API_SUCCESS;
  }  
};

static OccamModuleFactory<OccamUndistortFilterImpl> __module_factory
("duf","Undistort",OCCAM_MODULE_UNDISTORT_FILTER,0,0);
void init_undistort_filter() {
  __module_factory.registerModule();
}
