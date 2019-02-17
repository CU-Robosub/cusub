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

#include "module_utils.h"
#include "remap.h"
#include <vector>
#include <map>
#include <functional>
#include <assert.h>
#include <string.h>
#include <algorithm>
#include <memory>
#include <mutex>
#include <iostream>
#include <math.h>

#undef min
#undef max

struct BlendRemapSensor {
  int width;
  int height;
  double D[5];
  double K[9];
  double R[9];
  double T[3];
  bool operator== (const BlendRemapSensor& rhs) const {
    if (width != rhs.width ||
	height != rhs.height ||
	!std::equal(D,D+5,rhs.D) ||
	!std::equal(K,K+9,rhs.K) ||
	!std::equal(R,R+9,rhs.R) ||
	!std::equal(T,T+3,rhs.T))
      return false;
    return true;
  }
};

struct BlendRemapArgs {
  BlendRemapSensor sensors[5];
  int sensor_count;
  int dst_width;
  int dst_height;
  int cheight1k;
  int cradius1k;
  int cameraboundary1k;
  int stitching_rotation1k;
  int stitching_scalewidth1k;
  int scale_x;
  int scale_y;
  bool crop;
  BlendRemapArgs()
    : sensor_count(0),
      dst_width(0),
      dst_height(0),
      cheight1k(0),
      cradius1k(0),
      cameraboundary1k(0),
      stitching_rotation1k(0),
      stitching_scalewidth1k(0),
      scale_x(0),
      scale_y(0),
      crop(0) {
  }
  bool operator== (const BlendRemapArgs& rhs) const {
    if (sensor_count != rhs.sensor_count)
      return false;
    if (!std::equal(sensors,sensors+sensor_count,rhs.sensors))
      return false;
    return dst_width == rhs.dst_width &&
      dst_height == rhs.dst_height &&
      cheight1k == rhs.cheight1k &&
      cradius1k == rhs.cradius1k &&
      cameraboundary1k == rhs.cameraboundary1k &&
      stitching_rotation1k == rhs.stitching_rotation1k &&
      stitching_scalewidth1k == rhs.stitching_scalewidth1k &&
      crop == rhs.crop &&
      scale_x == rhs.scale_x &&
      scale_y == rhs.scale_y;
  }
  bool operator!= (const BlendRemapArgs& rhs) const {
    return !(*this == rhs);
  }
};

class BlendRemapper {
  BlendRemapArgs args;
  std::shared_ptr<ImageRemap> remap;
  std::mutex lock;

  double findextent(bool min_max, double theta) {
    double cradius = args.cradius1k / 1000.;
    double cameraboundary = args.cameraboundary1k / 1000.;
    int sensor_count = args.sensor_count;

    double dir = min_max?1:-1;
    double miny = 0;
    double maxy = dir*1e4;
    for (int j=0;j<20;++j) {
      double midy = (miny+maxy)/2;
      double X0[] = {cos(theta)*cradius,midy,sin(theta)*cradius};
      bool inbounds = false;
      for (int Si=0;Si<sensor_count;++Si) {
	const BlendRemapSensor& S = args.sensors[Si];
	const double* D = S.D;
	const double* K = S.K;
	const double* R = S.R;
	const double* T = S.T;
	double t1 = fabs(R[6]), t2 = fabs(R[7]), t3 = fabs(R[8]);
	double t4 = t1*t1 + t2*t2 + t3*t3;
	double Mdet =
	  K[4] * K[0] * ((R[5] * R[6] - R[3] * R[8]) * R[1] +
			 (-R[4] * R[6] + R[3] * R[7]) * R[2] + R[0] *
			 (-R[5] * R[7] + R[4] * R[8]));
	double m3norm = sqrt(t4);
	double w = T[2] + R[8] * X0[2] + R[7] * X0[1] + R[6] * X0[0];
	double sign_Mdet = Mdet>0?1:-1;
	double depth = (sign_Mdet * w) / (1 * m3norm);
	if (depth<=0)
	  continue;

	double xh[2];
	{
	  double X = X0[0], Y = X0[1], Z = X0[2];
	  double x = R[0]*X + R[1]*Y + R[2]*Z + T[0];
	  double y = R[3]*X + R[4]*Y + R[5]*Z + T[1];
	  double z = R[6]*X + R[7]*Y + R[8]*Z + T[2];
	  double r2, r4, r6, a1, a2, a3, cdist, icdist2;
	  double xd, yd;
	  double k[] = {D[0],D[1],D[2],D[3],D[4],0,0,0,0,0,0,0,0,0,0,0,0};
	  double fx=K[0],fy=K[4],cx=K[2],cy=K[5];

	  z = z ? 1./z : 1;
	  x *= z; y *= z;
	  if (x < -cameraboundary || x > cameraboundary ||
	      y < -cameraboundary || y > cameraboundary)
	    continue;

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
	if (xh[1]<0||xh[1]>=S.height)
	  continue;
	inbounds = true;
	break;
      }

      if (inbounds)
	miny = midy;
      else
	maxy = midy;
    }

    return miny;
  }

  void findextents(bool crop, double& miny, double& maxy) {
    bool first = true;
    miny = 0;
    maxy = 0;
    const double pi = 3.14159265358979323846;
    for (double theta=0;theta<2*pi;theta+=pi/1500) {
      double miny0 = findextent(false, theta);
      double maxy0 = findextent(true, theta);
      if (crop) {
	if (first) {
	  miny = miny0;
	  maxy = maxy0;
	} else {
	  miny = std::max(miny0,miny);
	  maxy = std::min(maxy0,maxy);
	}
      } else {
	miny = std::min(miny0,miny);
	maxy = std::max(maxy0,maxy);
      }
      first = false;
    }
  }

  void init() {
    double miny, maxy;
    findextents(args.crop, miny, maxy);

    int sensor_count = args.sensor_count;
    int dst_width = args.dst_width;
    int dst_height = args.dst_height;
    double cheight = args.cheight1k / 1000.;
    double cradius = args.cradius1k / 1000.;
    double cameraboundary = args.cameraboundary1k / 1000.;
    float scale_x = 1.f/args.scale_x;
    float scale_y = 1.f/args.scale_y;

    remap = std::make_shared<ImageRemap>(args.dst_width, args.dst_height);
    for (int Si=0;Si<args.sensor_count;++Si)
      remap->addImage(args.sensors[Si].width/args.scale_x,args.sensors[Si].height/args.scale_x);

    struct BlendRegion {
      int parent;
      int Si, Sj;
      float Si_max, Sj_max;
      int dstx_min, dstx_max;
    };
    std::vector<BlendRegion> regions;
    std::vector<int> bx;
    std::vector<float> sxx, syy;

    std::function<int(int)> findrep = [&](int i) {
      assert(i>=0&&i<regions.size());
      if (regions[i].parent<0)
	return i;
      int ir = findrep(regions[i].parent);
      regions[i].parent = ir;
      return ir;
    };
    auto connect = [&](int i, int j) {
      int ir = findrep(i), jr = findrep(j);
      if (regions[ir].Si != regions[jr].Si ||
	  regions[ir].Sj != regions[jr].Sj)
	return;
      regions[jr].parent = ir;
      regions[ir].Si_max = std::max(regions[ir].Si_max,regions[jr].Si_max);
      regions[ir].Sj_max = std::max(regions[ir].Sj_max,regions[jr].Sj_max);
      regions[ir].dstx_min = std::min(regions[ir].dstx_min,regions[jr].dstx_min);
      regions[ir].dstx_max = std::max(regions[ir].dstx_max,regions[jr].dstx_max);
    };

    const double pi = 3.14159265358979323846;
    double theta0 = 2*pi*args.stitching_rotation1k/360.;
    double theta_step = 2*pi/dst_width;
    double Y = miny;
    double Y_step = (maxy-miny)/dst_height;
    for (int dsty=0;dsty<dst_height;++dsty,Y+=Y_step) {

      double theta = theta0;
      sxx.assign(dst_width*sensor_count,-1);
      syy.assign(dst_width*sensor_count,-1);

      regions.resize(dst_width);
      for (BlendRegion& br : regions)
	br.parent = br.Si = br.Sj = -1;
      bx.clear();

      for (int dstx=0;dstx<dst_width;++dstx,theta+=theta_step) {
	double X0[] = {cos(theta)*cradius,Y,sin(theta)*cradius};

	for (int Si=0,soff=0;Si<sensor_count;++Si,soff+=dst_width) {
	  const BlendRemapSensor& S = args.sensors[Si];
	  const double* D = S.D;
	  const double* K = S.K;
	  const double* R = S.R;
	  const double* T = S.T;

	  double t1 = fabs(R[6]), t2 = fabs(R[7]), t3 = fabs(R[8]);
	  double t4 = t1*t1 + t2*t2 + t3*t3;
	  double Mdet =
	    K[4] * K[0] * ((R[5] * R[6] - R[3] * R[8]) * R[1] +
			   (-R[4] * R[6] + R[3] * R[7]) * R[2] + R[0] *
			   (-R[5] * R[7] + R[4] * R[8]));
	  double m3norm = sqrt(t4);
	  double w = T[2] + R[8] * X0[2] + R[7] * X0[1] + R[6] * X0[0];
	  double sign_Mdet = Mdet>0?1:-1;
	  double depth = (sign_Mdet * w) / (1 * m3norm);
	  if (depth<=0)
	    continue;

	  double xh[2];
	  {
	    double X = X0[0], Y = X0[1], Z = X0[2];
	    double x = R[0]*X + R[1]*Y + R[2]*Z + T[0];
	    double y = R[3]*X + R[4]*Y + R[5]*Z + T[1];
	    double z = R[6]*X + R[7]*Y + R[8]*Z + T[2];
	    double r2, r4, r6, a1, a2, a3, cdist, icdist2;
	    double xd, yd;
	    double k[] = {D[0],D[1],D[2],D[3],D[4],0,0,0,0,0,0,0,0,0,0,0,0};
	    	    double fx=K[0],fy=K[4],cx=K[2],cy=K[5];

	    z = z ? 1./z : 1;
	    x *= z; y *= z;
	    if (x < -cameraboundary || x > cameraboundary ||
	     	y < -cameraboundary || y > cameraboundary)
	      continue;

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

	  float srcx = float(xh[0]),srcy = float(xh[1]);
	  if (srcx<0||srcx>=S.width||srcy<0||srcy>=S.height)
	    continue;

	  sxx[soff+dstx] = srcx;
	  syy[soff+dstx] = srcy;
	}
      }

      for (int dstx=dst_width-1;dstx>=0;--dstx) {
	int Sj_count = 0;
	int Sj[2] = {-1,-1};
	for (int Si=0;Si<sensor_count;++Si) {
	  if (sxx[Si*dst_width+dstx]<0)
	    continue;
	  Sj[Sj_count++] = Si;
	  if (Sj_count>=2)
	    break;
	}
	if (Sj_count == 1) {
	  float srcx = sxx[Sj[0]*dst_width+dstx];
	  float srcy = syy[Sj[0]*dst_width+dstx];

	  remap->map(dst_width-dstx-1,dsty,Sj[0],srcx*scale_x,srcy*scale_y);

	} else if (Sj_count == 2) {
	  bx.push_back(dstx);
	  BlendRegion& br0 = regions[dstx];
	  br0.Si = Sj[0];
	  br0.Sj = Sj[1];
	  br0.Si_max = sxx[Sj[0]*dst_width+dstx];
	  br0.Sj_max = sxx[Sj[1]*dst_width+dstx];
	  br0.dstx_min = dstx;
	  br0.dstx_max = dstx;
	  if (dstx<dst_width-1)
	    connect(dstx+1,dstx);
	}
      }

      for (int dstx : bx) {
	int ir = findrep(dstx);
	BlendRegion& br = regions[ir];
	assert(br.Si>=0&&br.Sj>=0);
	if (br.Si_max < br.Sj_max) {
	  std::swap(br.Si,br.Sj);
	  std::swap(br.Si_max,br.Sj_max);
	}
	float Si_srcx = sxx[br.Si*dst_width+dstx];
	float Si_srcy = syy[br.Si*dst_width+dstx];
	float Sj_srcx = sxx[br.Sj*dst_width+dstx];
	float Sj_srcy = syy[br.Sj*dst_width+dstx];
	assert(dstx>=br.dstx_min&&dstx<=br.dstx_max);
	float zif = std::min(1.f,std::max(0.f,float(dstx - br.dstx_min) /
					  float(br.dstx_max - br.dstx_min)));
	remap->map(dst_width-dstx-1,dsty,
		   br.Si,Si_srcx*scale_x,Si_srcy*scale_y,
		   br.Sj,Sj_srcx*scale_x,Sj_srcy*scale_y,
		   zif);
      }
    }
  }
public:
  int operator() (const BlendRemapArgs& args0,
		  const OccamImage* const* img0, OccamImage** img1) {
    std::shared_ptr<ImageRemap> remap0;
    {
      std::unique_lock<std::mutex> g(lock);
      if (args0 != args) {
	args = args0;
	init();
      }
      remap0 = remap;
    }

    return (*remap0)(img0, img1);
  }
};

class OccamCylinderBlendFilterImpl : public OccamBlendFilter,
				     public OccamParameters {
  BlendRemapArgs args0;
  BlendRemapper blender;

  int get_stitching_radius() {
    return args0.cradius1k;
  }
  void set_stitching_radius(int stitching_radius1k0) {
    args0.cradius1k = stitching_radius1k0;
  }
  int get_stitching_rotation() {
    return args0.stitching_rotation1k;
  }
  void set_stitching_rotation(int stitching_rotation1k0) {
    args0.stitching_rotation1k = stitching_rotation1k0;
  }
  int get_stitching_scalewidth() {
    return args0.stitching_scalewidth1k;
  }
  void set_stitching_scalewidth(int stitching_scalewidth1k0) {
    args0.stitching_scalewidth1k = stitching_scalewidth1k0;
  }
  void set_stitching_crop(bool stitching_crop0) {
    args0.crop = stitching_crop0;
  }
  bool get_stitching_crop() {
    return args0.crop;
  }
public:
  OccamCylinderBlendFilterImpl() {
    args0.cradius1k = 1700000;
    args0.stitching_rotation1k = 0;
    args0.stitching_scalewidth1k = 1000;
    args0.crop = true;

    using namespace std::placeholders;
    registerParami(OCCAM_STITCHING_RADIUS,
		   "stitching_radius", OCCAM_SETTINGS, 1700000, 10000000,
		   std::bind(&OccamCylinderBlendFilterImpl::get_stitching_radius,this),
		   std::bind(&OccamCylinderBlendFilterImpl::set_stitching_radius,this,_1));
    registerParami(OCCAM_STITCHING_ROTATION,
		   "stitching_rotation", OCCAM_SETTINGS, 0, 360,
		   std::bind(&OccamCylinderBlendFilterImpl::get_stitching_rotation,this),
		   std::bind(&OccamCylinderBlendFilterImpl::set_stitching_rotation,this,_1));
    registerParami(OCCAM_STITCHING_SCALEWIDTH,
		   "stitching_scalewidth", OCCAM_SETTINGS, 1000, 8000,
		   std::bind(&OccamCylinderBlendFilterImpl::get_stitching_scalewidth,this),
		   std::bind(&OccamCylinderBlendFilterImpl::set_stitching_scalewidth,this,_1));
    registerParamb(OCCAM_STITCHING_CROP,
		   "stitching_crop", OCCAM_SETTINGS,
		   std::bind(&OccamCylinderBlendFilterImpl::get_stitching_crop,this),
		   std::bind(&OccamCylinderBlendFilterImpl::set_stitching_crop,this,_1));

    setDefaultValuei(OCCAM_STITCHING_RADIUS,1700000);
    setDefaultValuei(OCCAM_STITCHING_ROTATION,0);
    setDefaultValuei(OCCAM_STITCHING_SCALEWIDTH,1000);
    setDefaultValueb(OCCAM_STITCHING_CROP,true);
  }

  virtual int configure(int N,
			const int* sensor_width,
			const int* sensor_height,
			const double* const* D,
			const double* const* K,
			const double* const* R,
			const double* const* T) {
    const int max_sensors = sizeof(args0.sensors)/sizeof(args0.sensors[0]);
    if (N>max_sensors)
      return OCCAM_API_INVALID_COUNT;
    args0.sensor_count = 0;
    for (int j=0;j<N;++j) {
      BlendRemapSensor& S = args0.sensors[args0.sensor_count++];
      S.width = sensor_width[j];
      S.height = sensor_height[j];
      std::copy(D[j],D[j]+5,S.D);
      std::copy(K[j],K[j]+9,S.K);
      std::copy(R[j],R[j]+9,S.R);
      std::copy(T[j],T[j]+3,S.T);
    }
    return OCCAM_API_SUCCESS;
  }

  virtual int compute(const OccamImage* const* img0,OccamImage** img1) {
     BlendRemapArgs args = args0;
     float scalewidth = std::min(8.f,std::max(1.f,args0.stitching_scalewidth1k/1000.f));
     int width = 0;
     int height = 0;
     for (int j=0;j<args.sensor_count;++j) {
       width += img0[j]->width;
       height = std::max(height,img0[j]->height);
     }
     args.dst_width = int(width/scalewidth);
     args.dst_height = height;
     args.cheight1k = std::max(500*1000,args.cradius1k);
     args.cameraboundary1k = 1000;
     args.scale_x = args.sensors[0].width / img0[0]->width;
     args.scale_y = args.sensors[0].height / img0[0]->height;

     return blender(args,img0,img1);
  }
};

static OccamModuleFactory<OccamCylinderBlendFilterImpl> __module_factory
("cylb","Cylindrical",OCCAM_MODULE_BLEND_FILTER,0,0);
void init_cylinder_blend_filter() {
  __module_factory.registerModule();
}
