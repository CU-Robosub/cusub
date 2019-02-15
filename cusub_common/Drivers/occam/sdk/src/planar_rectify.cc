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

// various logic derived from OpenCV 1bdd86edeba4babff7a78586beba20841bb87fa9
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2010, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "module_utils.h"
#include "remap.h"
#include <algorithm>
#include <mutex>
#include <thread>
#include <memory>
#include <iostream>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <math.h>
#undef min
#undef max

class SVD {
  std::vector<double> _buf;

  static void compute0(int m, int n,
		       double* At, double* W, double* Vt,
		       int n1, double minval, double eps) {
    int i, j, k, iter, max_iter = std::max(m, 30);
    double c, s;
    double sd;
    int a_width = m;
    int v_width = n;

    for (i = 0; i < n; i++) {
      for (k = 0, sd = 0; k < m; k++) {
	double t = At[i*a_width + k];
	sd += (double)t*t;
      }
      W[i] = sd;

      if (Vt) {
	for (k = 0; k < n; k++)
	  Vt[i*v_width + k] = 0;
	Vt[i*v_width + i] = 1;
      }
    }

    for (iter = 0; iter < max_iter; iter++) {
      bool changed = false;

      for (i = 0; i < n-1; i++)
	for (j = i+1; j < n; j++) {
	  double *Ai = At + i*a_width, *Aj = At + j*a_width;
	  double a = W[i];
	  double p = 0;
	  double b = W[j];

	  for (k = 0; k < m; k++)
	    p += (double)Ai[k]*Aj[k];

	  if (std::abs(p) <= eps*std::sqrt((double)a*b))
	    continue;

	  p *= 2;
	  double beta = a - b, gamma = hypot((double)p, beta);
	  if (beta < 0) {
	    double delta = (gamma - beta)*0.5;
	    s = (double)std::sqrt(delta/gamma);
	    c = (double)(p/(gamma*s*2));
	  } else {
	    c = (double)std::sqrt((gamma + beta)/(gamma*2));
	    s = (double)(p/(gamma*c*2));
	  }

	  a = b = 0;
	  for (k = 0; k < m; k++) {
	    double t0 = c*Ai[k] + s*Aj[k];
	    double t1 = -s*Ai[k] + c*Aj[k];
	    Ai[k] = t0;
	    Aj[k] = t1;

	    a += (double)t0*t0;
	    b += (double)t1*t1;
	  }
	  W[i] = a;
	  W[j] = b;

	  changed = true;

	  if (Vt) {
	    double *Vi = Vt + i*v_width, *Vj = Vt + j*v_width;
	    for (k=0; k < n; k++) {
	      double t0 = c*Vi[k] + s*Vj[k];
	      double t1 = -s*Vi[k] + c*Vj[k];
	      Vi[k] = t0; Vj[k] = t1;
	    }
	  }
	}
      if (!changed)
	break;
    }

    for (i = 0; i < n; i++) {
      for (k = 0, sd = 0; k < m; k++) {
	double t = At[i*a_width + k];
	sd += (double)t*t;
      }
      W[i] = std::sqrt(sd);
    }

    for (i = 0; i < n-1; i++) {
      j = i;
      for (k = i+1; k < n; k++) {
	if (W[j] < W[k])
	  j = k;
      }
      if (i != j) {
	std::swap(W[i], W[j]);
	if (Vt) {
	  for (k = 0; k < m; k++)
	    std::swap(At[i*a_width + k], At[j*a_width + k]);

	  for (k = 0; k < n; k++)
	    std::swap(Vt[i*v_width + k], Vt[j*v_width + k]);
	}
      }
    }

    if (!Vt)
      return;

    uint64_t state = 0x12345678;
    auto next = [&state](){
      state = (uint64_t)(unsigned)state*4164903690U + (unsigned)(state >> 32);
      return (unsigned)state;
    };

    for (i = 0; i < n1; i++) {
      sd = i < n ? W[i] : 0;

      while (sd <= minval) {
	const double val0 = (double)(1./m);
	for (k = 0; k < m; k++) {
	  double val = (next() & 256) != 0 ? val0 : -val0;
	  At[i*a_width + k] = val;
	}
	for (iter = 0; iter < 2; iter++) {
	  for (j = 0; j < i; j++) {
	    sd = 0;
	    for (k = 0; k < m; k++)
	      sd += At[i*a_width + k]*At[j*a_width + k];
	    double asum = 0;
	    for (k = 0; k < m; k++) {
	      double t = (double)(At[i*a_width + k] - sd*At[j*a_width + k]);
	      At[i*a_width + k] = t;
	      asum += std::abs(t);
	    }
	    asum = asum ? 1/asum : 0;
	    for (k = 0; k < m; k++)
	      At[i*a_width + k] *= asum;
	  }
	}
	sd = 0;
	for (k = 0; k < m; k++) {
	  double t = At[i*a_width + k];
	  sd += (double)t*t;
	}
	sd = std::sqrt(sd);
      }

      s = (double)(1/sd);
      for (k = 0; k < m; k++)
	At[i*a_width + k] *= s;
    }
  }

  void compute0(int m, int n, double* A, double** W, double** Ut, double** Vt) {
    assert(m>=n);
    _buf.resize(m*m+n*n+n);
    double* temp_Ut = &_buf[0];
    double* temp_Vt = &_buf[m*m];
    double* temp_W = &_buf[m*m+n*n];
    std::fill(temp_Ut,temp_Ut+m*m,0);

    for (int i=0;i<m;++i)
      for (int j=0;j<n;++j)
   	temp_Ut[j*m+i] = A[i*n+j];

    compute0(m, n, temp_Ut, temp_W, temp_Vt, m,
	    std::numeric_limits<double>::min(),
	    10*std::numeric_limits<double>::epsilon());

    if (W)
      *W = temp_W;
    if (Ut)
      *Ut = temp_Ut;
    if (Vt)
      *Vt = temp_Vt;
  }

public:
  void operator() (int m, int n, double* A, double** W, double** Ut, double** Vt) {
    compute(m, n, A, W, Ut, Vt);
  }
  void operator() (int m, int n, double* A, double** W) {
    compute(m, n, A, W);
  }
  void compute(int m, int n, double* A, double** W, double** Ut, double** Vt) {
    compute0(m, n, A, W, Ut, Vt);
  }
  void compute(int m, int n, double* A, double** W) {
    compute0(m, n, A, W, 0, 0);
  }
};

static void unpackR(const double* r, double* R) {
  double rx=r[0], ry=r[1], rz=r[2];
  double theta = std::sqrt(rx*rx + ry*ry + rz*rz);
  if (theta < std::numeric_limits<double>::epsilon())
    R[0]=1,R[1]=0,R[2]=0, R[3]=0,R[4]=1,R[5]=0, R[6]=0,R[7]=0,R[8]=1;
  else {
    const double I[] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    double c = cos(theta);
    double s = sin(theta);
    double c1 = 1. - c;
    double itheta = theta ? 1./theta : 0.;
    rx *= itheta; ry *= itheta; rz *= itheta;
    double rrt[] = { rx*rx, rx*ry, rx*rz, rx*ry, ry*ry, ry*rz, rx*rz, ry*rz, rz*rz };
    double _r_x_[] = { 0, -rz, ry, rz, 0, -rx, -ry, rx, 0 };
    for (int k=0;k<9;k++)
      R[k] = c*I[k] + c1*rrt[k] + s*_r_x_[k];
  }
}

static void packR(const double* R, double* r) {
  SVD svd;
  double R0[] = {
    R[0],R[1],R[2],R[3],R[4],R[5],R[6],R[7],R[8]
  };
  double* W;
  double* Ut;
  double* Vt;
  svd(3, 3, R0, &W, &Ut, &Vt);

  double R1[] = {
    Ut[0] * Vt[0] + Ut[3] * Vt[3] + Ut[6] * Vt[6],
    Ut[0] * Vt[1] + Ut[3] * Vt[4] + Ut[6] * Vt[7],
    Ut[0] * Vt[2] + Ut[3] * Vt[5] + Ut[6] * Vt[8],
    Ut[1] * Vt[0] + Ut[4] * Vt[3] + Ut[7] * Vt[6],
    Ut[1] * Vt[1] + Ut[4] * Vt[4] + Ut[7] * Vt[7],
    Ut[1] * Vt[2] + Ut[4] * Vt[5] + Ut[7] * Vt[8],
    Ut[2] * Vt[0] + Ut[5] * Vt[3] + Ut[8] * Vt[6],
    Ut[2] * Vt[1] + Ut[5] * Vt[4] + Ut[8] * Vt[7],
    Ut[2] * Vt[2] + Ut[5] * Vt[5] + Ut[8] * Vt[8]
  };

  double rx = R1[7] - R1[5];
  double ry = R1[2] - R1[6];
  double rz = R1[3] - R1[1];

  double s = std::sqrt((rx*rx + ry*ry + rz*rz)*0.25);
  double c = (R1[0] + R1[4] + R1[8] - 1)*0.5;
  c = c > 1. ? 1. : c < -1. ? -1. : c;
  double theta = acos(c);

  if (s < 1e-5) {
    double t;

    if (c > 0)
      rx = ry = rz = 0;
    else {
      t = (R1[0] + 1)*0.5;
      rx = std::sqrt(std::max(t,0.));
      t = (R1[4] + 1)*0.5;
      ry = std::sqrt(std::max(t,0.))*(R1[1] < 0 ? -1. : 1.);
      t = (R1[8] + 1)*0.5;
      rz = std::sqrt(std::max(t,0.))*(R1[2] < 0 ? -1. : 1.);
      if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R1[5] > 0) != (ry*rz > 0) )
	rz = -rz;
      theta /= std::sqrt(rx*rx + ry*ry + rz*rz);
      rx *= theta;
      ry *= theta;
      rz *= theta;
    }
  }
  else {
    double vth = 1/(2*s);
    vth *= theta;
    rx *= vth;
    ry *= vth;
    rz *= vth;
  }

  r[0] = rx;
  r[1] = ry;
  r[2] = rz;
}

class OccamStereoRectifyImpl : public OccamStereoRectify, public OccamParameters {
  struct SensorPair {
    int width;
    int height;
    double D0[5];
    double D1[5];
    double K0[9];
    double K1[9];
    double R0[9];
    double R1[9];
    double T0[3];
    double T1[3];
    double Q[16];
    double H0[9];
    double C[16];
    double B0[9];
    double B1[9];
    std::shared_ptr<ImageRemap> rectifymap0;
    std::shared_ptr<ImageRemap> rectifymap1;
    std::shared_ptr<ImageRemap> unrectifymap0;
    std::shared_ptr<ImageRemap> unrectifymap1;
    std::vector<uint8_t> edge_mask0;
    std::vector<uint8_t> edge_mask1;
  };

  struct Rep {
    bool transposed;
    int scale;
    std::vector<SensorPair> pairs;
  };
  std::shared_ptr<Rep> rep;
  std::mutex lock;
  int scale;

  int get_scale() {
    return scale;
  }
  void set_scale(int value) {
    scale = value;
  }

  void initRectifyMap(int width, int height, int scale,
		      const double* D, const double* K,
		      const double* H, const double* P,
		      double* B, ImageRemap& rectifymap,
		      std::vector<uint8_t>& edge_mask,
		      bool transposed) {
    double B0[] = {
      P[0] * H[0] + P[1] * H[3] + P[2] * H[6],
      P[0] * H[1] + P[1] * H[4] + P[2] * H[7],
      P[0] * H[2] + P[1] * H[5] + P[2] * H[8],
      P[4] * H[0] + P[5] * H[3] + P[6] * H[6],
      P[4] * H[1] + P[5] * H[4] + P[6] * H[7],
      P[4] * H[2] + P[5] * H[5] + P[6] * H[8],
      P[8] * H[0] + P[9] * H[3] + P[10] * H[6],
      P[8] * H[1] + P[9] * H[4] + P[10] * H[7],
      P[8] * H[2] + P[9] * H[5] + P[10] * H[8]
    };
    double t1 = -B0[5] * B0[7] + B0[4] * B0[8];
    double t2 = B0[2] * B0[7] - B0[1] * B0[8];
    double t3 = -B0[2] * B0[4] + B0[1] * B0[5];
    double t4 = B0[0] * t1 + t2 * B0[3] + t3 * B0[6];
    t4 = 0.1e1 / t4;
    B[0] = t1 * t4;
    B[1] = t2 * t4;
    B[2] = t3 * t4;
    B[3] = -(-B0[5] * B0[6] + B0[3] * B0[8]) * t4;
    B[4] = (-B0[2] * B0[6] + B0[0] * B0[8]) * t4;
    B[5] = -(-B0[2] * B0[3] + B0[0] * B0[5]) * t4;
    B[6] = (-B0[4] * B0[6] + B0[3] * B0[7]) * t4;
    B[7] = -(-B0[1] * B0[6] + B0[0] * B0[7]) * t4;
    B[8] = (-B0[1] * B0[3] + B0[0] * B0[4]) * t4;

    edge_mask.resize(height*width);
    const int max_edge_dist = 20;

    double u0 = K[2];
    double v0 = K[5];
    double fx = K[0];
    double fy = K[4];
    double k1 = D[0];
    double k2 = D[1];
    double p1 = D[2];
    double p2 = D[3];
    double k3 = D[4];
    for (int i=0,ii=0;i<height;++i) {
      for (int j=0;j<width;++j,++ii) {
	int i0 = i * scale;
	int j0 = j * scale;
	if (transposed)
	  std::swap(i0,j0);
	double _x = j0*B[0] + i0*B[1] + B[2];
	double _y = j0*B[3] + i0*B[4] + B[5];
	double _w = j0*B[6] + i0*B[7] + B[8];
	double w = 1./_w, x = _x*w, y = _y*w;
	double x2 = x*x, y2 = y*y;
	double r2 = x2 + y2, _2xy = 2*x*y;
	double kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2);
	double u = fx*(x*kr + p1*_2xy + p2*(r2 + 2*x2)) + u0;
	double v = fy*(y*kr + p1*(r2 + 2*y2) + p2*_2xy) + v0;
	rectifymap.map(j,i,0,float(u),float(v));

	edge_mask[ii] = u>max_edge_dist&&u<height-max_edge_dist&&v>max_edge_dist&&v<width-max_edge_dist;
      }
    }
  }

  void initUnrectifyMap(int width, int height, int scale,
			const double* D, const double* K,
			const double* H, const double* P,
			double* B, ImageRemap& unrectifymap,
			bool transposed) {
    double B0[] = {
      P[0] * H[0] + P[1] * H[3] + P[2] * H[6],
      P[0] * H[1] + P[1] * H[4] + P[2] * H[7],
      P[0] * H[2] + P[1] * H[5] + P[2] * H[8],
      P[4] * H[0] + P[5] * H[3] + P[6] * H[6],
      P[4] * H[1] + P[5] * H[4] + P[6] * H[7],
      P[4] * H[2] + P[5] * H[5] + P[6] * H[8],
      P[8] * H[0] + P[9] * H[3] + P[10] * H[6],
      P[8] * H[1] + P[9] * H[4] + P[10] * H[7],
      P[8] * H[2] + P[9] * H[5] + P[10] * H[8]
    };

    double k[] = {D[0],D[1],D[2],D[3],0,0,0,0,0,0,0,0};
    double k1 = D[0];
    double k2 = D[1];
    double p1 = D[2];
    double p2 = D[3];
    double k3 = D[4];
    for (int i=0;i<height;++i) {
      for (int j=0;j<width;++j) {
    	int i0 = i;
    	int j0 = j;

    	double x,y;
    	double x0 = x = (j0-K[2])/K[0];
    	double y0 = y = (i0-K[5])/K[4];

    	for (int jj=0;jj<10;++jj) {
	  double x2 = x*x, y2 = y*y;
	  double r2 = x2 + y2, _2xy = 2*x*y;
	  double kr = (1 + ((k3*r2 + k2)*r2 + k1)*r2);
	  double u = x*kr + p1*_2xy + p2*(r2 + 2*x2);
	  double v = y*kr + p1*(r2 + 2*y2) + p2*_2xy;

	  double du = u - x0;
	  double dv = v - y0;
	  x -= du;
	  y -= dv;
    	}

    	double xx = B0[0]*x + B0[1]*y + B0[2];
    	double yy = B0[3]*x + B0[4]*y + B0[5];
    	double ww = 1./(B0[6]*x + B0[7]*y + B0[8]);
    	x = xx*ww;
    	y = yy*ww;
	if (transposed)
	  std::swap(x,y);

	unrectifymap.map(j,i,0,float(x)/scale,float(y)/scale);
      }
    }
  }

  void undistortPoints(const double* D, const double* K0, const double* K1,
		      int N,const double* xin,double* xout) {
    for (int j=0;j<N;++j) {
      double k[] = {D[0],D[1],D[2],D[3],0,0,0,0,0,0,0,0};
      double fx = K0[0];
      double fy = K0[4];
      double ifx = 1./fx;
      double ify = 1./fy;
      double cx = K0[2];
      double cy = K0[5];
      double x = xin[j*2+0];
      double y = xin[j*2+1];
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
      xout[j*2+0] = x;
      xout[j*2+1] = y;
    }
  }  

  void initRectify(int width, int height,
		   const double* D0, const double* K0,
		   const double* D1, const double* K1,
		   const double* R, const double* T,
		   double* H0, double* H1,
		   double* P0, double* P1,
		   double* Q,
		   bool zero_disparity) {
    double r[3];
    packR(R,r);
    r[0]*=-0.5;
    r[1]*=-0.5;
    r[2]*=-0.5;
    double r_r[9];
    unpackR(r,r_r);
    double _t[] = {
      r_r[0] * T[0] + r_r[1] * T[1] + r_r[2] * T[2],
      r_r[3] * T[0] + r_r[4] * T[1] + r_r[5] * T[2],
      r_r[6] * T[0] + r_r[7] * T[1] + r_r[8] * T[2]
    };
    int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;
    double c = _t[idx];
    double nt = sqrt(_t[0]*_t[0]+_t[1]*_t[1]+_t[2]*_t[2]);
    double _uu[] = {0,0,0};
    _uu[idx] = c > 0 ? 1 : -1;

    double _ww[] = {
      _t[1] * _uu[2] - _t[2] * _uu[1],
      -_t[0] * _uu[2] + _t[2] * _uu[0],
      _t[0] * _uu[1] - _t[1] * _uu[0]
    };
    double nw = sqrt(_ww[0]*_ww[0]+_ww[1]*_ww[1]+_ww[2]*_ww[2]);
    if (nw > 0) {
      double t0 = acos(fabs(c)/nt)/nw;
      _ww[0]*=t0;
      _ww[1]*=t0;
      _ww[2]*=t0;
    }
    double wR[9];
    unpackR(_ww,wR);

    H0[0] = r_r[2] * wR[2] + r_r[1] * wR[1] + r_r[0] * wR[0];
    H0[1] = r_r[5] * wR[2] + r_r[4] * wR[1] + r_r[3] * wR[0];
    H0[2] = r_r[8] * wR[2] + r_r[7] * wR[1] + r_r[6] * wR[0];
    H0[3] = r_r[2] * wR[5] + r_r[1] * wR[4] + r_r[0] * wR[3];
    H0[4] = r_r[5] * wR[5] + r_r[4] * wR[4] + r_r[3] * wR[3];
    H0[5] = r_r[8] * wR[5] + r_r[7] * wR[4] + r_r[6] * wR[3];
    H0[6] = r_r[2] * wR[8] + r_r[1] * wR[7] + r_r[0] * wR[6];
    H0[7] = r_r[5] * wR[8] + r_r[4] * wR[7] + r_r[3] * wR[6];
    H0[8] = r_r[8] * wR[8] + r_r[7] * wR[7] + r_r[6] * wR[6];
    H1[0] = r_r[6] * wR[2] + r_r[3] * wR[1] + r_r[0] * wR[0];
    H1[1] = r_r[7] * wR[2] + r_r[4] * wR[1] + r_r[1] * wR[0];
    H1[2] = r_r[8] * wR[2] + r_r[5] * wR[1] + r_r[2] * wR[0];
    H1[3] = r_r[6] * wR[5] + r_r[3] * wR[4] + r_r[0] * wR[3];
    H1[4] = r_r[7] * wR[5] + r_r[4] * wR[4] + r_r[1] * wR[3];
    H1[5] = r_r[8] * wR[5] + r_r[5] * wR[4] + r_r[2] * wR[3];
    H1[6] = r_r[6] * wR[8] + r_r[3] * wR[7] + r_r[0] * wR[6];
    H1[7] = r_r[7] * wR[8] + r_r[4] * wR[7] + r_r[1] * wR[6];
    H1[8] = r_r[8] * wR[8] + r_r[5] * wR[7] + r_r[2] * wR[6];

    double fc_new = std::numeric_limits<double>::max();
    for (int k=0;k<2;++k) {
      const double* Dk = k?D1:D0;
      const double* Kk = k?K1:K0;
      double dk1 = Dk[0];
      double fc = idx?Kk[0]:Kk[4];
      if (dk1 < 0)
    	fc *= 1 + dk1*(width*width + height*height)/(4*fc*fc);
      fc_new = std::min(fc_new, fc);
    }

    double cc_new[4];
    for (int k = 0; k < 2; k++) {
      const double* Dk = k?D1:D0;
      const double* Kk = k?K1:K0;
      double pts[2*4];
      for (int i=0;i<4;i++) {
    	int j = (i<2) ? 0 : 1;
    	pts[i*2+0] = (double)((i % 2)*(width-1));
    	pts[i*2+1] = (double)(j*(height-1));
      }
      const double KI[] = {1,0,0,0,1,0,0,0,1};
      undistortPoints(Dk, Kk, KI, 4, pts, pts);
      const double* Hk = k?H1:H0;
      double avg[2] = {0,0};
      for (int i=0;i<4;++i) {
    	const double* x = pts+i*2;
    	double w = 1./(Hk[6] * x[0] + Hk[7] * x[1] + Hk[8]);
    	avg[0] += w * (fc_new * Hk[0] * x[0] + fc_new * Hk[1] * x[1] + fc_new * Hk[2]);
    	avg[1] += w * (fc_new * Hk[3] * x[0] + fc_new * Hk[4] * x[1] + fc_new * Hk[5]);
      }
      avg[0] *= .25;
      avg[1] *= .25;

      cc_new[k*2+0] = (width-1)/2 - avg[0];
      cc_new[k*2+1] = (height-1)/2 - avg[1];
    }

    if (zero_disparity) {
      cc_new[0] = cc_new[2] = (cc_new[0] + cc_new[2])*0.5;
      cc_new[1] = cc_new[3] = (cc_new[1] + cc_new[3])*0.5;
    } else if (idx == 0)
      cc_new[1] = cc_new[3] = (cc_new[1] + cc_new[3])*0.5;
    else
      cc_new[0] = cc_new[2] = (cc_new[0] + cc_new[2])*0.5;

    P0[0] = fc_new;
    P0[1] = 0;
    P0[2] = cc_new[0];
    P0[3] = 0;
    P0[4] = 0;
    P0[5] = fc_new;
    P0[6] = cc_new[1];
    P0[7] = 0;
    P0[8] = 0;
    P0[9] = 0;
    P0[10] = 1;
    P0[11] = 0;
    P1[0] = fc_new;
    P1[1] = 0;
    P1[2] = cc_new[2];
    P1[3] = 0;
    P1[4] = 0;
    P1[5] = fc_new;
    P1[6] = cc_new[3];
    P1[7] = 0;
    P1[8] = 0;
    P1[9] = 0;
    P1[10] = 1;
    P1[11] = 0;
    P1[idx?7:3] = _t[idx]*fc_new;

    Q[0] = 1;
    Q[1] = 0;
    Q[2] = 0;
    Q[3] = -cc_new[0];
    Q[4] = 0;
    Q[5] = 1;
    Q[6] = 0;
    Q[7] = -cc_new[1];
    Q[8] = 0;
    Q[9] = 0;
    Q[10] = 0;
    Q[11] = fc_new;
    Q[12] = 0;
    Q[13] = 0;
    Q[14] = -1./_t[idx];
    Q[15] = (idx == 0 ? cc_new[0] - cc_new[2] : cc_new[1] - cc_new[3])/_t[idx];
  }

  void init(SensorPair& p, int width, int height, int scale,
	    const double* D0, const double* D1,
	    const double* K0, const double* K1,
	    const double* R0, const double* R1,
	    const double* T0, const double* T1,
	    bool transposed) {
    p.width = width;
    p.height = height;
    std::copy(D0,D0+5,p.D0);
    std::copy(D1,D1+5,p.D1);
    std::copy(K0,K0+9,p.K0);
    std::copy(K1,K1+9,p.K1);
    std::copy(R0,R0+9,p.R0);
    std::copy(R1,R1+9,p.R1);
    std::copy(T0,T0+3,p.T0);
    std::copy(T1,T1+3,p.T1);

    int map_width = width / scale;
    int map_height = height / scale;
    if (transposed)
      std::swap(map_width,map_height);

    double A1[] = {
      R0[0],R0[1],R0[2],T0[0],
      R0[3],R0[4],R0[5],T0[1],
      R0[6],R0[7],R0[8],T0[2],
      0,0,0,1
    };
    double A2[] = {
      R1[0],R1[1],R1[2],T1[0],
      R1[3],R1[4],R1[5],T1[1],
      R1[6],R1[7],R1[8],T1[2],
      0,0,0,1
    };

    SVD svd;
    double* W;
    double* Ut;
    double* Vt;
    svd(4,4,A1,&W,&Ut,&Vt);
    double A3[] = {
      Vt[0] / W[0] * Ut[0] + Vt[4] / W[1] * Ut[4] + Vt[8] / W[2] * Ut[8] + Vt[12] / W[3] * Ut[12],
      Vt[0] / W[0] * Ut[1] + Vt[4] / W[1] * Ut[5] + Vt[8] / W[2] * Ut[9] + Vt[12] / W[3] * Ut[13],
      Vt[0] / W[0] * Ut[2] + Vt[4] / W[1] * Ut[6] + Vt[8] / W[2] * Ut[10] + Vt[12] / W[3] * Ut[14],
      Vt[0] / W[0] * Ut[3] + Vt[4] / W[1] * Ut[7] + Vt[8] / W[2] * Ut[11] + Vt[12] / W[3] * Ut[15],
      Vt[1] / W[0] * Ut[0] + Vt[5] / W[1] * Ut[4] + Vt[9] / W[2] * Ut[8] + Vt[13] / W[3] * Ut[12],
      Vt[1] / W[0] * Ut[1] + Vt[5] / W[1] * Ut[5] + Vt[9] / W[2] * Ut[9] + Vt[13] / W[3] * Ut[13],
      Vt[1] / W[0] * Ut[2] + Vt[5] / W[1] * Ut[6] + Vt[9] / W[2] * Ut[10] + Vt[13] / W[3] * Ut[14],
      Vt[1] / W[0] * Ut[3] + Vt[5] / W[1] * Ut[7] + Vt[9] / W[2] * Ut[11] + Vt[13] / W[3] * Ut[15],
      Vt[2] / W[0] * Ut[0] + Vt[6] / W[1] * Ut[4] + Vt[10] / W[2] * Ut[8] + Vt[14] / W[3] * Ut[12],
      Vt[2] / W[0] * Ut[1] + Vt[6] / W[1] * Ut[5] + Vt[10] / W[2] * Ut[9] + Vt[14] / W[3] * Ut[13],
      Vt[2] / W[0] * Ut[2] + Vt[6] / W[1] * Ut[6] + Vt[10] / W[2] * Ut[10] + Vt[14] / W[3] * Ut[14],
      Vt[2] / W[0] * Ut[3] + Vt[6] / W[1] * Ut[7] + Vt[10] / W[2] * Ut[11] + Vt[14] / W[3] * Ut[15],
      Vt[3] / W[0] * Ut[0] + Vt[7] / W[1] * Ut[4] + Vt[11] / W[2] * Ut[8] + Vt[15] / W[3] * Ut[12],
      Vt[3] / W[0] * Ut[1] + Vt[7] / W[1] * Ut[5] + Vt[11] / W[2] * Ut[9] + Vt[15] / W[3] * Ut[13],
      Vt[3] / W[0] * Ut[2] + Vt[7] / W[1] * Ut[6] + Vt[11] / W[2] * Ut[10] + Vt[15] / W[3] * Ut[14],
      Vt[3] / W[0] * Ut[3] + Vt[7] / W[1] * Ut[7] + Vt[11] / W[2] * Ut[11] + Vt[15] / W[3] * Ut[15]
    };
    std::copy(A3,A3+16,p.C);

    double A4[] = {
      A3[0] * A2[0] + A3[1] * A2[4] + A3[2] * A2[8] + A3[3] * A2[12],
      A3[0] * A2[1] + A3[1] * A2[5] + A3[2] * A2[9] + A3[3] * A2[13],
      A3[0] * A2[2] + A3[1] * A2[6] + A3[2] * A2[10] + A3[3] * A2[14],
      A3[0] * A2[3] + A3[1] * A2[7] + A3[2] * A2[11] + A3[3] * A2[15],
      A3[4] * A2[0] + A3[5] * A2[4] + A3[6] * A2[8] + A3[7] * A2[12],
      A3[4] * A2[1] + A3[5] * A2[5] + A3[6] * A2[9] + A3[7] * A2[13],
      A3[4] * A2[2] + A3[5] * A2[6] + A3[6] * A2[10] + A3[7] * A2[14],
      A3[4] * A2[3] + A3[5] * A2[7] + A3[6] * A2[11] + A3[7] * A2[15],
      A3[8] * A2[0] + A3[9] * A2[4] + A3[10] * A2[8] + A3[11] * A2[12],
      A3[8] * A2[1] + A3[9] * A2[5] + A3[10] * A2[9] + A3[11] * A2[13],
      A3[8] * A2[2] + A3[9] * A2[6] + A3[10] * A2[10] + A3[11] * A2[14],
      A3[8] * A2[3] + A3[9] * A2[7] + A3[10] * A2[11] + A3[11] * A2[15],
      A3[12] * A2[0] + A3[13] * A2[4] + A3[14] * A2[8] + A3[15] * A2[12],
      A3[12] * A2[1] + A3[13] * A2[5] + A3[14] * A2[9] + A3[15] * A2[13],
      A3[12] * A2[2] + A3[13] * A2[6] + A3[14] * A2[10] + A3[15] * A2[14],
      A3[12] * A2[3] + A3[13] * A2[7] + A3[14] * A2[11] + A3[15] * A2[15]
    };
    double R[] = {
      A4[0],A4[1],A4[2],
      A4[4],A4[5],A4[6],
      A4[8],A4[9],A4[10],
    };
    double T[] = {
      A4[3],A4[7],A4[11]
    };

    double H0[9];
    double H1[9];
    double P0[12];
    double P1[12];

    p.rectifymap0 = std::make_shared<ImageRemap>(map_width,map_height);
    p.rectifymap1 = std::make_shared<ImageRemap>(map_width,map_height);
    p.unrectifymap0 = std::make_shared<ImageRemap>(width,height);
    p.unrectifymap1 = std::make_shared<ImageRemap>(width,height);

    p.rectifymap0->addImage(width,height);
    p.rectifymap1->addImage(width,height);
    p.unrectifymap0->addImage(map_width,map_height);
    p.unrectifymap1->addImage(map_width,map_height);

    initRectify(width, height, D0, K0, D1, K1, R, T, H0, H1, P0, P1, p.Q, true);
    initRectifyMap(map_width, map_height, scale, D0, K0, H0, P0, p.B0, *p.rectifymap0, p.edge_mask0, transposed);
    initRectifyMap(map_width, map_height, scale, D1, K1, H1, P1, p.B1, *p.rectifymap1, p.edge_mask1, transposed);
    initUnrectifyMap(width, height, scale, D0, K0, H0, P0, p.B0, *p.unrectifymap0, transposed);
    initUnrectifyMap(width, height, scale, D1, K1, H1, P1, p.B1, *p.unrectifymap1, transposed);

    std::copy(H0,H0+9,p.H0);
  }

public:
  OccamStereoRectifyImpl()
    :   scale(1) {
    using namespace std::placeholders;
    registerParami(OCCAM_RECTIFY_SCALE,"rectify_scale",OCCAM_SETTINGS,1,4,
		   std::bind(&OccamStereoRectifyImpl::get_scale,this),
		   std::bind(&OccamStereoRectifyImpl::set_scale,this,_1));
    std::vector<std::pair<std::string,int> > scale_values;
    scale_values.push_back(std::make_pair("1",1));
    scale_values.push_back(std::make_pair("2",2));
    scale_values.push_back(std::make_pair("4",4));
    setAllowedValues(OCCAM_RECTIFY_SCALE,scale_values);
    setDefaultValuei(OCCAM_RECTIFY_SCALE,1);
  }

  virtual int configure(int N,int width,int height,
			const double* const* D,const double* const* K,
			const double* const* R,const double* const* T,
			int transposed) {
    if ((N&1)||N<2)
      return OCCAM_API_INVALID_PARAMETER;

    {
      std::shared_ptr<Rep> rep0;
      {
	std::unique_lock<std::mutex> g(lock);
	rep0 = rep;
      }
      std::vector<SensorPair>& pairs = rep0->pairs;
      bool changed = false;
      if (!bool(rep0) ||
	  N/2 != pairs.size() ||
	  rep0->scale != scale)
	changed = true;
      else
	for (int j=0,k=0;j<pairs.size();++j,k+=2) {
	  if (!std::equal(pairs[j].D0,pairs[j].D0+5,D[k+0]) ||
	      !std::equal(pairs[j].D1,pairs[j].D1+5,D[k+1]) ||
	      !std::equal(pairs[j].K0,pairs[j].K0+9,K[k+0]) ||
	      !std::equal(pairs[j].K1,pairs[j].K1+9,K[k+1]) ||
	      !std::equal(pairs[j].R0,pairs[j].R0+9,R[k+0]) ||
	      !std::equal(pairs[j].R1,pairs[j].R1+9,R[k+1]) ||
	      !std::equal(pairs[j].T0,pairs[j].T0+3,T[k+0]) ||
	      !std::equal(pairs[j].T1,pairs[j].T1+3,T[k+1])) {
	    changed = true;
	    break;
	  }
	}
      if (!changed)
	return OCCAM_API_SUCCESS;
    }

    int N2 = N/2;
    auto rep0 = std::make_shared<Rep>();
    rep0->transposed = !!transposed;
    rep0->scale = scale;
    std::vector<SensorPair>& pairs = rep0->pairs;
    pairs.reserve(N2);
    std::vector<std::thread> init_threads;
    for (int j=0,k=0;j<N2;++j,k+=2) {
      SensorPair* p = &*pairs.emplace(pairs.end(),SensorPair());
            init_threads.push_back(std::thread([=](){
		  init(*p,width,height,rep0->scale,D[k+0],D[k+1],K[k+0],K[k+1],R[k+0],R[k+1],T[k+0],T[k+1],!!transposed);
		}));
    }
    for (std::thread& th : init_threads)
      th.join();

    {
      std::unique_lock<std::mutex> g(lock);
      rep = rep0;
    }

    return OCCAM_API_SUCCESS;
  }

  virtual int rectify(int index,const OccamImage* img0,OccamImage** img1) {
    if (index<0)
      return OCCAM_API_INVALID_PARAMETER;
    std::shared_ptr<Rep> rep0;
    {
      std::unique_lock<std::mutex> g(lock);
      rep0 = rep;
    }
    if (!bool(rep0))
      return OCCAM_API_NOT_INITIALIZED;
    int index0 = index>>1;
    int index1 = index&1;
    if (index0>=rep0->pairs.size())
      return OCCAM_API_INVALID_PARAMETER;
    SensorPair& p = rep0->pairs[index0];
    ImageRemap& rectifymap = index1 ? *p.rectifymap1 : *p.rectifymap0;
    return rectifymap(img0,img1);
  }

  virtual int unrectify(int index,const OccamImage* img0,OccamImage** img1) {
    if (index<0)
      return OCCAM_API_INVALID_PARAMETER;
    std::shared_ptr<Rep> rep0;
    {
      std::unique_lock<std::mutex> g(lock);
      rep0 = rep;
    }
    if (!bool(rep0))
      return OCCAM_API_NOT_INITIALIZED;
    int index0 = index>>1;
    int index1 = index&1;
    if (index0>=rep0->pairs.size())
      return OCCAM_API_INVALID_PARAMETER;
    SensorPair& p = rep0->pairs[index0];
    ImageRemap& unrectifymap = index1 ? *p.unrectifymap1 : *p.unrectifymap0;
    return unrectifymap(img0,img1);
  }

  virtual int generateCloud(int N,const int* indices,int transform,
			    const OccamImage* const* img0,const OccamImage* const* disp0,
			    OccamPointCloud** cloud1out) {
    if (N<=0)
      return OCCAM_API_INVALID_PARAMETER;
    std::shared_ptr<Rep> rep0;
    {
      std::unique_lock<std::mutex> g(lock);
      rep0 = rep;
    }
    if (!bool(rep0))
      return OCCAM_API_NOT_INITIALIZED;
    int max_points = 0;
    for (int j=0;j<N;++j) {
      if (disp0[j]->format != OCCAM_SHORT1)
	return OCCAM_API_INVALID_FORMAT;
      int index = indices[j];
      int index0 = index>>1;
      int index1 = index&1;
      if (index0>=rep0->pairs.size())
	return OCCAM_API_INVALID_PARAMETER;
      max_points += disp0[j]->width * disp0[j]->height;
    }

    OccamPointCloud* cloud1 = (OccamPointCloud*)occamAlloc(sizeof(OccamPointCloud));
    *cloud1out = cloud1;
    memset(cloud1,0,sizeof(OccamPointCloud));
    cloud1->cid = strdup(img0[0]->cid);
    memcpy(cloud1->timescale,img0[0]->timescale,sizeof(cloud1->timescale));
    cloud1->time_ns = img0[0]->time_ns;
    cloud1->index = img0[0]->index;
    cloud1->refcnt = 1;
    cloud1->xyz = (float*)occamAlloc(sizeof(float)*3*max_points);
    cloud1->rgb = (uint8_t*)occamAlloc(sizeof(uint8_t)*3*max_points);
    cloud1->point_count = 0;

    float* xyzp = cloud1->xyz;
    uint8_t* rgbp = cloud1->rgb;
    int scale = rep0->scale;
    bool transposed = rep0->transposed;

    for (int j=0;j<N;++j) {
      int index = indices[j];
      int index0 = index>>1;
      int index1 = index&1;
      SensorPair& p = rep0->pairs[index0];
      const double* Q = p.Q;
      const double* C = p.C;
      const double* H0 = p.H0;

      uint8_t* img0p0 = img0[j]->data[0];
      int img0_step = img0[j]->step[0];
      int bpp = 0;
      if (img0[j]->format == OCCAM_GRAY8)
	bpp = 1;
      else if (img0[j]->format == OCCAM_RGB24)
	bpp = 3;

      const uint8_t* srcp0 = (const uint8_t*)disp0[j]->data[0];
      int src_step = disp0[j]->step[0];
      for (int y=0,ii=0;y<disp0[j]->height;++y,srcp0+=src_step,img0p0+=img0_step) {
    	int16_t* srcp = (int16_t*)srcp0;
	uint8_t* img0p = img0p0;

    	for (int x=0,x3=0;x<disp0[j]->width;++x,img0p+=bpp,++ii) {

	  if (!p.edge_mask0[ii])
	    continue;

	  int x0 = x*scale;
	  int y0 = y*scale;
	  if (transposed)
	    std::swap(x0,y0);
	  double qx = Q[0]*x0 + Q[1]*y0 + Q[3];
	  double qy = Q[4]*x0 + Q[5]*y0 + Q[7];
	  double qz = Q[8]*x0 + Q[9]*y0 + Q[11];
	  double qw = Q[12]*x0 + Q[13]*y0 + Q[15];

    	  int16_t d = srcp[x]*scale;
    	  if (d<0)
    	    continue;
    	  double w = 1./(qw + Q[14]*d);
	  float x1 = float((qx + Q[2]*d)*w);
	  float y1 = float((qy + Q[6]*d)*w);
	  float z1 = float((qz + Q[10]*d)*w);

	  if (!std::isfinite(z1) || z1 > 500 || z1 < 0)
	    continue;

	  float scalemm = 10.f;
	  float x1a,x2;
	  float y1a,y2;
	  float z1a,z2;
	  if (transform) {
	    x1a = float(H0[0]*x1+H0[1]*y1+H0[2]*z1) * scalemm;
	    y1a = float(H0[3]*x1+H0[4]*y1+H0[5]*z1) * scalemm;
	    z1a = float(H0[6]*x1+H0[7]*y1+H0[8]*z1) * scalemm;
	    x2 = float(C[0]*x1a+C[1]*y1a+C[2]*z1a+C[3]);
	    y2 = float(C[4]*x1a+C[5]*y1a+C[6]*z1a+C[7]);
	    z2 = float(C[8]*x1a+C[9]*y1a+C[10]*z1a+C[11]);
	  } else {
	    x2 = x1 * scalemm;
	    y2 = y1 * scalemm;
	    z2 = z1 * scalemm;
	  }

    	  xyzp[0] = x2;
    	  xyzp[1] = y2;
    	  xyzp[2] = z2;
    	  xyzp+=3;

	  if (img0[j]->format == OCCAM_GRAY8) {
	    rgbp[0] = img0p[0];
	    rgbp[1] = img0p[0];
	    rgbp[2] = img0p[0];
	    rgbp+=3;
	  }
	  else if (img0[j]->format == OCCAM_RGB24) {
	    rgbp[0] = img0p[0];
	    rgbp[1] = img0p[1];
	    rgbp[2] = img0p[2];
	    rgbp+=3;
	  }

    	  ++cloud1->point_count;
    	}
      }
    }

    return OCCAM_API_SUCCESS;
  }
};


static OccamModuleFactory<OccamStereoRectifyImpl> __module_factory
("prec","Planar",OCCAM_MODULE_STEREO_RECTIFY,0,0);
void init_planar_rectify() {
  __module_factory.registerModule();
}
