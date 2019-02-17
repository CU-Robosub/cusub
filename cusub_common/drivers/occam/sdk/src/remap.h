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

#include "indigo.h"
#include <stdint.h>
#include <vector>

class ImageRemap {
  int map_width;
  int map_height;
  struct Image {
    int width;
    int height;
  };
  struct Segment {
    short dst_x;
    short dst_y;
    short src_indices[2];
    short length;
    bool inlier;
  };
  std::vector<Image> images;
  std::vector<Segment> segments;
  std::vector<float> xy;
  std::vector<short> ixy;
  std::vector<unsigned short> fxy;
  std::vector<float> fade;
  std::vector<short> ifade;
public:
  ImageRemap(int map_width,int map_height);
  int mapWidth() const;
  int mapHeight() const;

  int addImage(int width, int height);
  void map(int dst_x, int dst_y,
	   int src_index, float src_x, float src_y);
  void map(int dst_x, int dst_y,
	   int src_index0, float src_x0, float src_y0,
	   int src_index1, float src_x1, float src_y1,
	   float fade);

  int operator() (OccamImageFormat format,
		  const uint8_t* const* srcp,const int* src_step,
		  uint8_t* dstp,int dst_step);
  int operator() (const OccamImage* const* img0, OccamImage** img1);
  int operator() (const OccamImage* img0, OccamImage** img1);
};

// Local Variables:
// mode: c++
// End:

