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
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>

static void reportError(int error_code) {
  char str[1024] = {0};
  occamGetErrorString((OccamError)error_code, str, sizeof(str));
  std::cerr<<"Occam API Error: "<<str<<" ("<<error_code<<")"<<std::endl;
  abort();
}

class ImageFiles : public std::map<int, std::string> {
public:
  void enumerateFiles(int sensor_index) {
    WIN32_FIND_DATA search_data;
    memset(&search_data, 0, sizeof(WIN32_FIND_DATA));
    std::stringstream query;
    query<<"images-"<<sensor_index<<"\\*";
    HANDLE handle = FindFirstFile(query.str().c_str(), &search_data);
    while (handle != INVALID_HANDLE_VALUE) {
      //      std::cout<<"\n"<<search_data.cFileName;
      //      files.insert(search_data.cFileName);

      if (strcmp(search_data.cFileName,".") &&
	  strcmp(search_data.cFileName,"..")) {

      int image_index = atoi(search_data.cFileName);

      std::stringstream path;
      path<<"images-"<<sensor_index<<"\\"<<search_data.cFileName;

      insert(std::make_pair(image_index,path.str()));
      }

      if(FindNextFile(handle, &search_data) == FALSE)
	break;
    }
    FindClose(handle);
  }
};

int main(int argc, char* argv[]) {
  std::vector<ImageFiles> sensors;
  int sensor_count = 5;
  sensors.resize(sensor_count);
  for (int j=0;j<sensor_count;++j)
    sensors[j].enumerateFiles(j);

  //  cv::VideoWriter vid("output.avi", cv::VideoWriter::fourcc('P','I','M','1'), 60, cv::Size(752,480), true);
  //  cv::VideoWriter vid("output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 60, cv::Size(752,480), true);
  cv::VideoWriter vid("output.avi", cv::VideoWriter::fourcc('X','V','I','D'), 30, cv::Size(752*sensor_count,480), true);
  if (!vid.isOpened()) {
    std::cerr<<"failed to open output.avi"<<std::endl;
    return 1;
  }

  int frame_count = 0;

  for (auto&& v : sensors[0]) {
    int image_index = v.first;

    bool dont_have_all_frames = false;
    for (int j=1;j<sensor_count;++j)
      if (sensors[j].find(image_index) == sensors[j].end()) {
	dont_have_all_frames = true;
	break;
      }
    if (dont_have_all_frames)
      continue;

    std::cerr<<"gen "<<image_index<<", "<<v.second<<std::endl;

    cv::Mat img(480,sensor_count*752,CV_8UC3);

    for (int j=0;j<sensor_count;++j) {
      const std::string& fn = sensors[j][image_index];
      cv::Mat imgj = cv::imread(fn);

      cv::Mat dst_img(img, cv::Rect(j*752,0,752,480));
      imgj.copyTo(dst_img);
    }

    vid.write(img);
    imshow("indigosdk", img);

    if (++frame_count>=200)
      break;
      
    int key = cv::waitKey(1);
    if (key == 27 || key == 'Q' || key == 'q') // ESC/Q/q keys to quit
      break;

  }

  vid.release();
   
   return 0;
}
