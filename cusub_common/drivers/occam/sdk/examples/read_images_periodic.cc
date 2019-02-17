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

// This example shows how we can capture only subset of frames in response
// to a software based trigger. In particular, it does the CPU intensive
// host-side processing (color processing, stitching) only on those frames
// we are interested in keeping.

// E.g., capture a frame when external odometry device says we've moved N meters.

#include "indigo.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <list>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#ifdef _WIN32
#include <windows.h>
#else // _WIN32
#include <unistd.h>
#endif // _WIN32

static void reportError(int error_code) {
  char str[1024] = {0};
  occamGetErrorString((OccamError)error_code, str, sizeof(str));
  std::cerr<<"Occam API Error: "<<str<<" ("<<error_code<<")"<<std::endl;
  abort();
}

typedef std::shared_ptr<OccamImage> OccamImagePtr;

// PeriodicFrameGrabber generates two threads: a device reader thread that
// reads all raw images from a device as quickly as possible,
// and a post processing thread that performs debayer, image processing,
// and stitching only on a smaller set of those raw images. The user indicates which
// frames to keep and process by calling captureNextFrame which tells the class to
// take the next raw frame, perform all the post processing, and then make it
// available to the pop function.
class PeriodicFrameGrabber {
  OccamDevice* device;
  bool shutdown;

  std::shared_ptr<void> debayerf_handle;
  std::shared_ptr<void> imagef_handle;
  std::shared_ptr<void> blend_handle;
  int is_color;

  std::thread device_reader_thread;
  std::thread post_process_thread;
  std::mutex lock;
  std::condition_variable pending_cond;
  std::list<OccamImagePtr> pending;
  std::list<OccamImagePtr> reaping;
  bool capture_next;

  // Read all images raw from the device as fast as we can.
  void readerThread() {
    for (;;) {
      // Check if we're supposed to shut down
      {
	std::unique_lock<std::mutex> g(lock);
	if (shutdown)
	  break;
      }

      // Read next raw frame from camera
      OccamImage* image0;
      OccamDataName req[] = {OCCAM_RAW_IMAGE_TILES0};
      int r = occamDeviceReadData(device,1,req,0,(void**)&image0, 0);
      if (r == OCCAM_API_DATA_NOT_AVAILABLE ||
	  r == OCCAM_API_UNSUPPORTED_DATA) {
#ifdef _WIN32
	Sleep(1);
#else // _WIN32
	usleep(1);
#endif // _WIN32
	continue;
      } else if (r != OCCAM_API_SUCCESS)
	reportError(r);

      // if we're supposed to capture the frame, give it to the post process thread
      auto image = OccamImagePtr(image0,occamFreeImage);
      {
	std::unique_lock<std::mutex> g(lock);
	if (!capture_next)
	  continue;

	capture_next = false;
	pending.push_back(image);
	pending_cond.notify_one();
      }
    }
  }

  // Return a sub-image of the given image.
  OccamImagePtr subImage(const OccamImagePtr& img0,int x,int y,int width,int height) {
    OccamImage* img1 = 0;
    occamSubImage(img0.get(), &img1, x, y, width, height);
    return OccamImagePtr(img1,occamFreeImage);
  };

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

  // Process images posted to the pending list, and put the resulting images
  // onto the reaping list.
  void postProcessThread() {
    for (;;) {
      // Check if we're supposed to shut down. If not, pick up the next frame
      // we're supposed to work on. If there are no frames pending, sleep until
      // one appears.
      std::shared_ptr<OccamImage> img0;
      {
	std::unique_lock<std::mutex> g(lock);
	if (shutdown)
	  break;
	if (pending.empty()) {
	  pending_cond.wait(g);
	  continue;
	}
	img0 = *pending.begin();
	pending.pop_front();
      }

      // Post process the frame: for each sensor, perform debayer (if color
      // camera) and image processing, and then feed all the images to the blender
      std::vector<OccamImagePtr> processed_subimages;
      processed_subimages.reserve(img0->subimage_count);
      for (int j=0;j<img0->subimage_count;++j) {
	OccamImagePtr imgj = subImage
	  (img0,img0->si_x[j],img0->si_y[j],img0->si_width[j],img0->si_height[j]);
	imgj = processImage(imgj);
	processed_subimages.push_back(imgj);
      }
      OccamImagePtr stitched_image = blendImages(processed_subimages);

      // Push the resulting stitched image to the reaping list
      {
	std::unique_lock<std::mutex> g(lock);
	reaping.push_back(stitched_image);
      }
    }
  }
public:
  // Query the device for its debayer/image processing/blender modules,
  // then start the device reader and post processing threads.
  PeriodicFrameGrabber(OccamDevice* _device)
    : device(_device),
      shutdown(false),
      capture_next(false) {

    // Query the device for debayer, image processing, and blender modules.
    // These are configured by the device itself when you read read images
    // from it.
    void* debayerf_handle0 = 0;
    void* imagef_handle0 = 0;
    void* blend_handle0 = 0;
    int r = occamGetDeviceValuep(device, OCCAM_DEBAYER_FILTER0, &debayerf_handle0);
    if (r != OCCAM_API_SUCCESS)
      reportError(r);
    r = occamGetDeviceValuep(device, OCCAM_IMAGE_FILTER0, &imagef_handle0);
    if (r != OCCAM_API_SUCCESS)
      reportError(r);
    r = occamGetDeviceValuep(device, OCCAM_BLENDER0, &blend_handle0);
    if (r != OCCAM_API_SUCCESS)
      reportError(r);
    debayerf_handle = std::shared_ptr<void>(debayerf_handle0,occamReleaseModule);
    imagef_handle = std::shared_ptr<void>(imagef_handle0,occamReleaseModule);
    blend_handle = std::shared_ptr<void>(blend_handle0,occamReleaseModule);

    // Is this a color device? I.e., should we perform debayer step?
    r = occamGetDeviceValuei(device, OCCAM_COLOR, &is_color);
    if (r != OCCAM_API_SUCCESS)
      reportError(r);

    // start the device reader and image processing threads
    device_reader_thread = std::thread([this](){this->readerThread();});
    post_process_thread = std::thread([this](){this->postProcessThread();});
  }

  // Shut down both threads
  ~PeriodicFrameGrabber() {
    {
      std::unique_lock<std::mutex> g(lock);
      shutdown = true;
    }
    pending_cond.notify_all();
    device_reader_thread.join();
    post_process_thread.join();
  }
  PeriodicFrameGrabber(const PeriodicFrameGrabber& x) = delete;
  PeriodicFrameGrabber& operator= (const PeriodicFrameGrabber& rhs) = delete;

  // Tell the reader to capture and process the next frame from the camera.
  void captureNextFrame() {
    std::unique_lock<std::mutex> g(lock);
    capture_next = true;
  }

  // Take the oldest captured and processed frame.
  bool pop(OccamImagePtr& stitched_image) {
    std::unique_lock<std::mutex> g(lock);
    if (reaping.empty())
      return false;
    stitched_image = *reaping.begin();
    reaping.pop_front();
    return true;
  }
};

int main(int argc, const char** argv) {
  int r;
  int i;
  int dev_index = argc>=2?atoi(argv[1]):0;
  OccamDeviceList* device_list;
  OccamDevice* device;

  // Initialize the SDK
  if ((r = occamInitialize()) != OCCAM_API_SUCCESS)
    reportError(r);

  // Enumerate available devices
  if ((r = occamEnumerateDeviceList(2000, &device_list)) != OCCAM_API_SUCCESS)
    reportError(r);
  std::cerr<<device_list->entry_count<<" devices found"<<std::endl;
  for (i=0;i<device_list->entry_count;++i) {
    std::cerr<<"device["<<i<<"]: cid = "<<device_list->entries[i].cid<<std::endl;
  }
  if (dev_index<0 || dev_index >= device_list->entry_count) {
    std::cerr<<"device index "<<dev_index<<" out of range"<<std::endl;
    return 1;
  }

  // Open the selected (or first) device
  if ((r = occamOpenDevice(device_list->entries[dev_index].cid, &device)) != OCCAM_API_SUCCESS)
    reportError(r);

  // Grab a frame whenever the user presses a key, and display it.
  // Only the frames captured in response to key press will be post
  // processed (have debayer, image processing, and blending/stitching
  // performed on them).
  PeriodicFrameGrabber grabber(device);
  grabber.captureNextFrame();
  bool downsample = true;
  for (;;) {
    // handle key strokes, determine when to capture image
    int key = cv::waitKey(1);
    if (key == 27 || key == 'Q' || key == 'q') // ESC/Q/q keys to quit
      break;
    else if (key == 'g')
      grabber.captureNextFrame();
    else if (key == 'w')
      downsample = !downsample;

    // pop captured and fully processed images from the reader
    OccamImagePtr img0;
    if (!grabber.pop(img0))
      continue;

    // convert resulting image to opencv
    cv::Mat img1;
    if (img0->format == OCCAM_GRAY8)
      img1 = cv::Mat_<uchar>(img0->height,img0->width,(uchar*)img0->data[0],img0->step[0]);
    else if (img0->format == OCCAM_RGB24) {
      img1 = cv::Mat_<cv::Vec3b>(img0->height,img0->width,(cv::Vec3b*)img0->data[0],img0->step[0]);
      cv::Mat img11;
      cv::cvtColor(img1, img11, cv::COLOR_BGR2RGB);
      img1 = img11;
    } else if (img0->format == OCCAM_SHORT1) {
      img1 = cv::Mat_<short>(img0->height,img0->width,(short*)img0->data[0],img0->step[0]);
    } else {
      //      printf("img0 format not supported by this demo\n");
    }

    // optionally downsample the image
    if (downsample && img1.cols>1024) {
      int small_width = 1024;
      int small_height = img0->height * small_width / img0->width;
      cv::Mat small_img;
      cv::resize(img1, small_img, cv::Size(small_width,small_height));
      img1 = small_img;
    }

    // display the image
    if (!img1.empty())
      imshow("indigosdk", img1);
  }

  return 1;
}
