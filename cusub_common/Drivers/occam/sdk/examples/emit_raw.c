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

// ./emit_raw | ~/ffmpeg/inst/bin/ffmpeg -f rawvideo -pixel_format gray -video_size 3760x480 -framerate 1 -i - -me_method epzs -qmin 3 -r 1 -flags +global_header  http://192.168.1.130:8090/feed1.ffm

#include "indigo.h"
#include <stdio.h>
#include <stdlib.h>

static int dev_index = 0;

static void reportError(int error_code) {
  fprintf(stderr,"Occam API Error: %i\n",error_code);
  abort();
}

static void enumerateDevices() {
  OccamDeviceList* device_list;
  int i;
  int r;
  if ((r = occamEnumerateDeviceList(2000, &device_list)) != OCCAM_API_SUCCESS)
    reportError(r);
  printf("%i devices found\n", device_list->entry_count);
  for (i=0;i<device_list->entry_count;++i) {
    printf("device[%i]: cid = %s\n",
	   i,device_list->entries[i].cid);
  }
  occamFreeDeviceList(device_list);
}

static void printImageInfo() {
  OccamDeviceList* device_list;
  OccamDevice* device;
  OccamImage* image;
  int r;
  int i;
  if ((r = occamEnumerateDeviceList(2000, &device_list)) != OCCAM_API_SUCCESS)
    reportError(r);
  printf("%i devices found\n", device_list->entry_count);
  for (i=0;i<device_list->entry_count;++i) {
    printf("device[%i]: cid = %s\n",
	   i,device_list->entries[i].cid);
  }
  if ((r = occamOpenDevice(device_list->entries[dev_index].cid, &device)) != OCCAM_API_SUCCESS)
    reportError(r);

  OccamDataName req[] = {OCCAM_IMAGE_TILES0};
  if ((r = occamDeviceReadData(device, 1, req, 0, (void**)&image, 1)) != OCCAM_API_SUCCESS)
    reportError(r);

  const char* format_str = "??";
  switch (image->format) {
  case OCCAM_GRAY8: format_str = "gray8"; break;
  case OCCAM_RGB24: format_str = "rgb24"; break;
  }
  printf("image info: format = %s, width = %i, height = %i\n",
	 format_str, image->width, image->height);
  occamFreeImage(image);

  occamCloseDevice(device);
  occamFreeDeviceList(device_list);
}

int main(int argc, const char** argv) {
  int r;
  int i;
  OccamDeviceList* device_list;
  OccamDevice* device;
  OccamImage* image;

  if ((r = occamInitialize()) != OCCAM_API_SUCCESS)
    reportError(r);

  for (i=1;i<argc;++i) {
    if (!strcmp(argv[i],"--list")) {
      enumerateDevices();
      return 0;
    } else if (!strcmp(argv[i],"--stat")) {
      printImageInfo();
      return 0;
    } else if ((!strcmp(argv[i],"--device") || !strcmp(argv[i],"--d")) &&
	       i+1<argc) {
      dev_index = atoi(argv[++i]);
    }
  }

  if ((r = occamEnumerateDeviceList(2000, &device_list)) != OCCAM_API_SUCCESS)
    reportError(r);
  if (dev_index<0 || dev_index >= device_list->entry_count) {
    fprintf(stderr,"device index %i out of range\n",dev_index);
    return 1;
  }

  if (isatty(1)) {
    fprintf(stderr,"The output of this program cannot be a terminal. Pipe it to "
	    "another program (e.g., ffmpeg with rawvideo options, see --stat option for information)\n");
    return 1;
  }

  if ((r = occamOpenDevice(device_list->entries[dev_index].cid, &device)) != OCCAM_API_SUCCESS)
    reportError(r);

  int drop_count = 0;
  while (1) {
    OccamDataName req[] = {OCCAM_IMAGE_TILES0};
    if ((r = occamDeviceReadData(device, 1, req, 0, (void**)&image, 1)) != OCCAM_API_SUCCESS)
      reportError(r);
    if (drop_count++>0)
      continue;
    drop_count = 0;

    int offset = 0;
    int size = image->step[0]*image->height;
    while (offset < size) {
      int wrlen = write(1, image->data+offset, size-offset);
      if (wrlen < 0) {
	fprintf(stderr,"write to stdout failed\n");
	return 2;
      }
      offset += wrlen;
            fprintf(stderr,"wrlen %i, offset %i\n",wrlen,offset);
    }

    occamFreeImage(image);
  }

  occamCloseDevice(device);
  occamFreeDeviceList(device_list);
  occamShutdown();

  return 0;
}
