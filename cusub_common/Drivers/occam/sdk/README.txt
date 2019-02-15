
==========================================================================================
== Using the SDK

The entire API to the SDK is contained in include/indigo.h. That file contains usage documentation
for each of the structures and functions in the API. The examples directory contains a number of
examples demonstrating use of the API. The online knowledge base at

http://occamvisiongroup.com/knowledge-base/

contains further information about installation and usage. There is also a separate Windows-only
package called the Occam Indigo Tools (free download from the web site) that is UI based.
You can use that software to interactively inspect the camera outputs, change camera settings,
and recalibrate the camera.

It is recommended that you get the examples read_images and read_images_opencv working on
your hardware, and use those to get started with using the camera in your project. Those
implement the most common use case of video capture and interfacing with OpenCV.

You can directly link against the indigo.so (indigo.dll on Windows) generated in build/bin,
or you can include the CMake project as a subproject of your own CMake project.
indigo.so (indigo.dll on Windows) is the library produced by the SDK, and it doesn't have
any dependencies other than your compiler's runtime.

==========================================================================================
== Building the SDK

The Indigo SDK uses CMake to generate platform-specific makefiles and project files (e.g., for
GNU make or Microsoft Visual C++). You need to first install cmake (which is freely available
from cmake.org and is also included as a standard package on most Linux distributions). The
sections below give commands to configure and build the SDK.

==========================================================================================
== Visual Studio 2015 compilation

cd <base SDK directory>
mkdir build
cd build
del CMakeCache.txt && cmake -G "Visual Studio 14 2015" -DOpenCV_DIR=e:/opencv/build -DUSE_OPENCV=1 -DINDIGOSDK_MSVC_STATIC_CRT=OFF ..
cmake --build . --config Release -- /nologo /v:m

==========================================================================================
== Visual Studio 2015 compilation for non-X86 platform

cd <base SDK directory>
mkdir build
cd build
del CMakeCache.txt && cmake -G "Visual Studio 14 2015" -DOpenCV_DIR=e:/opencv/build -DUSE_OPENCV=1 -DINDIGOSDK_MSVC_STATIC_CRT=OFF -DOCCAM_SSE2=OFF ..
cmake --build . --config Release -- /nologo /v:m

==========================================================================================
== Ubuntu compilation

## build with OpenCV support
cd <base SDK directory>
mkdir build
cd build
rm -f CMakeCache.txt && cmake -DOpenCV_DIR=~/opencv/build -DUSE_OPENCV=1 ..
cmake --build . --config Release

## build with no OpenCV support
cd <base SDK directory>
mkdir build
cd build
rm -f CMakeCache.txt && cmake -DUSE_OPENCV=0 ..
cmake --build . --config Release

Use -DOCCAM_SSE2=OFF to build for ARM or other non-X86 platforms that don't have SSE support.

==========================================================================================
== Building with OpenCV

OpenCV is optional-- building the SDK with OpenCV is mostly useful for running some of the
examples (e.g., read_images_opencv) which can interactively show you camera output. The core
SDK does not require OpenCV, and does not need to be built with OpenCV even if your application
uses OpenCV to manipulate images from the camera (of course in that case *your* application
needs to build against OpenCV).

On the CMake command line, the <dir> given in -DOpenCV_DIR=<dir> should point to the build output
directory of OpenCV, or the install location that contains the OpenCVConfig.cmake file.

Version 3.2 of OpenCV as well as most recent versions should work. You can use one of the official
release tarballs, or "git clone https://github.com/opencv/opencv.git" to download the main OpenCV
git repository and build it yourself. "git checkout 3.2.0" to set your OpenCV directory to the 3.2.0
release tag, and then follow their instructions to build it (very similar usage of cmake as here).

