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

#ifndef __OCCAM_INDIGO_API_H__
#define __OCCAM_INDIGO_API_H__

#ifdef __cplusplus
#define OCCAM_SDK_LINKAGE extern "C"
#else // __cplusplus
#define OCCAM_SDK_LINKAGE
#endif // __cplusplus
#ifdef _WIN32
#ifdef OCCAM_SDK_BUILD
#define OCCAM_API OCCAM_SDK_LINKAGE __declspec(dllexport)
#else // OCCAM_SDK_BUILD
#define OCCAM_API OCCAM_SDK_LINKAGE __declspec(dllimport)
#endif // OCCAM_SDK_BUILD
#else // _WIN32
#define OCCAM_API OCCAM_SDK_LINKAGE
#endif // _WIN32

#include <stdint.h>

/*! Initialize the SDK.
  This function must be called before any other APIs.
  @return OCCAM_API_SUCCESS on success, otherwise OCCAM_API_NOT_INITIALIZED.
 */
OCCAM_API int occamInitialize();
/*!
  Shut down the SDK.
  This function can optionally be called to free SDK allocated resources.
  @return OCCAM_API_SUCCESS
 */
OCCAM_API int occamShutdown();
/*!
  Allocate memory.
  This is a wrapper around malloc(), useful if the library is built with static CRT. Any memory that is returned from any module call that is expected to be freed by the user, must be allocated with this function and freed with #occamFree.
  @return the pointer to the allocated memory, or NULL if the allocation failed.
 */
OCCAM_API void* occamAlloc(int size);
/*!
  Free memory previously allocated with occamAlloc.
  This is a wrapper around free(), useful if the library is built with static CRT. Any memory that is returned from any module call that is expected to be freed by the user, must be allocated with #occamAlloc and freed with this call.
 */
OCCAM_API void occamFree(void* ptr);

typedef struct _OccamDeviceListEntry {
  /*! Unique device identifier.
    This has the form MODEL:SERIAL where MODEL is the model name of the device (e.g., omni5u3mt9v022) and serial is an 8 byte unique number that uniquely identifies the device. The serial number is always read from the device itself and is transferable across host machines.
   */
  char* cid;
} OccamDeviceListEntry;

typedef struct _OccamDeviceList {
  /*! Array of #OccamDeviceListEntry.
   */
  OccamDeviceListEntry* entries;
  /*! The number of items in the #entries array.
   */
  int entry_count;
} OccamDeviceList;

/*!
  Enumerate the set of devices present on the system.
  Uses libusb (all platforms), cyusb (Windows), libusbk (Windows) to enumerate the Occam devices supported by the SDK.
  @param timeout_ms the maximum amount of time this call should block while performing IO to enumerate devices.
  @param ret_device_list the returned list of devices, on success. You must free this list using #occamFreeDeviceList.
  @return OCCAM_API_SUCCESS on success, and otherwise OCCAM_API_ERROR_ENUMERATING_DEVICES.
 */
OCCAM_API int occamEnumerateDeviceList(int timeout_ms, OccamDeviceList** ret_device_list);
/*!
  Free a device list previous returned by a successful call to #occamEnumerateDeviceList.
  @param device_list the device list to free.
  @return always returns OCCAM_API_SUCCESS.
 */
OCCAM_API int occamFreeDeviceList(OccamDeviceList* device_list);

typedef void OccamDevice;
/*!
  Open a device.
  @param cid the unique device identifier, as given in the device list returned by #occamEnumerateDeviceList.
  @param device the returned device, on success.
  @return OCCAM_API_SUCCESS if the device is successfully opened. OCCAM_API_INVALID_PARAMETER is returned if the unique device identifier is not valid
 */
OCCAM_API int occamOpenDevice(const char* cid, OccamDevice** device);
/*!
  Close a device.
  @param device the point to the open device, returned by occamOpenDevice.
  @return OCCAM_API_SUCCESS if successful.
 */
OCCAM_API int occamCloseDevice(OccamDevice* device);

/*!
  Enumeration of parameter data types
 */
typedef enum _OccamParamType {
  OCCAM_PARAM_BOOL = 0,
  OCCAM_PARAM_INT = 1,
  OCCAM_PARAM_REAL = 2,
  OCCAM_PARAM_STRING = 3,
  OCCAM_PARAM_BOOL_ARRAY = 4,
  OCCAM_PARAM_INT_ARRAY = 5,
  OCCAM_PARAM_REAL_ARRAY = 6,
  OCCAM_PARAM_STRING_ARRAY = 7
  // next value 8
} OccamParamType;

/*!
  Enumerate of parameter names.
  These are the names of configurable parameters, device status values, and calibration data.
  Devices may not support all of these parameters.
  Use #occamEnumerateParamList to discover which parameters a device supports.
 */
typedef enum _OccamParam {
  OCCAM_EXPOSURE = 1,
  OCCAM_EXPOSURE_MICROSECONDS = 114,
  OCCAM_GAIN = 2,
  OCCAM_AUTO_EXPOSURE = 3,
  OCCAM_AUTO_GAIN = 4,
  OCCAM_SENSOR_WIDTH = 5,
  OCCAM_SENSOR_HEIGHT = 6,
  OCCAM_SENSOR_COUNT = 7,
  OCCAM_PAIRED_DEVICES = 8,
  OCCAM_COLOR = 9,
  OCCAM_PREFERRED_BACKEND = 10,
  OCCAM_TRIGGER_MODE = 11,
  OCCAM_MAX_USB_PENDING_FRAMES = 12,
  OCCAM_MAX_DEFERRED_PENDING_FRAMES = 13,
  OCCAM_MAX_DEFERRED_REAPING_FRAMES = 14,
  OCCAM_MAX_PROCESSING_QUEUED_FRAMES = 15,
  OCCAM_IMAGE_PROCESSING_ENABLED = 16,
  OCCAM_BRIGHTNESS = 17,
  OCCAM_GAMMA = 18,
  OCCAM_BLACK_LEVEL = 19,
  OCCAM_WHITE_BALANCE_RED = 20,
  OCCAM_WHITE_BALANCE_GREEN = 21,
  OCCAM_WHITE_BALANCE_BLUE = 22,
  OCCAM_WIRE_FPS = 23,
  OCCAM_WIRE_BPS = 24,
  OCCAM_EMIT_FPS = 25,
  OCCAM_TARGET_FPS = 26,
  OCCAM_SENSOR_DISTORTION_COEFS0 = 27,
  OCCAM_SENSOR_DISTORTION_COEFS1 = 28,
  OCCAM_SENSOR_DISTORTION_COEFS2 = 29,
  OCCAM_SENSOR_DISTORTION_COEFS3 = 30,
  OCCAM_SENSOR_DISTORTION_COEFS4 = 31,
  OCCAM_SENSOR_DISTORTION_COEFS5 = 32,
  OCCAM_SENSOR_DISTORTION_COEFS6 = 33,
  OCCAM_SENSOR_DISTORTION_COEFS7 = 34,
  OCCAM_SENSOR_DISTORTION_COEFS8 = 35,
  OCCAM_SENSOR_DISTORTION_COEFS9 = 36,
  OCCAM_SENSOR_INTRINSICS0 = 37,
  OCCAM_SENSOR_INTRINSICS1 = 38,
  OCCAM_SENSOR_INTRINSICS2 = 39,
  OCCAM_SENSOR_INTRINSICS3 = 40,
  OCCAM_SENSOR_INTRINSICS4 = 41,
  OCCAM_SENSOR_INTRINSICS5 = 42,
  OCCAM_SENSOR_INTRINSICS6 = 43,
  OCCAM_SENSOR_INTRINSICS7 = 44,
  OCCAM_SENSOR_INTRINSICS8 = 45,
  OCCAM_SENSOR_INTRINSICS9 = 46,
  OCCAM_SENSOR_ROTATION0 = 47,
  OCCAM_SENSOR_ROTATION1 = 48,
  OCCAM_SENSOR_ROTATION2 = 49,
  OCCAM_SENSOR_ROTATION3 = 50,
  OCCAM_SENSOR_ROTATION4 = 51,
  OCCAM_SENSOR_ROTATION5 = 52,
  OCCAM_SENSOR_ROTATION6 = 53,
  OCCAM_SENSOR_ROTATION7 = 54,
  OCCAM_SENSOR_ROTATION8 = 55,
  OCCAM_SENSOR_ROTATION9 = 56,
  OCCAM_SENSOR_TRANSLATION0 = 57,
  OCCAM_SENSOR_TRANSLATION1 = 58,
  OCCAM_SENSOR_TRANSLATION2 = 59,
  OCCAM_SENSOR_TRANSLATION3 = 60,
  OCCAM_SENSOR_TRANSLATION4 = 61,
  OCCAM_SENSOR_TRANSLATION5 = 62,
  OCCAM_SENSOR_TRANSLATION6 = 63,
  OCCAM_SENSOR_TRANSLATION7 = 64,
  OCCAM_SENSOR_TRANSLATION8 = 65,
  OCCAM_SENSOR_TRANSLATION9 = 66,

  OCCAM_STITCHING_OFFSET_PRESET = 67,
  OCCAM_STITCHING_OFFSET_PRESET_COUNT = 68,
  OCCAM_STITCHING_OFFSET_X = 69,
  OCCAM_STITCHING_OFFSET_Y = 70,
  OCCAM_STITCHING_RADIUS = 71,
  OCCAM_STITCHING_ROTATION = 72,
  OCCAM_STITCHING_SCALEWIDTH = 73,
  OCCAM_STITCHING_CROP = 74,

  OCCAM_FIRMWARE_VERSION_A = 75,
  OCCAM_FIRMWARE_VERSION_B = 76,
  OCCAM_FIRMWARE_VERSION_C = 77,
  OCCAM_FIRMWARE_VERSION = 78,
  OCCAM_CID = 79,
  OCCAM_MODEL = 80,
  OCCAM_SERIAL = 81,

  OCCAM_STEREO_MATCHER0 = 82,
  OCCAM_STEREO_RECTIFIER0 = 83,
  OCCAM_DEBAYER_FILTER0 = 84,
  OCCAM_IMAGE_FILTER0 = 85,
  OCCAM_UNDISTORT_FILTER0 = 86,
  OCCAM_BLENDER0 = 87,

  OCCAM_THRESHOLD = 88,
  OCCAM_MAX_GAP = 89,

  OCCAM_RECTIFY_SCALE = 90,

  OCCAM_BM_PREFILTER_TYPE = 91,
  OCCAM_BM_PREFILTER_SIZE = 92,
  OCCAM_BM_PREFILTER_CAP = 93,
  OCCAM_BM_SAD_WINDOW_SIZE = 94,
  OCCAM_BM_MIN_DISPARITY = 95,
  OCCAM_BM_NUM_DISPARITIES = 96,
  OCCAM_BM_TEXTURE_THRESHOLD = 97,
  OCCAM_BM_UNIQUENESS_RATIO = 98,
  OCCAM_BM_SPECKLE_RANGE = 99,
  OCCAM_BM_SPECKLE_WINDOW_SIZE = 100,

  OCCAM_HDR_MODE = 101,
  OCCAM_HDR_SHUTTER1 = 102,
  OCCAM_HDR_SHUTTER2 = 103,
  OCCAM_HDR_T2_RATIO = 104,
  OCCAM_HDR_T3_RATIO = 105,
  OCCAM_HDR_VSTEP1 = 106,
  OCCAM_HDR_VSTEP2 = 107,
  OCCAM_HDR_VSTEP3 = 108,
  OCCAM_HDR_VSTEP4 = 109,

  OCCAM_BINNING_MODE = 110,

  OCCAM_ADC_VREF = 111,
  OCCAM_ADC_COMPANDING = 112,

  OCCAM_AEC_AGC_DESIRED_BIN = 127,
  OCCAM_AEC_UPDATE_FREQUENCY = 128,
  OCCAM_AEC_LOW_PASS_FILTER = 129,
  OCCAM_AGC_OUTPUT_UPDATE_FREQUENCY = 130,
  OCCAM_AGC_LOW_PASS_FILTER = 131,
  OCCAM_AGC_AEC_PIXEL_COUNT = 132,

  OCCAM_RESERVED0 = 113,
  OCCAM_RESERVED1 = 114,
  OCCAM_RESERVED2 = 115,
  OCCAM_RESERVED3 = 116,
  OCCAM_RESERVED4 = 117,
  OCCAM_RESERVED5 = 118

  // next value 141

} OccamParam;

/*!
  Enumeration of backend types.
 */
typedef enum _OccamBackend {
  OCCAM_CPU = 1,
  OCCAM_OPENGL = 2
} OccamBackend;

/*!
  Enumeration of backend types.
 */
typedef enum _OccamStorageClass {
  OCCAM_NOT_STORED = 1,
  OCCAM_SETTINGS = 2,
  OCCAM_CALIBRATION = 3
} OccamStorageClass;

/*!
  Enumeration of data types.
  These are the types of data that may be returned each frame.
 */
typedef enum _OccamDataType {
  OCCAM_MARKERS = 1,
  OCCAM_IMAGE = 2,
  OCCAM_POINT_CLOUD = 3
} OccamDataType;

/*!
  HDR modes.
 */
enum OccamHDRMode {
  OCCAM_HDR_MODE_DISABLED = 0,
  OCCAM_HDR_MODE_AUTO_SHUTTER = 1,
  OCCAM_HDR_MODE_MANUAL = 2
};

/*!
  Binning modes.
 */
enum OccamBinningMode {
  OCCAM_BINNING_DISABLED = 1,
  OCCAM_BINNING_2x2 = 2,
  OCCAM_BINNING_4x4 = 4
};

/*!
  Enumeration of data names.
  These are the names of data that may be returned each frame. Devices may not support all of these outputs.
  Use #occamDeviceAvailableData to discover which outputs a device supports.
 */
typedef enum _OccamDataName {
  OCCAM_IMAGE0 = 1,
  OCCAM_IMAGE1 = 2,
  OCCAM_IMAGE2 = 3,
  OCCAM_IMAGE3 = 4,
  OCCAM_IMAGE4 = 5,
  OCCAM_IMAGE5 = 6,
  OCCAM_IMAGE6 = 7,
  OCCAM_IMAGE7 = 8,
  OCCAM_IMAGE8 = 9,
  OCCAM_IMAGE9 = 10,
  OCCAM_IMAGE10 = 11,
  OCCAM_IMAGE11 = 12,
  OCCAM_IMAGE12 = 13,
  OCCAM_IMAGE13 = 14,
  OCCAM_IMAGE14 = 15,
  OCCAM_RAW_IMAGE0 = 16,
  OCCAM_RAW_IMAGE1 = 17,
  OCCAM_RAW_IMAGE2 = 18,
  OCCAM_RAW_IMAGE3 = 19,
  OCCAM_RAW_IMAGE4 = 20,
  OCCAM_RAW_IMAGE5 = 21,
  OCCAM_RAW_IMAGE6 = 22,
  OCCAM_RAW_IMAGE7 = 23,
  OCCAM_RAW_IMAGE8 = 24,
  OCCAM_RAW_IMAGE9 = 25,
  OCCAM_RAW_IMAGE10 = 26,
  OCCAM_RAW_IMAGE11 = 27,
  OCCAM_RAW_IMAGE12 = 28,
  OCCAM_RAW_IMAGE13 = 29,
  OCCAM_RAW_IMAGE14 = 30,
  OCCAM_IMAGE_TILES0 = 31,
  OCCAM_IMAGE_TILES1 = 32,
  OCCAM_IMAGE_TILES2 = 33,
  OCCAM_RAW_IMAGE_TILES0 = 34,
  OCCAM_RAW_IMAGE_TILES1 = 35,
  OCCAM_RAW_IMAGE_TILES2 = 36,
  OCCAM_UNDISTORTED_IMAGE_TILES0 = 37,
  OCCAM_UNDISTORTED_IMAGE_TILES1 = 38,
  OCCAM_UNDISTORTED_IMAGE_TILES2 = 39,
  OCCAM_UNDISTORTED_IMAGE0 = 40,
  OCCAM_UNDISTORTED_IMAGE1 = 41,
  OCCAM_UNDISTORTED_IMAGE2 = 42,
  OCCAM_UNDISTORTED_IMAGE3 = 43,
  OCCAM_UNDISTORTED_IMAGE4 = 44,
  OCCAM_UNDISTORTED_IMAGE5 = 45,
  OCCAM_UNDISTORTED_IMAGE6 = 46,
  OCCAM_UNDISTORTED_IMAGE7 = 47,
  OCCAM_UNDISTORTED_IMAGE8 = 48,
  OCCAM_UNDISTORTED_IMAGE9 = 49,
  OCCAM_UNDISTORTED_IMAGE10 = 50,
  OCCAM_UNDISTORTED_IMAGE11 = 51,
  OCCAM_UNDISTORTED_IMAGE12 = 52,
  OCCAM_UNDISTORTED_IMAGE13 = 53,
  OCCAM_UNDISTORTED_IMAGE14 = 54,
  OCCAM_STITCHED_IMAGE0 = 55,
  OCCAM_STITCHED_IMAGE1 = 56,
  OCCAM_STITCHED_IMAGE2 = 57,
  OCCAM_RECTIFIED_IMAGE0 = 58,
  OCCAM_RECTIFIED_IMAGE1 = 59,
  OCCAM_RECTIFIED_IMAGE2 = 60,
  OCCAM_RECTIFIED_IMAGE3 = 61,
  OCCAM_RECTIFIED_IMAGE4 = 62,
  OCCAM_RECTIFIED_IMAGE5 = 63,
  OCCAM_RECTIFIED_IMAGE6 = 64,
  OCCAM_RECTIFIED_IMAGE7 = 65,
  OCCAM_RECTIFIED_IMAGE8 = 66,
  OCCAM_RECTIFIED_IMAGE9 = 67,
  OCCAM_DISPARITY_IMAGE0 = 68,
  OCCAM_DISPARITY_IMAGE1 = 69,
  OCCAM_DISPARITY_IMAGE2 = 70,
  OCCAM_DISPARITY_IMAGE3 = 71,
  OCCAM_DISPARITY_IMAGE4 = 72,
  OCCAM_TILED_DISPARITY_IMAGE = 73,
  OCCAM_STITCHED_DISPARITY_IMAGE = 74,
  OCCAM_POINT_CLOUD0 = 75,
  OCCAM_POINT_CLOUD1 = 76,
  OCCAM_POINT_CLOUD2 = 77,
  OCCAM_POINT_CLOUD3 = 78,
  OCCAM_POINT_CLOUD4 = 79,
  OCCAM_STITCHED_POINT_CLOUD = 80

  // next value 71
} OccamDataName;

/*!
  Structure representing a single parameter supported by a device.
  Use #occamEnumerateParamList to discover the set of parameters supported by an open device.
 */
typedef struct _OccamParamEntry {
  /*!
    The name of the parameter, in integer/enumeration form.
   */
  OccamParam id;
  /*!
    The module index.
    This is zero for all device-global options, and for first module of a given type.
    Useful for modules that appear multiple times and duplicate parameters of other modules.
   */
  int module_index;
  /*!
    Null-terminated string giving name of the parameter.
   */
  char* name;
  /*!
    The data type of the parameter.
   */
  OccamParamType type;
  /*!
    The storage class of the parameter.
   */
  OccamStorageClass storage_class;
  /*!
    The minimum value of the parameter, if applicable.
   */
  double min_value;
  /*!
    The maximum value of the parameter, if applicable.
   */
  double max_value;
  /*!
    The number of elements in the parameter, applicable for array types.
   */
  int32_t count;
  /*!
    Boolean indicating whether the parameter is read-only.
   */
  int read_only;
} OccamParamEntry;

/*!
  Structure representing the list of parameters supported by a device.
  Use #occamEnumerateParamList to discover the set of parameters supported by an open device.
  Use #occamFreeParamList to free the memory used by this structure.
 */
typedef struct _OccamParamList {
  /*!
    The array of parameters.
   */
  OccamParamEntry* params;
  /*!
    The number of elements in the #params array.
   */
  int param_count;
} OccamParamList;

/*!
  OccamParam indices can be optionally coded with upper 8 bits indicating the module_index.
  This is useful if a module is instantiated multiple times and duplicates settings of its other
  instantiations or other modules.
 */
#define OCCAM_MAKE_PARAM(base_id,module_index) ((OccamParam)((uint32_t)(base_id)+(uint32_t)((module_index)<<24)))
#define OCCAM_PARAM_ID(id) OccamParam(((uint32_t)(id))&((1<<24)-1))
#define OCCAM_MODULE_INDEX(id) (((uint32_t)(id))>>24)

/*!
  Enumerate the set of parameters supported by a device.
  Use #occamFreeParamList to free the parameter list returned on success.
  @param device the open device to enumerate parameters of.
  @param ret_param_list the returned parameter list.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamEnumerateParamList(OccamDevice* device, OccamParamList** ret_param_list);
/*!
  Free memory associated with a parameter list returned by #occamEnumerateParamList.
 */
OCCAM_API int occamFreeParamList(OccamParamList* param_list);
/*!
  Set an integer parameter.
  @param device the device to set the parameter on.
  @param id the parameter to set.
  @param the new value of the parameter.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamSetDeviceValuei(OccamDevice* device, OccamParam id, int value);
/*!
  Set an double parameter.
  @param device the device to set the parameter on.
  @param id the parameter to set.
  @param the new value of the parameter.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamSetDeviceValuer(OccamDevice* device, OccamParam id, double value);
/*!
  Set an string parameter.
  @param device the device to set the parameter on.
  @param id the parameter to set.
  @param the new value of the parameter.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamSetDeviceValues(OccamDevice* device, OccamParam id, const char* value);
/*!
  Set a integer array parameter.
  @param device the device to set the parameter on.
  @param id the parameter to set.
  @param the new value of the parameter.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamSetDeviceValueiv(OccamDevice* device, OccamParam id, const int* values, int value_count);
/*!
  Set a double array parameter.
  @param device the device to set the parameter on.
  @param id the parameter to set.
  @param the new value of the parameter.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_INVALID_COUNT if the size of the array is incorrect, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamSetDeviceValuerv(OccamDevice* device, OccamParam id, const double* values, int value_count);
/*!
  Set a string array parameter.
  @param device the device to set the parameter on.
  @param id the parameter to set.
  @param the new value of the parameter.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_INVALID_COUNT if the size of the array is incorrect, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamSetDeviceValuesv(OccamDevice* device, OccamParam id, char** values, int value_count);
/*!
  Get an integer parameter.
  @param device the device to set the parameter on.
  @param id the parameter to get.
  @param pointer to value to be assigned.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_INVALID_COUNT if the size of the array is incorrect, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamGetDeviceValuei(OccamDevice* device, OccamParam id, int* value);
/*!
  Get an integer parameter.
  @param device the device to set the parameter on.
  @param id the parameter to get.
  @param pointer to value to be assigned.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamGetDeviceValuer(OccamDevice* device, OccamParam id, double* value);
/*!
  Get a double parameter.
  @param device the device to set the parameter on.
  @param id the parameter to get.
  @param pointer to value to be assigned.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamGetDeviceValues(OccamDevice* device, OccamParam id, char** value);
/*!
  Get an opaque pointer to internal parameter.
  This is typically used to get module handles of internal components.
  @param device the device to set the parameter on.
  @param id the parameter to get.
  @param pointer to value to be assigned.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamGetDeviceValuep(OccamDevice* device, OccamParam id, void** value);
/*!
  Get a string parameter.
  @param device the device to set the parameter on.
  @param id the parameter to get.
  @param pointer to value to be assigned.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamGetDeviceValueiv(OccamDevice* device, OccamParam id, int* values, int value_count);
/*!
  Get an integer array parameter.
  @param device the device to set the parameter on.
  @param id the parameter to get.
  @param pointer to value to be assigned.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_INVALID_COUNT if the size of the array is incorrect, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamGetDeviceValuerv(OccamDevice* device, OccamParam id, double* values, int value_count);
/*!
  Get a string array parameter.
  @param device the device to set the parameter on.
  @param id the parameter to get.
  @param pointer to value to be assigned.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_INVALID_COUNT if the size of the array is incorrect, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamGetDeviceValuesv(OccamDevice* device, OccamParam id, char** values, int value_count);
/*!
  Get a pointer array parameter.
  @param device the device to set the parameter on.
  @param id the parameter to get.
  @param pointer to value to be assigned.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device, OCCAM_API_INVALID_COUNT if the size of the array is incorrect, OCCAM_API_GENERIC_ERROR or a more specific error if there is a failure communicating with the hardware.
 */
OCCAM_API int occamGetDeviceValuepv(OccamDevice* device, OccamParam id, void** values, int value_count);
/*!
  Get the number of elements for array parameters.
  @param device the device to set the parameter on.
  @param id the parameter to get.
  @param value_count pointer to integer that contains the number of elements on return.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device.
 */
OCCAM_API int occamGetDeviceValueCount(OccamDevice* device, OccamParam id, int* value_count);
/*!
  Reset the given parameter to its factory default.
  @param device the device to set the parameter on.
  @param id the parameter to reset.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_INVALID_PARAMETER if the parameter is not supported by the device.
 */
OCCAM_API int occamResetDeviceValue(OccamDevice* device, OccamParam id);

/*!
  Read a single frame of data.
  Reads a single frame of data in one or more types of returned forms. The minimal set of operations to satisfy the request will be formed. If a supported data is not requested then it will not be computed if possible.
  @param device pointer to open device.
  @param req_count the number of outputs requested.
  @param req array of data names that are requested.
  @param ret_types the returned data types of the data returned. May be null.
  @param ret_data the returned data.
  @param block whether the function should block. If this is false and no data is available, then OCCAM_API_DATA_NOT_AVAILABLE is returned. Otherwise the calling code will block until data becomes ready or an error occurs (e.g., device goes away).
  @return OCCAM_API_SUCCESS on success, OCCAM_API_UNSUPPORTED_DATA if data is requested that is not supported by the device. Other errors may be returned by the driver in the case of hardware failure.
 */
OCCAM_API int occamDeviceReadData(OccamDevice* device, int req_count, const OccamDataName* req,
				  OccamDataType* ret_types, void** ret_data, int block);
/*!
  Query the driver for what data is available.
  The available data may depend on the configuration of the device according to device values.
  Note that on successful return, the values req and types must be freed via an #occamFree call.
  @param device pointer to open device.
  @param req_count pointer to returned value indicating number of elements in returned req and types arrays.
  @param req the set of data names returned.
  @param req the set of data types returned.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamDeviceAvailableData(OccamDevice* device, int* req_count, OccamDataName** req, OccamDataType** types);

/*!
  Write a register on the device.
  @param device pointer to open device.
  @param addr the register index.
  @param value the register value.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_WRITE_ERROR on error.
 */
OCCAM_API int occamWriteRegister(OccamDevice* device, uint32_t addr, uint32_t value);
/*!
  Write a register on the device.
  @param device pointer to open device.
  @param addr the register index.
  @param value pointer to returned register value.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_READ_ERROR or OCCAM_API_WRITE_ERROR on error.
 */
OCCAM_API int occamReadRegister(OccamDevice* device, uint32_t addr, uint32_t* value);

/*!
  Write a register on the device.
  @param device pointer to open device.
  @param target the device request type. These are device specific values.
  @param addr the data address. These are device specific values.
  @param len number of bytes in supplied data array.
  @param data array of bytes supplied for the request.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_WRITE_ERROR or OCCAM_API_READ_ERROR on error.
 */
OCCAM_API int occamWriteStorage(OccamDevice* device, uint32_t target, uint32_t addr, uint32_t len, const uint8_t* data);
/*!
  Write a register on the device.
  @param device pointer to open device.
  @param target the device request type. These are device specific values.
  @param addr the data address. These are device specific values.
  @param len number of bytes in supplied data array.
  @param data array of bytes supplied for the request.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_WRITE_ERROR or OCCAM_API_READ_ERROR on error.
 */
OCCAM_API int occamReadStorage(OccamDevice* device, uint32_t target, uint32_t addr, uint32_t len, uint8_t* data);
/*!
  Store settings to non-volatile memory on the device.
  @param device pointer to open device.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_WRITE_ERROR or OCCAM_API_READ_ERROR on error.
 */
OCCAM_API int occamSaveSettings(OccamDevice* device);
/*!
  Reset the device.
  This causes a soft reset on the hardware.
  @param device pointer to open device.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_WRITE_ERROR or OCCAM_API_READ_ERROR on error.
 */
OCCAM_API int occamResetDevice(OccamDevice* device);

/*! List of image (pixel) formats.
 */
typedef enum _OccamImageFormat {
  OCCAM_GRAY8 = 0,
  OCCAM_R8 = 0,
  OCCAM_RG16 = 1,
  OCCAM_RGB24 = 2,
  OCCAM_RGBA32 = 3,
  OCCAM_SHORT1 = 4,
  OCCAM_FLOAT1 = 5,
  OCCAM_FLOAT2 = 6,
  OCCAM_FLOAT3 = 7,
  OCCAM_YUV422_PACKED = 8,
  OCCAM_YUV420_PACKED = 9,
  OCCAM_YUV422_PLANAR = 10,
  OCCAM_YUV420_PLANAR = 11
} OccamImageFormat;

#define OCCAM_IMAGE_MAX_PLANES 3
#define OCCAM_IMAGE_MAX_SUBIMAGES 10

/*! Structure representing a video image.
 */
typedef struct _OccamImage {
  /*! The name of the device that generated the image.
   */
  char* cid;
  /*! The timescale that time_ns is an element of. Timescales are distinct/unique identifiers that are the same
     for images from two cameras only if time_ns refers to the same time space/scale.
   */
  uint8_t timescale[16];
  /*! The time the frame was triggered. If the driver or firmware doesn't support this, then it is the
     time the host received the full frame.
   */
  uint64_t time_ns;
  /*! A sequential/monotonic index that identifies this frame. Sub-images will have the same index. Corresponding
     frames from synchronized cameras will have the same index values.
   */
  uint32_t index;
  /*! The reference count of this image. When the count reaches zero the data is freed.
   */
  uint32_t refcnt;
  /*! For sub-images, this refers to the parent image. In non-null, then data[*] point to data allocated by our
     parent image (or its parent image, etc).
   */
  struct _OccamImage* owner;

  /*! The backend that hosts the data for this image. The value of this field data fields below are valid.
     In particular step[*] and data[*] will only be valid for OCCAM_CPU, and texture[*] and sync will only
     be valid for OCCAM_OPENGL.
   */
  OccamBackend backend;
  /*! The pixel format of this image.
   */
  OccamImageFormat format;
  /*! The width in pixels of this image.
   */
  int width;
  /*! The height in pixels of this image.
   */
  int height;

  /*! The number of sub-images. Multi-sensor drivers that emit images with tiles for each sensor populate these
     fields to indicate which rectangles in the image correspond to which sensor.
   */
  int subimage_count;
  /*! The x coordinate of the left of the sub-image.
   */
  int si_x[OCCAM_IMAGE_MAX_SUBIMAGES];
  /*! The y coordinate of the top fo the sub-image.
   */
  int si_y[OCCAM_IMAGE_MAX_SUBIMAGES];
  /*! The width in pixels of the sub-image.
   */
  int si_width[OCCAM_IMAGE_MAX_SUBIMAGES];
  /*! The height in pixels of the sub-image.
   */
  int si_height[OCCAM_IMAGE_MAX_SUBIMAGES];

  // local backend

  /*! The step of the image (bytes per scanline) for OCCAM_CPU backend. This field is only valid if backend equals OCCAM_CPU.
     Normally this is width * bytes_per_pixel rounded up to 16-byte boundary.
     There is a value per image plane. For single-plane (packed) images, only step[0] is valid.
   */
  int step[OCCAM_IMAGE_MAX_PLANES];
  /*! The pointer to the actual image data. For single-plane (packed) images, only data[0] is valid.
   */
  uint8_t* data[OCCAM_IMAGE_MAX_PLANES];

  // opengl backend

  /*! The name of the OpenGL texture that holds the image data.
   */
  uint32_t texture[OCCAM_IMAGE_MAX_PLANES];
  /*! The GLsync object that indicates when any previous operations on the textures backing this image
     will be completed. If non-zero, you must delete this value with glDeleteSync. You can either configure
     the device to issue glFinish each frame, or issue a glWaitSync in your context to get non-blocking completion
     before you do further work on the texture or display it.
   */
  void* sync;
} OccamImage;

/*!
  Frees the given image.
  This reduces the reference count on the image and actually frees memory when that reaches zero.
  @param image the image to free.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamFreeImage(OccamImage* image);
/*!
  Returns a sub-image of a given image.
  Note that this allocates a new OccamImage structure but points to the original image memory.
  @param image the image to use.
  @param new_image a pointer to the returned OccamImage.
  @param x the left of the sub-image in pixels.
  @param y the top of the sub-image in pixels.
  @param width the width of the sub-image in pixels.
  @param height the height of the sub-image in pixels.
  @return OCCAM_API_SUCCESS on success, or OCCAM_API_INVALID_PARAMETER.
 */
OCCAM_API int occamSubImage(OccamImage* image, OccamImage** new_image, int x, int y, int width, int height);
/*!
  Copies the given image.
  If deep_copy is non-zero, the actual image data will be copied. Otherwise the returned OccamImage points
  to the same memory and increases the reference count on the structure.
  @param image the image to copy.
  @param new_image a pointer to the returned OccamImage.
  @param deep_copy whether to actually copy image data or just increase reference count.  
  @return OCCAM_API_SUCCESS on success, or OCCAM_API_INVALID_PARAMETER.
 */
OCCAM_API int occamCopyImage(const OccamImage* image, OccamImage** new_image, int deep_copy);
/*!
  Returns the number of planes implied by the given image format.
  @param format the image format.
  @param planes pointer to integer with returned number of planes.
  @return OCCAM_API_SUCCESS on success, or OCCAM_API_INVALID_PARAMETER if format not known.
 */
OCCAM_API int occamImageFormatPlanes(OccamImageFormat format, int* planes);
/*!
  Returns the number of bytes per pixel implied by the given image format.
  @param format the image format.
  @param bpp pointer to integer with returned number of bytes per pixels.
  @return OCCAM_API_SUCCESS on success, or OCCAM_API_INVALID_PARAMETER if format not known.
 */
OCCAM_API int occamImageFormatBytesPerPixel(OccamImageFormat format, int* bpp);

/*! Structure representing a point cloud.
 */
typedef struct _OccamPointCloud {
  /*! The name of the device that generated the point cloud.
   */
  char* cid;
  /*! The timescale that time_ns is an element of. Timescales are distinct/unique identifiers that are the same
     for images from two cameras only if time_ns refers to the same time space/scale.
   */
  uint8_t timescale[16];
  /*! The time the frame was triggered. If the driver or firmware doesn't support this, then it is the
     time the host received the full frame.
   */
  uint64_t time_ns;
  /*! A sequential/monotonic index that identifies this frame. Sub-images will have the same index. Corresponding
     frames from synchronized cameras will have the same index values.
   */
  uint32_t index;
  /*! The reference count of this point cloud. When the count reaches zero the data is freed.
   */
  uint32_t refcnt;

  /*! An array of (x,y,z) triplets representing the actual points. The range [0,point_count) is valid.
   */
  float* xyz;
  /*! An optional array of (r,g,b) triplets representing the point colors. If non-null, the range [0,point_count) is valid.
   */
  uint8_t* rgb;
  /*! The number of points specified by this structure.
   */
  int point_count;
} OccamPointCloud;

/*!
  Frees the given point cloud.
  This reduces the reference count on the image and actually frees memory when that reaches zero.
  @param image the point cloud to free.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamFreePointCloud(OccamPointCloud* point_cloud);
/*!
  Copies the given point cloud.
  If deep_copy is non-zero, the actual data will be copied. Otherwise the returned OccamPointCloud points
  to the same memory and increases the reference count on the structure.
  @param point_cloud the point cloud to copy.
  @param new_point_cloud a pointer to the returned OccamPointCloud.
  @param deep_copy whether to actually copy data or just increase reference count.  
  @return OCCAM_API_SUCCESS on success, or OCCAM_API_INVALID_PARAMETER.
 */
OCCAM_API int occamCopyPointCloud(const OccamPointCloud* point_cloud, OccamPointCloud** new_point_cloud, int deep_copy);

/*!
  Marker field data types.
 */
typedef enum _OccamMarkerFieldType {
  OCCAM_FIELD_UINT32 = 1,
  OCCAM_FIELD_FLOAT = 2
} OccamMarkerFieldType;

/*!
  Marker field names.
 */
typedef enum _OccamMarkerFieldName {
  OCCAM_MARKER_X2 = 1,
  OCCAM_MARKER_Y2 = 2,
  OCCAM_MARKER_X3 = 3,
  OCCAM_MARKER_Y3 = 4,
  OCCAM_MARKER_Z3 = 5,
  OCCAM_MARKER_SIZE = 6,
  OCCAM_MARKER_PRODSUM_X = 7,
  OCCAM_MARKER_PRODSUM_Y = 8,
  OCCAM_MARKER_SUM = 9,
  OCCAM_MARKER_COUNT = 10,
  OCCAM_MARKER_MIN_X = 11,
  OCCAM_MARKER_MIN_Y = 12,
  OCCAM_MARKER_MAX_X = 13,
  OCCAM_MARKER_MAX_Y = 14,
  OCCAM_MARKER_ID = 15
  // next value 16
} OccamMarkerFieldName;

/*! Structure representing a marker field. A field is a named element of a certain type,
   such as an X or Y coordinate, or an ID.
 */
typedef struct _OccamMarkerField {
  OccamMarkerFieldType type;
  OccamMarkerFieldName name;
} OccamMarkerField;

/*! Structure representing a set of markers.
 */
typedef struct _OccamMarkers {
  /*! The name of the device that generated the image.
   */
  char* cid;
  /*! The timescale that time_ns is an element of. Timescales are distinct/unique identifiers that are the same
     for images from two cameras only if time_ns refers to the same time space/scale.
   */
  uint8_t timescale[16];
  /*! The time the frame was triggered. If the driver or firmware doesn't support this, then it is the
     time the host received the full frame.
   */
  uint64_t time_ns;
  /*! A sequential/monotonic index that identifies this frame. Sub-images will have the same index. Corresponding
     frames from synchronized cameras will have the same index values.
   */
  uint32_t index;
  /*! The reference count of this image. When the count reaches zero the data is freed.
   */
  uint32_t refcnt;

  /*! The width of the image that generated this marker set.
   */
  int width;
  /*! The height of the image that generated this marker set.
   */
  int height;

  /*! The number of fields in each marker.
   */
  int field_count;
  /*! The fields that describe each marker.
   */
  OccamMarkerField* fields;

  /*! The number of markers in this set.
   */
  int marker_count;
  /*! The marker data. The format of this array depends on the fields array above, and is simply the packed
     array of field elements per marker.
   */
  void* markers;
} OccamMarkers;

/*!
  Frees the given markers.
  This reduces the reference count on the markers and actually frees memory when that reaches zero.
  @param markers the markers to free.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamFreeMarkers(OccamMarkers* markers);
/*!
  Copies the given markers.
  If deep_copy is non-zero, the actual marker data will be copied. Otherwise the returned OccamMarkers points
  to the same memory and increases the reference count on the structure.
  @param markers the markers to copy.
  @param new_markers a pointer to the returned OccamMarkers.
  @param deep_copy whether to actually copy marker data or just increase reference count.  
  @return OCCAM_API_SUCCESS on success, or OCCAM_API_INVALID_PARAMETER.
 */
OCCAM_API int occamCopyMarkers(const OccamMarkers* markers, OccamMarkers** new_markers, int deep_copy);
/*!
  Read a types marker field.
  The data pointer must point to pre-allocated space or reside on the stack and be of sufficient size corresponding to specified type. The resulting field is converted into the specific data type as required.
  @param markers the marker to read from.
  @param i the index of the marker.
  @param name the name of the field within the marker.
  @param type the type of the field.
  @param value pointer that will contain the resulting value.
  @return OCCAM_API_SUCCESS on success, OCCAM_API_FIELD_NOT_FOUND if field is not found, OCCAM_API_INVALID_TYPE if implied type cast is not possible.
 */
OCCAM_API int occamGetMarkerField(const OccamMarkers* markers, int i,
				  OccamMarkerFieldName name, OccamMarkerFieldType type,
				  void* value);

/*! Create an OpenGL context.
  This generates a hidden window on default screen, generates an OpenGL context in it, and makes that context current.
  @param context pointer to returned context.
  All the contexts returned by this call are "shared" in the GL (share texture namespace, etc).
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamGLCreateContext(void** context);
/*! Free an OpenGL context.
  @param pointer to the context to free.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamGLFreeContext(void* context);
/*! Make the given context current on the current thread.
  @param context the context to make current.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamGLMakeCurrent(void* context);
/*! Add the current context to the share list of the SDK created context.
  The current context can be created by an external system. This is used for example to allow sharing of SDK returned textured with display libraries such as Qt, for rendering of results.
  Currently this is only supported on Windows.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamGLShareContext();
/*! Create a GL texture.
  Draws from a pool of recycled textures if any have previously been freed.
  @param width the required width of the texture.
  @param height the required height of the texture.
  @param format0 the internal format.
  @param format1 the user supplied format.
  @param type the user supplied data type.
  @param texid pointer to returned texture ID.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamGLCreateTexture(int width, int height,
				   uint32_t format0, uint32_t format1, uint32_t type,
				   uint32_t* texid);
/*! Frees a GL texture.
  Puts the texture into the pool of recycled textures (which can be recovered via #occamGLCreateTexture) to minimize the number of actual allocations that get to the GPU. When the pool exceeds a certain size, all the textures in the cache are reset.
  @param texid ID of the texture to free.
  @param width the required width of the texture.
  @param height the required height of the texture.
  @param format0 the internal format.
  @param format1 the user supplied format.
  @param type the user supplied data type.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamGLFreeTexture(uint32_t texid, int width, int height,
				 uint32_t format0, uint32_t format1, uint32_t type);
/*! User-supplied callback that is invoked once a texture is downloaded to CPU main memory.
 */
typedef int (*GLDownloadCallback)(OccamImage* img, void* cb_data);
/*! Download a texture from GPU texture memory to CPU main memory.
  @param img image to download.
  @param cb callback to invoke when download completes.
  @param cb_data optional application data passed to the callback.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamGLDownload(OccamImage* img, GLDownloadCallback cb, void* cb_data);
/*! User-supplied callback that is invoked once a texture is uploaded to GPU texture memory.
 */
typedef int (*GLUploadCallback)(OccamImage* img, void* cb_data);
/*! Upload an image from CPU main memory to GPU texture memory.
  @param img image to download.
  @param cb callback to invoke when download completes.
  @param cb_data optional application data passed to the callback.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int occamGLUpload(OccamImage* img, GLUploadCallback cb, void* cb_data);

/*! Device Data Cache Operations
 */
typedef enum _OccamDeviceDataCacheOp {
  OCCAM_DEVICEDATACACHE_STORE,
  OCCAM_DEVICEDATACACHE_LOAD
} OccamDeviceDataCacheOp;
/*! User-supplied callback that is invoked when a driver needs to read/write from the device data cache.
 */
typedef int (*DeviceDataCacheCallback)(const char* cid, OccamDeviceDataCacheOp op,
				       int data_type,
				       const void* hash, int hash_len,
				       void* data, int data_len,
				       void* cb_data);
/*! Set device data cache callback.
  This is to allow applications to optionally cache data that is read from device non-volatile memory.
  Reading large amounts of this data can be slow, and in many cases (e.g., geometric calibration) the data
  doesn't change often.
  @param cb the callback to set.
  @param cb_data optionally application data passed to the callback.
  @return OCCAM_API_SUCCESS on success.
 */
OCCAM_API int setDeviceDataCache(DeviceDataCacheCallback cb, void* cb_data);

typedef enum _OccamModuleClass {
  OCCAM_MODULE_STEREO = 1,
  OCCAM_MODULE_STEREO_RECTIFY = 2,
  OCCAM_MODULE_DEBAYER_FILTER = 3,
  OCCAM_MODULE_IMAGE_FILTER = 4,
  OCCAM_MODULE_UNDISTORT_FILTER = 5,
  OCCAM_MODULE_BLEND_FILTER = 6
} OccamModuleClass;

typedef enum _OccamModuleInterfaceType {
  IOCCAMMODULEINFO = 1,
  IOCCAMINTERFACEBASE = 2,
  IOCCAMPARAMETERS = 3,
  IOCCAMSTEREO = 4,
  IOCCAMSTEREORECTIFY = 5,
  IOCCAMIMAGEFILTER = 6,
  IOCCAMUNDISTORTFILTER = 7,
  IOCCAMBLENDFILTER = 8,
  IOCCAMDEBUGDATA = 9
} OccamModuleInterfaceType;

typedef struct _IOccamModuleInfo {
  int (*construct)(void* handle,const char* keys,void** ret_handle);
  int (*getName)(void* handle,char** ret_name);
  int (*getPrettyName)(void* handle,char** ret_name);
  int (*getClass)(void* handle,OccamModuleClass* ret_class);
  int (*getPriority)(void* handle,int* ret_priority);
  int (*getVersion)(void* handle,int* ret_version);
} IOccamModuleInfo;

typedef struct _IOccamInterfaceBase {
  int (*getInterface)(void* handle,OccamModuleInterfaceType iface_type,void** ret_iface);
  int (*retain)(void* handle);
  int (*release)(void* handle);
} IOccamInterfaceBase;

typedef struct _IOccamParameters {
  int (*enumerateParameters)(void* handle,OccamParamList** ret_param_list);
  int (*setValuei)(void* handle,OccamParam id,int value);
  int (*setValuer)(void* handle,OccamParam id,double value);
  int (*setValues)(void* handle,OccamParam id,const char* value);
  int (*setValueiv)(void* handle,OccamParam id,const int* values,int value_count);
  int (*setValuerv)(void* handle,OccamParam id,const double* values,int value_count);
  int (*setValuesv)(void* handle,OccamParam id,char** values,int value_count);
  int (*getValuei)(void* handle,OccamParam id,int* value);
  int (*getValuer)(void* handle,OccamParam id,double* value);
  int (*getValues)(void* handle,OccamParam id,char** value);
  int (*getValuep)(void* handle,OccamParam id,void** value);
  int (*getValueiv)(void* handle,OccamParam id,int* values,int value_count);
  int (*getValuerv)(void* handle,OccamParam id,double* values,int value_count);
  int (*getValuesv)(void* handle,OccamParam id,char** values,int value_count);
  int (*getValuepv)(void* handle,OccamParam id,void** values,int value_count);
  int (*getValueCount)(void* handle,OccamParam id,int* value_count);
  int (*setDefaultValueb)(void* handle,OccamParam id,int value);
  int (*setDefaultValuei)(void* handle,OccamParam id,int value);
  int (*setDefaultValuer)(void* handle,OccamParam id,double value);
  int (*setDefaultValues)(void* handle,OccamParam id,const char* value);
  int (*resetValue)(void* handle,OccamParam id);
} IOccamParameters;

typedef struct _IOccamStereo {
  int (*configure)(void* handle,int N,int width,int height,
		   const double* const* D,const double* const* K,
		   const double* const* R,const double* const* T);
  int (*compute)(void* handle,int index,const OccamImage* img0,const OccamImage* img1,
		 OccamImage** disp);
} IOccamStereo;

typedef struct _IOccamStereoRectify {
  int (*configure)(void* handle,int N,int width,int height,
		   const double* const* D,const double* const* K,
		   const double* const* R,const double* const* T,
		   int transposed);
  int (*rectify)(void* handle,int index,const OccamImage* img0,OccamImage** img1);
  int (*unrectify)(void* handle,int index,const OccamImage* img0,OccamImage** img1);
  int (*generateCloud)(void* handle,int N,const int* indices,int transform,
		       const OccamImage* const* img0,const OccamImage* const* disp0,
		       OccamPointCloud** cloud1);
} IOccamStereoRectify;

typedef struct _IOccamImageFilter {
  int (*compute)(void* handle,const OccamImage* img0,OccamImage** img1);
} IOccamImageFilter;

typedef struct _IOccamUndistortFilter {
  int (*configure)(void* handle,int N,const int* si_x,const int* si_y,
		   const int* si_width,const int* si_height,
		   const double* const* D,const double* const* K0,const double* const* K1);
  int (*compute)(void* handle,const OccamImage* img0,OccamImage** img1);
  int (*undistortPoints)(void* handle, int N,const int* sensor_indices,
			 const float* x0,const float* y0,float* x1,float* y1);
} IOccamUndistortFilter;

typedef struct _IOccamBlendFilter {
  int (*configure)(void* handle,int N,
		   const int* sensor_width,const int* sensor_height,
		   const double* const* D,const double* const* K,
		   const double* const* R,const double* const* T);
  int (*compute)(void* handle,const OccamImage* const* img0,OccamImage** img1);
} IOccamBlendFilter;

typedef struct _IOccamDebugData {
  int (*setp)(void* handle,int index,void* v);
  int (*seti)(void* handle,int index,int v);
  int (*setr)(void* handle,int index,double v);
  int (*getp)(void* handle,int index,void** v);
  int (*geti)(void* handle,int index,int* v);
  int (*getr)(void* handle,int index,double* v);
} IOccamDebugData;

OCCAM_API int occamRegisterModule(IOccamModuleInfo* module_info);
OCCAM_API int occamUnregisterModule(IOccamModuleInfo* module_info);
OCCAM_API int occamEnumerateModules(OccamModuleClass module_class, IOccamModuleInfo*** ret_module_info);
OCCAM_API int occamLoadModules(const char* path);
OCCAM_API int occamAddModuleKeys(const char* keys);
OCCAM_API int occamConstructModule(OccamModuleClass module_class, const char* name, void** ret_handle);
OCCAM_API int occamGetInterface(void* handle, OccamModuleInterfaceType iface_type, void** ret_iface);
OCCAM_API int occamRetainModule(void* handle);
OCCAM_API int occamReleaseModule(void* handle);

/*! The set of status codes that functions in this API may return.
 */
typedef enum _OccamError {
  OCCAM_API_SUCCESS = 0,
  OCCAM_API_GENERIC_ERROR = 1,
  OCCAM_API_NOT_INITIALIZED = 2,
  OCCAM_API_ALREADY_INITIALIZED = 3,
  OCCAM_API_ERROR_ENUMERATING_DEVICES = 4,
  OCCAM_API_NOT_SUPPORTED = 5,
  OCCAM_API_UNSUPPORTED_DATA = 6,
  OCCAM_API_INVALID_PARAMETER = 7,
  OCCAM_API_INVALID_TYPE = 8,
  OCCAM_API_INVALID_COUNT = 9,
  OCCAM_API_INVALID_FORMAT = 10,
  OCCAM_API_DATA_NOT_AVAILABLE = 11,
  OCCAM_API_WRITE_ERROR = 12,
  OCCAM_API_READ_ERROR = 13,
  OCCAM_API_FIELD_NOT_FOUND = 14,
  OCCAM_API_MODULE_ALREADY_LOADED = 15,
  OCCAM_API_MODULE_NOT_FOUND = 16,
  OCCAM_API_MODULE_FAILED_TO_LOAD = 17,
  OCCAM_API_MODULE_FAILED_TO_ENUMERATE = 18,
  OCCAM_API_INVALID_MODULE_PATH = 19,
  OCCAM_API_INTERFACE_NOT_FOUND = 20,
  OCCAM_API_PARTIAL_DEVICE = 21
  // next value 22
} OccamError;

OCCAM_API int occamGetErrorString(OccamError error, char* str, int max_len);

#endif // __OCCAM_INDIGO_API_H__

// Local Variables:
// mode: c++
// End:
