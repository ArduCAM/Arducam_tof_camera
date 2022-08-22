/* Copyright 2021 Arducam Technology co., Ltd. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#ifndef _ARDUCAM_DEPTH_CAMERA_H_
#define _ARDUCAM_DEPTH_CAMERA_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define Status int

	// struct ArducamDepthCamera;
	/**
	 * @brief Some types of frame data
	 *
	 */
	typedef enum
	{
		RAW_FRAME = 0,
		AMPLITUDE_FRAME,
		DEPTH_FRAME,
		FRAME_TYPE_COUNT,
	} FrameType;
	/**
	 * @brief The output type of the frame, the RAW type outputs RAW_FRAME, the DEPTH type outputs DEPTH_FRAME and AMPLITUDE_FRAME
	 *
	 */
	typedef enum
	{
		RAW_TYPE = 0,
		DEPTH_TYPE,
		OUTPUT_TYPE_COUNT,
	} OutputType;
	/**
	 * @brief Description of frame data format
	 *
	 */
	typedef struct
	{
		uint16_t width;	 //! width of frame
		uint16_t height; //! height of frame
		FrameType type;	 //! type of frame
		uint8_t bitdepth;
	} FrameFormat;
#ifndef DOXYGEN_SHOULD_SKIP_THIS

	// typedef struct {
	// 	int32_t frameid;
	// 	int64_t timestamp;	//! timestamp
	// 	FrameFormat format;
	// 	uint32_t length;
	// 	uint8_t *data;
	// 	void *priv;
	// } FrameBuffer;

	// typedef enum {
	// 	EXPOSURE,
	// } ControlID;
	// /**
	//  * @struct CameraInfo
	//  * @brief Basic information of the camera module
	//  */
	// struct CameraInfo{
	// 	char* cameraId;		/**<Model of camera module */
	// };

	// typedef uint8_t (*BUFFER_CALLBACK)(ArducamDepthCamera *camera, FrameBuffer *fb);		/**<Callback function prototype  */

	// /**
	//  * @struct ArducamDepthCamera
	//  * @brief Camera drive interface and information
	//  */
	// typedef struct ArducamDepthCamera {
	// 	int fd;												/**< File handle to hold the device (should not be accessed by the user) */
	// 	struct CameraInfo cameraInfo;						/**< Basic information of the current camera */
	// 	const struct CameraOperations* arducamCameraOp;		/**< Camera function interface */
	// 	BUFFER_CALLBACK callBackFunction;					/**< Camera callback function */
	// } ArducamDepthCamera;

	typedef void *ArducamDepthCamera;

	typedef void *FrameBuffer;

/// @cond
// struct CameraOperations{
// 	Status (*open)(ArducamDepthCamera *camera, int index);
// 	Status (*close)(ArducamDepthCamera *camera);
// 	Status (*start)(ArducamDepthCamera *camera);
// 	Status (*stop)(ArducamDepthCamera *camera);
// 	Status (*setFormat)(ArducamDepthCamera *camera, FrameFormat *format);
// 	Status (*getFormat)(ArducamDepthCamera *camera, FrameFormat *format);
// 	Status (*setControl)(ArducamDepthCamera *camera, ControlID id, int64_t val);
// 	Status (*readFrame)(ArducamDepthCamera *camera, FrameType type, FrameBuffer *fb, int timeout);
// 	Status (*releaseBuffer)(ArducamDepthCamera *camera, FrameBuffer *fb);
// 	Status (*registerCallback)(ArducamDepthCamera *camera, FrameType type, BUFFER_CALLBACK callback);
// };
/// @endcond
#endif
	/**
	 * @brief Create a camera instance.
	 *
	 * @return Return a ArducamDepthCamera instance.
	 */
	extern ArducamDepthCamera createArducamDepthCamera();

	/**
	 * @brief Initialize the camera configuration and turn on the camera, set the initialization frame according to the @ref type.
	 *
	 * @param camera Camera instance, obtained through @ref createArducamDepthCamera().
	 * @param type Type of camera output fame.
	 * @param path Device node, the default value is video0.
	 *
	 * @return Return Status code.
	 */
	extern Status initialize(ArducamDepthCamera camera, OutputType type, char const* path);

	// /**
	//  * @brief Close camera.
	//  *
	//  * @param camera Camera instance.
	//  *
	//  * @return Return Status code.
	//  */
	// extern Status close_camera(ArducamDepthCamera *camera);

	/**
	 * @brief Start the camera stream and start processing.
	 *
	 * @param camera Camera instance.
	 *
	 * @return Return Status code.
	 */
	extern Status start(ArducamDepthCamera camera);

	/**
	 * @brief Stop camera stream and processing.
	 *
	 * @param camera Camera instance.
	 *
	 * @return Return Status code.
	 */
	extern Status stop(ArducamDepthCamera camera);

	// /**
	//  * @brief Specifies the frame format.
	//  *
	//  * @param camera Camera instance.
	//  * @param format Specifies the frame format. If the set format is not supported,
	//  * the function will modify the value of this parameter to return the actual value used.
	//  *
	//  * @return Return Status code.
	//  */
	// extern Status setFormat(ArducamDepthCamera *camera, FrameFormat *format);

	/**
	 * @brief Get the format of the specified frame.
	 *
	 * @param camera Frame instance.
	 * @param format Frame type.
	 * This parameter can be one of the following values:
	 *          @arg RAW_FRAME
	 *          @arg AMPLITUDE_FRAME
	 *          @arg DEPTH_FRAME
	 *
	 * @return Return frame format.
	 */
	extern FrameFormat getFormat(FrameBuffer fb, FrameType type);

	// /**
	//  * @brief Get the current camera output format.
	//  *
	//  * @param camera Camera instance.
	//  * @param id The id of the control.
	//  * @param val The value that needs to be set.
	//  *
	//  * @return Return Status code.
	//  */
	// extern Status setControl(ArducamDepthCamera *camera, ControlID id, int64_t val);

	/**
	 * @brief Read frame from the camera.
	 *
	 * @param camera Camera instance.
	 * @param timeout Timeout time, -1 means to wait all the time, 0 means immediate range,
	 * other values indicate the maximum waiting time, the unit is milliseconds.
	 *
	 * @return Return Status code.
	 */
	extern FrameBuffer requestFrame(ArducamDepthCamera camera, int timeout);

	/**
	 * @brief Release the FrameBuffer.
	 *
	 * @param camera Camera instance.
	 * @param fb  FrameBuffer.
	 *
	 * @return Return Status code.
	 */
	extern Status releaseFrame(ArducamDepthCamera camera, FrameBuffer fb);

	// /**
	//  * @brief Register the callback to the camera,
	//  *
	//  * @param camera Camera instance.
	//  * @param type FrameType to receive
	//  * @param callback A pointer to the callback function.
	//  *
	//  * @return Return Status code.
	//  */
	// extern Status registerCallback(ArducamDepthCamera *camera, FrameType type, BUFFER_CALLBACK callback);

	/**
	 * @brief Read depth data from the frame.
	 * @note The output mode is the depth type, and the function can be called to obtain data
	 *
	 * @param fb dataframe object.
	 *
	 * @return Return Status code.
	 */
	extern void *getDepthData(FrameBuffer fb);
	/**
	 * @brief Read depth data from the frame.
	 * @note The output mode is the depth type, and the function can be called to obtain data.
	 *
	 * @param fb dataframe object.
	 *
	 * @return Return Status code.
	 */
	extern void *getAmplitudeData(FrameBuffer fb);

	/**
	 * @brief Read raw data from the frame.
	 * @note The output mode is the raw type, and the function can be called to obtain data.
	 *
	 * @param fb dataframe object.
	 *
	 * @return Return Status code.
	 */
	extern void *getRawData(FrameBuffer fb);

#ifdef __cplusplus
}
#endif
#endif /*__ARDUCAM_DEPTH_CAMERA_H_*/
