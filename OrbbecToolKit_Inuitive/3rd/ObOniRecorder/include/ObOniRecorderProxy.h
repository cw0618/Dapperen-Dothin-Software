/*****************************************************************************
 *  Copyright (C) 2019 by ORBBEC Technology., Inc.                           *
 *                                                                           *
 *  This file is part of ObOniRecorder.                                      *
 *                                                                           *
 *  This file belongs to ORBBEC Technology., Inc.                            *
 *  It is considered a trade secret, and is not to be divulged or use by     *
 *  parties who have not received written authorization from the owner.      *
 *                                                                           *
 *****************************************************************************/

 /**
  * @file ObOniRecorderProxy.h
  * @brief Used to record streams to an ONI file.
  * @date Feb 22, 2019
  * @version V1.0.1.3
  * @author linweihua<changyin@orbbec.com>
  */

#ifndef _OB_ONI_RECORDER_PROXY_H_
#define _OB_ONI_RECORDER_PROXY_H_

#include "ObOniTypes.h"
#include "ObOniDefines.h"


namespace ob_oni_record_api
{
	/**
	 * @class ObOniRecorderProxy
	 * @brief Responsible for providing recording API.
	 */
	class OB_ONI_API ObOniRecorderProxy
	{
	public:
		ObOniRecorderProxy();
		~ObOniRecorderProxy();

		/**
		 * @brief Initialize the recorder.
		 *
		 * @param[in] file_name Specify the file name that will store the recording.
		 * @retval OB_ONI_STATUS_OK Means success, otherwise failure.
		 *
		 * @note It can only be initialized once, and it's forbidden to call any other
		 *       method before calling this method.
		 */
		ObOniStatus Initialize(const char* file_name);

		/**
		 * @brief Get the version number of the API.
		 *
		 * @param[out] ver Store the version information.
		 * @retval OB_ONI_STATUS_OK Means success, otherwise failure.
		 */
		ObOniStatus GetVersion(ObOniVersion& ver);

		/**
		 * @brief Attaches a stream to the recorder, can attach multiple streams.
		 *
		 * @param[in] pinfo The properties of the stream that to be recorded.
		 * @param[in] blossy If this value is true, the recorder might use a lossy compresion,
		 *            which means that when the recording will be play-back, there might be a
		 *            small differences from the original frame. Default value is false.
		 * @retval OB_ONI_STATUS_OK Means success, otherwise failure.
		 *
		 * @note This method won't start recording, as soon as the recording process has been
		 *       started, no more streams can be attached to the recorder.
		 */
		ObOniStatus AttachStream(const ObOniAttachInfoHandle pinfo, const bool blossy = false);

		/**
		 * @brief Save the property value of the recorded stream into the recording file.
		 *
		 * @param[in] pinfo The properties of the stream that to be recorded.
		 * @param[in] id The numerical ID of the property that to be recorded.
		 * @param[in] pdata Place to store the data that to be written to the property.
		 * @param[in] data_size Size of the data that to be written to the property.
		 * @retval OB_ONI_STATUS_OK Means success, otherwise failure.
		 *
		 * @note Call this method whenever the property value changes.
		 */
		ObOniStatus RecordStreamProperty(const ObOniAttachInfoHandle pinfo, int id, const void* pdata, int data_size);

		/**
		 * @brief Starts recording.
		 *
		 * @retval OB_ONI_STATUS_OK Means success, otherwise failure.
		 *
		 * @note There's a known side effect related to AttachStream() method.
		 */
		ObOniStatus Start();

		/**
		 * @brief Records a frame into the ONI file.
		 *
		 * @param[in] pinfo The properties of the stream that to be recorded.
		 * @param[in] pframe Place to store the frame that to be written.
		 * @retval OB_ONI_STATUS_OK Means success, otherwise failure.
		 *
		 * @note It can only be called after calling Start() method, can be used for
		 *       both single-stream and multi-stream recording. No data copy inside the
		 *       API, it just pushes the data pointer to the message queue of the write
		 *       thread and then return. The caller is responsible for making a copy of
		 *       the data if necessary.
		 */
		ObOniStatus Record(const ObOniAttachInfoHandle pinfo, const ObOniFrameHandle pframe);

		/**
		 * @brief Records a frame into the ONI file for single stream recording.
		 *
		 * @param[in] pinfo The properties of the stream that to be recorded.
		 * @param[in] pdata Place to store the frame data that to be written.
		 * @param[in] data_size Size of the frame data that to be written.
		 * @param[in] width The width of the frame.
		 * @param[in] height The height of the frame.
		 * @param[in] bit_depth The bit depth of the frame.
		 * @retval OB_ONI_STATUS_OK Means success, otherwise failure.
		 *
		 * @note It can only be called after calling Start() method, just used for
		 *       single-stream recording. No data copy inside the API, It just pushes
		 *       the data pointer to the message queue of the write thread, and then
		 *       return. The caller is responsible for making a copy of the data if necessary.
		 */
		ObOniStatus Record(const ObOniAttachInfoHandle pinfo, const void* pdata, int data_size, int width, int height, int bit_depth);

		/**
		 * @brief Stops recording and detaches all streams from the recorder.
		 *
		 * @retval OB_ONI_STATUS_OK Means success, otherwise failure.
		 *
		 * @note It must be called to release resources after recording ends.
		 */
		ObOniStatus Destroy();

	private:
		bool m_binit;
		ObOniFrame m_frame;
		ObOniRecorderHandle m_handle;
	};
}

#endif //_OB_ONI_RECORDER_PROXY_H_
