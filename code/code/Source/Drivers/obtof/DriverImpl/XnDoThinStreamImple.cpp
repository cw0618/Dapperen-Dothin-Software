#include "XnDothinStreamImple.h"



XnDothinStreamImple::XnDothinStreamImple()
{

}


XN_THREAD_PROC xnDothinReadDataMain(XN_THREAD_PARAM pThreadParam)
{
	xnDothinReadThreadData* threadData = (xnDothinReadThreadData*)pThreadParam;

	//Improving Thread Priority
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

	XnStatus status = XN_STATUS_OK;
	while (TRUE)
	{
		if (threadData->bKillReadThread)
		{
			//xnLogVerbose(XN_MASK_DONTHIN_STREAM_IMP, "Dothin read data main thread exit...");
			return XN_STATUS_OK;
		}

		if (threadData->m_pConnectHelper != NULL)
		{
			unsigned long realSize = 0;
            status = threadData->m_pConnectHelper->ReadFrame(threadData->grabBuffer, threadData->grabBufferSize, &realSize);

            if (threadData->grabBufferSize != realSize)
            {
				xnLogError(XN_MASK_DONTHIN_STREAM_IMP, "Stream read frame size: %d, realSize: %d, result code : %d.", threadData->grabBufferSize, realSize, status);
				continue;
			}
			else
			{
				//xnLogInfo(XN_MASK_DONTHIN_STREAM_IMP, "Stream read frame size:%d.", threadData->grabBuffer);
			}

			//call back data to stream process.
			if (threadData->dotDataProcess != NULL && !threadData->bKillReadThread)
			{
                threadData->dotDataProcess->XnDothinFrameDataCallback(threadData->grabBuffer, threadData->grabBufferSize, threadData->requiredFrameSize);
			}
		}
	}
}

 
XnStatus XnDothinStreamImple::XnDothinInitReadThread(XnUInt32 grabBufferSize, XnUInt32 requiredFrameSize, 
	XnDothinConnectHelper* dtConnectHelper, XnDothinDataProcessor* dothinDataProcess)
{
	XN_VALIDATE_INPUT_PTR(dothinDataProcess);
	XN_VALIDATE_INPUT_PTR(dtConnectHelper);

	XnStatus nRetVal = XN_STATUS_OK;

	// Wipe the ThreadData memory
	xnOSMemSet(&dThreadData, 0, sizeof(xnDothinReadThreadData));
	dThreadData.isRunning = FALSE;
    dThreadData.bKillReadThread = FALSE;
    dThreadData.grabBufferSize = grabBufferSize;
    dThreadData.requiredFrameSize = requiredFrameSize;
	dThreadData.m_pConnectHelper = dtConnectHelper;
	dThreadData.dotDataProcess = dothinDataProcess;
	
    dThreadData.grabBuffer = (XnChar*)xnOSMalloc(grabBufferSize);
    xnOSMemSet(dThreadData.grabBuffer, 0, grabBufferSize);

	//start thread
	nRetVal = xnOSCreateThread(xnDothinReadDataMain, &dThreadData, &dThreadData.hReadThread);
	if (nRetVal != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_DONTHIN_STREAM_IMP, "%s", xnGetStatusString(nRetVal));
		return XN_STATUS_OS_THREAD_CREATION_FAILED;
	}

	dThreadData.isRunning = TRUE;
	return XN_STATUS_OK;
}




XnStatus XnDothinStreamImple::XnDothinShutDownThread()
{
	xnLogInfo(XN_MASK_DONTHIN_STREAM_IMP, "Dothin read data thread shut down start...");

	XN_VALIDATE_INPUT_PTR(&dThreadData);

	if (!dThreadData.bKillReadThread)
	{
		dThreadData.bKillReadThread = TRUE;

		// close thread
		xnLogInfo(XN_MASK_DONTHIN_STREAM_IMP, "Wait AndTerminateThread start...");
		xnOSWaitAndTerminateThread(&dThreadData.hReadThread, XN_DOTHIN_READ_THREAD_KILL_TIMEOUT);
		dThreadData.hReadThread = NULL;
		xnLogInfo(XN_MASK_DONTHIN_STREAM_IMP, "Wait AndTerminateThread send...");
	}


	if (dThreadData.grabBuffer != NULL)
	{
		xnOSFree(dThreadData.grabBuffer);
	}

	dThreadData.isRunning = FALSE;

	xnLogInfo(XN_MASK_DONTHIN_STREAM_IMP, "Dothin read data thread shut down end...");
	return XN_STATUS_OK;
}



XnDothinStreamImple::~XnDothinStreamImple()
{


}
