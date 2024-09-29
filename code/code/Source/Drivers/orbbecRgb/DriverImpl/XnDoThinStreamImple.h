#pragma once
#ifndef XN_DOTHIN_STREAM_IMPLE
#define XN_DOTHIN_STREAM_IMPLE

#include "XnOS.h"
#include "XnLog.h"
#include "XnDothinConnectHelper.h"
#include "XnDoThinDataProcessor.h"

#define XN_DOTHIN_READ_THREAD_KILL_TIMEOUT 10000


typedef struct xnDothinReadThreadData
{
	/*Is running*/
	XnBool isRunning;
	/*Grab buffer*/
	XnUChar* grabBuffer;
    /*grab data size*/
    XnUInt32 grabBufferSize;
	/*Output buffer size*/
	XnUInt32 requiredFrameSize;
	/*Threads for reading data*/
	XN_THREAD_HANDLE  hReadThread;

	XnBool bKillReadThread;

	XnDothinConnectHelper* m_pConnectHelper;
	XnDothinDataProcessor* dotDataProcess;

} xnDothinReadThreadData;


class XnDothinStreamImple
{
public:
	XnDothinStreamImple();

	~XnDothinStreamImple();

    XnStatus XnDothinInitReadThread(XnUInt32 grabBufferSize, XnUInt32 requiredFrameSize, XnDothinConnectHelper* dtConnectHelper, XnDothinDataProcessor* dothinDataProcess);

	XnStatus XnDothinShutDownThread();


private:
	const XnChar* XN_MASK_DONTHIN_STREAM_IMP = "DothinStreamImple";
	xnDothinReadThreadData dThreadData;
	//XnDothinConnectHelper* dtConnectHelper;


};


#endif //XN_DOTHIN_STREAM_IMPLE

