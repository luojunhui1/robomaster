#pragma once

#ifdef __unix__
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

#include "RGBConvert.h"
#include "GenICam/Frame.h"
#include "Infra/Thread.h"
#include "GenICam/StreamSource.h"
#include "Media/RecordVideo.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

extern HANDLE g_hHandle;

using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace cv;


class StreamRetrieve : public CThread
{
public:
	explicit StreamRetrieve(IStreamSourcePtr& streamSptr);
	bool start();
	bool stop();
private:
	void threadProc() override;
	bool m_isLoop;
	IStreamSourcePtr m_streamSptr;
	bool m_bThreadFinished;
};
