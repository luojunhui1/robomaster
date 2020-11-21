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

struct MapBuffer
{
    Mat image[10];
    int curWIndex = 0;
    int curRIndex = 0;
    std::mutex _mutexs[10];
};

MapBuffer mapBufferG;

class StreamRetrieve : public CThread
{
public:
	StreamRetrieve(IStreamSourcePtr& streamSptr);
	bool start();
	bool stop();
private:
	void threadProc();
	bool m_isLoop;
	IStreamSourcePtr m_streamSptr;
	bool m_bThreadFinished;
};


StreamRetrieve::StreamRetrieve(IStreamSourcePtr& streamSptr)
        : CThread("streamRetrieve")
        , m_isLoop(false)
        , m_streamSptr(streamSptr)
        , m_bThreadFinished(false)
{

}

bool StreamRetrieve::start()
{
    m_isLoop = true;
    return createThread();
}

bool StreamRetrieve::stop()
{
    m_isLoop = false;
    while (m_bThreadFinished == false)
    {
        CThread::sleep(100);
    }
    m_streamSptr.reset();
    return destroyThread();
}

void StreamRetrieve::threadProc()
{
    int frameCount = 0;
    while (m_isLoop)
    {
        // 此frame对象必须为临时变量，对象的生命周期与驱动帧缓存相关。
        // 此对象生命周期结束意味着:驱动可以回收此帧缓存并用于存放后续获取到的帧。
        // 如没有及时释放，获取到的帧与设置的帧缓存相同时，将无法获取到帧（因为驱动已没有可用的帧缓存）。
        // This frame object must be a temporary variable, and its lifecycle is related to the drive frame cache.
        // The end of this object's life cycle means that the driver can reclaim this frame cache and use it to store subsequent acquired frames.
        // If it is not released in time, the acquired frame will be the same as the set frame cache, and the frame will not be acquired (because the driver has no available frame cache).
        CFrame frame;

        // 获取一帧
        // Get one frame
        if (!m_streamSptr)
        {
            printf("m_streamPtr is NULL.\n");
            return;
        }
        bool isSuccess = m_streamSptr->getFrame(frame, 300);
        if (!isSuccess)
        {
            printf("getFrame  fail.\n");
            continue;
        }

        // 判断帧的有效性
        // Judge the validity of frame
        bool isValid = frame.valid();
        if (!isValid)
        {
            printf("frame is invalid!\n");
            continue;
        }

        FrameBufferSPtr displayFrame;
        if (ConvertImage(frame, displayFrame))
        {
            //try for 2ms to lock
//            unique_lock<timed_mutex> lock(mapBufferG._mutexs[mapBufferG.curWIndex],chrono::milliseconds(10));
            if(!mapBufferG._mutexs[mapBufferG.curWIndex].try_lock())
            {
                continue;
            }
            mapBufferG.image[mapBufferG.curWIndex] = Mat(Size(displayFrame->Width(), displayFrame->Height()),\
                                               CV_8UC3, displayFrame->bufPtr());
            imshow("cur",mapBufferG.image[mapBufferG.curWIndex]);
            mapBufferG._mutexs[mapBufferG.curWIndex].unlock();
            mapBufferG.curWIndex = (mapBufferG.curWIndex + 1)%10;

        }else
        {
            perror("Image Convert Failure！");
        }
    }

    m_bThreadFinished = true;

}


