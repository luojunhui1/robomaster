//
// Created by luojunhui on 1/28/20.
//

#include "mythread.hpp"
#include <csignal>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <memory>

using namespace std;
using namespace cv;

namespace rm
{
bool ImgProdCons::quitFlag = false;

FrameBuffer::FrameBuffer(int32_t size_) :
        frames(size_),//initialize a vector with size numgbers elements
	mutexs(size_),
        tailIdx(0),
        headIdx(0),
        lastGetTimeStamp(0.0),
        size(size_)
{
}

bool FrameBuffer::push(const Frame &frame)//push a frame into the queue
{
	const int32_t newHeadIdx = (headIdx + 1) % size;

	//try for 2ms to lock
	unique_lock<timed_mutex> lock(mutexs[newHeadIdx], chrono::milliseconds(2));
	if (!lock.owns_lock())
	{
		return false;
	}

    frames[newHeadIdx] = frame;
	if (newHeadIdx == tailIdx)
	{
        tailIdx = (tailIdx + 1) % size;
	}
    headIdx = newHeadIdx;
	return true;
}

bool FrameBuffer::getLatest(Frame &frame)
{
	volatile const size_t headIdx = headIdx;

	//try for 2ms to lock
	unique_lock<timed_mutex> lock(mutexs[headIdx], chrono::milliseconds(2));
	if (!lock.owns_lock() ||
        frames[headIdx].img.empty() ||
        frames[headIdx].timeStamp == lastGetTimeStamp)//it means the same frame
	{
		return false;
	}

	frame = frames[headIdx];
    lastGetTimeStamp = frames[headIdx].timeStamp;

	return true;
}

    Status::Status()
{
    curArmorState = BIG_ARMOR;
    lastArmorState = BIG_ARMOR;

    curControlState = AUTO_SHOOT_STATE;
    lastControlState = AUTO_SHOOT_STATE;

    curFindState = FIND_ARMOR_NO;
    lastFindState = FIND_ARMOR_NO;

    curAttackMode = SEARCH_MODE;
    lastAttackMode = SEARCH_MODE;
}
void Status::UpdateStates(int8_t armor_state, int8_t find_state, int8_t control_state, int8_t attack_mode)
{
	//last status update
	lastArmorState = curArmorState;
    lastFindState = curFindState;
    lastControlState = curControlState;
    lastAttackMode = curAttackMode;
	//current status update
	curArmorState = armor_state;
    curFindState = find_state;
    curControlState = control_state;
    curAttackMode = attack_mode;
}
void ImgProdCons::SignalHandler(int)
{
	perror("sigaction error:");
	ImgProdCons::quitFlag = true;
}
void ImgProdCons::InitSignals(void)
{
	ImgProdCons::quitFlag = false;
	struct sigaction sigact;
	sigact.sa_handler = SignalHandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, (struct sigaction *)NULL);// if interrupt occurs,set quitFlag as true;
}
ImgProdCons::ImgProdCons() :
        videoCapturePtr(make_unique<RMVideoCapture>()),
        frameBuffer(6), /*frame size is 6*/
	serialPtr(make_unique<SerialPort>()),
        solverPtr(make_unique<SolveAngle>()),
        armorDetectorPtr(make_unique<ArmorDetector>())
{
}

void ImgProdCons::Init()
{
	//initialize signal
    InitSignals();

	//initialize camera
	videoCapturePtr->RMopen(0, 2); //camera id 0 buffersize 2
	videoCapturePtr->setVideoFormat(640, 480, 2);//1 is mjpg,2 is jepg,3 is yuyv
	videoCapturePtr->setExposureTime(100);
	videoCapturePtr->setFPS(200);
	videoCapturePtr->startStream();
	videoCapturePtr->info();
}
void ImgProdCons::Produce()
{
	auto startTime = chrono::high_resolution_clock::now();
	static uint32_t seq = 1;
	for (;;)
	{
		if (ImgProdCons::quitFlag)
			return;
		if (!videoCapturePtr->grab())
			continue;
		Mat newImg;
		double timeStamp = (static_cast<chrono::duration<double, std::milli>>(chrono::high_resolution_clock::now() -startTime)).count();
		/*write somthing, try to send serial some infomation,to make sure serial is working well,
		becase when interrupte happens we need to restart it from the beginning*/
		if (!videoCapturePtr->retrieve(newImg))
		{
			continue;
		}
		frameBuffer.push(Frame{newImg, seq, timeStamp});
		seq++;
	}
}
void ImgProdCons::Consume()
{
	Frame frame;
	Mat image0;
	cv::VideoCapture cap;
    armorDetectorPtr->Init();
	if (!runWithCamera)
	{
		if(blueTarget)
			cap.open("E://RMHOME//RM_last//Robomaster_last//video//blue.mov");
		else
			cap.open("E://RMHOME//RM_last//Robomaster_last//video//red.MOV");
	}

	do
	{
		if (runWithCamera)
		{
			if (!frameBuffer.getLatest(frame))
				continue;
			image0 = frame.img;
		}
		else
		{
			image0 = cap.read(image0);
			if (image0.empty())
				continue;
		}
		if (status->curControlState == BIG_ENERGY_STATE)
		{

		}
		else if (status->curControlState == SMALL_ENERGY_STATE)
		{

		}
		else if (status->curControlState == AUTO_SHOOT_STATE)
		{
			switch (status->curAttackMode)
			{
			case SEARCH_MODE:
			{

				if (armorDetectorPtr->ArmorDetectTask(image0))
				{
					Rect armor_rect = armorDetectorPtr->GetArmorRect();
					bool is_small_ = armorDetectorPtr->IsSmall();
					int8_t armor_type_ = (is_small_) ? (SMALL_ARMOR) : (BIG_ARMOR);
                    solverPtr->GetAngle(armor_rect, 15, is_small_);
                    serialPtr->UpdateSerialOut(solverPtr->distance,solverPtr->yaw,
                                               solverPtr->pitch, FIND_ARMOR_YES);
                    serialPtr->SendData();
                    status->UpdateStates(armor_type_, FIND_ARMOR_YES, AUTO_SHOOT_STATE, TRACKING_MODE);
                    armorDetectorPtr->tracker = TrackerKCF::create();
					armorDetectorPtr->tracker->init(image0, armor_rect);
                    solverPtr->lastDistance = solverPtr->distance;
				}
				else
				{
                    serialPtr->UpdateSerialOut(solverPtr->lastDistance,solverPtr->yaw,
                                               solverPtr->pitch, FIND_ARMOR_NO);
                    serialPtr->SendData();
                    status->UpdateStates(SMALL_ARMOR, FIND_ARMOR_NO, AUTO_SHOOT_STATE, SEARCH_MODE);
				}

				//如果成功找到，将roi区域视为tracking区域,下一次进入tracking模式
				break;
			}
			case TRACKING_MODE:
			{

				//这里一开始就是trakcking，若追踪到目标，继续，未追踪到，进入searching,进入tracking函数时要判断分类器是否可用
				if (armorDetectorPtr->trackingTarget(image0, armorDetectorPtr->GetArmorRect()))
				{
					Rect armor_rect = armorDetectorPtr->GetArmorRect();
					bool is_small_ = armorDetectorPtr->IsSmall();
					int8_t armor_type_ = (is_small_) ? (SMALL_ARMOR) : (BIG_ARMOR);
                    solverPtr->GetAngle(armor_rect, 15, is_small_);
                    serialPtr->UpdateSerialOut(solverPtr->distance, solverPtr->lastDistance, solverPtr->yaw,
                                               solverPtr->pitch, FIND_ARMOR_YES);
                    serialPtr->SendData();
                    status->UpdateStates(armor_type_, FIND_ARMOR_YES, AUTO_SHOOT_STATE, TRACKING_MODE);
				}
				else
				{
                    serialPtr->UpdateSerialOut(solverPtr->distance, solverPtr->lastDistance, solverPtr->yaw,
                                               solverPtr->pitch, FIND_ARMOR_NO);
                    serialPtr->SendData();
                    status->UpdateStates(SMALL_ARMOR, FIND_ARMOR_NO, AUTO_SHOOT_STATE, SEARCH_MODE);
				}

				break;
			}
			}
		}
	} while (ImgProdCons::quitFlag);
}
void ImgProdCons::Feedback()
{
}
} // namespace rm
