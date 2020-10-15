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
bool ImgProdCons::_quit_flag = false;
FrameBuffer::FrameBuffer(int32_t size) :	
	_frames(size),//initialize a vector with size numgbers elements 
	_mutexs(size),
	_tailIdx(0),
	_headIdx(0),
	_lastGetTimeStamp(0.0),
	this->size(size)
{
}

bool FrameBuffer::push(const Frame &frame)//push a frame into the queue
{
	const int32_t newHeadIdx = (_headIdx + 1) % size;

	//try for 2ms to lock
	unique_lock<timed_mutex> lock(_mutexs[newHeadIdx], chrono::milliseconds(2));
	if (!lock.owns_lock())
	{
		return false;
	}

	_frames[newHeadIdx] = frame;
	if (newHeadIdx == _tailIdx)
	{
		_tailIdx = (_tailIdx + 1) % size;
	}
	_headIdx = newHeadIdx;
	return true;
}

bool FrameBuffer::getLatest(Frame &frame)
{
	volatile const size_t headIdx = _headIdx;

	//try for 2ms to lock
	unique_lock<timed_mutex> lock(_mutexs[headIdx], chrono::milliseconds(2));
	if (!lock.owns_lock() ||
		_frames[headIdx].img.empty() ||
		_frames[headIdx].timeStamp == _lastGetTimeStamp)//it means the same frame
	{
		return false;
	}

	frame = _frames[headIdx];
	_lastGetTimeStamp = _frames[headIdx].timeStamp;

	return true;
}
States::States()
{
	cur_armor_state = BIG_ARMOR;
	last_armor_state = BIG_ARMOR;

	cur_control_state = AUTO_SHOOT_STATE;
	last_control_state = AUTO_SHOOT_STATE;

	cur_find_state = FIND_ARMOR_NO;
	last_find_state = FIND_ARMOR_NO;

	cur_attack_mode = SEARCH_MODE;
	last_attack_mode = SEARCH_MODE;
}
void States::updateStates(int8_t& armor_state, int8_t& find_state, int8_t& control_state, int8_t& attack_mode)
{
	//last states update
	last_armor_state = cur_armor_state;
	last_find_state = cur_find_state;
	last_control_state = cur_control_state;
	last_attack_mode = cur_attack_mode;
	//current states update
	cur_armor_state = armor_state;
	cur_find_state = find_state;
	cur_control_state = control_state;
	cur_attack_mode = attack_mode;
}
void ImgProdCons::signal_handler(int)
{
	perror("sigaction error:");
	ImgProdCons::_quit_flag = true;
}
void ImgProdCons::init_signals(void)
{
	ImgProdCons::_quit_flag = false;
	struct sigaction sigact;
	sigact.sa_handler = signal_handler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, (struct sigaction *)NULL);// if interrupt occurs,set _quit_flag as true;
}
ImgProdCons::ImgProdCons() : 
	_videoCapturePtr(make_unique<RMVideoCapture>()),
	_buffer(6), /*frame size is 6*/
	_serialPtr(make_unique<SerialPort>()),
	_solverPtr(make_unique<SolveAngle>()),
	_armorDetectorPtr(make_unique<ArmorDetector>())
{
}

void ImgProdCons::init()
{
	//initialize signal
	init_signals();

	//initialize camera
	_videoCapturePtr->open(0, 2); //camera id 0 buffersize 2
	_videoCapturePtr->setVideoFormat(640, 480, 2);//1 is mjpg,2 is jepg,3 is yuyv
	_videoCapturePtr->setExposureTime(100);
	_videoCapturePtr->setFPS(200);
	_videoCapturePtr->startStream();
	_videoCapturePtr->info();
}
void ImgProdCons::produce()
{
	auto startTime = chrono::high_resolution_clock::now();
	static uint32_t seq = 1;
	for (;;)
	{
		if (ImgProdCons::_quit_flag)
			return;
		if (!_videoCapturePtr->grab())
			continue;
		Mat newImg;
		double timeStamp = (static_cast<chrono::duration<double, std::milli>>(chrono::high_resolution_clock::now() -startTime)).count();
		/*write somthing, try to send serial some infomation,to make sure serial is working well,
		becase when interrupte happens we need to restart it from the beginning*/
		if (!_videoCapturePtr->retrieve(newImg))
		{
			continue;
		}
		_buffer.push(Frame{newImg, seq, timeStamp});
		seq++;
	}
}
void ImgProdCons::consume()
{
	Frame frame;
	Mat image0;
	cv::VideoCapture cap;
	_armorDetectorPtr->init();
	if (!run_with_camera)
	{
		if(blue_target)
			cap.open("E://RMHOME//RM_last//Robomaster_last//video//blue.mov");
		else
			cap.open("E://RMHOME//RM_last//Robomaster_last//video//red.MOV");
	}

	do
	{
		if (run_with_camera)
		{
			if (!_buffer.getLatest(frame))
				continue;
			image0 = frame.img;
		}
		else
		{
			image0 = cap.read(image0);
			if (image0.empty())
				continue;
		}
		if (_states->cur_control_state == BIG_ENERGY_STATE)
		{

		}
		else if (_states->cur_control_state == SMALL_ENERGY_STATE)
		{

		}
		else if (_states->cur_control_state == AUTO_SHOOT_STATE)
		{
			switch (_states->cur_attack_mode)
			{
			case SEARCH_MODE:
			{

				if (_armorDetectorPtr->ArmorDetectTask(image0))
				{
					Rect armor_rect = _armorDetectorPtr->getArmorRect();
					bool is_small_ = _armorDetectorPtr->isSmall();
					int8_t armor_type_ = (is_small_) ? (SMALL_ARMOR) : (BIG_ARMOR);
					_solverPtr->getAngle(armor_rect, 15, _armorDetectorPtr->angle_x_, _armorDetectorPtr->angle_y_, _armorDetectorPtr->distance_, is_small_);
					_serialPtr->update_serial_out(_armorDetectorPtr->distance_, _armorDetectorPtr->last_distance, _armorDetectorPtr->angle_x_, _armorDetectorPtr->angle_y_, FIND_ARMOR_YES);
					_serialPtr->send_data();
					_states->updateStates(armor_type_, FIND_ARMOR_YES, AUTO_SHOOT_STATE, TRACKING_MODE);
					_armorDetectorPtr->tracker = TrackerKCF::create();
					_armorDetectorPtr->tracker->init(image0, armor_rect);
					_armorDetectorPtr->last_distance = _armorDetectorPtr->distance_;
				}
				else
				{
					_serialPtr->update_serial_out(_armorDetectorPtr->distance_, _armorDetectorPtr->last_distance, _armorDetectorPtr->angle_x_, _armorDetectorPtr->angle_y_, FIND_ARMOR_NO);
					_serialPtr->send_data();
					_states->updateStates(SMALL_ARMOR, FIND_ARMOR_NO, AUTO_SHOOT_STATE, SEARCH_MODE);
				}

				//如果成功找到，将roi区域视为tracking区域,下一次进入tracking模式
				break;
			}
			case TRACKING_MODE:
			{

				//这里一开始就是trakcking，若追踪到目标，继续，未追踪到，进入searching,进入tracking函数时要判断分类器是否可用
				if (_armorDetectorPtr->trackingTarget(image0, _armorDetectorPtr->getArmorRect()))
				{
					Rect armor_rect = _armorDetectorPtr->getArmorRect();
					bool is_small_ = _armorDetectorPtr->isSmall();
					int8_t armor_type_ = (is_small_) ? (SMALL_ARMOR) : (BIG_ARMOR);
					_solverPtr->getAngle(armor_rect, 15, _armorDetectorPtr->angle_x_, _armorDetectorPtr->angle_y_, _armorDetectorPtr->distance_, is_small_);
					_serialPtr->update_serial_out(_armorDetectorPtr->distance_, _armorDetectorPtr->last_distance, _armorDetectorPtr->angle_x_, _armorDetectorPtr->angle_y_, FIND_ARMOR_YES);
					_serialPtr->send_data();
					_states->updateStates(armor_type_, FIND_ARMOR_YES, AUTO_SHOOT_STATE, TRACKING_MODE);
				}
				else
				{
					_serialPtr->update_serial_out(_armorDetectorPtr->distance_, _armorDetectorPtr->last_distance, _armorDetectorPtr->angle_x_, _armorDetectorPtr->angle_y_, FIND_ARMOR_NO);
					_serialPtr->send_data();
					_states->updateStates(SMALL_ARMOR, FIND_ARMOR_NO, AUTO_SHOOT_STATE, SEARCH_MODE);
				}

				break;
			}
			}
		}
	} while (ImgProdCons::_quit_flag);
}
void ImgProdCons::feedback()
{
}
} // namespace rm
