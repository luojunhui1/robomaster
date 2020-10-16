//
// Created by luojunhui on 1/28/20.
//
#include <iostream>
#include<opencv2/opencv.hpp>
#include<chrono>
#include<mutex>
#include<memory>

#include "RMVideoCapture.hpp"
#include "ArmorDetector.hpp"
#include "SerialPort.hpp"
#include "SolveAngle.hpp"
#include "preoptions.h"
#include "mydefine.h"
using namespace std;

#ifndef ROBOMASTER_THREAD_H
#define ROBOMASTER_THREAD_H
namespace rm
{
	struct Frame/*min unit to describe figure*/
	{
		cv::Mat img;
		uint32_t seq;         //count from 1
		double timeStamp;	//time in ms, from initialization to now
	};
	class FrameBuffer/*figure queue*/
	{
	public:
	    explicit FrameBuffer(int32_t size_);

		~FrameBuffer() = default;

		bool push(const Frame& frame);

		bool getLatest(Frame& frame);

	private:
		std::vector<Frame> frames;
		std::vector<std::timed_mutex> mutexs;

		uint32_t tailIdx;
		uint32_t headIdx;
		uint32_t size;
		double lastGetTimeStamp;
	};
    class Status//describe the run status
    {
    public:
        Status();
        int8_t curArmorState;// current armor is big or small
        int8_t lastArmorState;

        int8_t curFindState; //current image find armor or not
        int8_t lastFindState;

        int8_t curControlState; //current control mode
        int8_t lastControlState;

        uint8_t curAttackMode; //tracking or searching
        uint8_t lastAttackMode;
		void UpdateStates(int8_t armor_state, int8_t find_state, int8_t control_state, int8_t attack_mode);
    };
	class ImgProdCons
	{
	public:
		ImgProdCons();
		~ImgProdCons() = default;;
		/*
		initialize
		*/
		void Init();
		/*
		* @Brief: Receive self state from the serail port, update task mode if commanded
		*/
		void Feedback();

		/*
		* @Brief: keep reading image from the camera into the buffer
		*/
		void Produce();

		/*
		* @Brief: run tasks
		*/
		void Consume();
	private:
		/*
		* To prevent camera from dying!
		*/
		static bool quitFlag;
		static void SignalHandler(int);
		void InitSignals();

		/* Camera */
		std::unique_ptr<RMVideoCapture> videoCapturePtr;

		FrameBuffer frameBuffer;

		/* Serial */
		std::unique_ptr<SerialPort> serialPtr;

		/* Angle solver */
		std::unique_ptr<SolveAngle> solverPtr;

		/* Armor detector */
		std::unique_ptr<ArmorDetector> armorDetectorPtr;

        std::unique_ptr<Status> status;
	};
}

#endif //ROBOMASTER_INFANRY_H
