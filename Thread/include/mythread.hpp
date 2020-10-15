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
		FrameBuffer(int32_t size);

		~FrameBuffer() = default;

		bool push(const Frame& frame);

		bool getLatest(Frame& frame);

	private:
		std::vector<Frame> _frames;
		std::vector<std::timed_mutex> _mutexs;

		uint32_t _tailIdx;
		uint32_t _headIdx;
		uint32_t size;
		double _lastGetTimeStamp;
	};
    class States//describe the run states
    {
    public:
        States();
        int8_t cur_armor_state;// current armor is big or small
        int8_t last_armor_state;

        int8_t cur_find_state; //current image find armor or not
        int8_t last_find_state;

        int8_t cur_control_state; //current control mode
        int8_t last_control_state;

        uint8_t cur_attack_mode; //tracking or searching
        uint8_t last_attack_mode;
		void updateStates(int8_t& armor_state,int8_t& find_state,int8_t& control_state,int8_t& attack_mode);
    };
	class ImgProdCons
	{
	public:
		ImgProdCons();
		~ImgProdCons() {};
		/*
		initialize
		*/
		void init();
		/*
		* @Brief: Receive self state from the serail port, update task mode if commanded
		*/
		void feedback();

		/*
		* @Brief: keep reading image from the camera into the buffer
		*/
		void produce();

		/*
		* @Brief: run tasks
		*/
		void consume();
	private:
		/*
		* To prevent camera from dying!
		*/
		static bool _quit_flag;
		static void signal_handler(int);
		void init_signals(void);

		/* Camera */
		std::unique_ptr<RMVideoCapture> _videoCapturePtr;

		FrameBuffer _buffer;

		/* Serial */
		std::unique_ptr<SerialPort> _serialPtr;

		/* Angle solver */
		std::unique_ptr<SolveAngle> _solverPtr;

		/* Armor detector */
		std::unique_ptr<ArmorDetector> _armorDetectorPtr;

        std::unique_ptr<States> _states;
	};
}

#endif //ROBOMASTER_INFANRY_H
