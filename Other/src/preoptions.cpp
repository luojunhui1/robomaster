#include "preoptions.h"
#include "log.h"
#include <map>

using namespace std;

bool show_armor_box = false;//����ѡ��װ�װ�
bool show_armor_boxes = false;//��������װ�װ�
bool show_light_blobs = false;//��������
bool show_origin = false;//չʾԭͼ
bool save_video = false;//������Ƶ
bool save_labelled_boxes = false;//�����ǵ�����
bool show_bianryimg = false;//չʾ��ֵ��ͼ��
bool show_energy = false;//չʾ������������
bool blue_target = false;
bool red_target = false;
bool hsv_mode = false;
bool run_with_camera = false;

std::map<std::string, std::pair<std::string, void(*)(void)>> options = {
	{"-blue",{
		"enemy is blue.",[]()
		{
			blue_target = true;
			LOGM("Chose blue armor enemy.");
		}
	}},
		{"-red",{
		"enemy is blue.",[]()
		{
			red_target = true;
			LOGM("Chose red armor enemy.");
		}
	}},
    {"-hsv",{
      "use HSV color mode",[]()
       {
           hsv_mode = true;
           LOGM("chose HSV mode ");
       }
    }},
	{"-help",{
		"show the help information.", []() {
			LOG(LOG_MSG, "<HELP>: " STR_CTR(WORD_BLUE, "All options below are for debug use."));
			for (const auto& option : options) {
				LOG(LOG_MSG, "<HELP>: " STR_CTR(WORD_GREEN, "%s: %s"), option.first.data(), option.second.first.data());
			}
		}
	}},
	{"-box", {
		"show the aim box.", []() {
			show_armor_box = true;
			LOGM("Enable show armor box");
		}
	}},
	{"-boxes",{
		"show the candidate aim boxes.", []() {
			show_armor_boxes = true;
			LOGM("Enable show armor boxes");
		}
	}},
	{"-blobs",{
		"show the candidate light blobs.", []() {
			show_light_blobs = true;
			LOGM("Enable show light blobs");
		}
	}},
	{"-origin", {
		"show the origin image.", []() {
			show_origin = true;
			LOGM("Enable show origin");
		}
	}},
	{"-camera", {
		"start the program with camera directly without asking.", []() {
			run_with_camera = true;
			LOGM("Run with camera!");
		}
	}},
	{"-save", {
		"save the video.", []() {
			save_video = true;
			LOGM("Enable save video!");
		}
	}},
	{"-save2",{
		"save the candidate boxes with their id labels.", []() {
			save_labelled_boxes = true;
			LOGM("labelled armor boxes will be saved!");
		}
	}},
	{"-energy", {
		"",[]() {
			show_energy = true;
			LOGM("Enable show energy part!");
		}
	}},
	{"-debug", {
		"show armors,energy and video.", []() {
			show_armor_box = true;
			LOGM("Enable show armor box");
			show_armor_boxes = true;
			LOGM("Enable show armor boxes");
			show_bianryimg = true;
			LOGM("Enable show binary image");
			show_energy = true;
			LOGM("Enable show energy part");
		}
	}}
};

void preOptions(int argc, char** argv) {
	if (argc >= 2) {
		for (int i = 1; i < argc; i++) {
			auto key = options.find(std::string(argv[i])); // argv[i]是运行程序时的选项
			if (key != options.end()) {
				key->second.second();
			}
			else {
				LOGW("Unknown option: %s. Use --help to see options.", argv[i]);
			}
		}
		if (!red_target && !blue_target)
		{
			LOGW("Forget to choose target color!");
		}
	}
}