#include "preoptions.h"
#include "log.h"
#include <map>

using namespace std;

std::map<std::string, std::pair<std::string, void(*)(void)>> options = {
	{"-blue",{
		"enemy is blue.",[]()
		{
            blueTarget = true;
			LOGM("Chose blue armor enemy.");
		}
	}},
		{"-red",{
		"enemy is blue.",[]()
		{
            redTarget = true;
			LOGM("Chose red armor enemy.");
		}
	}},
    {"-hsv",{
      "use HSV color mode",[]()
       {
           hsvMode = true;
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
    {"-l", {
         "show the lamps", []() {
                showLamps = true;
            LOGM("Enable show lamps");
            }
    }},
	{"-box", {
		"show the aim box.", []() {
                showArmorBox = true;
			LOGM("Enable show armor box");
		}
	}},
	{"-boxes",{
		"show the candidate aim boxes.", []() {
                showArmorBoxes = true;
			LOGM("Enable show armor boxes");
		}
	}},
	{"-blobs",{
		"show the candidate light blobs.", []() {
                showLightBlobs = true;
			LOGM("Enable show light blobs");
		}
	}},
	{"-origin", {
		"show the origin image.", []() {
                showOrigin = true;
			LOGM("Enable show origin");
		}
	}},
	{"-camera", {
		"start the program with camera directly without asking.", []() {
                runWithCamera = true;
			LOGM("Run with camera!");
		}
	}},
	{"-save", {
		"save the video.", []() {
                saveVideo = true;
			LOGM("Enable save video!");
		}
	}},
	{"-save2",{
		"save the candidate boxes with their id labels.", []() {
                saveLabelledBoxes = true;
			LOGM("labelled armor boxes will be saved!");
		}
	}},
	{"-energy", {
		"",[]() {
                showEnergy = true;
			LOGM("Enable show energy part!");
		}
	}},
	{"-debug", {
		"show armors,energy and video.", []() {
                showArmorBox = true;
			LOGM("Enable show armor box");
                showArmorBoxes = true;
			LOGM("Enable show armor boxes");
                showBianryImg = true;
			LOGM("Enable show binary image");
                showEnergy = true;
			LOGM("Enable show energy part");
                showLamps = true;
            LOGM("Enable show lamps part");
		}
	}}
};

void PreOptions(int argc, char** argv) {
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
		if (!redTarget && !blueTarget)
		{
			LOGW("Forget to choose target color!");
		}
	}
}