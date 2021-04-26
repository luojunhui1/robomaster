#include "preoptions.h"
#include "log.h"

#include <cstdlib>
#include <cstring>
#include <dirent.h>

#include <iostream>
#include <map>

using namespace std;

std::map<std::string, std::pair<std::string, void(*)(void)>> options = {
	{"-blue",{
		"enemy is blue.",[]()
		{
            blueTarget = true;
			LOGM("Choose blue armor enemy.");
		}
	}},
		{"-red",{
		"enemy is red.",[]()
		{
            blueTarget = false;
			LOGM("Choose red armor enemy.");
		}
	}},
	{"-help",{
		"show the help information.", []() {
			LOG(LOG_MSG, "<HELP>: " STR_CTR(WORD_BLUE, "All options below are for debug use."));
			for (const auto& option : options) {
				LOG(LOG_MSG, "<HELP>: " STR_CTR(WORD_GREEN, "%s: %s"), option.first.data(), option.second.first.data());
			}
			exit(0);
		}
	}},
    {"-l", {
         "show the lamps", []() {
                showLamps = true;
            LOGM("Enable show lamps");
            }
    }},
	{"-box", {
		"show the armor box.", []() {
                showArmorBox = true;
			LOGM("Enable show armor box");
		}
	}},
	{"-origin", {
		"show the original image.", []() {
                showOrigin = true;
			LOGM("Enable show origin");
		}
	}},
	{"-video", {
		"start the program with reading video directly without asking.", []() {
                carName = VIDEO;
			LOGM("Run with Video!");
		}
	}},
	{"-energy", {
		"show energy",[]() {
                showEnergy = true;
			LOGM("Enable show energy part!");
		}
	}},
    {"-hero", {
                        "HERO SET SAIL! COMMANDER!",[]() {
                carName = HERO;
                LOGM("HERO SET SAIL! COMMANDER!");
            }
                }},
    {"-uav", {
                        "UAV SET SAIL! COMMANDER!",[]() {
                carName = UAV;
                LOGM("UAV SET SAIL! COMMANDER!");
            }
                }},
    {"-melee", {
                        "MELEE INFANTRY SET SAIL! COMMANDER!",[]() {
                carName = INFANTRY_MELEE;
                LOGM("MELEE INFANTRY SET SAIL! COMMANDER!");
            }
                }},
    {"-track", {
                       "TRACK INFANTRY SET SAIL! COMMANDER!",[]() {
                carName = INFANTRY_TRACK;
                LOGM("TRACK INFANTRY SET SAIL! COMMANDER!");
            }
               }},
    {"-sentry", {
                          "SENTRY SET SAIL! COMMANDER!",[]() {
                carName = SENTRY;
                LOGM("SENTRY SET SAIL! COMMANDER!");
            }
                  }},
    {"-binary", {
                        "Show Binary ROI Image",[]() {
                showBianryImg = true;
                LOGM("Show Binary ROI Image!");
            }
                }},
	{"-all", {
		"show armors,energy and video.", []() {
		        showOrigin = true;
                showArmorBox = true;
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
        char *token;
        string str;

        for (int i = 1; i < argc; i++)
        {
            token = strtok(argv[i],"=");
			if(!strcmp(token,"-path"))
            {
                token = strtok(NULL,"=");
                LOGM("Set Video Path as %s",token);
                videoPath = std::string(token);
                continue;
            }else if(!strcmp(token,"-index"))
            {
                token = strtok(NULL,"=");
                LOGM("Set Camera Index as %s",token);
                cameraIndex = stoi(token, nullptr,10);
                continue;
            }else if(!strcmp(token,"-delta"))
            {
                token = strtok(NULL,"=");
                LOGM("Set feedback delta as %f",feedbackDelta);
                feedbackDelta = stof(token);
                continue;
            }
            auto key = options.find(std::string(argv[i])); //argv[i]是运行程序时的选项
			if (key != options.end()) {
				key->second.second();
			}
			else {
				LOGW("Unknown option: %s. Use --help to see options.", argv[i]);
			}
		}

		if(carName == NOTDEFINED)
        {
		    LOGW("HAVE NOT DEFINED CAR NAME! LITTLE FULL!");
		    exit(0);
        }
	}
}