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
	{"-video", {
		"start the program with reading video directly without asking.", []() {
                carName = VIDEO;
			LOGM("Run with Video!");
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
		"show energy",[]() {
                showEnergy = true;
			LOGM("Enable show energy part!");
		}
	}},
    {"-hero", {
                        "",[]() {
                carName = HERO;
                LOGM("HERO SET SAIL! COMMANDER!");
            }
                }},
    {"-uav", {
                        "",[]() {
                carName = UAV;
                LOGM("UAV SET SAIL! COMMANDER!");
            }
                }},
    {"-infantry", {
                        "",[]() {
                carName = INFANTRY;
                LOGM("INFANTRY SET SAIL! COMMANDER!");
            }
                }},
    {"-sentry", {
                          "",[]() {
                carName = SENTRY;
                LOGM("SENTRY SET SAIL! COMMANDER!");
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
        char* token;
        string str;

        for (int i = 1; i < argc; i++)
        {
            token = strtok(argv[i],"= ");
			if(!strcmp(token,"-path"))
            {
                token = strtok(NULL,"= ");
                LOGM("Set Video Path as %s",token);
                videoPath = std::string(token);
                continue;
            }else if(!strcmp(token,"-cameraIndex"))
            {
                token = strtok(NULL,"= ");
                LOGM("Set Camera Index as %s",token);
                stoi(token,&cameraIndex,10);
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
		if (!redTarget && !blueTarget)
		{
			LOGW("Forget to choose target color!");
			exit(0);
		}
		if(carName == NOTDEFINED)
        {
		    LOGW("HAAVE NOT DEFINED CAR NAME! LITTLE FULL!");
		    exit(0);
        }
	}
}