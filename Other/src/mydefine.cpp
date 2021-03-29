//
// Created by root on 2020/12/17.
//

#include "mydefine.h"

bool showArmorBox = false;
bool showArmorBoxes = false;
bool showLightBlobs = false;
bool showOrigin = true;
bool runWithCamera = false;
bool showLamps = false;
bool saveVideo = false;
bool saveLabelledBoxes = false;
bool showBianryImg = false;
bool showEnergy = false;
bool blueTarget = false;
bool redTarget = true;
bool hsvMode = false;

int FRAMEWIDTH;
int FRAMEHEIGHT;

CARNAME carName = SENTRY;
int cameraIndex = 0;
std::string videoPath = "/home/ljh/视频/Videos/Xavier_12_19.avi";

float feedbackDelta  = 1.11;
bool saveTrackingImage;