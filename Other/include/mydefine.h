//
// Created by luojunhui on 2/15/20.
//

#ifndef ROBOMASTER_MYDEFINE_H
#define ROBOMASTER_MYDEFINE_H

#include <string>

#ifndef AUTO_SHOOT_STATE
#define  AUTO_SHOOT_STATE 0
#endif

#ifndef FAR_DISTANCE_SHOOT
#define  FAR_DISTANCE_SHOOT 1
#endif

#ifndef SMALL_ENERGY_STATE
#define  SMALL_ENERGY_STATE 2
#endif

#ifndef BIG_ENERGY_STATE
#define  BIG_ENERGY_STATE 2
#endif

#ifndef SEARCH_MODE
#define   SEARCH_MODE 3
#endif

#ifndef TRACKING_MODE
#define  TRACKING_MODE 4
#endif

#ifndef BIG_ARMOR
#define  BIG_ARMOR 5
#endif

#ifndef SMALL_ARMOR
#define  SMALL_ARMOR 6
#endif

#ifndef FIND_ARMOR_YES
#define  FIND_ARMOR_YES 7
#endif

#ifndef FIND_ARMOR_NO
#define  FIND_ARMOR_NO 8
#endif

#ifndef BLUE_ENEMY
#define  BLUE_ENEMY 9
#endif

#ifndef USEROI
#define  USEROI 1
#endif

/*IMAGEHEIGHT AND IMAGEWIDTH just for initialize the camera, maybe not the real frame format*/
#ifndef IMAGEWIDTH
#define  IMAGEWIDTH 640
#endif

#ifndef IMAGEHEIGHT
#define  IMAGEHEIGHT 480
#endif

#ifndef CARNAME_
    #define CARNAME_
    enum CARNAME {HERO, INFANTRY, SENTRY, UAV, VIDEO,NOTDEFINED};
#endif

#ifndef RECORD
#define  RECORD 0
#endif

#ifndef GPUMODE
#define  GPUMODE 1
#endif

extern bool showArmorBox;
extern bool showArmorBoxes;
extern bool showLightBlobs;
extern bool showOrigin;
extern bool runWithCamera;
extern bool showLamps;
extern bool saveVideo;
extern bool saveLabelledBoxes;
extern bool showBianryImg;
extern bool showEnergy;
extern bool blueTarget;
extern bool redTarget;
extern bool hsvMode;
extern bool saveTrackingImage;
extern int FRAMEWIDTH;
extern int FRAMEHEIGHT;
extern CARNAME carName;
extern std::string videoPath;
extern int cameraIndex;
extern float feedbackDelta;

#endif //ROBOMASTER_MYDEFINE_H
