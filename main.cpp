#include <thread>
#include "MyThread.hpp"
#include "preoptions.h"

using namespace rm;
using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    PreOptions(argc,argv);

    ImgProdCons pro;
    pro.Init();

//    runWithCamera = false;
//    blueTarget = false;
//    showArmorBox = true;
//    showBianryImg = true;
//    showOrigin = true;
//    showLamps = true;

    std::thread produceThread(&rm::ImgProdCons::Produce, &pro);
    std::thread detectThread(&rm::ImgProdCons::Detect, &pro);
    std::thread compareThread(&rm::ImgProdCons::Compare, &pro);
    std::thread energyThread(&rm::ImgProdCons::Energy, &pro);
    std::thread feedbackThread(&rm::ImgProdCons::Feedback, &pro);
    //std::thread senseThread(&rm::ImgProdCons::feedback, &pro);

    produceThread.join();
    detectThread.join();
    compareThread.join();
    energyThread.join();
    feedbackThread.join();
    //senseThread.join();

    return 0;
}
