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
    std::thread consumeThread(&rm::ImgProdCons::Consume, &pro);
    //std::thread senseThread(&rm::ImgProdCons::feedback, &pro);

    produceThread.join();
    consumeThread.join();

    //senseThread.join();

    return 0;
}
