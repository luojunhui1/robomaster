#include <thread>
#include "MyThread.hpp"
#include "preoptions.h"

using namespace rm;
using namespace cv;
using namespace std;

int main()
{
    ImgProdCons pro;
    pro.Init();

    runWithCamera = true;
    blueTarget = false;
    showArmorBox = true;
    showBianryImg = false;
    showOrigin = false;

    //std::thread produceThread(&rm::ImgProdCons::Produce, &pro);
    std::thread consumeThread(&rm::ImgProdCons::Consume, &pro);
    //std::thread senseThread(&rm::ImgProdCons::feedback, &pro);

    //produceThread.join();
    consumeThread.join();
    //senseThread.join();

    return 0;
}
