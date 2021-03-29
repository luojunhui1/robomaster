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

    std::thread produceThread(&rm::ImgProdCons::Produce, &pro);
    std::thread detectThread(&rm::ImgProdCons::Detect, &pro);
    std::thread energyThread(&rm::ImgProdCons::Energy, &pro);
    std::thread feedbackThread(&rm::ImgProdCons::Feedback, &pro);

    produceThread.join();
    detectThread.join();
    energyThread.join();
    feedbackThread.join();

    return 0;
}
