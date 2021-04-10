#include <thread>
#include "MyThread.hpp"
#include "preoptions.h"

using namespace rm;
using namespace cv;
using namespace std;

pthread_t producePThreadHandler;
pthread_t detectPThreadHandler;
pthread_t energyPThreadHandler;
pthread_t feedbackPThreadHandler;


int main(int argc, char** argv)
{
    PreOptions(argc,argv);

    ImgProdCons pro;
    pro.Init();

    std::thread produceThread(&rm::ImgProdCons::Produce, &pro);
    producePThreadHandler = produceThread.native_handle();

    std::thread detectThread(&rm::ImgProdCons::Detect, &pro);
    detectPThreadHandler = detectThread.native_handle();

    std::thread energyThread(&rm::ImgProdCons::Energy, &pro);
    energyPThreadHandler = energyThread.native_handle();

    std::thread feedbackThread(&rm::ImgProdCons::Feedback, &pro);
    feedbackPThreadHandler = feedbackThread.native_handle();

    produceThread.join();
    detectThread.join();
    energyThread.join();
    feedbackThread.join();

    return 0;
}
