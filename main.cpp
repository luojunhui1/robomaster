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

#if SAVE_VIDEO == 1
    int video_save_count;
#endif

int main(int argc, char** argv)
{
    PreOptions(argc,argv);

#if SAVE_VIDEO == 1
    std::ifstream fileVideoCountRead("../Log/video_count_file.txt", ios::in);
    if(!fileVideoCountRead.is_open())
    {
        LOGE("VIDEO SAVE FAILED\n");
        video_save_count = 0;
    }
    else
    {
        fileVideoCountRead >> video_save_count;
    }
    fileVideoCountRead.close();

    std::ofstream fileVideoCountWrite("../Log/video_count_file.txt", ios::out);
    if(!fileVideoCountWrite.is_open())
        LOGE("VIDEO SAVE FAILED\n");
    fileVideoCountWrite << video_save_count + 1 << endl;
    fileVideoCountWrite.close();
#endif

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
