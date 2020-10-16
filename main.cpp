//
// Created by luojunhui on 1/28/20.
//
#include<thread>
#include "mythread.hpp"
#include "preoptions.h"

using namespace std;
using namespace rm;

int main(int argc, char** argv)
{
    PreOptions(argc, argv);
    rm::ImgProdCons imgProdCons;

    imgProdCons.Init();

    std::thread produceThread(&rm::ImgProdCons::Produce, &imgProdCons);
    std::thread consumeThread(&rm::ImgProdCons::Consume, &imgProdCons);
    std::thread senseThread(&rm::ImgProdCons::Feedback, &imgProdCons);

    produceThread.join();
    consumeThread.join();
    senseThread.join();
    return 0;
}


