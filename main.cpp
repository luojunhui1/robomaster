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
    preOptions(argc,argv);
    rm::ImgProdCons imgProdCons;

    imgProdCons.init();

    std::thread produceThread(&rm::ImgProdCons::produce, &imgProdCons);
    std::thread consumeThread(&rm::ImgProdCons::consume, &imgProdCons);
    std::thread senseThread(&rm::ImgProdCons::feedback, &imgProdCons);

    produceThread.join();
    consumeThread.join();
    senseThread.join();
    return 0;
}


