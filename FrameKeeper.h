//
// Created by yurii on 03/12/2019.
//

#ifndef TRACKING_FRAMEKEEPER_H
#define TRACKING_FRAMEKEEPER_H

#include "Tracker.h"
#include <vector>

using namespace std;
/* common interface for logging frames in Tracker.h:
 * Is bool logging is enable, each frame of sequence is logged using this class
 *
 * It could be mock object, that just send frame to trash, or could be object that keeps only 5 last frames, or keeps all, or keeps every second frame */

/* Это нужный класс, но только я решил, что не должно быть зависимости Трекера от ФрэймКипера, обязанности трекера -- это просто предоставить множество хороших особых точек,
 * для этого ему нужно хранить только предыдущий "кадр"
 * Сохранять кадры для дальнейшей оптимизации в его обязанности не входит, поэтому ФрэймКипер будет вызываться из мэйна, а трекер ничего знать про него не будет */

/* МОжно сделать декоратор над треккингом который логгирует точки */

class FrameKeeper {
public:
    void push_back();
private:
//    vector<Tracker::RegularFrame> frames;
};


#endif //TRACKING_FRAMEKEEPER_H
