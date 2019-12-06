//
// Created by yurii on 06/12/2019.
//

#include "IdGenerator.h"

int IdGenerator::getId()
{
    if (this->isFree.empty()) {
        this->isFree.push_back(false);
        return 0;
    }

    for (int i = 0; i < this->isFree.size(); i++) {
        if (isFree[i] == true) {
            isFree[i] = false;
            return i;
        }
    }

    isFree.push_back(false);
    return isFree.size() - 1;
}

void IdGenerator::release(int id)
{
    if (id == this->isFree.size() - 1) {
        isFree.pop_back();
        return;
    }

    isFree[id] = false;
}
