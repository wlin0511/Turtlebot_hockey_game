/*
 * cction.cpp
 *
 *  Created on: 30 Dec 2016
 *      Author: Kris
 */

#include "path_planning/action.h"

Action::Action():x_(0), y_(0), theta_(0){
}

Action::Action(double x, double y, double theta){
    x_ = x;
    y_ = y;
    theta_ = theta;
}

Action::~Action(){
}

double Action::x(){
    return x_;
}

double Action::y(){
    return y_;
}

double Action::theta(){
    return theta_;
}
