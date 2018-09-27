/*
 * action.h
 *
 *  Created on: 21 Dec 2016
 *      Author: Kris
 */

#ifndef ACTION_H_
#define ACTION_H_

class Action{
private:
    double x_;           // x translation
    double y_;           // y translation
    double theta_;       // rotation
public:
	Action();
    Action(double x, double y, double theta);
	~Action();
    double x();
    double y();
    double theta();
};

#endif /* ACTION_H_ */
