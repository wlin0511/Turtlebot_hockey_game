/*
 * node.cpp
 *
 *  Created on: 30 Dec 2016
 *      Author: Kris
 */

#include "path_planning/node.h"

Node::Node():m_dCost(0),m_dHeuristic(0), m_Action(Action()), Faw_n_(KDL::Frame2::Identity()){

}

Node::Node(std::vector<int> net_index, Action act, double c, double h, KDL::Frame2 Faw_n){
	m_viNetIndex = net_index;
	m_Action = act;
	m_dCost = c;
	m_dHeuristic = h;
    Faw_n_ = Faw_n;
}

Node::~Node(){
}

std::vector<int> Node::getSuccessorState(){
	return m_viNetIndex;
}

Action Node::getAction(){
	return m_Action;
}

double Node::getCost(){
	return m_dCost;
}

double Node::getHeuristic(){
	return m_dHeuristic;
}

KDL::Frame2 Node::getFrame(){
    return Faw_n_;
}

std::vector<int> Node::getNetIndex(){
    return m_viNetIndex;
}
