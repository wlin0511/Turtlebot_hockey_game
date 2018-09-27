/*
 * node.h
 *
 *  Created on: 30 Dec 2016
 *      Author: Kris
 *  Modified by Dimitar 05 Jan 2017
 */

#ifndef NODE_H_
#define NODE_H_

#include "action.h"
#include <vector>
#include <kdl/frames.hpp>

class Node{
	std::vector<int> m_viNetIndex;	// Index of this node in the network
	Action m_Action;			// action to this node
	double m_dCost;				// cost that take the action to the state
	double m_dHeuristic;		// heuristic value of this state
    KDL::Frame2 Faw_n_;         // trafo node wrt to active window
public:
	Node();
    Node(std::vector<int> net_index, Action act, double c, double h, KDL::Frame2 Faw_n);
	~Node();
	std::vector<int> getSuccessorState();
	Action getAction();
	double getCost();
	double getHeuristic();
	void setNumChildren(int num_children);
    KDL::Frame2 getFrame();
    std::vector<int> getNetIndex();
};

#endif /* NODE_H_ */
