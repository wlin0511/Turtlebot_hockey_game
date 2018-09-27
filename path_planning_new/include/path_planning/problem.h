/*
 * problem.h
 *
 *  Created on: 31 Dec 2016
 *      Author: Kris
 */

#ifndef PROBLEM_H_
#define PROBLEM_H_

#include <map>
#include <vector>

#include "node.h"

class Problem{
    bool goal_is_found_;
	std::vector<int> m_viStartState;
	std::vector<int> m_viGoalState;
	std::map<std::vector<int>, std::vector<Node> > m_mProblem;
	std::map<std::vector<int>, Node> m_mStateNodeMap;
	std::map<std::vector<int>, std::vector<int> > m_mChildParentMap;
	std::vector<std::vector<int> > m_vLastLayerStates;
	std::vector<std::vector<std::vector<int> > > m_v3iNet;

public:
	Problem();
	~Problem();
	void eraseProblem();
    void resizeNet(int num_layer);
	std::vector<int> getStartState();
	std::vector<Node> getSuccessors(std::vector<int> state);
	void setStartState(std::vector<int> start_state);
	void setGoalState(std::vector<int> goal_state);
	std::vector<int> getGoalState();
	void setSuccessors(std::vector<int> state, std::vector<Node> );
	bool isGoalState(std::vector<int> state);
    void setParent(std::vector<int> child_state, std::vector<int> parent_state);
	std::vector<int> getParent(std::vector<int> parent);
    void setStateNodeMap(std::vector<int> state, Node node);
	Node getNode(std::vector<int> state);
	void setLastLayerStateVector(std::vector<std::vector<int> > v_layer_state);
	std::vector<std::vector<int> > getLastLayerStateVector();
	void addNodeStateToLayer(int layer_num, std::vector<int> node_state);
    std::vector<std::vector <int> > getLayer(int layer_number);
    bool goalFound();
    void setGoalFound();
};

#endif /* PROBLEM_H_ */
