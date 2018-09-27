/*
 * problem.cpp
 *
 *  Created on: 31 Dec 2016
 *      Author: Kris
 */

#include "path_planning/problem.h"

Problem::Problem(){
}

Problem::~Problem(){
}

void Problem::eraseProblem(){
    goal_is_found_ =false;
	m_viStartState.clear();
	m_viGoalState.clear();
	m_mProblem.clear();
	m_mStateNodeMap.clear();
	m_mChildParentMap.clear();
	m_vLastLayerStates.clear();
	m_v3iNet.clear();
}

void Problem::resizeNet(int num_layer){
    m_v3iNet.resize(num_layer);
}

std::vector<int> Problem::getStartState(){
	return m_viStartState;
}

std::vector<Node> Problem::getSuccessors(std::vector<int> state){
	return m_mProblem[state];
}

void Problem::setStartState(std::vector<int> start_state){
	m_viStartState = start_state;
}

void Problem::setGoalState(std::vector<int> goal_state){
	m_viGoalState = goal_state;
}

std::vector<int> Problem::getGoalState(){
	return m_viGoalState;
}

void Problem::setSuccessors(std::vector<int> state, std::vector<Node> v_node){
	m_mProblem[state] = v_node;
}

bool Problem::isGoalState(std::vector<int> state){
	return (m_viGoalState==state);
}

void Problem::setParent(std::vector<int> child_state, std::vector<int> parent_state){
    m_mChildParentMap[child_state] = parent_state;
}

std::vector<int> Problem::getParent(std::vector<int> child){
	return m_mChildParentMap[child];
}

void Problem::setStateNodeMap(std::vector<int> state, Node node){
    m_mStateNodeMap[state] = node;
}

Node Problem::getNode(std::vector<int> state){
	return m_mStateNodeMap[state];
}

void Problem::setLastLayerStateVector(std::vector<std::vector<int> > v_layer_state){
	m_vLastLayerStates = v_layer_state;
}
std::vector<std::vector<int> > Problem::getLastLayerStateVector(){
	return m_vLastLayerStates;
}

void Problem::addNodeStateToLayer(int layer_number, std::vector<int> node_state){
	m_v3iNet[layer_number].push_back(node_state);
}

std::vector<std::vector <int> >  Problem::getLayer(int layer_number){
    return m_v3iNet[layer_number];
}

bool Problem::goalFound(){
    return goal_is_found_;
}

void Problem::setGoalFound(){
    goal_is_found_ = true;
}
