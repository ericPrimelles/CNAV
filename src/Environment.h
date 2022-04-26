#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#pragma once

#include "RVO.h"
#include "vector"
#include "torch/torch.h"
#include "chrono"

/*
* \brief Environment class. Extension of RVOSimulator class to following the perspective of a RL Environment
*/
class Environment : RVO::RVOSimulator
{
public:
    Environment(size_t n_agents);
    Environment(size_t n_agents, float timeStep, float neighborDist, size_t maxNeighbors,
					 float timeHorizon, float timeHorizonObst, float radius,
					 float maxSpeed, const RVO::Vector2 &velocity = RVO::Vector2());
    using RVOSimulator::RVOSimulator;
    ~Environment();
    /*
    * \brief Returns a Tensor with the position and Pref velocities of every agent in the environment
    */
    torch::Tensor getObservation();
    /*
    * \brief Execute an step and returns the reward asociated to every action.
    * \param actions Joint actions tensor.
    */
    torch::Tensor step(std::vector<torch::Tensor> actions);
    /*
    *   \brief  Add a set of agents to the environment (RVO Agents)
    *   \param positions Set of two-dimensional positions of agents to be settled
    */
    void addAgents(std::vector<RVO::Vector2> positions);
    /*
    *   \brief Returns true if all agents reach their respective goals, otherwise returns false
    */
    bool isDone();
    /*
    *   \brief  Resets the inner RVOSImulator and Restarts the timer
    */
    void reset();
private:

    /*
    * \brief Returns a global reward calculated from time spended when the environment is done
    */
    torch::Tensor calculateGlobalReward();
    /*
    * \brief Returns a local reward based on the local actions of the agents.
    */
    torch::Tensor calculateInstantReward();
    clock_t start, end;
    float time= 0.0f;
    size_t n_agents;
};

#endif