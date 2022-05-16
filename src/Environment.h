#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#pragma once

#include "RVO.h"
#include "vector"
#include "torch/torch.h"
#include "chrono"
#include <exception>

struct dimensionError: public std::exception{
    const char* what()const throw(){
        return "Position or goal vectors size don't match with number of agents in environment.";
    }
};

/*
* \brief Environment class. Extension of RVOSimulator class to following the perspective of a RL Environment
*/
class Environment : public RVO::RVOSimulator
{
public:
    Environment(size_t n_agents, float timestep,float neighbor_dists, size_t max_neig, float time_horizont,
                         float time_horizont_obst, float radius, float max_speeds, std::vector<RVO::Vector2> positions,
                         std::vector<RVO::Vector2> goals);
    
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
    *   \brief Returns true if all agents reach their respective goals, otherwise returns false
    */
    bool isDone();
    /*
    *   \brief  Resets the inner RVOSImulator and Restarts the timer
    */
    void reset();

    torch::Tensor sample();
    void setup(std::vector<RVO::Vector2> positions, std::vector<RVO::Vector2> goals);
    // Inline funtions;
    inline size_t getNAgents() {return this->getNumAgents();}
    inline RVO::Vector2 getAgentPos(size_t i) {return this->getAgentPosition(i);}
private:
    /*
    *   \brief  Add a set of agents to the environment (RVO Agents)
    *   \param positions Set of two-dimensional positions of agents to be settled
    */
    void addAgents(std::vector<RVO::Vector2> positions);
    void addAgentsGoals(std::vector<RVO::Vector2> goals);
    /*
    * \brief Returns a global reward calculated from time spended when the environment is done
    */
    torch::Tensor calculateGlobalReward();
    void setPrefferedVelocities(std::vector<torch::Tensor> velocities);
    /*
    * \brief Returns a local reward based on the local actions of the agents.
    */
    torch::Tensor calculateInstantReward();
    clock_t start, end;
    float time= 0.0f;
    size_t n_agents;
    std::vector<RVO::Vector2> positions, goals;
    
};

#endif