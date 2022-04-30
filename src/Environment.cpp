#include "Environment.h"
#include <math.h>

#if _OPENMP
#include <omp.h>
#endif

Environment::Environment(size_t n_agents, size_t time_step, float neighbor_dists, size_t max_neig, float time_horizon,
                         float time_horizon_obst, float radius, float max_speed,std::vector<RVO::Vector2> positions, 
                         std::vector<RVO::Vector2> goals)
{
    this->start = clock();
    this->end = 0;
    this->n_agents = n_agents;
    this->setAgentDefaults(neighbor_dists, max_neig, time_horizon, time_horizon_obst, radius, max_speed);
   
    
}

Environment::~Environment()
{
}

void Environment::setup(std::vector<RVO::Vector2> positions, std::vector<RVO::Vector2> goals){
    try
    {
        if(positions.size() == this->n_agents and goals.size() == this->n_agents){
            this->addAgents(positions);
            this->addAgentsGoals(goals);
        }
        else{
            throw dimensionError();
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

void Environment::setPrefferedVelocities(std::vector<torch::Tensor> actions)
{
    float x = 0.0f, y = 0.0f;
    RVO::Vector2 velPlaceholder;
#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (size_t i = 0; i < this->getNumAgents(); i++)
    {
        x = actions[i][0].item<float>();
        y = actions[i][1].item<float>();

        velPlaceholder = RVO::Vector2(x, y);
        if (RVO::absSq(velPlaceholder) > 1.0f)
        {
            velPlaceholder = RVO::normalize(velPlaceholder);
        }
        this->setAgentPrefVelocity(i, velPlaceholder);
    }
}

torch::Tensor Environment::step(std::vector<torch::Tensor> actions)
{

    // Executing Actions
    this->setPrefferedVelocities(actions);
    this->doStep();

    return this->calculateGlobalReward() + this->calculateInstantReward();
}


torch::Tensor Environment::sample(){
    std::vector<torch::Tensor> actions(this->n_agents);
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif 
    for (size_t i = 0; i < this->n_agents; i++)
    {
        actions[i] = torch::rand((int64_t)2, torch::dtype(torch::kFloat32));
    }
    
    return this->step(actions);
    //return this->step(torch::rand({this->n_agents, 2}, torch::dtype(torch::kFloat32)));
}
torch::Tensor Environment::getObservation()
{
    int64_t nAgents = this->getNumAgents();
    torch::Tensor observation = torch::zeros({nAgents, 2, 2}, torch::dtype(torch::kFloat32));
    std::vector<std::vector<float>> data(2);
    int64_t size;

    for (size_t i = 0; i < this->getNumAgents(); i++)
    {
        data = {{this->getAgentPosition(i).x(), this->getAgentPosition(i).y()}, {this->getAgentPrefVelocity(i).x(), this->getAgentPrefVelocity(i).y()}};
        size = data.size();
        observation[i] = torch::from_blob(data.data(), {1, size});
    }
    return observation;
}

torch::Tensor Environment::calculateGlobalReward()
{

    this->end = clock();
    this->time = ((float)this->end - this->start) / CLOCKS_PER_SEC;

    for (size_t i = 0; i < this->getNumAgents(); i++)
    {
        if (this->getAgentPosition(i) != this->getAgentGoal(i))
        {
            return torch::zeros(this->getNumAgents());
        }
    }

    return torch::full((int64_t)this->getNumAgents(), 100.0 - time);
}

torch::Tensor Environment::calculateInstantReward()
{
    std::vector<float> rewards(this->getNumAgents());
    float r_goal, r_coll_a, r_coll_obs = 0, r_cong = 0;
    int64_t size = rewards.size();
    float abs = 0;
    auto calcDist = [](RVO::Vector2 x, RVO::Vector2 y) -> float

    {
        return std::sqrt(std::pow(x.x() - y.x(), 2) + std::pow(x.y() - y.y(), 2));
    };
    for (size_t i = 0; i < this->getNumAgents(); i++)
    {
        r_goal = calcDist(this->getAgentPosition(i), this->getAgentGoal(i));
        abs = std::sqrt(this->getAgentVelocity(i) * this->getAgentVelocity(i));
        r_cong = -1 / (abs / this->getAgentMaxSpeed(i));
        for (size_t j = 0; j < this->getNumAgents() && j != i; j++)
        {
            if (calcDist(this->getAgentPosition(i), this->getAgentPosition(j)) <= 2 * this->getAgentRadius(i))
                r_coll_a += -3;
        }

        for (size_t j = 0; j < this->getAgentNumObstacleNeighbors(i); j++)
        {
            if (calcDist(this->getAgentPosition(i),
                         this->getObstacleVertex(this->getAgentObstacleNeighbor(i, j))) < this->getAgentRadius(i))
                r_coll_obs += -1;
        }

        rewards[i] = r_goal + r_coll_a + r_coll_obs + r_cong;
    }

    return torch::from_blob(rewards.data(), {1, size}, torch::dtype(torch::kFloat32));
}

bool Environment::isDone()
{
    for (size_t i = 0; i < this->getNumAgents(); i++)
    {
        if (this->getAgentPosition(i) != this->getAgentGoal(i))
            return false;
    }
    return true;
}

void Environment::addAgents(std::vector<RVO::Vector2> positions)
{

    for (auto pos : positions)
    {
        this->addAgent(pos);
    }
}

void Environment::addAgentsGoals(std::vector<RVO::Vector2> goals){
    for (size_t i = 0; i < n_agents; i++)
    {
        this->setAgentGoal(i, goals[i]);
    }
    
}
void Environment::reset()
{
    this->reset();
    this->start = clock();
    this->end = 0;
}