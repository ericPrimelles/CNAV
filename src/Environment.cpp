#include "Environment.h"
#include <math.h>

Environment::Environment(size_t n_agents)
{
    this->start = clock();
    this->end = 0;
    this->n_agents = n_agents;
}

Environment::Environment(size_t n_agents, float timeStep, float neighborDist, size_t maxNeighbors,
                         float timeHorizon, float timeHorizonObst, float radius,
                         float maxSpeed, const RVO::Vector2 &velocity)
{
    this->start = clock();
    this->end = 0;
    this->n_agents = n_agents;
}

Environment::~Environment()
{
}

torch::Tensor Environment::step(std::vector<torch::Tensor> actions)
{

    // Executing Actions
    RVO::Vector2 velPlaceholder;
    float x = 0.0f, y = 0.0f;
    for (size_t i = 0; i < this->getNumAgents(); i++)
    {
        x = actions[i][0].item<float>();
        y = actions[i][1].item<float>();

        velPlaceholder = RVO::Vector2(x, y);

        this->setAgentPrefVelocity(i, velPlaceholder);
    }
    this->doStep();

    return this->calculateGlobalReward() + this->calculateInstantReward();
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
void Environment::reset()
{
    this->reset();
    this->start = clock();
    this->end = 0;
}