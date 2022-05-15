#include <iostream>
#include <vector>
#include <math.h>
#include "Environment.h"
#include "MADDPG.h"
#include "definitions.h"
using std::cout, std::endl;

Environment *setupEnvironment()
{

    if (scenario == 2) // Circle Scenario
    {

        for (int i = 0; i < Agents; ++i)
        {
            positions.push_back(float(Agents) *
                          RVO::Vector2(std::cos(i * 2.0f * M_PI / float(Agents)) + (std::rand() * 0.01f / (float)RAND_MAX), std::sin(i * 2.0f * M_PI / float(Agents)) + (std::rand() * 0.01f / (float)RAND_MAX)));

            goals.push_back(-positions[positions.end()]);
            //sim->setAgentGoal(i, -sim->getAgentPosition(i));
        }
    }
}

int main(int argc, char const *argv[])
{

    Environment env(250, 0.25f, 15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f, positions, goals);
    // env.sample();
    std::vector<int64_t> hdims = {32, 32, 32};

    MADDPG program(&env, 4, 2, {32, 16, 8}, 6, 1, {32, 16, 8}, 250, 0);
    program.Train();
    return 0;
}
