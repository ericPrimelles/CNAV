#include <iostream>
#include <vector>
#include <math.h>
#include "Environment.h"
#include "MADDPG.h"

using std::cout, std::endl;

void updateVisualize(Environment* env){
    for (size_t i = 0; i < env->getNAgents(); i++)
    {
        cout << " " << env->getAgentPos(i);
    }
    cout << endl;
    
}
int main(int argc, char const *argv[])
{
    
    // Seting up a circle scenario for debug
    std::vector<RVO::Vector2> positions, goals;
    
    float x, y;
    RVO::Vector2 placeholder;
    for (size_t i = 0; i < 250; i++)
    {
        x = std::cos(i * 2.0f * 3.1416 / 250.0f);
        y = std::sin(i * 2.0f * 3.1416 / 250.0f);
        placeholder = RVO::Vector2(x, y);
        //cout << placeholder.x() << endl;
        positions.push_back(placeholder);
        goals.push_back(-positions[i]);
    }
    
    Environment env (250, 0.25f, 15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f, positions, goals);
    //env.sample();
    std::vector<int64_t> hdims = {32, 32, 32};

  
    MADDPG  program(&env, 4, 2, {32, 16, 8},  6, 1, {32, 16, 8}, 250, 0);
    program.Train();
    return 0;    
}
