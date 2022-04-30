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
    cout << "Positions size: " << positions.size() << endl;
    Environment* env = new Environment(250, 0.25f, 15.0f, 10, 10.0f, 10.0f, 1.5f, 2.0f, positions, goals);
    cout << env->getNAgents() << endl;
    for (size_t i = 0; i < 100; i++)
    {
        env->sample();
        updateVisualize(env);
    }
    
    return 0;    
}
