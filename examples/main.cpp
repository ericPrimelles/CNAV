#include <iostream>
#include <vector>
#include <math.h>
#include <random>
#include <time.h>
#include "Environment.h"
#include "MADDPG.h"
#include "RVO.h"

using std::cout, std::endl;

// Environments params
size_t Agents = 50;
float timestep = 0.25f;
float neighbor_dist = 1.0f;
size_t max_neig = Agents;
float time_horizont = 10.0f;
float time_horizont_obst = 20.0f;
float radius = 2.0f;
float max_speed = 1.5f;

std::vector<RVO::Vector2> positions, goals;

int scenario = 0;

Environment *setupEnvironment()
{

    positions.reserve(Agents);
    goals.reserve(Agents);

    // Set Agent positions
    if (scenario == 2) // Circle Scenario

    {

        for (int i = 0; i < Agents; ++i)
        {
            positions.push_back(float(Agents) *
                                RVO::Vector2(std::cos(i * 2.0f * M_PI / float(Agents)) + (std::rand() * 0.01f / (float)RAND_MAX), std::sin(i * 2.0f * M_PI / float(Agents)) + (std::rand() * 0.01f / (float)RAND_MAX)));

            goals.push_back(-positions[i]);
        }
    }
    if (scenario == 3)
    { // Bidirectional flow

        positions.push_back(RVO::Vector2(-20.0f, 30.0f));
        positions.push_back(RVO::Vector2(-17.0f, 30.0f));
        positions.push_back(RVO::Vector2(-14.0f, 30.0f));
        positions.push_back(RVO::Vector2(-20.0f, 29.0f));
        positions.push_back(RVO::Vector2(-17.0f, 29.0f));

        positions.push_back(RVO::Vector2(-14.0f, 29.0f));
        positions.push_back(RVO::Vector2(-20.0f, 28.0f));
        positions.push_back(RVO::Vector2(-17.0f, 28.0f));
        positions.push_back(RVO::Vector2(-14.0f, 28.0f));
        positions.push_back(RVO::Vector2(20.0f, 30.0f));

        positions.push_back(RVO::Vector2(17.0f, 30.0f));
        positions.push_back(RVO::Vector2(14.0f, 30.0f));
        positions.push_back(RVO::Vector2(20.0f, 29.0f));
        positions.push_back(RVO::Vector2(17.0f, 29.0f));
        positions.push_back(RVO::Vector2(14.0f, 29.0f));

        positions.push_back(RVO::Vector2(20.0f, 28.0f));
        positions.push_back(RVO::Vector2(17.0f, 28.0f));
        positions.push_back(RVO::Vector2(14.0f, 28.0f));

        for (size_t i = 0; i < Agents; i++)
        {
            goals.push_back(RVO::Vector2(-positions[i].x(), positions[i].y()));
        }
    }
    if (scenario == 4)

    {
        positions.push_back(RVO::Vector2(-30.0f, 0.5f));
        positions.push_back(RVO::Vector2(-26.0f, 0.5f));
        positions.push_back(RVO::Vector2(-22.0f, 0.5f));
        positions.push_back(RVO::Vector2(-18.0f, 0.5f));
        positions.push_back(RVO::Vector2(-14.0f, 0.5f));

        positions.push_back(RVO::Vector2(-30.0f, -0.5f));
        positions.push_back(RVO::Vector2(-26.0f, -0.5f));
        positions.push_back(RVO::Vector2(-22.0f, -0.5f));
        positions.push_back(RVO::Vector2(-18.0f, -0.5f));
        positions.push_back(RVO::Vector2(-14.0f, -0.5f));

        positions.push_back(RVO::Vector2(-30.0f, -1.5f));
        positions.push_back(RVO::Vector2(-26.0f, -1.5f));
        positions.push_back(RVO::Vector2(-22.0f, -1.5f));
        positions.push_back(RVO::Vector2(-18.0f, -1.5f));
        positions.push_back(RVO::Vector2(-14.0f, -1.5f));

        positions.push_back(RVO::Vector2(-30.0f, 1.5f));
        positions.push_back(RVO::Vector2(-26.0f, 1.5f));
        positions.push_back(RVO::Vector2(-22.0f, 1.5f));
        positions.push_back(RVO::Vector2(-18.0f, 1.5f));
        positions.push_back(RVO::Vector2(-14.0f, 1.5f));

        positions.push_back(RVO::Vector2(30.0f, 0.5f));
        positions.push_back(RVO::Vector2(26.0f, 0.5f));
        positions.push_back(RVO::Vector2(22.0f, 0.5f));
        positions.push_back(RVO::Vector2(18.0f, 0.5f));
        positions.push_back(RVO::Vector2(14.0f, 0.5f));

        positions.push_back(RVO::Vector2(30.0f, -0.5f));
        positions.push_back(RVO::Vector2(26.0f, -0.5f));
        positions.push_back(RVO::Vector2(22.0f, -0.5f));
        positions.push_back(RVO::Vector2(18.0f, -0.5f));
        positions.push_back(RVO::Vector2(14.0f, -0.5f));

        positions.push_back(RVO::Vector2(30.0f, -1.5f));
        positions.push_back(RVO::Vector2(26.0f, -1.5f));
        positions.push_back(RVO::Vector2(22.0f, -1.5f));
        positions.push_back(RVO::Vector2(18.0f, -1.5f));
        positions.push_back(RVO::Vector2(14.0f, -1.5f));

        positions.push_back(RVO::Vector2(30.0f, 1.5f));
        positions.push_back(RVO::Vector2(26.0f, 1.5f));
        positions.push_back(RVO::Vector2(22.0f, 1.5f));
        positions.push_back(RVO::Vector2(18.0f, 1.5f));
        positions.push_back(RVO::Vector2(14.0f, 1.5f));

        positions.push_back(RVO::Vector2(0.5f, 30.0f));
        positions.push_back(RVO::Vector2(0.5f, 26.0f));
        positions.push_back(RVO::Vector2(0.5f, 22.0f));
        positions.push_back(RVO::Vector2(0.5f, 18.0f));
        positions.push_back(RVO::Vector2(0.5f, 14.0f));

        positions.push_back(RVO::Vector2(-0.5f, 30.0f));
        positions.push_back(RVO::Vector2(-0.5f, 26.0f));
        positions.push_back(RVO::Vector2(-0.5f, 22.0f));
        positions.push_back(RVO::Vector2(-0.5f, 18.0f));
        positions.push_back(RVO::Vector2(-0.5f, 14.0f));

        positions.push_back(RVO::Vector2(-1.5f, 30.0f));
        positions.push_back(RVO::Vector2(-1.5f, 26.0f));
        positions.push_back(RVO::Vector2(-1.5f, 22.0f));
        positions.push_back(RVO::Vector2(-1.5f, 18.0f));
        positions.push_back(RVO::Vector2(-1.5f, 14.0f));

        positions.push_back(RVO::Vector2(1.5f, 30.0f));
        positions.push_back(RVO::Vector2(1.5f, 26.0f));
        positions.push_back(RVO::Vector2(1.5f, 22.0f));
        positions.push_back(RVO::Vector2(1.5f, 18.0f));
        positions.push_back(RVO::Vector2(1.5f, 14.0f));

        positions.push_back(RVO::Vector2(0.5f, -30.0f));
        positions.push_back(RVO::Vector2(0.5f, -26.0f));
        positions.push_back(RVO::Vector2(0.5f, -22.0f));
        positions.push_back(RVO::Vector2(0.5f, -18.0f));
        positions.push_back(RVO::Vector2(0.5f, -14.0f));

        positions.push_back(RVO::Vector2(-0.5f, -30.0f));
        positions.push_back(RVO::Vector2(-0.5f, -26.0f));
        positions.push_back(RVO::Vector2(-0.5f, -22.0f));
        positions.push_back(RVO::Vector2(-0.5f, -18.0f));
        positions.push_back(RVO::Vector2(-0.5f, -14.0f));

        positions.push_back(RVO::Vector2(-1.5f, -30.0f));
        positions.push_back(RVO::Vector2(-1.5f, -26.0f));
        positions.push_back(RVO::Vector2(-1.5f, -22.0f));
        positions.push_back(RVO::Vector2(-1.5f, -18.0f));
        positions.push_back(RVO::Vector2(-1.5f, -14.0f));

        positions.push_back(RVO::Vector2(1.5f, -30.0f));
        positions.push_back(RVO::Vector2(1.5f, -26.0f));
        positions.push_back(RVO::Vector2(1.5f, -22.0f));
        positions.push_back(RVO::Vector2(1.5f, -18.0f));
        positions.push_back(RVO::Vector2(1.5f, -14.0f));

        for (int i = 0; i < Agents; ++i)
        {

            if (i < (Agents / 4)) // 10
            {
                goals.push_back(RVO::Vector2(10, positions[i].y()));
            }

            if ((i >= (Agents / 4)) && (i < (Agents / 2)))
            {
                goals.push_back(RVO::Vector2(-10, positions[i].y()));
            }

            if ((i >= (Agents / 2)) && (i < (Agents / ((float)4 / (float)3))))
            {
                goals.push_back(RVO::Vector2(positions[i].x(), -10));
            }

            if ((i >= (Agents / ((float)4 / (float)3))) && (i < Agents))
            {
                goals.push_back(RVO::Vector2(positions[i].x(), 10));
            }
        }
    }
    if (scenario == 5 || scenario == 1)
    {
        srand(time(NULL));
        for (size_t i = 0; i < Agents; i++)
        {
            positions.push_back(RVO::Vector2(rand() % 25, rand() % 25));
            goals.push_back(RVO::Vector2(rand() % 25, rand() % 25));
        }
    }
    if (scenario == 6)
    {
        positions.push_back(RVO::Vector2(-500.0f, -10.0f));
        positions.push_back(RVO::Vector2(-400.0f, -10.0f));
        positions.push_back(RVO::Vector2(-300.0f, -10.0f));
        positions.push_back(RVO::Vector2(-200.0f, -10.0f));
        positions.push_back(RVO::Vector2(-100.0f, -10.0f));

        positions.push_back(RVO::Vector2(-500.0f, -9.0f));
        positions.push_back(RVO::Vector2(-4.0f, -9.0f));
        positions.push_back(RVO::Vector2(-3.0f, -9.0f));
        positions.push_back(RVO::Vector2(-2.0f, -9.0f));
        positions.push_back(RVO::Vector2(-1.0f, -9.0f));

        positions.push_back(RVO::Vector2(-5.0f, -8.0f));
        positions.push_back(RVO::Vector2(-4.0f, -8.0f));
        positions.push_back(RVO::Vector2(-3.0f, -8.0f));
        positions.push_back(RVO::Vector2(-2.0f, -8.0f));
        positions.push_back(RVO::Vector2(-1.0f, -8.0f));

        positions.push_back(RVO::Vector2(-3.0f, 3.0f));

        for (int i = 0; i < Agents - 1; ++i)
        {
            // goals.push_back(RVO::Vector2(positions[i].x(),24+sim->getAgentPosition(i).y() ) );
            goals.push_back(RVO::Vector2(positions[i].x(), 24 + positions[i].y()));
        }

        goals.push_back(RVO::Vector2(positions[15].x(), -20));
    }

    if (scenario == 7)
    {

        positions.push_back(RVO::Vector2(-21.0f, -0.0f));
        positions.push_back(RVO::Vector2(21.0f, 0.0f));

        positions.push_back(RVO::Vector2(-23.0f, 0.0f));
        positions.push_back(RVO::Vector2(23.0f, 0.0f));

        positions.push_back(RVO::Vector2(-25.0f, 0.0f));
        positions.push_back(RVO::Vector2(25.0f, 0.0f));

        positions.push_back(RVO::Vector2(-27.0f, 0.0f));
        positions.push_back(RVO::Vector2(27.0f, 0.0f));

        positions.push_back(RVO::Vector2(-29.0f, 0.0f));
        positions.push_back(RVO::Vector2(29.0f, 0.0f));

        goals.push_back(RVO::Vector2(21.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(-21.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(21.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(-21.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(21.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(-21.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(21.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(-21.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(21.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(-21.0f, positions[0].y()));
    }
    if (scenario == 8)
    {
        positions.push_back(RVO::Vector2(-10.0f, 0.0f));
        positions.push_back(RVO::Vector2(-15.0f, 5.0f));
        positions.push_back(RVO::Vector2(-12.0f, -1.0f));
        positions.push_back(RVO::Vector2(-15.0f, 2.5f));
        positions.push_back(RVO::Vector2(-13.0f, -2.5f));

        positions.push_back(RVO::Vector2(-7.0f, 0.0f));
        positions.push_back(RVO::Vector2(-11.0f, 5.0f));
        positions.push_back(RVO::Vector2(-10.0f, -1.0f));
        positions.push_back(RVO::Vector2(-9.0f, 2.5f));
        positions.push_back(RVO::Vector2(-11.0f, -2.5f));

        goals.push_back(RVO::Vector2(15.0f, positions[0].y()));

        goals.push_back(RVO::Vector2(15.0f, positions[1].y()));

        goals.push_back(RVO::Vector2(15.0f, positions[2].y()));

        goals.push_back(RVO::Vector2(15.0f, positions[3].y()));

        goals.push_back(RVO::Vector2(15.0f, positions[4].y()));

        goals.push_back(RVO::Vector2(15.0f, positions[5].y()));

        goals.push_back(RVO::Vector2(15.0f, positions[6].y()));

        goals.push_back(RVO::Vector2(15.0f, positions[7].y()));

        goals.push_back(RVO::Vector2(15.0f, positions[8].y()));

        goals.push_back(RVO::Vector2(15.0f, positions[9].y()));
    }

    if (scenario == 9)
    {
        positions.push_back(RVO::Vector2(-9.5f, 1.5f));
        positions.push_back(RVO::Vector2(-9.5f, 2.5f));
        positions.push_back(RVO::Vector2(-9.5f, 3.5f));
        positions.push_back(RVO::Vector2(-9.5f, 4.5f));
        for (int i = 0; i < Agents; ++i)
        {
            goals.push_back(RVO::Vector2(-10.0, 0.0));
        }
    }

    if (scenario == 10)
    {
        positions.push_back(RVO::Vector2(3.0f, 0.5f));
        positions.push_back(RVO::Vector2(27.0f, 0.5f));

        goals.push_back(RVO::Vector2(27.0f, 0.5f));
        goals.push_back(RVO::Vector2(3.0f, 0.5f));

        positions.push_back(RVO::Vector2(3.0f, 3.5f));
        positions.push_back(RVO::Vector2(27.0f, 3.5f));

        goals.push_back(RVO::Vector2(27.0f, 3.5f));
        goals.push_back(RVO::Vector2(3.0f, 3.5f));

        positions.push_back(RVO::Vector2(3.0f, 6.5f));
        positions.push_back(RVO::Vector2(27.0f, 6.5f));

        goals.push_back(RVO::Vector2(27.0f, 6.5f));
        goals.push_back(RVO::Vector2(3.0f, 6.5f));

        positions.push_back(RVO::Vector2(3.0f, 9.5f));
        positions.push_back(RVO::Vector2(27.0f, 9.5f));

        goals.push_back(RVO::Vector2(27.0f, 9.5f));
        goals.push_back(RVO::Vector2(3.0f, 9.5f));
    }

    Environment *sim = new Environment(Agents, timestep, neighbor_dist, max_neig, time_horizont, time_horizont_obst, radius, max_speed, positions, goals);

    std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4, obstacle5, obstacle6, obstacle7, obstacle8, obstacle9, obstacle10, obstacle11, obstacle12, obstacle13;

    if (scenario == 3)
    {
        obstacle1.push_back(RVO::Vector2(200.0f, 31.0f));
        obstacle1.push_back(RVO::Vector2(-200.0f, 31.0f));
        obstacle1.push_back(RVO::Vector2(-200.0f, 30.6f));
        obstacle1.push_back(RVO::Vector2(200.0f, 30.6f));

        obstacle2.push_back(RVO::Vector2(200.0f, 27.4f));
        obstacle2.push_back(RVO::Vector2(-200.0f, 27.4f));
        obstacle2.push_back(RVO::Vector2(-200.0f, 27.0f));
        obstacle2.push_back(RVO::Vector2(200.0f, 27.0f));

        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);

        sim->processObstacles();
    }

    if (scenario == 4)
    {

        obstacle1.push_back(RVO::Vector2(200.0f, 3.0f));
        obstacle1.push_back(RVO::Vector2(2.0f, 3.0f));
        obstacle1.push_back(RVO::Vector2(2.0f, 2.0f));
        obstacle1.push_back(RVO::Vector2(200.0f, 2.0f));

        obstacle2.push_back(RVO::Vector2(200.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(2.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(2.0f, -3.0f));
        obstacle2.push_back(RVO::Vector2(200.0f, -3.0f));

        obstacle3.push_back(RVO::Vector2(-2.0f, 3.0f));
        obstacle3.push_back(RVO::Vector2(-200.0f, 3.0f));
        obstacle3.push_back(RVO::Vector2(-200.0f, 2.0f));
        obstacle3.push_back(RVO::Vector2(-2.0f, 2.0f));

        obstacle4.push_back(RVO::Vector2(-2.0f, -2.0f));
        obstacle4.push_back(RVO::Vector2(-200.0f, -2.0f));
        obstacle4.push_back(RVO::Vector2(-200.0f, -3.0f));
        obstacle4.push_back(RVO::Vector2(-2.0f, -3.0f));

        obstacle5.push_back(RVO::Vector2(3.0f, 200.0f));
        obstacle5.push_back(RVO::Vector2(2.0f, 200.0f));
        obstacle5.push_back(RVO::Vector2(2.0f, 3.0f));
        obstacle5.push_back(RVO::Vector2(3.0f, 3.0f));

        obstacle6.push_back(RVO::Vector2(-2.0f, 200.0f));
        obstacle6.push_back(RVO::Vector2(-3.0f, 200.0f));
        obstacle6.push_back(RVO::Vector2(-3.0f, 3.0f));
        obstacle6.push_back(RVO::Vector2(-2.0f, 3.0f));

        obstacle7.push_back(RVO::Vector2(3.0f, -3.0f));
        obstacle7.push_back(RVO::Vector2(2.0f, -3.0f));
        obstacle7.push_back(RVO::Vector2(2.0f, -200.0f));
        obstacle7.push_back(RVO::Vector2(3.0f, -200.0f));

        obstacle8.push_back(RVO::Vector2(-2.0f, -3.0f));
        obstacle8.push_back(RVO::Vector2(-3.0f, -3.0f));
        obstacle8.push_back(RVO::Vector2(-3.0f, -200.0f));
        obstacle8.push_back(RVO::Vector2(-2.0f, -200.0f));

        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);

        sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        sim->addObstacle(obstacle7);
        sim->addObstacle(obstacle8);

        sim->processObstacles();
    }

    if (scenario == 5)
    {

        obstacle1.push_back(RVO::Vector2(-10.0f, 350.0f));
        obstacle1.push_back(RVO::Vector2(-11.0f, 350.0f));
        obstacle1.push_back(RVO::Vector2(-11.0f, 0.7f));
        obstacle1.push_back(RVO::Vector2(-10.0f, 0.7f));

        obstacle2.push_back(RVO::Vector2(-10.0f, -0.7f));
        obstacle2.push_back(RVO::Vector2(-11.0f, -0.7f));
        obstacle2.push_back(RVO::Vector2(-11.0f, -150.0f));
        obstacle2.push_back(RVO::Vector2(-10.0f, -150.0f));

        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);

        sim->processObstacles();
    }

    if (scenario == 10)
    {
        obstacle1.push_back(RVO::Vector2(0.0f, 12.0f));
        obstacle1.push_back(RVO::Vector2(-2.0f, 12.0f));

        obstacle1.push_back(RVO::Vector2(-2.0f, -2.0f));
        obstacle1.push_back(RVO::Vector2(0.0f, -2.0f));

        obstacle2.push_back(RVO::Vector2(30.0f, 0.0f));
        obstacle2.push_back(RVO::Vector2(0.0f, 0.0f));

        obstacle2.push_back(RVO::Vector2(0.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(30.0f, -2.0f));

        obstacle3.push_back(RVO::Vector2(30.0f, 12.0f));
        obstacle3.push_back(RVO::Vector2(0.0f, 12.0f));

        obstacle3.push_back(RVO::Vector2(0.0f, 10.0f));
        obstacle3.push_back(RVO::Vector2(30.0f, 10.0f));

        obstacle4.push_back(RVO::Vector2(32.0f, 12.0f));
        obstacle4.push_back(RVO::Vector2(30.0f, 12.0f));

        obstacle4.push_back(RVO::Vector2(30.0f, -2.0f));
        obstacle4.push_back(RVO::Vector2(32.0f, -2.0f));

        obstacle5.push_back(RVO::Vector2(11.0f, 2.95f));
        obstacle5.push_back(RVO::Vector2(5.0f, 2.95f));

        obstacle5.push_back(RVO::Vector2(5.0f, 1.05f));
        obstacle5.push_back(RVO::Vector2(11.0f, 1.05f));

        obstacle6.push_back(RVO::Vector2(11.0f, 5.95f));
        obstacle6.push_back(RVO::Vector2(5.0f, 5.95f));

        obstacle6.push_back(RVO::Vector2(5.0f, 4.05f));
        obstacle6.push_back(RVO::Vector2(11.0f, 4.05f));

        obstacle7.push_back(RVO::Vector2(11.0f, 8.95f));
        obstacle7.push_back(RVO::Vector2(5.0f, 8.95f));

        obstacle7.push_back(RVO::Vector2(5.0f, 7.05f));
        obstacle7.push_back(RVO::Vector2(11.0f, 7.05f));

        obstacle8.push_back(RVO::Vector2(18.0f, 8.95f));
        obstacle8.push_back(RVO::Vector2(12.0f, 8.95f));

        obstacle8.push_back(RVO::Vector2(12.0f, 7.05f));
        obstacle8.push_back(RVO::Vector2(18.0f, 7.05f));

        obstacle9.push_back(RVO::Vector2(18.0f, 2.95f));
        obstacle9.push_back(RVO::Vector2(12.0f, 2.95f));

        obstacle9.push_back(RVO::Vector2(12.0f, 1.05f));
        obstacle9.push_back(RVO::Vector2(18.0f, 1.05f));

        obstacle10.push_back(RVO::Vector2(18.0f, 5.95f));
        obstacle10.push_back(RVO::Vector2(12.0f, 5.95f));

        obstacle10.push_back(RVO::Vector2(12.0f, 4.05f));
        obstacle10.push_back(RVO::Vector2(18.0f, 4.05f));

        obstacle11.push_back(RVO::Vector2(25.0f, 8.95f));
        obstacle11.push_back(RVO::Vector2(19.0f, 8.95f));

        obstacle11.push_back(RVO::Vector2(19.0f, 7.05f));
        obstacle11.push_back(RVO::Vector2(25.0f, 7.05f));

        obstacle12.push_back(RVO::Vector2(25.0f, 2.95f));
        obstacle12.push_back(RVO::Vector2(19.0f, 2.95f));

        obstacle12.push_back(RVO::Vector2(19.0f, 1.05f));
        obstacle12.push_back(RVO::Vector2(25.0f, 1.05f));

        obstacle13.push_back(RVO::Vector2(25.0f, 5.95f));
        obstacle13.push_back(RVO::Vector2(19.0f, 5.95f));

        obstacle13.push_back(RVO::Vector2(19.0f, 4.05f));
        obstacle13.push_back(RVO::Vector2(25.0f, 4.05f));

        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);

        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);

        sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);
        sim->addObstacle(obstacle7);

        sim->addObstacle(obstacle8);
        sim->addObstacle(obstacle9);
        sim->addObstacle(obstacle10);

        sim->addObstacle(obstacle11);
        sim->addObstacle(obstacle12);
        sim->addObstacle(obstacle13);

        sim->processObstacles();
    }

    if (scenario == 1)
    {

        obstacle1.push_back(RVO::Vector2(-1.5f, -0.5f));
        obstacle1.push_back(RVO::Vector2(-1.5f, -1.5f));
        obstacle1.push_back(RVO::Vector2(25 + 1.5f, -1.5f));
        obstacle1.push_back(RVO::Vector2(25 + 1.5f, -0.5f));

        obstacle2.push_back(RVO::Vector2(25 + 0.5f, 25 + 1.5f));
        obstacle2.push_back(RVO::Vector2(25 + 0.5f, -1.5f));
        obstacle2.push_back(RVO::Vector2(25 + 1.5f, -1.5f));
        obstacle2.push_back(RVO::Vector2(25 + 1.5f, 25 + 1.5f));

        obstacle3.push_back(RVO::Vector2(-1.5f, 25 + 1.5f));
        obstacle3.push_back(RVO::Vector2(-1.5f, 25 + 0.5f));
        obstacle3.push_back(RVO::Vector2(25 + 1.5f, 25 + 0.5f));
        obstacle3.push_back(RVO::Vector2(25 + 1.5f, 25 + 1.5f));

        obstacle4.push_back(RVO::Vector2(-1.5f, 25 + 1.5f));
        obstacle4.push_back(RVO::Vector2(-1.5f, -1.5f));
        obstacle4.push_back(RVO::Vector2(-0.5f, -1.5f));
        obstacle4.push_back(RVO::Vector2(-0.5f, 25 + 1.5f));

        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);

        sim->processObstacles();

        /*if(iteration>1)
        {
            getScenario();

        }

        if(iteration==1)
        {
            genScenario();

        }*/
    }

    if (scenario == 7)
    {
        obstacle1.push_back(RVO::Vector2(-10.0f, 2.0f));
        obstacle1.push_back(RVO::Vector2(-10.0f, 0.6f));
        obstacle1.push_back(RVO::Vector2(10.0f, 0.6f));
        obstacle1.push_back(RVO::Vector2(10.0f, 2.0f));

        obstacle2.push_back(RVO::Vector2(-10.0f, -0.6f));
        obstacle2.push_back(RVO::Vector2(-10.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(10.0f, -2.0f));
        obstacle2.push_back(RVO::Vector2(10.0f, -0.6f));

        obstacle3.push_back(RVO::Vector2(-100.0f, 100.0f));
        obstacle3.push_back(RVO::Vector2(-100.0f, 10.0f));
        obstacle3.push_back(RVO::Vector2(-10.0f, 0.6f));
        obstacle3.push_back(RVO::Vector2(-10.0f, 100.0f));

        obstacle4.push_back(RVO::Vector2(-100.0f, -10.0f));
        obstacle4.push_back(RVO::Vector2(-100.0f, -100.0f));
        obstacle4.push_back(RVO::Vector2(-10.0f, -100.0f));
        obstacle4.push_back(RVO::Vector2(-10.0f, -0.6f));

        obstacle5.push_back(RVO::Vector2(10.0f, 100.0f));
        obstacle5.push_back(RVO::Vector2(10.0f, 0.6f));
        obstacle5.push_back(RVO::Vector2(100.0f, 10.0f));
        obstacle5.push_back(RVO::Vector2(100.0f, 100.0f));

        obstacle6.push_back(RVO::Vector2(10.0f, -0.6f));
        obstacle6.push_back(RVO::Vector2(10.0f, -100.0f));
        obstacle6.push_back(RVO::Vector2(100.0f, -100.0f));
        obstacle6.push_back(RVO::Vector2(100.0f, -10.0f));

        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);

        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);

        sim->addObstacle(obstacle5);
        sim->addObstacle(obstacle6);

        sim->processObstacles();
    }

    if (scenario == 8)
    {

        obstacle1.push_back(RVO::Vector2(-1.0f, 0.5f));
        obstacle1.push_back(RVO::Vector2(-1.0f, -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, -1.5f));
        obstacle1.push_back(RVO::Vector2(1.0f, 0.5f));

        obstacle2.push_back(RVO::Vector2(3.0f, 3.5f));
        obstacle2.push_back(RVO::Vector2(3.0f, 1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 1.5f));
        obstacle2.push_back(RVO::Vector2(5.0f, 3.5f));

        obstacle3.push_back(RVO::Vector2(-4.5f, -2.0f));
        obstacle3.push_back(RVO::Vector2(-4.5f, -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -4.0f));
        obstacle3.push_back(RVO::Vector2(-2.5f, -2.0f));

        obstacle4.push_back(RVO::Vector2(-1.5f, 6.5f));
        obstacle4.push_back(RVO::Vector2(-1.5f, 4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 4.5f));
        obstacle4.push_back(RVO::Vector2(0.5f, 6.5f));

        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);
        sim->addObstacle(obstacle3);
        sim->addObstacle(obstacle4);
        sim->processObstacles();
    }

    if (scenario == 9)
    {

        obstacle1.push_back(RVO::Vector2(-10.0f, 350.0f));
        obstacle1.push_back(RVO::Vector2(-11.0f, 350.0f));
        obstacle1.push_back(RVO::Vector2(-11.0f, 0.7f));
        obstacle1.push_back(RVO::Vector2(-10.0f, 0.7f));

        obstacle2.push_back(RVO::Vector2(-10.0f, -0.7f));
        obstacle2.push_back(RVO::Vector2(-11.0f, -0.7f));
        obstacle2.push_back(RVO::Vector2(-11.0f, -150.0f));
        obstacle2.push_back(RVO::Vector2(-10.0f, -150.0f));

        sim->addObstacle(obstacle1);
        sim->addObstacle(obstacle2);

        sim->processObstacles();
    }

    return sim;
}

int main(int argc, char const *argv[])
{
    cout << argc << endl;
    if (argc > 4)
    {
        cout << "Too many flags" << endl;
        return -1;
    }
    else if (argc < 4)
    {
        cout << "Input the following flags:\n. Scenario: [1 - 10] \n. Testing: 0 - false, 1- True\n. Testing epochs: int";
        return -1;
    }

    scenario = atoi(argv[1]);
    switch (scenario)
    {
    case 1:
        Agents = 300;
        break;

    case 2:
        Agents = 128;
        break;

    case 3:
        Agents = 18;
        break;

    case 4:
        Agents = 80;
        break;

    case 5:
        Agents = 64;
        break;

    case 6:
        Agents = 16;
        break;

    case 7:
        Agents = 10;
        break;
    case 8:
        Agents = 10;
        break;
    case 9:
        Agents = 4;
        break;

    case 10:
        Agents = 8;
        ;
        break;
    }
    Environment *env = setupEnvironment();

    std::vector<int64_t> hdims = {32, 32, 32};

    MADDPG program(env, 4, 2, {32, 16, 8}, 6, 1, {32, 16, 8}, 0);

    if (argv[2] == 0)
    {
        program.Train();
        delete env;
        return 0;
    }

    program.Test(atoi(argv[3]));
    delete env;
    return 0;
}
