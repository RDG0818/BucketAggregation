#pragma once

#include <domains/robot_arm.hpp>

#include <fstream>
#include <string>
#include <sstream>

namespace parsers
{
    class robot_arm_parser
    {
    public:
        template<typename dof, bool uniform_cost = true>
        static search::domains::robot_arm<dof, uniform_cost> parse(std::ifstream& inFile)
        {
            std::vector<char> environment;
            std::pair<float, float> env_dims;
            std::pair<int, int> cell_scale_factors;
            int baseX;
            std::array<float, dof::num_joints> lengths;
            std::array<float, dof::num_joints> start_angles;
            std::pair<int, int> goal;

            // Use misc to remove info from beginning of the line
            std::string misc;

            inFile >> misc >> env_dims.first >> env_dims.second;
            inFile >> misc >> cell_scale_factors.first >> cell_scale_factors.second;
            inFile >> misc >> baseX;
            inFile >> misc;

            std::string line;
            std::getline(inFile, line);

            std::stringstream ss(line);
            auto i = 0;
            while(!ss.eof())
            {
                float length;
                ss >> length;
                lengths[i] = length;
                ++i;
            }

            inFile >> misc;
            std::getline(inFile, line);

            ss = std::stringstream(line);
            i = 0;
            while(!ss.eof())
            {
                float angle;
                ss >> angle;
                start_angles[i] = angle;
                ++i;
            }

            inFile >> misc >> goal.first >> goal.second;
            inFile >> misc;

            unsigned int cell;
            while (inFile >> cell)
            {
                environment.push_back(cell);
            }

            return search::domains::robot_arm<dof, uniform_cost>(environment, env_dims, cell_scale_factors, baseX, lengths, start_angles, goal);
        }
    };
}