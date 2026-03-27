#pragma once

#include <filesystem>
#include <fstream>
#include <random>
#include <algorithm>
#include <ranges>
#include <iostream>

#include <search/solve.hpp>
#include <domains/grid_2d.hpp>

[[nodiscard]] search::domains::grid_2d<true> generate_grid_uc(std::mt19937& gen, std::pair<int, int> dimensions, float obstacle_density)
{
    auto const [width, height] = dimensions;
    auto const obstacle_percentage = obstacle_density;
    int const num_obstacles = width * height * obstacle_percentage;

    std::vector<char> grid(width * height, 0);
    std::ranges::generate_n(grid.begin(), num_obstacles, [](){ return 1; });

    std::ranges::shuffle(grid, gen);
    std::uniform_int_distribution<> distr(0, width);

    auto start = std::pair{0, 0};
    auto goal = std::pair{dimensions.first - 1, dimensions.second - 1};

    // auto start = std::pair{distr(gen), distr(gen)};
    // auto goal = std::pair{distr(gen), distr(gen)};
    // auto start = std::pair{0,0};
    // auto goal = std::pair{2,3};

    return search::domains::grid_2d<true>(grid, {width, height}, start, goal);
}

[[nodiscard]] search::domains::grid_2d<false> generate_grid(std::mt19937& gen, std::pair<int, int> dimensions)
{
    auto const [width, height] = dimensions;

    std::vector<char> grid;
    grid.reserve(width*height);

    std::uniform_int_distribution<> distr(1, 4);
    std::ranges::generate_n(std::back_inserter(grid), width * height, [&](){ return distr(gen); });

    auto start = std::pair{0, 0};
    auto goal = std::pair{dimensions.first - 1, dimensions.second - 1};

    return search::domains::grid_2d<false>(grid, {width, height}, start, goal);
}

template<template<typename> typename A, template<typename...> typename Q, typename time_queue>
void test_grid_2d(std::filesystem::path out_directory, std::pair<int, int> dimensions)
{
    std::string out_name;
    constexpr float density = 0.2f;

    if constexpr(time_queue::value)
    {
        out_name = std::to_string(dimensions.first) + "x" + std::to_string(dimensions.second) + "_" + std::to_string(density) + "_detailed_data.csv";
    }
    else
    {
        out_name = std::to_string(dimensions.first) + "x" + std::to_string(dimensions.second) + "_" + std::to_string(density) + "_data.csv";
    }
    
    auto copy = out_directory;
    out_directory.append(out_name);


    decltype(solve<A, Q, time_queue>(std::declval<search::domains::grid_2d<false>>())) data{};

    int solved = 0;
    constexpr auto num_to_solve = 1;

    std::mt19937 gen;
    gen.seed(dimensions.first * dimensions.second);

    while(solved < num_to_solve)
    {
        auto new_data = solve<A, Q, time_queue>(generate_grid_uc(gen, dimensions, density));

        if(new_data.cost == 0)
        {
            std::cout << "No solution." << std::endl;
            continue;
        }

        data += new_data;
        ++solved;

        std::cout << "Solved " << solved << "." << std::endl;
    }

    // std::ofstream out_file(out_directory);

    // out_file << data;

    std::string per_bound_out_file = "per_bound_data.csv";
    copy.append(per_bound_out_file);

    std::ofstream bound_out(copy);

    // <expanded, error, UB, nodes, buckets>
    bound_out << "expanded,error,UB,nodes,buckets\n";

    for(auto [expanded, error, UB, nodes, buckets] : data.per_bound_data)
    {
        bound_out << expanded << "," << error << "," << UB << "," << nodes << "," << buckets << "\n";
    }
}