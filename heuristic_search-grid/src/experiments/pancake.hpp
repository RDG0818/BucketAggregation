#pragma once

#include <filesystem>
#include <fstream>
#include <random>
#include <algorithm>
#include <ranges>
#include <iostream>

#include <domains/parsers/pancake_parser.hpp>
#include <domains/parsers/pancake_48_parser.hpp>
#include <search/solve.hpp>

template<std::size_t size>
[[nodiscard]] std::array<short, size> generate_pancakes()
{
    std::array<short, size> p;
    for(auto i = 0; i < size; ++i) p[i] = i;

    std::random_device rd;
    std::mt19937 g(rd()); 
    std::shuffle(p.begin(), p.end(), g);

    return p;
}

template<std::size_t size>
void create_pancake_set(int stacks_in_set, std::filesystem::path out_directory)
{
    constexpr auto file_name = "pancakes.txt";
    out_directory.append(file_name);

    std::ofstream out_file(out_directory);

    for(auto i = 0; i < stacks_in_set; ++i)
    {
        auto const pancakes = generate_pancakes<size>();
        std::ranges::copy(pancakes, std::ostream_iterator<short>{out_file, " "});
        out_file << "\n";
    }
}

template<template<typename> typename A, template<typename...> typename Q>
void test_pancake_problem(std::filesystem::path out_directory)
{
    constexpr auto out_name = "data.csv";
    out_directory.append(out_name);

    constexpr auto num_pancakes = 16;
    constexpr auto uniform_cost = true;

    auto const pancakes = generate_pancakes<num_pancakes>();

    // auto const pancakes = std::array<short, num_pancakes>{ 0, 3, 6, 4, 7, 2, 5, 1 };
    // auto const pancakes = std::array<short, num_pancakes>{ 4,3,11,8,6,12,14,10,0,15,9,2,7,13,5,1 };
}

template<template<typename> typename A, template<typename...> typename Q, typename time_queue>
void test_heavy_pancake_problem(std::filesystem::path out_directory)
{
    std::string out_name;

    if constexpr(time_queue::value)
    {
        out_name = "detailed_data.csv";
    }
    else
    {
        out_name = "data.csv";
    }
    
    auto copy = out_directory;
    out_directory.append(out_name);

    constexpr auto num_pancakes = 16;
    constexpr auto uniform_cost = false;

    auto const file_name = "/home/gmfer/repos/heuristic_search/src/domains/benchmarks/pancakes" + std::to_string(num_pancakes) + ".txt";

    std::ifstream in_file(file_name);

    std::cout << "Solving " << num_pancakes << "-pancake problem." << std::endl;

    auto parser = parsers::pancake_parser<num_pancakes>(in_file);

    decltype(solve<A, Q, time_queue>(parser.get_next())) data{};

    int i = 1;

    auto solved = 0;

    while(parser.has_next())
    {
        std::cout << "Solving instance " << i << "." << std::endl;

        auto const problem = parser.get_next();

        auto new_data = solve<A, Q, time_queue>(problem);
        data += new_data;
        ++i;

        solved += (new_data.cost > 0);
        if(new_data.cost > 0)
        {
            std::cout << "solved." << std::endl;
        }
        else
        {
            std::cout << "failed." << std::endl;
        }
    }

    std::cout << "Num solved: " << solved << std::endl;

    // std::ofstream out_file(out_directory);

    // out_file << data;

    // std::string per_bound_out_file = "per_bound_data.csv";
    // copy.append(per_bound_out_file);

    // std::ofstream bound_out(copy);

    // <expanded, error, UB, nodes, buckets>
    // bound_out << "expanded,error,UB,nodes,buckets\n";

    // for(auto [expanded, error, UB, nodes, buckets] : data.per_bound_data)
    // {
    //     bound_out << expanded << "," << error << "," << UB << "," << nodes << "," << buckets << "\n";
    // }
}

template<template<typename> typename A, template<typename...> typename Q, typename time_queue>
void test_pancake_48(std::filesystem::path out_directory)
{
    std::string out_name;

    if constexpr(time_queue::value)
    {
        out_name = "detailed_data.csv";
    }
    else
    {
        out_name = "data.csv";
    }
    
    auto copy = out_directory;
    out_directory.append(out_name);

    constexpr auto num_pancakes = 48;
    constexpr auto uniform_cost = false;

    auto const file_name = "/home/gmfer/repos/heuristic_search/src/domains/benchmarks/pancakes" + std::to_string(num_pancakes) + ".txt";

    std::ifstream in_file(file_name);

    std::cout << "Solving " << num_pancakes << "-pancake problem." << std::endl;

    auto parser = parsers::pancake_48_parser(in_file);

    decltype(solve<A, Q, time_queue>(parser.get_next())) data{};

    int i = 1;

    auto solved = 0;

    // while(parser.has_next())
    {
        std::cout << "Solving instance " << i << "." << std::endl;

        auto const problem = parser.get_next();

        auto new_data = solve<A, Q, time_queue>(problem);
        data += new_data;
        ++i;

        // solved += (new_data.cost > 0);
        // if(new_data.cost > 0)
        // {
        //     std::cout << "solved." << std::endl;
        // }
        // else
        // {
        //     std::cout << "failed." << std::endl;
        // }
    }

    // std::cout << "Num solved: " << solved << std::endl;

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