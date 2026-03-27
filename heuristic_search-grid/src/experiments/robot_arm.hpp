#pragma once

#include <filesystem>
#include <fstream>

#include <domains/parsers/robot_arm_parser.hpp>
#include <search/solve.hpp>

template<template<typename> typename A, template<typename...> typename Q, typename time_queue>
void test_robot_arm_6_uniform(std::filesystem::path out_directory)
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
    
    out_directory.append(out_name);

    constexpr auto file_name = "/home/gmfer/repos/heuristic_search/src/domains/benchmarks/env1_6d.cfg";

    std::cout << "Solving Robot Arm 6-DoF Uniform." << std::endl;

    std::ifstream in_file(file_name);

    auto const problem = parsers::robot_arm_parser::parse<search::domains::DoF6, true>(in_file);

    auto data = solve<A, Q, time_queue>(problem);

    std::ofstream out_file(out_directory);

    out_file << data;
}

template<template<typename> typename A, template<typename...> typename Q, typename time_queue>
void test_robot_arm_6(std::filesystem::path out_directory)
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

    constexpr auto file_name = "/home/gmfer/repos/heuristic_search/src/domains/benchmarks/env1_6d.cfg";

    std::cout << "Solving Robot Arm 6-DoF." << std::endl;

    std::ifstream in_file(file_name);

    auto const problem = parsers::robot_arm_parser::parse<search::domains::DoF6, false>(in_file);

    auto data = solve<A, Q, time_queue>(problem);

    std::ofstream out_file(out_directory);

    out_file << data;

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

template<template<typename> typename A, template<typename...> typename Q, typename time_queue>
void test_robot_arm_20(std::filesystem::path out_directory)
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

    constexpr auto file_name = "/home/gmfer/repos/heuristic_search/src/domains/benchmarks/env1_20d.cfg";

    std::cout << "Solving Robot Arm 20-DoF." << std::endl;

    std::ifstream in_file(file_name);

    auto const problem = parsers::robot_arm_parser::parse<search::domains::DoF20, false>(in_file);

    auto data = solve<A, Q, time_queue>(problem);

    std::ofstream out_file(out_directory);

    out_file << data;

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