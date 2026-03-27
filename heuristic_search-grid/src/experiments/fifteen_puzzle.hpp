#pragma once

#include <filesystem>
#include <fstream>

#include <domains/parsers/n_puzzle_parser.hpp>
#include <search/solve.hpp>

template<template<typename> typename A, template<typename...> typename Q, typename time_queue>
void test_fifteen_puzzle(std::filesystem::path out_directory)
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

    constexpr auto file_name = "/home/gmfer/repos/heuristic_search/src/domains/benchmarks/Korf.txt";

    std::ifstream in_file(file_name);

    auto parser = parsers::n_puzzle_parser(in_file);

    decltype(solve<A, Q, time_queue>(parser.get_next())) data{};

    constexpr auto to_skip = std::array<int, 1>{100};//{97, 99, 100};

    int i = 1;
    while(parser.has_next())
    {
        auto const problem = parser.get_next();

        if(!std::ranges::contains(to_skip, i))
        {
            std::cout << "Skipping instance " << i << "." << std::endl;
            ++i;
            continue;
        }

        std::cout << "Solving instance " << i << "." << std::endl;

        auto new_data = solve<A, Q, time_queue>(problem);
        data += new_data;
        ++i;
    }

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