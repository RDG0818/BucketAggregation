#pragma once

#include <filesystem>
#include <fstream>
#include <ranges>
#include <bitset>

#include <domains/msa.hpp>
#include <search/solve.hpp>

template<template<typename> typename A, template<typename...> typename Q, typename time_queue>
void test_msa(std::filesystem::path out_directory)
{
    constexpr auto num_sequences = 6;
    constexpr auto num_to_align = 4;

    std::cout << "Solving MSA " << num_to_align << "." << std::endl;

    std::string out_name = std::to_string(num_to_align) + "_";

    if constexpr(time_queue::value)
    {
        out_name += "detailed_data.csv";
    }
    else
    {
        out_name += "data.csv";
    }
    
    auto copy = out_directory;
    out_directory.append(out_name);

    constexpr auto in_directory = "/home/gmfer/repos/heuristic_search/src/domains/benchmarks/sequences/";

    // Load in all sequences
    std::array<std::vector<char>, num_sequences> sequences;

    for(auto i = 0; i < num_sequences; ++i)
    {
        std::ifstream in_file(in_directory + std::to_string(i) + ".faa");
        std::istream_iterator<char> begin(in_file), end;

        std::vector<char> v{'_'};
        v.insert(v.end(), begin, end);
        sequences[i] = std::move(v);
    }

    // Solve all num_sequences choose num_to_align combinations
    std::string bitmask(num_to_align, 1); // K leading 1's
    bitmask.resize(num_sequences, 0); // N-K trailing 0's

    decltype(solve<A, Q, time_queue>(std::declval<std::array<std::vector<char>, num_to_align>>())) data{};

    do
    {
        std::array<std::vector<char>, num_to_align> sequences_to_align;
        std::array<int, num_to_align> indices;

        for(auto i = 0, j = 0; i < num_sequences && j < num_to_align; ++i)
        {
            if(bitmask[i])
            {
                sequences_to_align[j] = sequences[i];
                indices[j] = i;
                ++j;
            }
        }

        std::cout << "Solving combination {";
        std::ranges::copy(indices, std::ostream_iterator<int>(std::cout, ","));
        std::cout << "}" << std::endl;

        auto new_data = solve<A, Q, time_queue>(search::domains::msa(sequences_to_align));
        data += new_data;
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));

    std::cout << std::endl;

    std::ofstream out_file(out_directory);

    out_file << data;

    // std::string per_bound_out_file = "per_bound_data.csv";
    // copy.append(per_bound_out_file);

    // std::ofstream bound_out(copy);

    // // <expanded, error, UB, nodes, buckets>
    // bound_out << "expanded,error,UB,nodes,buckets\n";

    // for(auto [expanded, error, UB, nodes, buckets] : data.per_bound_data)
    // {
    //     bound_out << expanded << "," << error << "," << UB << "," << nodes << "," << buckets << "\n";
    // }
}