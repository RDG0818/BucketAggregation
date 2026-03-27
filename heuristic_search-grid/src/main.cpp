#include <search/solve.hpp>
#include <domains/n_puzzle.hpp>
#include <domains/robot_arm.hpp>
#include <domains/parsers/robot_arm_parser.hpp>
#include <domains/heavy_pancake.hpp>
#include <domains/pancake_48.hpp>
#include <domains/grid_2d.hpp>

#include <search/algorithms/gbfs.hpp>
#include <search/algorithms/mdps.hpp>
#include <search/algorithms/mawa.hpp>
#include <search/algorithms/moptimistic.hpp>
#include <priority_queues/policies/gilon_heap.hpp>
#include <priority_queues/policies/two_level_bucket_queue_ll.hpp>
#include <priority_queues/policies/lazy_bucket_queue.hpp>

#include <fstream>
#include <random>
#include <iostream>
#include <format>
#include <print>
#include <filesystem>

#include <experiments/fifteen_puzzle.hpp>
#include <experiments/pancake.hpp>
#include <experiments/robot_arm.hpp>
#include <experiments/grid_2d.hpp>
#include <experiments/msa.hpp>

enum algorithm
{
    A,
    OPTIMISTIC,
    MOPTIMISTIC,
    DPS,
    MDPS,
    ANA,
    BAWA,
    MAWA,
    IAWA,
    MIAWA,
    MANA
};

enum domain
{
    FIFTEEN_PUZZLE,
    GRID2D,
    GRID2D_UC,
    ROBOT_ARM_6,
    ROBOT_ARM_6UC,
    ROBOT_ARM_20,
    ROBOT_ARM_20UC,
    PANCAKE,
    HEAVY_PANCAKE,
    PANCAKE_48,
    MSA
};

enum queue
{
    BINARY_HEAP,
    TWO_LEVEL_QUEUE,
    DUAL_HEAP,
    BUCKET_HEAP,
    GILON_HEAP
};

template<template<typename> typename A, template<typename...> typename Q, typename time_queue, typename... Args>
void switch_domain(domain d, std::filesystem::path out_directory, Args&& ...args)
{
    switch(d)
    {
        // case domain::FIFTEEN_PUZZLE:
        //     out_directory.append("15-puzzle/");
        //     test_fifteen_puzzle<A, Q, time_queue>(out_directory);
        //     break;

        case domain::HEAVY_PANCAKE:
            out_directory.append("heavy pancake/");
            test_heavy_pancake_problem<A, Q, time_queue>(out_directory);
            break;

        // case domain::PANCAKE_48:
        //     out_directory.append("pancake/");
        //     test_pancake_48<A, Q, time_queue>(out_directory);
        //     break;

        // case domain::ROBOT_ARM_6UC:
        //     out_directory.append("robot arm/6dof/uniform/");
        //     test_robot_arm_6_uniform<A, Q, time_queue>(out_directory);
        //     break;

        // case domain::ROBOT_ARM_6:
        //     out_directory.append("robot arm/6dof/non-uniform/");
        //     test_robot_arm_6<A, Q, time_queue>(out_directory);
        //     break;

        // case domain::ROBOT_ARM_20:
        //     out_directory.append("robot arm/20dof/non-uniform/");
        //     test_robot_arm_20<A, Q, time_queue>(out_directory);
        //     break;

        // case domain::GRID2D:
        //     out_directory.append("grid2d/");
        //     test_grid_2d<A, Q, time_queue>(out_directory, std::forward<Args>(args)...);
        //     break;

        // case domain::GRID2D_UC:
        //     out_directory.append("grid2d_uc/");
        //     test_grid_2d_uc<A, Q, time_queue>(out_directory, std::forward<Args>(args)...);
        //     break;

        // case domain::MSA:
        //     out_directory.append("msa/");
        //     test_msa<A, Q, time_queue>(out_directory);
        //     break;

        default:
            std::cerr << "!!Domain does not exist!!" << std::endl;
            break;
    }
}

template<template<typename...> typename Q, typename time_queue, typename... Args>
void switch_algorithm(domain d, algorithm a, std::filesystem::path out_directory, Args&& ...args)
{
    switch(a)
    {
        // case algorithm::A:
        //     out_directory.append("A*/");
        //     switch_domain<search::algorithms::a, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        // case algorithm::ANA:
        //     out_directory.append("ANA*/");
        //     switch_domain<search::algorithms::ana, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        // case algorithm::DPS:
        //     out_directory.append("DPS/");
        //     switch_domain<search::algorithms::dps, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        case algorithm::MDPS:
            out_directory.append("MDPS/");
            switch_domain<search::algorithms::mdps, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
            break;

        // case algorithm::MANA:
        //     out_directory.append("MANA*/");
        //     switch_domain<search::algorithms::mana, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        // case algorithm::BAWA:
        //     out_directory.append("BAWA*/");
        //     switch_domain<search::algorithms::awa, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        // case algorithm::MAWA:
        //     out_directory.append("MAWA*/");
        //     switch_domain<search::algorithms::mawa, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        // case algorithm::IAWA:
        //     out_directory.append("IAWA*/");
        //     switch_domain<search::algorithms::iawa, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        // case algorithm::MIAWA:
        //     out_directory.append("MIAWA*/");
        //     switch_domain<search::algorithms::miawa, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        // case algorithm::OPTIMISTIC:
        //     out_directory.append("Optimistic/");
        //     switch_domain<search::algorithms::optimistic, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        // case algorithm::MOPTIMISTIC:
        //     out_directory.append("Moptimistic/");
        //     switch_domain<search::algorithms::moptimistic, Q, time_queue>(d, out_directory, std::forward<Args>(args)...);
        //     break;

        default:
            std::cerr << "!!Algorithm does not exist!!" << std::endl;
            break;
    }
}

template<typename time_queue, typename... Args>
void switch_queue(domain d, queue q, algorithm a, std::filesystem::path out_directory, Args&& ...args)
{
    switch(q)
    {
        // case queue::BINARY_HEAP:
        //     out_directory.append("binary heap/");
        //     switch_algorithm<priority_queue_policies::binary_heap, time_queue>(d, a, out_directory, std::forward<Args>(args)...);
        //     break;

        // case queue::TWO_LEVEL_QUEUE:
        //     out_directory.append("two level queue/");
        //     switch_algorithm<priority_queue_policies::two_level_bucket_queue, time_queue>(d, a, out_directory, std::forward<Args>(args)...);
        //     break;

        // case queue::DUAL_HEAP:
        //     out_directory.append("dual heap/");
        //     switch_algorithm<priority_queue_policies::dual_heap, time_queue>(d, a, out_directory, std::forward<Args>(args)...);
        //     break;

        case queue::BUCKET_HEAP:
            out_directory.append("bucket heap/");
            switch_algorithm<priority_queue_policies::bucket_heap, time_queue>(d, a, out_directory, std::forward<Args>(args)...);
            break;

        // case queue::GILON_HEAP:
        //     out_directory.append("gilon heap/");
        //     switch_algorithm<priority_queue_policies::gilon_heap, time_queue>(d, a, out_directory, std::forward<Args>(args)...);
        //     break;

        default:
            std::cerr << "!!Queue type does not exist!!" << std::endl;
            break;
    }
}

int main(int argc, char** argv)
{
    constexpr auto d = domain::ROBOT_ARM_6UC;
    // constexpr auto q = queue::BUCKET_HEAP;
    // constexpr auto a = algorithm::MDPS;
    auto const out_directory = std::filesystem::path{"/home/gmfer/Desktop/socs/experiments/"};

    using yes = std::true_type;
    using no = std::false_type;

    // switch_queue<no>(domain::GRID2D, queue::BUCKET_HEAP, algorithm::ANA, out_directory, std::pair{5000,5000});
    // switch_queue<no>(domain::GRID2D, queue::BUCKET_HEAP, algorithm::MANA, out_directory, std::pair{5000,5000});

    switch_queue<no>(domain::HEAVY_PANCAKE, queue::BUCKET_HEAP, algorithm::MDPS, out_directory);


    // switch_queue<no>(domain::ROBOT_ARM_6, queue::BUCKET_HEAP, algorithm::ANA, out_directory);
    // switch_queue<no>(domain::FIFTEEN_PUZZLE, queue::BUCKET_HEAP, algorithm::ANA, out_directory);

    // switch_queue<yes>(domain::FIFTEEN_PUZZLE, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<yes>(domain::FIFTEEN_PUZZLE, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);
    // switch_queue<no>(domain::FIFTEEN_PUZZLE, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<no>(domain::FIFTEEN_PUZZLE, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);

    // switch_queue<yes>(domain::ROBOT_ARM_6UC, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<yes>(domain::ROBOT_ARM_6UC, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6UC, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6UC, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);

    // switch_queue<yes>(domain::ROBOT_ARM_6, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<yes>(domain::ROBOT_ARM_6, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);

    // switch_queue<yes>(domain::GRID2D, queue::BINARY_HEAP, algorithm::A, out_directory, std::pair{5000, 5000});
    // switch_queue<yes>(domain::GRID2D, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory, std::pair{5000, 5000});
    // switch_queue<no>(domain::GRID2D, queue::BINARY_HEAP, algorithm::A, out_directory, std::pair{5000, 5000});
    // switch_queue<no>(domain::GRID2D, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory, std::pair{5000, 5000});

    // switch_queue<yes>(domain::MSA, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<yes>(domain::MSA, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);
    // switch_queue<no>(domain::MSA, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<no>(domain::MSA, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);

    // switch_queue<yes>(domain::PANCAKE_48, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<yes>(domain::PANCAKE_48, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);
    // switch_queue<no>(domain::PANCAKE_48, queue::BINARY_HEAP, algorithm::A, out_directory);
    // switch_queue<no>(domain::PANCAKE_48, queue::TWO_LEVEL_QUEUE, algorithm::A, out_directory);


    // switch_queue<yes>(domain::PANCAKE_48, queue::DUAL_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<yes>(domain::PANCAKE_48, queue::GILON_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<yes>(domain::PANCAKE_48, queue::BUCKET_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::PANCAKE_48, queue::DUAL_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::PANCAKE_48, queue::GILON_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::PANCAKE_48, queue::BUCKET_HEAP, algorithm::OPTIMISTIC, out_directory);

    // switch_queue<yes>(domain::PANCAKE_48, queue::DUAL_HEAP, algorithm::DPS, out_directory);
    // switch_queue<yes>(domain::PANCAKE_48, queue::GILON_HEAP, algorithm::DPS, out_directory);
    // switch_queue<yes>(domain::PANCAKE_48, queue::BUCKET_HEAP, algorithm::DPS, out_directory);
    // switch_queue<no>(domain::PANCAKE_48, queue::DUAL_HEAP, algorithm::DPS, out_directory);
    // switch_queue<no>(domain::PANCAKE_48, queue::GILON_HEAP, algorithm::DPS, out_directory);
    // switch_queue<no>(domain::PANCAKE_48, queue::BUCKET_HEAP, algorithm::DPS, out_directory);
    
    // switch_queue<no>(domain::FIFTEEN_PUZZLE, queue::BUCKET_HEAP, algorithm::MANA, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6UC, queue::BUCKET_HEAP, algorithm::MANA, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6, queue::BUCKET_HEAP, algorithm::MANA, out_directory);
    // switch_queue<no>(domain::MSA, queue::BUCKET_HEAP, algorithm::MANA, out_directory);

    // switch_queue<no>(domain::FIFTEEN_PUZZLE, queue::DUAL_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::FIFTEEN_PUZZLE, queue::BUCKET_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::FIFTEEN_PUZZLE, queue::GILON_HEAP, algorithm::OPTIMISTIC, out_directory);

    // Change epsilon = 1.02
    // switch_queue<no>(domain::ROBOT_ARM_6UC, queue::DUAL_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6UC, queue::GILON_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6UC, queue::BUCKET_HEAP, algorithm::OPTIMISTIC, out_directory);

    // switch_queue<no>(domain::ROBOT_ARM_6, queue::DUAL_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6, queue::GILON_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::ROBOT_ARM_6, queue::BUCKET_HEAP, algorithm::OPTIMISTIC, out_directory);

    // switch_queue<no>(domain::MSA, queue::DUAL_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::MSA, queue::BUCKET_HEAP, algorithm::OPTIMISTIC, out_directory);
    // switch_queue<no>(domain::MSA, queue::GILON_HEAP, algorithm::OPTIMISTIC, out_directory);

    // switch_queue<yes>(domain::ROBOT_ARM_20, queue::BINARY_HEAP, algorithm::ANA, out_directory);
    // switch_queue<yes>(domain::HEAVY_PANCAKE, queue::BUCKET_HEAP, algorithm::ANA, out_directory);
    // switch_queue<yes>(domain::HEAVY_PANCAKE, queue::BUCKET_HEAP, algorithm::MANA, out_directory);

    // Change epsilon = 1.02
    // switch_queue<no>(domain::GRID2D, queue::DUAL_HEAP, algorithm::OPTIMISTIC, out_directory, std::pair{5000, 5000});
    // switch_queue<no>(domain::GRID2D, queue::GILON_HEAP, algorithm::OPTIMISTIC, out_directory, std::pair{5000, 5000});
    // switch_queue<no>(domain::GRID2D, queue::BUCKET_HEAP, algorithm::OPTIMISTIC, out_directory, std::pair{5000, 5000});

    // switch_queue<yes>(domain::GRID2D, queue::DUAL_HEAP, algorithm::DPS, out_directory, std::pair{5000, 5000});
    // switch_queue<yes>(domain::GRID2D, queue::GILON_HEAP, algorithm::DPS, out_directory, std::pair{5000, 5000});
    // switch_queue<yes>(domain::GRID2D, queue::BUCKET_HEAP, algorithm::DPS, out_directory, std::pair{5000, 5000});

    // switch_queue<yes>(domain::GRID2D, queue::DUAL_HEAP, algorithm::OPTIMISTIC, out_directory, std::pair{5000, 5000});
    // switch_queue<yes>(domain::GRID2D, queue::GILON_HEAP, algorithm::OPTIMISTIC, out_directory, std::pair{5000, 5000});
    // switch_queue<yes>(domain::GRID2D, queue::BUCKET_HEAP, algorithm::OPTIMISTIC, out_directory, std::pair{5000, 5000});


    // Fix ordered_set, lazy_ordered_set, lazy_bucket_queue

    return 0;
}