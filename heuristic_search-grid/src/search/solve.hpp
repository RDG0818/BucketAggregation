#pragma once

#include <search/algorithms/a.hpp>
#include <search/algorithms/wa.hpp>
#include <search/algorithms/awa.hpp>
#include <search/algorithms/iawa.hpp>
#include <search/algorithms/miawa.hpp>
#include <search/algorithms/ana.hpp>
#include <search/algorithms/mana.hpp>
#include <search/algorithms/dps.hpp>
#include <search/algorithms/optimistic.hpp>

#include <priority_queues/policies/binary_heap.hpp>
#include <priority_queues/policies/dual_heap.hpp>
#include <priority_queues/policies/bucket_heap.hpp>
#include <priority_queues/policies/buffered_bucket_heap.hpp>
#include <priority_queues/policies/timing_queue.hpp>

#include <search/search_traits.hpp>

#include <iostream>
#include <chrono>

struct solve_data
{
    std::vector<std::tuple<int, float, int, int, int>> per_bound_data;
    // total time (us)
    float search_time{};

    // search stats
    float generated{}, expanded{}, pruned{}, error{}, cost{}, h0{};

    // queue stats. Time in us
    float enqueue_time{}, dequeue_time{}, decrease_time{}, reorder_time{};
    float num_reorders{}, max_nodes{}, max_buckets{};

    // hash_table time (us)
    float hash_time{};

    float apply_time{};

    solve_data& operator+=(solve_data& other)
    {
        ++averaged_over;

        generated = average(generated, other.generated);
        expanded = average(expanded, other.expanded);
        pruned = average(pruned, other.pruned);
        error = average(error, other.error);
        cost = average(cost, other.cost);
        h0 = average(h0, other.h0);

        enqueue_time = average(enqueue_time, other.enqueue_time);
        dequeue_time = average(dequeue_time, other.dequeue_time);
        decrease_time = average(decrease_time, other.decrease_time);
        reorder_time = average(reorder_time, other.reorder_time);
        num_reorders = average(num_reorders, other.num_reorders);
        max_nodes = average(max_nodes, other.max_nodes);
        max_buckets = average(max_buckets, other.max_buckets);

        hash_time = average(hash_time, other.hash_time);

        apply_time = average(apply_time, other.apply_time);

        search_time = average(search_time, other.search_time);

        std::swap(per_bound_data, other.per_bound_data);

        return *this;
    }

    friend std::ostream& operator<<(std::ostream& os, solve_data const& data)
    {
        os << "time,hash_time,generated,expanded,pruned,error,cost,h0,enqueue_time,dequeue_time,decrease_time,reorder_time,num_reorders,max_nodes,max_buckets\n";
        os << data.search_time << ",";
        os << data.hash_time << ",";
        os << data.generated << ",";
        os << data.expanded << ",";
        os << data.pruned << ",";
        os << data.error << ",";
        os << data.cost << ",";
        os << data.h0 << ",";
        os << data.enqueue_time << ",";
        os << data.dequeue_time << ",";
        os << data.decrease_time << ",";
        os << data.reorder_time << ",";
        os << data.num_reorders << ",";
        os << data.max_nodes << ",";
        os << data.max_buckets;

        return os;
    }

private:
    int averaged_over{};

    float average(float old_avg, float val)
    {
        return (old_avg * (averaged_over - 1)) / averaged_over + val / averaged_over;
    }
};

template<template<typename> typename Algorithm, template<typename...> typename Queue = priority_queue_policies::binary_heap, typename time_queue, typename D>
solve_data solve(const D& domain)
{
    using traits = search_traits<Algorithm<D>, Queue, time_queue>;
    typename traits::search_data search_data;

    solve_data sd{};

    auto const start = std::chrono::steady_clock::now();

    traits::initialize(search_data, domain);
    
    while(!traits::should_terminate(search_data))
    {
        auto const node_handle = traits::get_fringe_node(search_data);

        if(node_handle == typename traits::node_handle{}) break;
        
        traits::expand_node(node_handle, search_data, domain);
    }

    auto const end = std::chrono::steady_clock::now();

    sd.search_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    sd.cost = search_data.incumbent.f_value;
    sd.generated = search_data.generated;
    sd.expanded = search_data.expanded;
    sd.h0 = search_data.h0;

    if constexpr(traits::is_anytime_algorithm::value)
    {
        sd.pruned = search_data.pruned;
        sd.error = search_data.error;
        sd.per_bound_data = std::move(search_data.per_bound_data);

        sd.apply_time = std::chrono::duration_cast<std::chrono::microseconds>(search_data.apply_time).count(); // hacky test
    }

    if constexpr(time_queue::value)
    {
        sd.enqueue_time = std::chrono::duration_cast<std::chrono::microseconds>(search_data.open_list.push_time).count();
        sd.dequeue_time = std::chrono::duration_cast<std::chrono::microseconds>(search_data.open_list.pop_time).count();
        sd.decrease_time = std::chrono::duration_cast<std::chrono::microseconds>(search_data.open_list.decrease_time).count();
        sd.reorder_time = std::chrono::duration_cast<std::chrono::microseconds>(search_data.open_list.reorder_time).count();
        sd.num_reorders = search_data.open_list.num_reorders;
        sd.max_nodes = search_data.open_list.max_nodes;
        sd.max_buckets = search_data.open_list.max_buckets;
    }

    // sd.hash_time = std::chrono::duration_cast<std::chrono::microseconds>(search_data.hash_time).count();

    return sd;
}