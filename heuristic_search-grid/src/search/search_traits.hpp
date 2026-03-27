#pragma once

#include <type_traits>

#include <utils/detector.hpp>
#include <search/search_traits_checks.hpp>
#include <search/default_search_traits.hpp>
#include <search/anytime_search_traits.hpp>

#include <priority_queues/policies/binary_heap.hpp>
#include <priority_queues/policies/two_level_bucket_queue.hpp>
#include <priority_queues/policies/lazy_bucket_queue.hpp>
#include <priority_queues/policies/two_level_bucket_queue_ll.hpp>
#include <priority_queues/policies/dual_heap.hpp>
#include <priority_queues/policies/bucket_heap.hpp>
#include <priority_queues/policies/buffered_bucket_heap.hpp>
#include <priority_queues/policies/gilon_heap.hpp>
#include <priority_queues/policies/timing_queue.hpp>

template<typename Algorithm, template<typename...> typename Queue, typename time_queue>
struct search_traits
{
private:
    template<typename T>
    using _node_handle = typename T::node_handle;

    template<typename T, typename... Args>
    using _search_node = typename T::template search_node<Args...>;

    template<typename T, typename... Args>
    using _node_compare = typename T::template node_compare<Args...>;

    template<typename T, typename... Args>
    using _queue_info_accessor = typename T::template queue_info_accessor<Args...>;

    template<typename T, typename... Args>
    using _search_data = typename T::template search_data<Args...>;

    template<typename T>
    using _is_anytime_algorithm = typename T::is_anytime_algorithm;

    

    template<template<typename...> typename Q, typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct _queue_type
    {
        using type = Q<NodeHandle, NodeCompare, InfoAccessor>;
    };

    template<typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct _queue_type<priority_queue_policies::two_level_bucket_queue, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>
    {
        using type = priority_queue_policies::two_level_bucket_queue<NodeHandle, NodePriorityPair, InfoAccessor>;
    };

    template<typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct _queue_type<priority_queue_policies::two_level_bucket_queue_ll, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>
    {
        using type = priority_queue_policies::two_level_bucket_queue_ll<NodeHandle, NodePriorityPair, InfoAccessor>;
    };

    template<typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct _queue_type<priority_queue_policies::lazy_bucket_queue, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>
    {
        using type = priority_queue_policies::lazy_bucket_queue<NodeHandle, NodePriorityPair, InfoAccessor>;
    };

    template<typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct _queue_type<priority_queue_policies::bucket_heap, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>
    {
        using type = priority_queue_policies::bucket_heap<NodeHandle, NodePriorityPair, NodeCompare, InfoAccessor>;
    };

    template<typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct _queue_type<priority_queue_policies::buffered_bucket_heap, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>
    {
        using type = priority_queue_policies::buffered_bucket_heap<NodeHandle, NodePriorityPair, NodeCompare, InfoAccessor>;
    };

    template<typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct _queue_type<priority_queue_policies::gilon_heap, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>
    {
        using type = priority_queue_policies::gilon_heap<NodeHandle, NodePriorityPair, NodeCompare, InfoAccessor>;
    };

    template<typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct _queue_type<priority_queue_policies::dual_heap, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>
    {
        using type = priority_queue_policies::dual_heap<NodeHandle, NodeCompare, default_node_compare<search_traits<Algorithm, Queue, time_queue>>, InfoAccessor>;
    };

    template<typename timing_queue, template<typename...> typename Q, typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct __queue_type
    {
        using type = typename _queue_type<Q, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>::type;
    };

    template<template<typename...> typename Q, typename NodeHandle, typename NodeCompare, typename NodePriorityPair, typename InfoAccessor>
    struct __queue_type<std::true_type, Q, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>
    {
        using type = priority_queue_policies::timing_queue<typename _queue_type<Q, NodeHandle, NodeCompare, NodePriorityPair, InfoAccessor>::type>;
    };

public:
    using algorithm = Algorithm;

    using domain = typename Algorithm::domain_type;

    using node_handle = detected_or_t<default_node_handle, _node_handle, Algorithm>;

    using queue_loc_info = typename priority_queue_traits<node_handle, Queue>::location_info_type;

    using search_node = detected_or_t<default_search_node<search_traits<Algorithm, Queue, time_queue>>, _search_node,
                                        Algorithm, typename Algorithm::domain_type, node_handle, queue_loc_info>;

    using node_compare = detected_or_t<default_node_compare<search_traits<Algorithm, Queue, time_queue>>, _node_compare, Algorithm, search_traits<Algorithm, Queue, time_queue>>;

    using node_priority_pair = default_node_priority_pair<search_traits<Algorithm, Queue, time_queue>>;

    using queue_info_accessor = detected_or_t<default_queue_info_accessor<search_traits<Algorithm, Queue, time_queue>>, _queue_info_accessor, Algorithm, node_handle, search_node>;

    // hack for timing queue
    // using use_timing_queue = std::true_type;
    using queue_type = typename __queue_type<time_queue, Queue, node_handle, node_compare, node_priority_pair, queue_info_accessor>::type;


    using search_data = detected_or_t<default_search_data<search_traits<Algorithm, Queue, time_queue>>, _search_data, Algorithm, search_traits<Algorithm, Queue, time_queue>>;

    using is_anytime_algorithm = detected_or_t<std::false_type, _is_anytime_algorithm, Algorithm>;

    inline static auto initialize_structures(search_data& data) -> typename search_data::open_list_type
    {
        if constexpr(has_initialize_structures<search_traits<Algorithm, Queue, time_queue>>)
        {
            return Algorithm::template initialize_structures<search_traits<Algorithm, Queue, time_queue>>(data);
        }
        else
        {
            return default_initialize_structures<search_traits<Algorithm, Queue, time_queue>>(data);
        }
    }

    inline static void initialize(search_data& data, domain const& domain)
    {
        if constexpr(has_initialize_search<search_traits<Algorithm, Queue, time_queue>>)
        {
            Algorithm::template initialize_search<search_traits<Algorithm, Queue, time_queue>>(data, domain);
        }
        else
        {
            default_initialize_search<search_traits<Algorithm, Queue, time_queue>>(data, domain);
        }
    }

    [[nodiscard]] inline static bool should_terminate(search_data const& data) noexcept
    {
        if constexpr(has_should_terminate<search_traits<Algorithm, Queue, time_queue>>)
        {
            return Algorithm::template should_terminate<search_traits<Algorithm, Queue, time_queue>>(data);
        }
        else
        {
            if constexpr(is_anytime_algorithm::value)
            {
                return anytime_should_terminate<search_traits<Algorithm, Queue, time_queue>>(data);
            }
            else
            {
                return default_should_terminate<search_traits<Algorithm, Queue, time_queue>>(data);
            }
        }
    }

    [[nodiscard]] inline static auto get_fringe_node(search_data& data) -> node_handle
    {
        if constexpr(has_get_fringe_node<search_traits<Algorithm, Queue, time_queue>>)
        {
            return Algorithm::template get_fringe_node<search_traits<Algorithm, Queue, time_queue>>(data);
        }
        else
        {
            if constexpr(is_anytime_algorithm::value)
            {
                return anytime_get_fringe_node<search_traits<Algorithm, Queue, time_queue>>(data);
            }
            else
            {
                return default_get_fringe_node<search_traits<Algorithm, Queue, time_queue>>(data);
            }
        }
    }

    inline static void expand_node(node_handle const handle, search_data& data, domain const& domain)
    {
        if constexpr(has_expand_node<search_traits<Algorithm, Queue, time_queue>>)
        {
            Algorithm::template expand_node<search_traits<Algorithm, Queue, time_queue>>(handle, data, domain);
        }
        else
        {
            if constexpr(is_anytime_algorithm::value)
            {
                return anytime_expand_node<search_traits<Algorithm, Queue, time_queue>>(handle, data, domain);
            }
            else
            {
                return default_expand_node<search_traits<Algorithm, Queue, time_queue>>(handle, data, domain);
            }
        }
    }

    inline static void handle_successor(typename domain::packed_state&& packed_successor, typename domain::h_type const h_value, node_handle const parent_handle, typename domain::action const& action,
                                        domain const& domain, search_data& data)
    {
        if constexpr(has_handle_successor<search_traits<Algorithm, Queue, time_queue>>)
        {
            algorithm::template handle_successor<search_traits<Algorithm, Queue, time_queue>>(std::move(packed_successor), h_value, parent_handle, action, domain, data);
        }
        else
        {
            if constexpr(is_anytime_algorithm::value)
            {
                anytime_handle_successor<search_traits<Algorithm, Queue, time_queue>>(std::move(packed_successor), h_value, parent_handle, action, domain, data);
            }
            else
            {
                default_handle_successor<search_traits<Algorithm, Queue, time_queue>>(std::move(packed_successor), h_value, parent_handle, action, domain, data);
            }
        }
    }

    inline static void update_open_closed(typename domain::packed_state&& packed_successor, node_handle const parent_handle, typename search_node::f_type const f_value,
                                            typename search_node::g_type const g_value, typename search_node::h_type const h_value, typename domain::action const& action, search_data& data)
    {
        if constexpr(has_update_open_closed<search_traits<Algorithm, Queue, time_queue>>)
        {
            algorithm::template update_open_closed<search_traits<Algorithm, Queue, time_queue>>(std::move(packed_successor), parent_handle, f_value, g_value, h_value, action, data);
        }
        else
        {
            default_update_open_closed<search_traits<Algorithm, Queue, time_queue>>(std::move(packed_successor), parent_handle, f_value, g_value, h_value, action, data);
        }
    }
};