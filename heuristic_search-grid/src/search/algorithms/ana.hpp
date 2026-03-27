#pragma once

#include <vector>
#include <memory>
#include <utils/parallel_hashmap/phmap.h>
#include <utils/ordered_set.hpp>
#include <chrono>
#include <utils/time_op.hpp>
#include <search/search_helpers.hpp>
#include <search/anytime_search_traits.hpp>

namespace search::algorithms
{
    template<typename Domain>
    struct ana
    {
        using domain_type = Domain;
        using is_anytime_algorithm = std::true_type;

        template<typename SearchTraits>
        struct node_compare
        {
            node_compare(std::vector<typename SearchTraits::search_node> const& nodes, typename SearchTraits::search_node const& incumbent) : nodes(nodes), incumbent(incumbent)
            {}

            [[nodiscard]] inline bool operator()(typename SearchTraits::node_handle const lhs, typename SearchTraits::node_handle const rhs) const noexcept
            {
                auto const& lnode = nodes[lhs];
                auto const& rnode = nodes[rhs];

                if(incumbent.parent_handle == typename SearchTraits::node_handle{})
                {
                    return lnode.h_value < rnode.h_value;
                }

                // Compare potentials: (C - g(n)) / h(n)
                float const left_potential = (lnode.h_value > 0) ? (incumbent.f_value - lnode.f_value + lnode.h_value) / static_cast<float>(lnode.h_value) : std::numeric_limits<float>::infinity();
                float const right_potential = (rnode.h_value > 0) ? (incumbent.f_value - rnode.f_value + rnode.h_value) / static_cast<float>(rnode.h_value) : std::numeric_limits<float>::infinity();

                if(left_potential == right_potential) return lnode.h_value < rnode.h_value;

                return left_potential > right_potential;

                // Rearrange u(lhs) > u(rhs) to arrive at this, which removes the need for division and static_cast<float>
                // auto const l = lnode.h_value * (rnode.f_value - incumbent.f_value);
                // auto const r = rnode.h_value * (lnode.f_value - incumbent.f_value);

                // return lnode.h_value * (rnode.f_value - incumbent.f_value) > rnode.h_value * (lnode.f_value - incumbent.f_value);
            }

        private:
            std::vector<typename SearchTraits::search_node> const& nodes;
            typename SearchTraits::search_node const& incumbent;
        };

        template<typename SearchTraits>
        using search_data = anytime_search_data<SearchTraits>;

        template<typename SearchTraits>
        [[nodiscard]] static constexpr auto initialize_structures(typename SearchTraits::search_data& data) -> SearchTraits::search_data::open_list_type
        {
            if constexpr(std::is_base_of_v<priority_queue_policies::binary_heap<typename SearchTraits::node_handle,
                                                            typename SearchTraits::node_compare,
                                                            typename SearchTraits::queue_info_accessor>,
                                        typename SearchTraits::search_data::open_list_type>)
            {
                return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_compare(data.search_nodes, data.incumbent),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
            }
            else if constexpr(std::is_base_of_v<priority_queue_policies::dual_heap<typename SearchTraits::node_handle,
                                                            typename SearchTraits::node_compare,
                                                            default_node_compare<SearchTraits>,
                                                            typename SearchTraits::queue_info_accessor>,
                                            typename SearchTraits::search_data::open_list_type>)
            {
                return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_compare(data.search_nodes, data.incumbent),
                                                                        default_node_compare<SearchTraits>(data.search_nodes),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
            }
            else
            {
                return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_priority_pair(data.search_nodes),
                                                                        typename SearchTraits::node_compare(data.search_nodes, data.incumbent),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
            }
        }

        template<typename SearchTraits>
        static inline void update_error_bound(typename SearchTraits::node_handle const handle, typename SearchTraits::search_data& data) noexcept
        {
            if constexpr(dual_queue<typename SearchTraits::queue_type> && !std::is_base_of_v<priority_queue_policies::binary_heap<typename SearchTraits::node_handle,
                                                            typename SearchTraits::node_compare,
                                                            typename SearchTraits::queue_info_accessor>,
                                        typename SearchTraits::search_data::open_list_type>)
            {
                if(!data.open_list.is_empty())
                {
                    auto const old_error = data.error;

                    float const min_f = std::min(data.search_nodes[handle].f_value, data.search_nodes[data.open_list.template top<1>()].f_value);
                    data.error = (data.incumbent.f_value / min_f) - 1.0f;

                    if(old_error != data.error)
                    {
                        // std::cerr << "UB: " << data.incumbent.f_value << ", LB: " << min_f << ", Error: " << data.error << std::endl;
                        // <expanded, error, UB, nodes, buckets>
                        data.per_bound_data.emplace_back(data.expanded, data.error, data.incumbent.f_value, data.open_list.size(), data.open_list.count());
                    }

                    // utils::time_op(data.queue_time, [&](){
                    //     auto const min_f = std::min(data.search_nodes[handle].f_value, data.search_nodes[data.open_list.template top<1>()].f_value);
                    //     data.error = (data.incumbent.f_value / static_cast<float>(min_f)) - 1.0f;
                    // });
                }
                else data.error = 0.0f;
            }
            else
            {
                // (G - g(n)) / h(n) = (G - f(n)) / h(n) + 1 -> subtract 1 to get error
                auto const old_error = data.error;

                float const e = (data.incumbent.f_value - data.search_nodes[handle].f_value) / static_cast<float>(data.search_nodes[handle].h_value);

                data.error = std::min(data.error, e);

                if(data.error != old_error)
                {
                    // <expanded, error, UB, nodes, buckets>
                    data.per_bound_data.emplace_back(data.expanded, data.error, data.incumbent.f_value, data.open_list.size(), data.open_list.count());
                }
            }
        }

        template<typename SearchTraits>
        static inline void update_incumbent(typename SearchTraits::search_data& data) noexcept
        {
            if constexpr(dual_queue<typename SearchTraits::queue_type> && !std::is_base_of_v<priority_queue_policies::binary_heap<typename SearchTraits::node_handle,
                                                            typename SearchTraits::node_compare,
                                                            typename SearchTraits::queue_info_accessor>,
                                        typename SearchTraits::search_data::open_list_type>)
            {
                if(!data.open_list.is_empty())
                {
                    float const min_f = data.search_nodes[data.open_list.template top<1>()].f_value;
                    data.error = (data.incumbent.f_value / min_f) - 1.0f;

                    // std::cerr << "UB: " << data.incumbent.f_value << ", LB: " << min_f << ", Error: " << data.error << std::endl;
                }
                // utils::time_op(data.queue_time, [&](){
                //     if(!data.open_list.is_empty())
                //     {
                //         data.error = (data.incumbent.f_value / static_cast<float>(data.search_nodes[data.open_list.template top<1>()].f_value)) - 1.0f;
                //     }
                // });
            }
            // <expanded, error, UB, nodes, buckets>
            data.per_bound_data.emplace_back(data.expanded, data.error, data.incumbent.f_value, data.open_list.size(), data.open_list.count());

            // utils::time_op(data.queue_time, [&](){
            //     data.open_list.reorder(data.incumbent.f_value);
            // });
            // ++data.reorders;
            data.open_list.reorder(data.incumbent.f_value);
        }

        // template<typename SearchTraits>
        // static void update_open_closed(typename SearchTraits::domain::packed_state&& packed_successor, typename SearchTraits::node_handle const parent_handle,
        //                                 typename SearchTraits::search_node::f_type f_value, typename SearchTraits::search_node::g_type g_value, typename SearchTraits::search_node::h_type h_value,
        //                                 typename SearchTraits::domain::action const& action, typename SearchTraits::search_data& data)
        // {
        //     auto& nodes = data.search_nodes;
        //     // auto const pair = data.hash_table[h_value].try_emplace(packed_successor, typename SearchTraits::node_handle{static_cast<int>(nodes.size())});

        //     // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //     // Might be better to do a find op, then give a (small) custom object as the key, instead of the packed_state
        //     // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            
        //     auto const pair = data.hash_table[h_value].try_emplace(std::move(packed_successor), typename SearchTraits::node_handle{static_cast<int>(nodes.size())});
        //     // auto const pair = utils::time_op(data.closed_time, [&](){
        //     //     return data.hash_table[h_value].try_emplace(std::move(packed_successor), typename SearchTraits::node_handle{static_cast<int>(nodes.size())});
        //     // });

        //     // Newly encountered state
        //     if(pair.second)
        //     {
        //         nodes.emplace_back(
        //             pair.first->first,         // pstate
        //             parent_handle,                             // parent_handle
        //             typename SearchTraits::queue_loc_info{},   // queue_info
        //             f_value,  // f_value
        //             h_value,                            // h_value
        //             action                              // generating_action
        //         );
        //         // utils::time_op(data.emplace_time, [&](){
        //         //     nodes.emplace_back(
        //         //         pair.first->first,         // pstate
        //         //         parent_handle,                             // parent_handle
        //         //         typename SearchTraits::queue_loc_info{},   // queue_info
        //         //         f_value,  // f_value
        //         //         h_value,                            // h_value
        //         //         action                              // generating_action
        //         //     );
        //         // });

        //         data.open_list.push(typename SearchTraits::node_handle{static_cast<int>(nodes.size()) - 1});
        //         // utils::time_op(data.queue_time, [&](){
        //         //     data.open_list.push(typename SearchTraits::node_handle{static_cast<int>(nodes.size()) - 1});
        //         // });
        //         // ++data.enqueues;
        //     }
        //     // Improved path to existing node
        //     else if(g_value < nodes[pair.first->second].f_value - nodes[pair.first->second].h_value)
        //     {
        //         auto const old_f_value = nodes[pair.first->second].f_value;
        //         nodes[pair.first->second].f_value = f_value;
        //         nodes[pair.first->second].parent_handle = parent_handle;
        //         nodes[pair.first->second].generating_action = action;

        //         // Exists in OPEN
        //         if(nodes[pair.first->second].queue_info != typename SearchTraits::queue_loc_info{})
        //         {
        //             data.open_list.decrease_key(nodes[pair.first->second].queue_info);
        //             // utils::time_op(data.queue_time, [&](){
        //             //     data.open_list.decrease_key(nodes[pair.first->second].queue_info);
        //             // });
        //             // ++data.decreases;
        //         }
        //         // Exists in CLOSED
        //         else
        //         {
        //             data.open_list.push(pair.first->second);
        //             // utils::time_op(data.queue_time, [&](){
        //             //     data.open_list.push(pair.first->second);
        //             // });
        //             // ++data.enqueues;
        //         }
                
        //         // // Exists in OPEN
        //         // if(nodes[pair.first->second].queue_info != typename SearchTraits::queue_loc_info{}
        //         // && (data.incumbent.parent_handle != typename SearchTraits::node_handle{} && old_f_value < data.incumbent.f_value))
        //         // {
        //         //     data.open_list.decrease_key(nodes[pair.first->second].queue_info);
        //         // }
        //         // // Exists in CLOSED
        //         // else
        //         // {
        //         //     data.open_list.push(pair.first->second);
        //         // }

        //         // THIS IS WRONG!!!
        //         // old_f_value can be greater without a reorder having pruned it.
        //         // if((data.incumbent.parent_handle != typename SearchTraits::node_handle{} && old_f_value >= data.incumbent.f_value)
        //         // || nodes[pair.first->second].queue_info == typename SearchTraits::queue_loc_info{})
        //         // {
        //         //     data.open_list.push(pair.first->second);
        //         // }
        //         // else
        //         // {
        //         //     data.open_list.decrease_key(nodes[pair.first->second].queue_info);
        //         // }
        //     }
        // }
    };
}