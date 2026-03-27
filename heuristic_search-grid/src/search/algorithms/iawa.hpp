#pragma once

#include <vector>
#include <memory>
#include <priority_queues/policies/binary_heap.hpp>
#include <utils/parallel_hashmap/phmap.h>

namespace search::algorithms
{
    template<typename Domain>//, template<typename, typename> typename QueueType>
    struct iawa
    {
        using domain_type = Domain;
        using is_anytime_algorithm = std::true_type;

        template<typename SearchTraits>
        struct node_compare
        {
            node_compare(std::vector<typename SearchTraits::search_node> const& nodes, float const& w) : nodes(nodes), w(w)
            {}

            [[nodiscard]] inline bool operator()(typename SearchTraits::node_handle const lhs, typename SearchTraits::node_handle const rhs) const noexcept
            {
                auto const& lnode = nodes[lhs];
                auto const& rnode = nodes[rhs];

                if(w == std::numeric_limits<float>::infinity()) return lnode.h_value < rnode.h_value;

                // Rearrange f_w(lnode) < f_w(rnode)
                return (w - 1.0f) * (lnode.h_value - rnode.h_value) < rnode.f_value - lnode.f_value;
            }

        private:
            std::vector<typename SearchTraits::search_node> const& nodes;
            float const& w;
        };

        template<typename SearchTraits>
        struct search_data : public anytime_search_data<SearchTraits>
        {
            float w;

            search_data() : w(std::numeric_limits<float>::infinity()), anytime_search_data<SearchTraits>()
            {}
        };

        template<typename SearchTraits>
        [[nodiscard]] static constexpr auto initialize_structures(typename SearchTraits::search_data& data) -> SearchTraits::search_data::open_list_type
        {
            if constexpr(std::is_base_of_v<priority_queue_policies::binary_heap<typename SearchTraits::node_handle,
                                                            typename SearchTraits::node_compare,
                                                            typename SearchTraits::queue_info_accessor>,
                                        typename SearchTraits::search_data::open_list_type>)
            {
                throw;
            }
            else if constexpr(std::is_base_of_v<priority_queue_policies::dual_heap<typename SearchTraits::node_handle,
                                                            typename SearchTraits::node_compare,
                                                            default_node_compare<SearchTraits>,
                                                            typename SearchTraits::queue_info_accessor>,
                                            typename SearchTraits::search_data::open_list_type>)
            {
                return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_compare(data.search_nodes, data.w),
                                                                        default_node_compare<SearchTraits>(data.search_nodes),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
            }
            else
            {
                return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_priority_pair(data.search_nodes),
                                                                        typename SearchTraits::node_compare(data.search_nodes, data.w),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
            }
        }

        template<typename SearchTraits>
            requires dual_queue<typename SearchTraits::queue_type>
        static inline void update_error_bound(typename SearchTraits::node_handle const handle, typename SearchTraits::search_data& data) noexcept
        {
            if(!data.open_list.is_empty())
            {
                auto const old_error = data.error;

                float const min_f = std::min(data.search_nodes[handle].f_value, data.search_nodes[data.open_list.template top<1>()].f_value);
                data.w = data.incumbent.f_value / min_f;

                data.error = data.w - 1.0f;

                if(data.error != old_error)
                {
                    // <expanded, error, UB, nodes, buckets>
                    data.per_bound_data.emplace_back(data.expanded, data.error, data.incumbent.f_value, data.open_list.size(), data.open_list.count());

                    data.open_list.reorder(data.incumbent.f_value);
                }
            }
            else data.error = 0.0f;
        }

        template<typename SearchTraits>
            requires dual_queue<typename SearchTraits::queue_type>
        static inline void update_incumbent(typename SearchTraits::search_data& data) noexcept
        {
            if(!data.open_list.is_empty())
            {
                float const min_f = data.search_nodes[data.open_list.template top<1>()].f_value;
                data.w = data.incumbent.f_value / min_f;
                data.error = data.w - 1.0f;

                // <expanded, error, UB, nodes, buckets>
                data.per_bound_data.emplace_back(data.expanded, data.error, data.incumbent.f_value, data.open_list.size(), data.open_list.count());
            }

            data.open_list.reorder(data.incumbent.f_value);
        }

        // template<typename SearchTraits>
        // static void update_open_closed(typename SearchTraits::domain::packed_state&& packed_successor, typename SearchTraits::node_handle const parent_handle,
        //                                 typename SearchTraits::search_node::f_type f_value, typename SearchTraits::search_node::g_type g_value, typename SearchTraits::search_node::h_type h_value,
        //                                 typename SearchTraits::domain::action const& action, typename SearchTraits::search_data& data)
        // {
        //     auto& nodes = data.search_nodes;
            
        //     auto const pair = data.hash_table[h_value].try_emplace(std::move(packed_successor), typename SearchTraits::node_handle{static_cast<int>(nodes.size())});

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

        //         data.open_list.push(typename SearchTraits::node_handle{static_cast<int>(nodes.size()) - 1});
        //     }
        //     // Improved path to existing node
        //     else if(g_value < nodes[pair.first->second].f_value - nodes[pair.first->second].h_value)
        //     {
        //         auto const old_f_value = nodes[pair.first->second].f_value;
        //         nodes[pair.first->second].f_value = f_value;
        //         nodes[pair.first->second].parent_handle = parent_handle;
        //         nodes[pair.first->second].generating_action = action;
                
        //         // THIS IS WRONG!!! Why?
        //         // old_f_value can be greater without a reorder having pruned it. How?
        //         if((data.incumbent.parent_handle != typename SearchTraits::node_handle{} && old_f_value >= data.incumbent.f_value)
        //         || nodes[pair.first->second].queue_info == typename SearchTraits::queue_loc_info{})
        //         {
        //             data.open_list.push(pair.first->second);
        //         }
        //         else
        //         {
        //             data.open_list.decrease_key(nodes[pair.first->second].queue_info);
        //         }
        //     }
        // }
    };
}