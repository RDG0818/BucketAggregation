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
    struct mana
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

                // return lnode.h_value * (rnode.f_value - incumbent.f_value) > rnode.h_value * (lnode.f_value - incumbent.f_value);
            }

        private:
            std::vector<typename SearchTraits::search_node> const& nodes;
            typename SearchTraits::search_node const& incumbent;
        };

        template<typename SearchTraits>
        using search_data = anytime_search_data<SearchTraits>;

        template<typename SearchTraits>
            // requires dual_queue<typename SearchTraits::queue_type>
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
            requires dual_queue<typename SearchTraits::queue_type>
        [[nodiscard]] static auto get_fringe_node(typename SearchTraits::search_data& data) -> typename SearchTraits::node_handle
        {
            static int count = 0;
            typename SearchTraits::node_handle handle{};

            ++count;

            if(count & 0x01)
            {
                do
                {
                    handle = data.open_list.top();
                    data.open_list.pop();

                    if(data.incumbent.parent_handle != typename SearchTraits::node_handle{})
                    {
                        SearchTraits::algorithm::template update_error_bound<SearchTraits>(handle, data);

                        // Proven desired optimality.
                        if(data.error <= data.epsilon) return typename SearchTraits::node_handle{};

                        // Prune this node?
                        if(data.search_nodes[handle].f_value >= data.incumbent.f_value)
                        {
                            ++data.pruned;
                            handle = typename SearchTraits::node_handle{};
                        }
                    }
                } while (handle == typename SearchTraits::node_handle{} && !data.open_list.is_empty());
            }
            else
            {
                handle = data.open_list.template top<1>();
                data.open_list.template pop<1>();
            }

            return handle;
        }

        template<typename SearchTraits>
            requires dual_queue<typename SearchTraits::queue_type>
        static inline void update_error_bound(typename SearchTraits::node_handle const handle, typename SearchTraits::search_data& data) noexcept
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
                data.error = (data.incumbent.f_value / min_f) - 1.0f;

                // std::cerr << "UB: " << data.incumbent.f_value << ", LB: " << min_f << ", Error: " << data.error << std::endl;

                // <expanded, error, UB, nodes, buckets>
                data.per_bound_data.emplace_back(data.expanded, data.error, data.incumbent.f_value, data.open_list.size(), data.open_list.count());
            }

            // ++data.reorders;
            data.open_list.reorder(data.incumbent.f_value);
        }
    };
}