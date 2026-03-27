#pragma once

#include <search/search_traits.hpp>
#include <priority_queues/priority_queue_traits.hpp>
#include <priority_queues/policies/binary_heap.hpp>
#include <priority_queues/policies/bucket_heap.hpp>
#include <priority_queues/policies/gilon_heap.hpp>

namespace search::algorithms
{
    template<typename Domain>
    struct mdps
    {
        using domain_type = Domain;

        template<typename SearchTraits>
        struct node_compare
        {
            node_compare(std::vector<typename SearchTraits::search_node> const& nodes, float const& cost_bound) : nodes(nodes), cost_bound(cost_bound)
            {}

            [[nodiscard]] inline bool operator()(typename SearchTraits::node_handle const lhs, typename SearchTraits::node_handle const rhs) const noexcept
            {
                auto const& lnode = nodes[lhs];
                auto const& rnode = nodes[rhs];

                // // Compare potentials: (C - g(n)) / h(n)
                // return lnode.h_value * (rnode.f_value - cost_bound) > rnode.h_value * (lnode.f_value - cost_bound);

                float left_potential = (lnode.h_value > 0) ? (cost_bound - lnode.f_value + lnode.h_value) / static_cast<float>(lnode.h_value) : std::numeric_limits<float>::infinity();
                float right_potential = (rnode.h_value > 0) ? (cost_bound - rnode.f_value + rnode.h_value) / static_cast<float>(rnode.h_value) : std::numeric_limits<float>::infinity();

                if(lnode.f_value > cost_bound && lnode.h_value == 0) left_potential = (cost_bound - lnode.f_value + lnode.h_value);
                if(rnode.f_value > cost_bound && rnode.h_value == 0) right_potential = (cost_bound - rnode.f_value + rnode.h_value);

                if(left_potential == right_potential) 
                {
                    return lnode.h_value < rnode.h_value;
                }

                return left_potential > right_potential;
            }

        private:
            std::vector<typename SearchTraits::search_node> const& nodes;
            float const& cost_bound;
        };

        template<typename SearchTraits>
        struct search_data : public default_search_data<SearchTraits>
        {
            float cost_bound;
            float epsilon;

            search_data() : epsilon(1.0f), default_search_data<SearchTraits>()
            {}
        };

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
                                                            node_compare<SearchTraits>,
                                                            default_node_compare<SearchTraits>,
                                                            default_queue_info_accessor<SearchTraits>>,
                                            typename SearchTraits::search_data::open_list_type>)
            {
                return typename SearchTraits::search_data::open_list_type(node_compare<SearchTraits>(data.search_nodes, data.cost_bound),
                                                                        default_node_compare<SearchTraits>(data.search_nodes),
                                                                        default_queue_info_accessor<SearchTraits>(data.search_nodes));
            }
            else
            {
                return typename SearchTraits::search_data::open_list_type(default_node_priority_pair<SearchTraits>(data.search_nodes),
                                                                        node_compare<SearchTraits>(data.search_nodes, data.cost_bound),
                                                                        default_queue_info_accessor<SearchTraits>(data.search_nodes));
            }
        }

        template<typename SearchTraits>
            requires dual_queue<typename SearchTraits::queue_type>
        [[nodiscard]] static auto get_fringe_node(typename SearchTraits::search_data& data) -> typename SearchTraits::node_handle
        {
            static int count = 0;
            typename SearchTraits::node_handle handle;

            ++count;

            auto const old_bound = data.cost_bound;
            data.cost_bound = (1.0f + data.epsilon) * data.search_nodes[data.open_list.template top<1>()].f_value;

            if(old_bound != data.cost_bound) data.open_list.reorder(data.cost_bound);

            if(count & 0x01)
            {
                handle = data.open_list.top();
                data.open_list.pop();
            }
            else
            {
                handle = data.open_list.template top<1>();
                data.open_list.template pop<1>();
            }

            // Check goal condition and update incumbent solution
            if(data.search_nodes[handle].h_value == 0) 
            {
                data.incumbent = data.search_nodes[handle];
            }

            return handle;
        }
    };
}