#pragma once

#include <search/search_traits.hpp>
#include <priority_queues/priority_queue_traits.hpp>
#include <priority_queues/policies/binary_heap.hpp>
#include <priority_queues/policies/bucket_heap.hpp>

namespace search::algorithms
{
    template<typename Domain>
    struct wa
    {
        using domain_type = Domain;

        template<typename SearchTraits>
        struct node_compare
        {
            node_compare(std::vector<typename SearchTraits::search_node> const& nodes, float const& w) : nodes(nodes), w(w)
            {}

            [[nodiscard]] inline bool operator()(typename SearchTraits::node_handle const lhs, typename SearchTraits::node_handle const rhs) const noexcept
            {
                auto const& lnode = nodes[lhs];
                auto const& rnode = nodes[rhs];

                return (w - 1.0f) * (lnode.h_value - rnode.h_value) < rnode.f_value - lnode.f_value;
            }

        private:
            std::vector<typename SearchTraits::search_node> const& nodes;
            float const& w;
        };

        template<typename SearchTraits>
        struct search_data : public default_search_data<SearchTraits>
        {
            float w;

            search_data() : w(16.5f), default_search_data<SearchTraits>()
            {}
        };

        template<typename SearchTraits>
        [[nodiscard]] static constexpr auto initialize_structures(typename SearchTraits::search_data& data) -> SearchTraits::search_data::open_list_type
        {
            if constexpr(std::is_same_v<priority_queue_policies::binary_heap<typename SearchTraits::node_handle,
                                                            node_compare<SearchTraits>,
                                                            default_queue_info_accessor<SearchTraits>>,
                                        typename SearchTraits::search_data::open_list_type>)
            {
                return typename SearchTraits::search_data::open_list_type(node_compare<SearchTraits>(data.search_nodes, data.w),
                                                                        default_queue_info_accessor<SearchTraits>(data.search_nodes));
            }
            else if constexpr(std::is_same_v<priority_queue_policies::dual_heap<typename SearchTraits::node_handle,
                                                            node_compare<SearchTraits>,
                                                            default_node_compare<SearchTraits>,
                                                            default_queue_info_accessor<SearchTraits>>,
                                            typename SearchTraits::search_data::open_list_type>)
            {
                return typename SearchTraits::search_data::open_list_type(node_compare<SearchTraits>(data.search_nodes, data.w),
                                                                        default_node_compare<SearchTraits>(data.search_nodes),
                                                                        default_queue_info_accessor<SearchTraits>(data.search_nodes));
            }
            else
            {
                return typename SearchTraits::search_data::open_list_type(default_node_priority_pair<SearchTraits>(data.search_nodes),
                                                                        node_compare<SearchTraits>(data.search_nodes, data.w),
                                                                        default_queue_info_accessor<SearchTraits>(data.search_nodes));
            }
        }
    };
}