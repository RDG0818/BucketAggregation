#pragma once

#include <search/search_traits.hpp>
#include <priority_queues/priority_queue_traits.hpp>
#include <priority_queues/policies/binary_heap.hpp>
#include <priority_queues/policies/bucket_heap.hpp>

namespace search::algorithms
{
    template<typename Domain>
    struct moptimistic
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
        struct search_data final : public default_search_data<SearchTraits>
        {
            float epsilon = 4.0f;
            float error{std::numeric_limits<float>::infinity()};
            float w;

            search_data() : w(1.0f * epsilon + 1.0f), default_search_data<SearchTraits>()
            {}
        };

        template<typename SearchTraits>
        [[nodiscard]] static constexpr auto initialize_structures(typename SearchTraits::search_data& data) -> SearchTraits::search_data::open_list_type
        {
            if constexpr(std::is_base_of_v<priority_queue_policies::dual_heap<typename SearchTraits::node_handle,
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

        template<typename SearchTraits>
        [[nodiscard]] static inline bool should_terminate(typename SearchTraits::search_data const& data) noexcept
        {
            return data.open_list.is_empty() || data.error <= data.epsilon
                || data.expanded >= 5'000'000;
        }

        template<typename SearchTraits>
        [[nodiscard]] static auto get_fringe_node(typename SearchTraits::search_data& data) -> typename SearchTraits::node_handle
        {
            static int count = 0;
            typename SearchTraits::node_handle handle;
            ++count;

            if(count & 0x01)
            {
                handle = data.open_list.top();

                if(data.incumbent.parent_handle != typename SearchTraits::node_handle{})
                {
                    data.error = data.incumbent.f_value / static_cast<float>(data.search_nodes[data.open_list.template top<1>()].f_value) - 1.0f;

                    float const top_fw = data.search_nodes[handle].f_value - data.search_nodes[handle].h_value + data.w * data.search_nodes[handle].h_value;
                    float const incumbent_fw = data.incumbent.f_value - data.incumbent.h_value + data.w * data.incumbent.h_value;

                    if(top_fw < incumbent_fw)
                    {
                        if(data.search_nodes[handle].h_value == 0) data.incumbent = data.search_nodes[handle];

                        data.open_list.pop();
                    }
                    else
                    {
                        handle = data.open_list.template top<1>();
                        data.open_list.template pop<1>();
                    }
                }
                else
                {
                    if(data.search_nodes[handle].h_value == 0) 
                    {
                        data.incumbent = data.search_nodes[handle];
                    }

                    data.open_list.pop();
                }
            }
            else
            {
                handle = data.open_list.template top<1>();
                data.open_list.template pop<1>();
            }

            return handle;
        }
    };
}