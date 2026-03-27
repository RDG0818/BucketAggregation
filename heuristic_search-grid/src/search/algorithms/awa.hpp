#pragma once

#include <vector>
#include <memory>
#include <priority_queues/policies/binary_heap.hpp>
#include <utils/parallel_hashmap/phmap.h>

namespace search::algorithms
{
    template<typename Domain>
    struct awa
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

            search_data() : w(1.25f), anytime_search_data<SearchTraits>()
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
                return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_compare(data.search_nodes, data.w),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
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
        static inline void update_error_bound(typename SearchTraits::node_handle const handle, typename SearchTraits::search_data& data) noexcept
        {
            if constexpr(dual_queue<typename SearchTraits::queue_type>)
            {
                if(!data.open_list.is_empty())
                {
                    auto const old_error = data.error;
                    auto const min_f = std::min(data.search_nodes[handle].f_value, data.search_nodes[data.open_list.template top<1>()].f_value);
                    data.error = (data.incumbent.f_value / static_cast<float>(min_f)) - 1.0f;

                    if(old_error != data.error)
                    {
                        // std::cerr << "UB: " << data.incumbent.f_value << ", LB: " << min_f << ", Error: " << data.error << std::endl;
                        // <expanded, error, UB, nodes, buckets>
                        data.per_bound_data.emplace_back(data.expanded, data.error, data.incumbent.f_value, data.open_list.size(), data.open_list.count());
                    }
                }
                else data.error = 0.0f;
            }
        }

        template<typename SearchTraits>
        static inline void update_incumbent(typename SearchTraits::search_data& data) noexcept
        {
            if constexpr(dual_queue<typename SearchTraits::queue_type>)
            {
                if(!data.open_list.is_empty())
                {
                    float const min_f = data.search_nodes[data.open_list.template top<1>()].f_value;
                    data.error = (data.incumbent.f_value / min_f) - 1.0f;

                    // std::cerr << "UB: " << data.incumbent.f_value << ", LB: " << min_f << ", Error: " << data.error << std::endl;

                    // <expanded, error, UB, nodes, buckets>
                    data.per_bound_data.emplace_back(data.expanded, data.error, data.incumbent.f_value, data.open_list.size(), data.open_list.count());
                }
            }
        }
    };
}