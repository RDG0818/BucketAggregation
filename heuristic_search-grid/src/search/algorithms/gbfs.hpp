#pragma once

#include <vector>
#include <memory>
#include <priority_queues/policies/binary_heap.hpp>
#include <utils/parallel_hashmap/phmap.h>

namespace search::algorithms
{
    template<typename Domain>//, template<typename, typename> typename QueueType>
    struct gbfs
    {
        using domain_type = Domain;
        using is_anytime_algorithm = std::true_type;

        template<typename SearchTraits>
        struct node_compare
        {
            node_compare(std::vector<typename SearchTraits::search_node> const& nodes) : nodes(nodes)
            {}

            [[nodiscard]] inline bool operator()(typename SearchTraits::node_handle const lhs, typename SearchTraits::node_handle const rhs) const noexcept
            {
                auto const& lnode = nodes[lhs];
                auto const& rnode = nodes[rhs];

                return lnode.h_value < rnode.h_value;
            }

        private:
            std::vector<typename SearchTraits::search_node> const& nodes;
        };

        template<typename SearchTraits>
        struct search_data : public anytime_search_data<SearchTraits>
        {
            search_data() : anytime_search_data<SearchTraits>()
            {}
        };

        template<typename SearchTraits>
        [[nodiscard]] static constexpr auto initialize_structures(typename SearchTraits::search_data& data) -> SearchTraits::search_data::open_list_type
        {
            if constexpr(std::is_same_v<priority_queue_policies::dual_heap<typename SearchTraits::node_handle,
                                                            typename SearchTraits::node_compare,
                                                            default_node_compare<SearchTraits>,
                                                            typename SearchTraits::queue_info_accessor>,
                                            typename SearchTraits::search_data::open_list_type>)
            {
                return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_compare(data.search_nodes),
                                                                        default_node_compare<SearchTraits>(data.search_nodes),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
            }
            else
            {
                return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_priority_pair(data.search_nodes),
                                                                        typename SearchTraits::node_compare(data.search_nodes),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
            }
        }

        template<typename SearchTraits>
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
        static inline void update_error_bound(typename SearchTraits::node_handle const handle, typename SearchTraits::search_data& data) noexcept
        {
            if(!data.open_list.is_empty())
            {
                auto const old_error = data.error;

                float const min_f = std::min(data.search_nodes[handle].f_value, data.search_nodes[data.open_list.template top<1>()].f_value);

                data.error = data.incumbent.f_value / min_f - 1.0f;

                if(data.error != old_error)
                {
                    std::cout << "UB: " << data.incumbent.f_value << ", LB: " << min_f << ", Error: " << data.error << std::endl;
                    data.open_list.reorder(data.incumbent.f_value);
                }
            }
            else data.error = 0.0f;
        }

        template<typename SearchTraits>
        static inline void update_incumbent(typename SearchTraits::search_data& data) noexcept
        {
            if(!data.open_list.is_empty())
            {
                float const min_f = data.search_nodes[data.open_list.template top<1>()].f_value;
                data.error = data.incumbent.f_value / min_f - 1.0f;

                std::cout << "UB: " << data.incumbent.f_value << ", LB: " << min_f << ", Error: " << data.error << std::endl;
            }

            data.open_list.reorder(data.incumbent.f_value);
        }
    };
}