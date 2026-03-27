#pragma once

#include <search/search_traits_checks.hpp>
#include <search/search_helpers.hpp>
#include <search/default_search_traits.hpp>

template<typename SearchTraits>
struct anytime_search_data : public default_search_data<SearchTraits>
{
    // <expanded, error, UB, nodes, buckets>
    std::vector<std::tuple<int, float, int, int, int>> per_bound_data;

    float epsilon = 0.0f;
    float error{std::numeric_limits<float>::infinity()};

    std::size_t pruned{};

    std::chrono::nanoseconds apply_time{};

    anytime_search_data() : default_search_data<SearchTraits>()
    {}
};

template<typename SearchTraits>
[[nodiscard]] inline bool anytime_should_terminate(typename SearchTraits::search_data const& data) noexcept
{
    return data.open_list.is_empty() || data.error <= data.epsilon;
        // || data.expanded >= 10'000'000; // || data.generated >= 60'000'000; || (data.incumbent.f_value == 0 && data.expanded >= 25'000'000);
}

template<typename SearchTraits>
[[nodiscard]] auto anytime_get_fringe_node(typename SearchTraits::search_data& data) -> typename SearchTraits::node_handle
{
    typename SearchTraits::node_handle handle{};

    do
    {
        handle = data.open_list.top();
        data.open_list.pop();

        if(data.incumbent.parent_handle != typename SearchTraits::node_handle{})
        {
            if constexpr(has_update_error_bound<SearchTraits>)
            {
                SearchTraits::algorithm::template update_error_bound<SearchTraits>(handle, data);

                // Proven desired optimality.
                if(data.error <= data.epsilon) return typename SearchTraits::node_handle{};
            }

            // Prune this node?
            if(data.search_nodes[handle].f_value >= data.incumbent.f_value)
            {
                handle = typename SearchTraits::node_handle{};
            }
        }
    } while (handle == typename SearchTraits::node_handle{} && !data.open_list.is_empty());

    return handle;
}

template<typename SearchTraits>
void anytime_handle_successor(typename SearchTraits::domain::packed_state&& packed_successor, typename SearchTraits::domain::h_type const h_value,
                            typename SearchTraits::node_handle const parent_handle, typename SearchTraits::domain::action const& action,
                            typename SearchTraits::domain const& domain, typename SearchTraits::search_data& data)
{
    auto& nodes = data.search_nodes;
    auto const g_value = nodes[parent_handle].f_value - nodes[parent_handle].h_value + action.cost();
    auto const f_value = g_value + h_value;

    if(data.incumbent.parent_handle != typename SearchTraits::node_handle{}
    && f_value >= data.incumbent.f_value)
    {
        ++data.pruned;
        return;
    }
    else
    if(h_value == 0)
    {
        data.incumbent = typename SearchTraits::search_node{
            std::move(packed_successor),         // pstate
            parent_handle,                             // parent_handle
            typename SearchTraits::queue_loc_info{},   // queue_info
            f_value,  // f_value
            h_value,                            // h_value
            action                              // generating_action
        };

        if constexpr(has_update_incumbent<SearchTraits>)
        {
            SearchTraits::algorithm::template update_incumbent<SearchTraits>(data);
        }
    }
    else
    {
        SearchTraits::update_open_closed(std::move(packed_successor), parent_handle, f_value, g_value, h_value, action, data);
    }
}

template<typename SearchTraits>
void anytime_expand_node(typename SearchTraits::node_handle const handle, typename SearchTraits::search_data& data, typename SearchTraits::domain const& domain)
{
    ++data.expanded; //

    // Technically results look a bit better the other way, because the bucket heap has less cache misses than the binary/dual heaps...
    auto state = domain.unpack(data.search_nodes[handle].pstate);
    
    std::vector<std::tuple<typename SearchTraits::domain::packed_state, typename SearchTraits::domain::h_type, typename SearchTraits::domain::action>> successors;

    successors.reserve(64);

    for(auto const& action : domain.actions(state, data.search_nodes[handle].generating_action))
    {
        ++data.generated; //

        auto const g_value = data.search_nodes[handle].f_value - data.search_nodes[handle].h_value + action.cost();
        auto const f_value = g_value + data.search_nodes[handle].h_value - domain.max_heuristic_delta; // minus maximum change in h, for consistency reasons

        if(data.incumbent.parent_handle != typename SearchTraits::node_handle{}
        && f_value >= data.incumbent.f_value)
        {
            ++data.pruned; //
            continue;
        }

        auto packed_successor_optional = domain.apply(action, state);

        // auto packed_successor_optional = utils::time_op(data.apply_time, [&](){
        //     return domain.apply(action, state);
        // });

        if(packed_successor_optional)
        {
            successors.emplace_back(std::move(*packed_successor_optional), domain.heuristic_value(state, action, data.search_nodes[handle].h_value), action);
        }

        domain.undo(action, state);
    }

    for(auto&& [ps, h, a] : successors)
    {
        SearchTraits::handle_successor(std::move(ps), h, handle, a, domain, data);
    }
}