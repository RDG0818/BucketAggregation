#pragma once

#include <limits>

#include <utils/parallel_hashmap/phmap.h>
#include <utils/time_op.hpp>
#include <search/search_traits_checks.hpp>
#include <search/search_helpers.hpp>

#include <priority_queues/policies/binary_heap.hpp>
#include <priority_queues/policies/dual_heap.hpp>
#include <priority_queues/policies/bucket_heap.hpp>
#include <priority_queues/policies/buffered_bucket_heap.hpp>

template<typename SearchTraits>
struct default_search_data
{
    using node_handle_type = typename SearchTraits::node_handle;
    using search_node_type = typename SearchTraits::search_node;
    using node_compare_type = typename SearchTraits::node_compare;
    using node_priority_pair_type = typename SearchTraits::node_priority_pair;
    using queue_info_accessor_type = typename SearchTraits::queue_info_accessor;

    using open_list_type = typename SearchTraits::queue_type;
    using hash_table_type = search::helpers::hash_table_t<search_node_type, node_handle_type>;
    // using hash_table_type = phmap::flat_hash_map<typename search_node_type::packed_state, node_handle_type>;

    open_list_type open_list;
    hash_table_type hash_table;
    std::vector<search_node_type> search_nodes;
    search_node_type incumbent{};
    std::size_t generated{}, expanded{}, h0;
    // std::chrono::nanoseconds hash_time{};

    default_search_data()
        : open_list(SearchTraits::initialize_structures(static_cast<typename SearchTraits::search_data&>(*this)))
    {}
};

struct default_node_handle
{
    int index{std::numeric_limits<int>::max()};

    [[nodiscard]] inline operator int() const noexcept
    {
        return index;
    }
};

template<typename SearchTraits>
struct default_search_node
{
    using g_type = SearchTraits::domain::g_type;
    using h_type = typename SearchTraits::domain::h_type;
    using f_type = decltype(g_type{} + h_type{});
    using packed_state = typename SearchTraits::domain::packed_state;
    using action = typename SearchTraits::domain::action;

    packed_state pstate;
    typename SearchTraits::node_handle parent_handle;
    typename SearchTraits::queue_loc_info queue_info;
    f_type f_value;
    h_type h_value;
    action generating_action;
};

template<typename SearchTraits>
struct default_node_compare
{
    default_node_compare(std::vector<typename SearchTraits::search_node> const& nodes) : nodes(nodes)
    {}

    [[nodiscard]] inline bool operator()(SearchTraits::node_handle const lhs, SearchTraits::node_handle const rhs) const noexcept
    {
        auto const& lnode = nodes[lhs];
        auto const& rnode = nodes[rhs];

        return  (lnode.f_value < rnode.f_value) || ((lnode.f_value == rnode.f_value) && (lnode.h_value < rnode.h_value));
    }

private:
    std::vector<typename SearchTraits::search_node> const& nodes;
};

template<typename SearchTraits>
struct default_node_priority_pair
{
    default_node_priority_pair(std::vector<typename SearchTraits::search_node> const& nodes) : nodes(nodes)
    {}

    [[nodiscard]] inline auto operator()(SearchTraits::node_handle const handle) const noexcept
    {
        return std::make_pair(nodes[handle].f_value, nodes[handle].h_value);
    }

private:
    std::vector<typename SearchTraits::search_node> const& nodes;
};

template<typename SearchTraits>
struct default_queue_info_accessor
{
    default_queue_info_accessor(std::vector<typename SearchTraits::search_node>& nodes) : nodes(nodes)
    {}

    [[nodiscard]] inline auto& operator()(SearchTraits::node_handle const handle) const noexcept
    {
        return nodes[handle].queue_info;
    }

private:
    std::vector<typename SearchTraits::search_node>& nodes;
};

template<typename SearchTraits>
[[nodiscard]] auto default_initialize_structures(typename SearchTraits::search_data& data) -> SearchTraits::search_data::open_list_type
{
    if constexpr(std::is_base_of_v<priority_queue_policies::binary_heap<typename SearchTraits::node_handle,
                                                            typename SearchTraits::node_compare,
                                                            typename SearchTraits::queue_info_accessor>,
                                        typename SearchTraits::search_data::open_list_type>)
    {
        return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_compare(data.search_nodes),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
    }
    else
    {
        return typename SearchTraits::search_data::open_list_type(typename SearchTraits::node_priority_pair(data.search_nodes),
                                                                        typename SearchTraits::queue_info_accessor(data.search_nodes));
    }
}

template<typename SearchTraits>
void default_initialize_search(typename SearchTraits::search_data& data, typename SearchTraits::domain const& domain)
{
    auto const start_state = domain.initial_state();
    auto const h_value = domain.heuristic_value(start_state);
    data.h0 = h_value;

    data.search_nodes.emplace_back(
        domain.pack(start_state),                                                                                           // pstate
        typename SearchTraits::node_handle{},                                                                                                 // parent_handle
        typename SearchTraits::queue_loc_info{},  // queue_info
        h_value,                                                                                                            // f_value
        h_value,                                                                                                            // h_value
        typename SearchTraits::domain::action{}                                                                                           // generating_action
    );

    data.open_list.push(typename SearchTraits::node_handle{0});
    
    if constexpr(search::helpers::Small<typename SearchTraits::domain::packed_state>)
    {
        data.hash_table[h_value].emplace(data.search_nodes[0].pstate, typename SearchTraits::node_handle{0});
        // utils::time_op(data.hash_time, [&](){
        //     data.hash_table[h_value].emplace(data.search_nodes[0].pstate, typename SearchTraits::node_handle{0});
        //     // data.hash_table.emplace(data.search_nodes[0].pstate, typename SearchTraits::node_handle{0});
        // });
    }
    else
    {
        data.hash_table[h_value].emplace(data.search_nodes, typename SearchTraits::node_handle{0});
    }
}

template<typename SearchTraits>
[[nodiscard]] inline bool default_should_terminate(typename SearchTraits::search_data const& data) noexcept
{
    return data.open_list.is_empty() || data.incumbent.parent_handle != typename SearchTraits::node_handle{}
         || data.expanded >= 5'000'000;
}

template<typename SearchTraits>
[[nodiscard]] inline auto default_get_fringe_node(typename SearchTraits::search_data& data) -> typename SearchTraits::node_handle
{
    typename SearchTraits::node_handle handle;

    handle = data.open_list.top();
    data.open_list.pop();

    // Check goal condition and update incumbent solution
    if(data.search_nodes[handle].h_value == 0) data.incumbent = data.search_nodes[handle];

    return handle;
}

template<typename SearchTraits>
void default_update_open_closed(typename SearchTraits::domain::packed_state&& packed_successor, typename SearchTraits::node_handle const parent_handle,
                                typename SearchTraits::search_node::f_type const f_value, typename SearchTraits::search_node::g_type const g_value, typename SearchTraits::search_node::h_type const h_value,
                                typename SearchTraits::domain::action const& action, typename SearchTraits::search_data& data)
{
    auto& nodes = data.search_nodes;

    if constexpr(search::helpers::Small<typename SearchTraits::domain::packed_state>)
    {
        auto const pair = data.hash_table[h_value].try_emplace(std::move(packed_successor), typename SearchTraits::node_handle{static_cast<int>(nodes.size())});
        // auto const pair = utils::time_op(data.hash_time, [&](){
        //     return data.hash_table[h_value].try_emplace(std::move(packed_successor), typename SearchTraits::node_handle{static_cast<int>(nodes.size())});
        //     // return data.hash_table.try_emplace(std::move(packed_successor), typename SearchTraits::node_handle{static_cast<int>(nodes.size())});
        // });

        // Newly encountered state
        if(pair.second)
        {
            nodes.emplace_back(
                pair.first->first,         // pstate
                parent_handle,                             // parent_handle
                typename SearchTraits::queue_loc_info{},   // queue_info
                f_value,  // f_value
                h_value,                            // h_value
                action                              // generating_action
            );

            data.open_list.push(typename SearchTraits::node_handle{static_cast<int>(nodes.size()) - 1});
        }
        // Improved path to existing node
        else if(g_value < nodes[pair.first->second].f_value - nodes[pair.first->second].h_value)
        {
            nodes[pair.first->second].f_value = f_value;
            nodes[pair.first->second].parent_handle = parent_handle;
            nodes[pair.first->second].generating_action = action;

            // Exists in OPEN
            if(nodes[pair.first->second].queue_info != typename SearchTraits::queue_loc_info{})
            {
                data.open_list.decrease_key(nodes[pair.first->second].queue_info);
            }
            // Exists in CLOSED
            else
            {
                data.open_list.push(pair.first->second);
            }
        }
    }
    else
    {
        auto const it = data.hash_table[h_value].find(packed_successor, packed_successor.key);

        // Newly encountered state
        if(it == data.hash_table[h_value].end())
        {
            nodes.emplace_back(
                std::move(packed_successor),         // pstate
                parent_handle,                             // parent_handle
                typename SearchTraits::queue_loc_info{},   // queue_info
                f_value,  // f_value
                h_value,                            // h_value
                action                              // generating_action
            );

            data.open_list.push(typename SearchTraits::node_handle{static_cast<int>(nodes.size()) - 1});
            data.hash_table[h_value].emplace(data.search_nodes, typename SearchTraits::node_handle{static_cast<int>(nodes.size()) - 1});
        }
        // Improved path to existing node
        else if(g_value < nodes[it->handle].f_value - nodes[it->handle].h_value)
        {
            nodes[it->handle].f_value = f_value;
            nodes[it->handle].parent_handle = parent_handle;
            nodes[it->handle].generating_action = action;

            // Exists in OPEN
            if(nodes[it->handle].queue_info != typename SearchTraits::queue_loc_info{})
            {
                data.open_list.decrease_key(nodes[it->handle].queue_info);
            }
            // Exists in CLOSED
            else
            {
                data.open_list.push(it->handle);
            }
        }
    }
}

template<typename SearchTraits>
inline void default_handle_successor(typename SearchTraits::domain::packed_state&& packed_successor, typename SearchTraits::domain::h_type const h_value,
                            typename SearchTraits::node_handle const parent_handle, typename SearchTraits::domain::action const& action,
                            typename SearchTraits::domain const& domain, typename SearchTraits::search_data& data)
{
    auto const g_value = data.search_nodes[parent_handle].f_value - data.search_nodes[parent_handle].h_value + action.cost();
    auto const f_value = g_value + h_value;

    default_update_open_closed<SearchTraits>(std::move(packed_successor), parent_handle, f_value, g_value, h_value, action, data);
}

template<typename SearchTraits>
void default_expand_node(typename SearchTraits::node_handle const handle, typename SearchTraits::search_data& data, typename SearchTraits::domain const& domain)
{
    ++data.expanded;

    auto state = domain.unpack(data.search_nodes[handle].pstate);
    
    std::vector<std::tuple<typename SearchTraits::domain::packed_state, typename SearchTraits::domain::h_type, typename SearchTraits::domain::action>> successors;

    for(auto const& action : domain.actions(state, data.search_nodes[handle].generating_action))
    {
        ++data.generated;
        
        auto packed_successor_optional = domain.apply(action, state);

        if(packed_successor_optional)
        {
            successors.emplace_back(std::move(*packed_successor_optional), domain.heuristic_value(state, action, data.search_nodes[handle].h_value), action);
        }

        domain.undo(action, state);
    }

    for(auto&& [ps, h, a] : successors)
    {
        SearchTraits::handle_successor(std::move(ps), h, handle, std::move(a), domain, data);
    }
}