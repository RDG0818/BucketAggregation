#pragma once

#include <concepts>
#include <type_traits>
#include <vector>

#include <utils/parallel_hashmap/phmap.h>
#include <utils/ordered_set.hpp>

namespace search::helpers
{
    template<typename T>
    concept Hashable = requires (T const& val) {
        { hash_value(val) } -> std::convertible_to<std::size_t>;
    };

    template<typename NodeType, typename HandleType>
    struct hash_key
    {
        hash_key(std::vector<NodeType>& nodes, HandleType handle) :
            nodes(nodes),
            handle(handle)
        {}

        [[nodiscard]] inline constexpr friend auto hash_value(hash_key const& hk) noexcept -> std::size_t
        {
            return hk.nodes[hk.handle].pstate.key;
        }

        [[nodiscard]] constexpr friend bool operator==(hash_key const& lhs, hash_key const& rhs) noexcept
        {
            return lhs.nodes[lhs.handle].pstate == rhs.nodes[rhs.handle].pstate;
        }

        template<typename PackedState>
        [[nodiscard]] constexpr friend bool operator==(hash_key const& lhs, PackedState const& rhs) noexcept
        {
            return lhs.nodes[lhs.handle].pstate == rhs;
        }

        template<typename PackedState>
        [[nodiscard]] constexpr friend bool operator==(PackedState const& lhs, hash_key const& rhs) noexcept
        {
            return lhs == rhs.nodes[rhs.handle].pstate;
        }

        std::vector<NodeType>& nodes;
        HandleType handle;
    };

    template<typename T>
    [[nodiscard]] inline constexpr auto hash_value(T&& val) noexcept -> std::size_t
    {
        return val.key;
    }

    struct transparent_hash
    {
        using is_transparent = void;

        template<Hashable T>
        [[nodiscard]] inline constexpr auto operator()(T&& val) const noexcept -> std::size_t
        {
            return hash_value(std::forward<T>(val));
        }
    };

    template<typename T>
    concept Small = sizeof(T) <= 1000000000;

    // template<typename SearchNode, typename NodeHandle, bool IsSmall>
    // struct hash_table
    // {
    //     using type = utils::ordered_set<
    //                     phmap::flat_hash_set<
    //                         hash_key<SearchNode, NodeHandle>,
    //                         transparent_hash,
    //                         std::equal_to<>
    //                     >
    //                 >;
    // };

    // template<typename SearchNode, typename NodeHandle>
    // struct hash_table<SearchNode, NodeHandle, true>
    // {
    //     using type = utils::ordered_set<phmap::flat_hash_map<typename SearchNode::packed_state, NodeHandle>>;
    // };

    // template<typename SearchNode, typename NodeHandle>
    // using hash_table_t = hash_table<SearchNode, NodeHandle, Small<typename SearchNode::packed_state>>::type;

    template<typename SearchNode, typename NodeHandle>
    using hash_table_t = utils::ordered_set<phmap::flat_hash_map<typename SearchNode::packed_state, NodeHandle>>;
}