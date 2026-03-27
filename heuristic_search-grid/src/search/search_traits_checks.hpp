#pragma once

#include <type_traits>
#include <concepts>

template<typename SearchTraits>
concept has_update_error_bound = requires (SearchTraits::node_handle h, SearchTraits::search_data sd) {
    SearchTraits::algorithm::template update_error_bound<SearchTraits>(h, sd);
};

template<typename SearchTraits>
concept has_update_incumbent = requires (SearchTraits::search_data sd) {
    SearchTraits::algorithm::template update_incumbent<SearchTraits>(sd);
};

template<typename SearchTraits>
concept has_initialize_structures = requires (SearchTraits::search_data sd) {
    SearchTraits::algorithm::template initialize_structures<SearchTraits>(sd);
};

template<typename SearchTraits>
concept has_initialize_search = requires (SearchTraits::search_data sd, SearchTraits::domain d) {
    SearchTraits::algorithm::template initialize_search<SearchTraits>(sd, d);
};

template<typename SearchTraits>
concept has_should_terminate = requires(SearchTraits::search_data sd){
    { SearchTraits::algorithm::template should_terminate<SearchTraits>(sd) } -> std::same_as<bool>;
};

template<typename SearchTraits>
concept has_get_fringe_node = requires(SearchTraits::search_data sd){
    SearchTraits::algorithm::template get_fringe_node<SearchTraits>(sd);
};

template<typename SearchTraits>
concept has_expand_node = requires(SearchTraits::node_handle h, SearchTraits::search_data sd, SearchTraits::domain d){
    SearchTraits::algorithm::template expand_node<SearchTraits>(h, sd, d);
};

template<typename SearchTraits>
concept has_handle_successor = requires(SearchTraits::domain::packed_state&& ps, SearchTraits::domain::h_type h, SearchTraits::node_handle ph, SearchTraits::domain::action a, SearchTraits::domain d,
                                            SearchTraits::search_data sd){
    SearchTraits::algorithm::template handle_successor<SearchTraits>(ps, h, ph, a, d, sd);
};

template<typename SearchTraits>
concept has_update_open_closed = requires(SearchTraits::domain::packed_state&& ps, SearchTraits::node_handle ph, SearchTraits::search_node::f_type f, SearchTraits::search_node::g_type g,
                                            SearchTraits::search_node::h_type h,  SearchTraits::domain::action a, SearchTraits::search_data sd){
    SearchTraits::algorithm::template update_open_closed<SearchTraits>(ps, ph, f, g, h, a, sd);
};