#pragma once

#include <type_traits>

// Implementation of the detection idiom (negative case).
template<typename Default, typename AlwaysVoid, template<typename...> class Op, typename... Args>
struct detector
{
    using type = Default;
    using is_detected = std::false_type;
};

// Implementation of the detection idiom (positive case).
template<typename Default, template<typename...> typename Op, typename... Args>
struct detector<Default, std::void_t<Op<Args...>>, Op, Args...>
{
    using type = Op<Args...>;
    using is_detected = std::true_type;
};

template<typename Default, template<typename...> typename Op, typename... Args>
using detected_or = detector<Default, void, Op, Args...>;

template<typename Default, template<typename...> typename Op, typename... Args>
using detected_or_t = detected_or<Default, Op, Args...>::type;



// template<typename T, typename Default, typename AlwaysVoid, typename Other>
// struct detect
// {
//     using type = Default;
//     using is_detected = std::false_type;
// };

// template<typename T, typename Default, typename Other>
// struct detect<T, Default, std::void_t<T<Other>>, Other>
// {
//     using type = Default;
//     using is_detected = std::false_type;
// };

// template<typename T, typename Default, typename Other>
// using default_or_detected = detect<T, Default, void, Other>;

// template<typename T, typename Default, typename Other>
// using default_or_detected_t = default_or_detected<T, Default, Other>::type;