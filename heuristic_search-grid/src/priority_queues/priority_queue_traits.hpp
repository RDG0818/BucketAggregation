#pragma once

template<typename T>
concept priority_queue = requires(T t, typename T::value_type const& cv){
    { t.top() } -> std::same_as<typename T::const_reference>;
    { t.pop() };
    { t.push(cv) };
    { t.size() } -> std::integral;
};

template<typename T>
concept dual_queue = requires(T t, typename T::value_type const& cv){
    { t.template top<0>() } -> std::same_as<typename T::const_reference>;
    { t.template pop<0>() };
    { t.push(cv) };
    { t.size() } -> std::integral;
};

template<typename T, template<typename... Args> typename Queue>
class priority_queue_traits
{
public:
    using value_type = T;
    using reference = value_type&;
    using const_reference = value_type const&;
    using pointer = value_type*;
    using const_pointer = value_type const*;
    using size_type = unsigned int;
    using difference_type = std::make_signed_t<size_type>;

private:
    struct location_info
    {
        difference_type index{std::numeric_limits<difference_type>::max()};

        [[nodiscard]] inline operator difference_type() const noexcept { return index; }
    };

public:
    using location_info_type = location_info;
};

// template<typename T>
// class priority_queue_traits<T, priority_queue_policies::binary_heap>
// {

// };