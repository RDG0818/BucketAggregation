#pragma once

#include "Policies/SharedBlockPool.h"
#include "Policies/SharedDoublingPool.h"
#include "Policies/BlockPool.h"

namespace CustomAllocation
{
    template<typename T, typename Policy = Policies::SharedBlockPool<T>>
    class Allocator : public Policy
    {
        using AllocationPolicy = Policy;

    public:
        using size_type = typename AllocationPolicy::size_type;
        using difference_type = typename AllocationPolicy::difference_type;
        using pointer = typename AllocationPolicy::pointer;
        using const_pointer = typename AllocationPolicy::const_pointer;
        using reference = typename AllocationPolicy::reference;
        using const_reference = typename AllocationPolicy::const_reference;
        using value_type = typename AllocationPolicy::value_type;

        template<typename U>
        struct rebind
        {
            //typedef Allocator<U> other;// , typename AllocationPolicy::rebind<U>::other > other;
            using other = Allocator<U, typename Policy::template rebind<U>::other>;// , typename Policy::rebind<U>::other > ;
        };

        Allocator() = default;

        pointer allocate(size_type n)
        {
            return AllocationPolicy::allocate(1);
        }

        void deallocate(pointer p, size_type n)
        {
            AllocationPolicy::deallocate(p, n);
        }

        void construct(pointer p, const_reference val)
        {
            new (p) T(val);
        }

        void destroy(pointer p)
        {
            p->~T();
        }
    };
}