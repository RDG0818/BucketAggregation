#pragma once

#include <cstddef>
#include <limits>

namespace CustomAllocation
{
    namespace Policies
    {
        namespace
        {
            template<typename T, std::size_t BlockSize>
            class BlockMemoryPool
            {
                struct Element
                {
                    Element* next;
                };

                class Buffer
                {
                private:
                    static const std::size_t elementSize = sizeof(T) > sizeof(Element) ? sizeof(T) : sizeof(Element);
                    uint8_t data[elementSize * BlockSize];

                public:
                    Buffer* const next;

                    Buffer(Buffer* next) :
                        next(next)
                    {}

                    T* getBlock(std::size_t index)
                    {
                        return reinterpret_cast<T*>(&data[elementSize * index]);
                    }
                };

                Element* firstFreeElement = nullptr;
                Buffer* firstBuffer = nullptr;
                std::size_t bufferedElements = BlockSize;

            public:
                BlockMemoryPool() = default;
                BlockMemoryPool(BlockMemoryPool&&) = delete;
                BlockMemoryPool(const BlockMemoryPool&) = delete;
                BlockMemoryPool operator =(BlockMemoryPool&&) = delete;
                BlockMemoryPool operator =(const BlockMemoryPool&) = delete;

                ~BlockMemoryPool()
                {
                    while (firstBuffer)
                    {
                        Buffer* buffer = firstBuffer;
                        firstBuffer = buffer->next;
                        delete buffer;
                    }
                }

                inline T* allocate(std::size_t n)
                {
                    if (firstFreeElement)
                    {
                        Element* element = firstFreeElement;
                        firstFreeElement = element->next;
                        return reinterpret_cast<T*>(element);
                    }

                    if (bufferedElements >= BlockSize)
                    {
                        firstBuffer = new Buffer(firstBuffer);
                        bufferedElements = 0;
                    }

                    return firstBuffer->getBlock(bufferedElements++);
                }

                inline void deallocate(T* p, std::size_t)
                {
                    Element* element = reinterpret_cast<Element*>(p);
                    element->next = firstFreeElement;
                    firstFreeElement = element;
                }
            };
        }

        template<typename T, std::size_t BlockSize = 1024>
        class SharedBlockPool
        {
            inline static BlockMemoryPool<T, BlockSize> pool;

        public:
            using value_type = T;
            using pointer = value_type*;
            using const_pointer = const value_type*;
            using reference = value_type&;
            using const_reference = const value_type&;
            using size_type = std::size_t;
            using difference_type = std::ptrdiff_t;

            template<typename U>
            struct rebind
            {
                using other = SharedBlockPool<U, BlockSize>;
            };

            SharedBlockPool() = default;

            inline pointer allocate(size_type n)
            {
                if (n != 1) throw std::bad_alloc();

                return pool.allocate(n);
            }

            inline void deallocate(pointer p, size_type n)
            {
                pool.deallocate(p, n);
            }

            inline size_type max_size() const
            {
                return std::numeric_limits<size_type>::max();
            }
        };
    }
}