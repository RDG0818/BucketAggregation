#pragma once

#include <cstddef>
#include <limits>

namespace CustomAllocation
{
    namespace Policies
    {
        namespace
        {
            template<typename T, std::size_t BlockSize = 1024>
            class BlockPool
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
                using value_type = T;
                using pointer = value_type*;
                using const_pointer = const value_type*;
                using reference = value_type&;
                using const_reference = const value_type&;
                using size_type = std::size_t;
                using difference_type = std::ptrdiff_t;

                BlockPool() = default;
                BlockPool(BlockPool&&) noexcept = default;
                BlockPool(const BlockPool&) = delete;
                BlockPool& operator =(BlockPool&&) noexcept = default;
                BlockPool& operator =(const BlockPool&) = delete;

                // void swap(BlockPool& first, BlockPool& second)
                // {
                //     using std::swap;
                //     swap(first.firstFreeElement, second.firstFreeElement);
                //     swap(first.firstBuffer, second.firstBuffer);
                //     swap(first.bufferedElements, second.bufferedElements);
                // }

                ~BlockPool()
                {
                    while (firstBuffer)
                    {
                        Buffer* buffer = firstBuffer;
                        firstBuffer = buffer->next;
                        delete buffer;
                    }
                }

                template<typename U>
                struct rebind
                {
                    using other = BlockPool<U, BlockSize>;
                };

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
    }
}