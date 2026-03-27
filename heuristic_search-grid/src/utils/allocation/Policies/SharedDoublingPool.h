#pragma once

#include<cstddef>
#include<iostream>
namespace CustomAllocation
{
    namespace Policies
    {
        namespace
        {
            template<typename T, std::size_t InitialSize>
            class DoublingMemoryPool
            {
                struct Element
                {
                    Element* next;
                };

                class Buffer
                {
                private:
                    static const std::size_t elementSize = sizeof(T) > sizeof(Element) ? sizeof(T) : sizeof(Element);
                    uint8_t* data;
                    std::size_t bufferSize;

                public:
                    Buffer* const next;

                    Buffer(Buffer* next, std::size_t size) :
                        next(next), bufferSize(size)
                    {
                        data = new uint8_t[elementSize * bufferSize];
                    }

                    ~Buffer()
                    {
                        delete[] data;
                    }

                    inline std::size_t GetSize() const
                    {
                        return bufferSize;
                    }

                    T* getBlock(std::size_t index)
                    {
                        return reinterpret_cast<T*>(&data[elementSize * index]);
                    }
                };

                Element* firstFreeElement = nullptr;
                Buffer* firstBuffer = nullptr;
                std::size_t bufferedElements = InitialSize;
                std::size_t currentBufferSize = 0;
                std::size_t totalPoolSize = 0;

            public:
                DoublingMemoryPool() = default;
                DoublingMemoryPool(DoublingMemoryPool&&) = delete;
                DoublingMemoryPool(const DoublingMemoryPool&) = delete;
                DoublingMemoryPool operator =(DoublingMemoryPool&&) = delete;
                DoublingMemoryPool operator =(const DoublingMemoryPool&) = delete;

                ~DoublingMemoryPool()
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

                    if (bufferedElements >= currentBufferSize)
                    {
                        firstBuffer = new Buffer(firstBuffer, std::max(totalPoolSize, InitialSize));
                        currentBufferSize = firstBuffer->GetSize();
                        bufferedElements = 0;
                        totalPoolSize += currentBufferSize;
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

        template<typename T, std::size_t InitialSize = 1024>
        class SharedDoublingPool
        {
            inline static DoublingMemoryPool<T, InitialSize> pool;

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
                using other = SharedDoublingPool<U, InitialSize>;
            };

            SharedDoublingPool() = default;

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