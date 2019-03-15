#pragma once

#include <cstdint>

namespace r2d2 {
    /**
     * Simple queue implementation.
     * 
     * @tparam T 
     * @tparam MaxSize 
     */
    template<typename T, size_t MaxSize>
    class queue_c {
    protected:
        T buffer[MaxSize] = {};
        size_t index = 0;

    public:
        /**
         * Put an item on the queue.
         * 
         * @param item 
         */
        void push(const T &item) {
            buffer[index++] = item;
        }

        /**
         * Pop an item from the queue.
         */
        void pop() {
            if (index > 0) {
                index -= 1;
            }
        }

        /**
         * Get the next item from the queue
         * and pop.
         */
        T copy_and_pop() {
            return buffer[--index];
        }

        /**
         * Get the next item on the queue.
         */
        void back() const {
            return buffer[index - 1];
        }

        /**
         * Get the current size of the queue.
         * 
         * @return size_t 
         */
        size_t size() const {
            return index;
        }

        /**
         * Is the queue empty?
         * 
         * @return
         */
        bool empty() const {
            return size() == 0;
        }

        /**
         * Is the queue full?
         * 
         * @return
         */
        bool full() const {
            return size() == max_size();
        }

        /**
         * Get the maximum size of the queue.
         * 
         * @return constexpr size_t 
         */
        constexpr size_t max_size() const {
            return MaxSize;
        }
    };
}