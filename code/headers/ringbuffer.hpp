#pragma once

#include <cstddef>

namespace r2d2 {
    /**
     * Simple ringbuffer implementation.
     *
     * @tparam T
     * @tparam MaxSize
     */
    template<typename T, size_t MaxSize>
    class ringbuffer_c {
    protected:
        T buffer[MaxSize] = {};

        size_t head = 0;
        size_t tail = 0;
        size_t used = 0;

    public:
        /**
         * Default constructor.
         */
        constexpr ringbuffer_c() = default;

        /**
         * Add an item to the ringbuffer.
         *
         * @param val
         * @return
         */
        constexpr void push(const T &val) {
            if (tail >= MaxSize) {
                tail = 0;
            }

            buffer[tail++] = val;

            if (full()) {
                tail = head;
                head = (head + 1) % MaxSize;
            } else {
                used += 1;
            }
        }

        /**
         * Copy the last item from the buffer
         * and pop.
         *
         * @return
         */
        constexpr T copy_and_pop() {
            size_t pos = 0;

            // get previous position
            if(tail){
                pos = tail - 1;
            } else {
                pos = MaxSize - 1;
            }

            T item = buffer[pos];

            if(!used){
                // at last item
                return item;
            }

            // remove last item
            used--;

            // move tail back 1 or back to maxsize - 1
            tail = !(tail) ? (MaxSize - 1) : (tail - 1);

            return item;
        }

        /**
         * Reset (empty) the ringbuffer.
         *
         * @return
         */
        constexpr void reset() {
            head = 0;
            tail = 0;
            used = 0;
        }

        /**
         * Get the value at the given index.
         *
         * @param index
         * @return
         */
        constexpr T &operator[](const size_t index) {
            return buffer[(head + index) % MaxSize];
        }

        /**
         * Get the value at the given index.
         *
         * @param index
         * @return
         */
        constexpr T const &operator[](const size_t index) const {
            return buffer[(head + index) % MaxSize];
        }

        /**
         * Return whether the ringbuffer is empty.
         *
         * @return
         */
        constexpr bool empty() const {
            return used == 0;
        }

        /**
         * Return whether the ringbuffer is full.
         *
         * @return
         */
        constexpr bool full() const {
            return used == MaxSize;
        }

        /**
         * Get the current size of the ringbuffer.
         *
         * @return
         */
        constexpr size_t size() const {
            return used;
        }

        /**
         * Get the maximum size of the ringbuffer.
         *
         * @return
         */
        constexpr size_t max_size() const {
            return MaxSize;
        }
    };
}