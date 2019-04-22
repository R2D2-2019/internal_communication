#pragma once

#include <queue.hpp>
#include <sam3.h>
#include <hwlib.hpp>

#include "priority.hpp"
#include "can.hpp"

namespace r2d2 {
    namespace can_bus::detail {
        constexpr uint16_t _small_buffer_size = 64;
        constexpr uint8_t _small_buffer_count = 16;
        constexpr uint16_t _large_buffer_size = 256;
        constexpr uint8_t _large_buffer_count = 4;

        struct _uid_index {
            uint8_t uid;
            uint8_t frame_type;
            uint8_t *data;
        };

        /**
         * NFC memory area layout.
         * Max 4224 bytes.
         *
         * @internal
         */
        struct _nfc_memory_area_s {
            using queue_type = queue_c<detail::_can_frame_s, 16, queue_optimization::READ>;

            using small_buffer = uint8_t[_small_buffer_size];
            using large_buffer = uint8_t[_large_buffer_size];

            queue_type tx_queues[4]; // 912 bytes

            small_buffer small_buffers[_small_buffer_count]; // 1024 bytes
            size_t small_buffer_counters[_small_buffer_count]; // 32 bytes

            large_buffer large_buffers[_large_buffer_count]; // 1024 bytes
            size_t large_buffer_counters[_large_buffer_count]; // 16 bytes

            _uid_index uid_indices[_small_buffer_count + _large_buffer_count];
        };

        /**
         * Pointer to the NFC memory area.
         *
         * @internal
         */
        _nfc_memory_area_s *const _nfc_mem = ((_nfc_memory_area_s *) NFC_RAM_ADDR);

        /**
         * Helper function to cleanly get the queue for the channel
         * with the given priority.
         *
         * @internal
         * @tparam P
         * @return
         */
        template<priority P>
        constexpr _nfc_memory_area_s::queue_type &_get_tx_queue_for_channel() {
            return _nfc_mem->tx_queues[
                static_cast<uint8_t>(P)
            ];
        }

        /**
         * Initialize the memory area that is normally reserved
         * for NFC. This area is used to store send and receive
         * buffers.
         *
         * @internal
         */
        void _init_nfc_memory_area() {
            // The NFC memory area cannot exceed 4224 bytes
            static_assert(sizeof(_nfc_memory_area_s) <= 4224);

            static bool initialized = false;

            if (initialized) {
                return;
            }

            // Set ram size to maximum (4096 + 128 bytes)
            SMC->SMC_CFG = SMC_CFG_PAGESIZE_PS4096;

            // Disable nand flash controller
            SMC->SMC_CTRL = SMC_CTRL_NFCDIS;

            // Enable peripheral clock
            PMC->PMC_PCER0 |= 1U << ID_SMC;

            // Clear all memory for first use
            memset((void *) _nfc_mem, 0, 4224);

            initialized = true;
        }

        /**
         * Helper struct that manages the memory
         * of the NFC memory area.
         */
        struct _memory_manager_s {
            static size_t *get_counter(const uint8_t *ptr) {
                const size_t offset = reinterpret_cast<size_t>(ptr) - reinterpret_cast<size_t>(_nfc_mem);

                if (offset < offsetof(_nfc_memory_area_s, large_buffers)) {
                    // Small buffers
                    const size_t array_offset =
                        (offset - offsetof(_nfc_memory_area_s, small_buffers)) / _small_buffer_size;
                    return &_nfc_mem->small_buffer_counters[array_offset];
                } else {
                    // Large buffers
                    const size_t array_offset =
                        (offset - offsetof(_nfc_memory_area_s, large_buffers)) / _large_buffer_size;
                    return &_nfc_mem->large_buffer_counters[array_offset];
                }
            }

            /**
             * Deallocate the block, starting at the given pointer.
             * This won't zero the memory, but simply mark it as free.
             * 
             * @param ptr 
             */
            static void dealloc(const uint8_t *ptr) {
                if (!ptr) {
                    return;
                }

                size_t offset = reinterpret_cast<size_t>(ptr) - reinterpret_cast<size_t>(_nfc_mem);

                if (offset < offsetof(_nfc_memory_area_s, large_buffers)) {
                    // Small buffers
                    const size_t array_offset =
                        (offset - offsetof(_nfc_memory_area_s, small_buffers)) / _small_buffer_size;
                    _nfc_mem->small_buffer_counters[array_offset] = 0;
                } else {
                    // Large buffers
                    const size_t array_offset =
                        (offset - offsetof(_nfc_memory_area_s, large_buffers)) / _large_buffer_size;
                    _nfc_mem->large_buffer_counters[array_offset] = 0;
                }
            }

            /**
             * @brief gets the pointer for the data for the current type and uid
             * 
             * @param uid 
             * @param type 
             * @return uint8_t* 
             */
            static uint8_t *_get_data_for_uid(const uint8_t uid, const uint8_t type) {
                for (const auto &index : _nfc_mem->uid_indices) {
                    if (index.uid == uid && index.frame_type == type) {
                        return index.data;
                    }
                }

                return nullptr;
            }

            /**
             * @brief tries to set the data for a specific uid and type
             * 
             * @param ptr 
             * @param uid 
             * @param type 
             */
            static void _set_data_for_uid(uint8_t *ptr, const uint8_t uid, const uint8_t type) {
                for (auto &index : _nfc_mem->uid_indices) {
                    if (index.data == nullptr) {
                        index.uid = uid;
                        index.frame_type = type;
                        index.data = ptr;
                        return;
                    }
                }
            }

            /**
             * @brief tries clears the data for a specific uid and type
             * 
             * @param uid 
             * @param type 
             */
            static void _clear_data_for_uid(const uint8_t uid, const uint8_t type) {
                // remove pointer from active uid_indices
                for (auto &index : _nfc_mem->uid_indices) {
                    if (index.uid == uid && index.frame_type == type) {
                        index.uid = 0;
                        index.frame_type = 0;
                        index.data = nullptr;
                        break;
                    }
                }
            }

            /**
             * Allocate a memory block for the given size.
             * If no memory could be allocated, a nullptr is returned.
             * 
             * @param size 
             * @return uint8_t* 
             */
            static uint8_t *alloc(const size_t size) {
                if (size <= _small_buffer_size) {
                    for (size_t i = 0; i < _small_buffer_count; i++) {
                        if (!(_nfc_mem->small_buffer_counters[i])) {
                            return reinterpret_cast<uint8_t *>(&_nfc_mem->small_buffers[i]);
                        }
                    }
                } else {
                    for (size_t i = 0; i < _large_buffer_count; i++) {
                        if (!(_nfc_mem->large_buffer_counters[i])) {
                            return reinterpret_cast<uint8_t *>(&_nfc_mem->large_buffers[i]);
                        }
                    }
                }

                return nullptr;
            }

            static void print_memory_statistics() {
                hwlib::cout << "Counters: \r\n\tSmall buffers:\r\n\t\t";

                for (size_t i = 0; i < _small_buffer_count; i++) {
                    hwlib::cout << _nfc_mem->small_buffer_counters[i] << ' ';
                }

                hwlib::cout << "\r\n\r\n\tLarge buffers:\r\n\t\t";

                for (size_t i = 0; i < _large_buffer_count; i++) {
                    hwlib::cout << _nfc_mem->large_buffer_counters[i] << ' ';
                }

                hwlib::cout << "\r\n";
            }
        };
    }

#ifdef HWLIB_TARGET_native
#include <memory>

    using shared_nfc_ptr_c = std::shared_ptr<uint8_t*>;
#else

    class shared_nfc_ptr_c {
    protected:
        uint8_t *ptr = nullptr;
        size_t *counter = nullptr;

        void increment() {
            (*counter) += 1;
        }

        void decrement() {
            (*counter) -= 1;
        }

    public:
        shared_nfc_ptr_c() = default;

        explicit shared_nfc_ptr_c(uint8_t *ptr) : ptr(ptr) {
            counter = can_bus::detail::_memory_manager_s::get_counter(ptr);
            (*counter) = 1;
        }

        ~shared_nfc_ptr_c() {
            decrement();

            if ((*counter) == 0) {
                hwlib::cout << "Deallocating!\r\n";

                can_bus::detail::_memory_manager_s::dealloc(ptr);
            }
        }

        shared_nfc_ptr_c(const shared_nfc_ptr_c &other)
            : ptr(other.ptr), counter(other.counter) {
            increment();
        }

        shared_nfc_ptr_c(shared_nfc_ptr_c &&other)
            : ptr(other.ptr), counter(other.counter) {}

        shared_nfc_ptr_c &operator=(const shared_nfc_ptr_c &other) {
            shared_nfc_ptr_c tmp(other);
            swap(tmp);

            return *this;
        }

        uint8_t const *get() const {
            return ptr;
        }

        uint8_t *get() {
            return ptr;
        }

        size_t get_counter() const {
            return *counter;
        }

        uint8_t *operator*() {
            return ptr;
        }

        uint8_t const *operator*() const {
            return ptr;
        }

        void swap(shared_nfc_ptr_c &other) {
            std::swap(ptr, other.ptr);
            std::swap(counter, other.counter);
        }
    };

#endif
}