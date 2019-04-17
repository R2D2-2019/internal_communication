#pragma once

#include <queue.hpp>
#include <sam3.h>

#include "priority.hpp"
#include "can.hpp"

namespace r2d2::can_bus::detail {
    constexpr uint16_t _small_buffer_size = 64;
    constexpr uint8_t _small_buffer_count = 8;
    constexpr uint16_t _large_buffer_size = 256;
    constexpr uint8_t _large_buffer_count = 4;

    struct _uid_index {
        uint8_t uid;
        uint8_t frame_type;
        uint8_t *data;
    };

    /**
     * NFC memory area layout.
     *
     * @internal
     */
    struct _nfc_memory_area_s {
        using queue_type = queue_c<detail::_can_frame_s, 16, queue_optimization::READ>;

        queue_type tx_queues[4]; // 912 bytes

        uint8_t small_buffers[_small_buffer_count][_small_buffer_size]; // 512 bytes
        bool small_buffers_in_use[_small_buffer_count]; // 8 bytes

        uint8_t large_buffers[_large_buffer_count][_large_buffer_size]; // 1024 bytes
        bool large_buffers_in_use[_large_buffer_count]; // 4 bytes

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
            _priority_index<P>
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
        memset((void *)_nfc_mem, 0, 4224);

        initialized = true;
    }

    /**
     * Helper struct that manages the memory
     * of the NFC memory area.
     */
    struct _memory_manager_s {
        /**
         * Deallocate the block, starting at the given pointer.
         * This won't zero the memory, but simply mark it as free.
         * 
         * @param ptr 
         */
        static void dealloc(uint8_t *ptr) {
            // nullptr check
            if (!ptr) {
                return;
            }

            size_t offset = reinterpret_cast<size_t>(ptr) - reinterpret_cast<size_t>(_nfc_mem);

            if (offset < offsetof(_nfc_memory_area_s, large_buffers)) {
                // Small buffers
                const size_t array_offset = (offset - offsetof(_nfc_memory_area_s, small_buffers)) / _small_buffer_size;
                _nfc_mem->small_buffers_in_use[array_offset] = false;
            } else {
                // Large buffers
                const size_t array_offset = (offset - offsetof(_nfc_memory_area_s, large_buffers)) / _large_buffer_size;
                _nfc_mem->large_buffers_in_use[array_offset] = false;
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
        static void _set_data_for_uid(uint8_t *ptr, const uint8_t uid, const uint8_t type){
            for(auto &index : _nfc_mem->uid_indices) {
                if(index.data == nullptr){
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
        static void _clear_data_for_uid(const uint8_t uid, const uint8_t type){
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
                    if (!(_nfc_mem->small_buffers_in_use[i])) {
                        _nfc_mem->small_buffers_in_use[i] = true;
                        return reinterpret_cast<uint8_t*>(&_nfc_mem->small_buffers[i]);
                    }
                }
            } else {
                for (size_t i = 0; i < _large_buffer_count; i++) {
                    if (!(_nfc_mem->large_buffers_in_use[i])) {
                        _nfc_mem->large_buffers_in_use[i] = true;
                        return reinterpret_cast<uint8_t*>(&_nfc_mem->large_buffers[i]);
                    }
                }
            }

            return nullptr;
        }
    };
}