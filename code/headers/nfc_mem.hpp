#pragma once

#include <ringbuffer.hpp>
#include <queue.hpp>
#include <sam3.h>
#include <hwlib.hpp>
#include <array>

#include "priority.hpp"
#include "can.hpp"

namespace r2d2::can_bus::detail {
    /**
     * This struct is packet to prevent padding
     * to 32 bits. This would require a lot more size, which
     * is sparse in the NFC memory area in which this is used.
     */
    #pragma pack(1)
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
        using queue_type = queue_c<
            detail::_can_frame_s, 16, queue_optimization::READ
        >;

        queue_type tx_queues[4]; // 912 bytes

        template<size_t Packets, size_t Amount>
        using buffer_type = ringbuffer_c<
            std::array<uint8_t, Packets * 8>,
            Amount
        >;

        constexpr static size_t p1_buffers_count = 96;
        constexpr static size_t p4_buffers_count = 18;
        constexpr static size_t p16_buffers_count = 4;
        constexpr static size_t p32_buffers_count = 2;

        buffer_type<1, p1_buffers_count>    p1_buffers;
        buffer_type<4, p4_buffers_count>    p4_buffers;
        buffer_type<16, p16_buffers_count>  p16_buffers;
        buffer_type<32, p32_buffers_count>  p32_buffers;

        _uid_index uid_indices[
            p1_buffers_count + 
            p4_buffers_count +
            p16_buffers_count +
            p32_buffers_count
        ];

        uint8_t *allocate(uint8_t size) {
            if (size <= 1 * 8) {
                return &(p1_buffers.emplace()[0]); 
            } else if (size <= 4 * 8) {
                return &(p4_buffers.emplace()[0]); 
            } else if (size <= 16 * 8) {
                return &(p16_buffers.emplace()[0]); 
            } else {
                return &(p32_buffers.emplace()[0]); 
            }
        }
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
    };
}