#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include "can.hpp"
#include <queue.hpp>
#include <ringbuffer.hpp>
#include <cmath>
#include "base_comm.hpp"

#include <hwlib.hpp>

namespace r2d2::can_bus {
    /**
     * The maximum amount of modules allowed
     * on a single microcontroller.
     */
    constexpr uint8_t max_modules = 8;

    /**
     * This register stores all communication modules
     * that are on this microcontroller.
     *
     * @internal
     */
    struct comm_module_register_s {
        inline static uint8_t count = 0;
        inline static std::array<base_comm_c *, max_modules> reg;

        /**
         * Clear the registered modules in the array.
         * Primarily used for initialization.
         */
        static void clear_register() {
            reg.fill(nullptr);
        }

        /**
         * Register a module within the registration.
         *
         * @param c
         */
        static void register_module(base_comm_c *c) {
            reg[count++] = c;
        }
    };

    namespace detail {
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

            uint8_t small_buffers[_small_buffer_count]; // 512 bytes
            bool small_buffers_in_use[_small_buffer_count]; // 8 bytes

            uint8_t large_buffers[_large_buffer_count]; // 1024 bytes
            bool large_buffers_in_use[_large_buffer_count]; // 4 bytes

            _uid_index uid_indices[_small_buffer_count + _large_buffer_count];
        };

        /**
         * Helper to get the array index of a given
         * priority.
         *
         * @internal
         * @tparam P
         */
        template<priority P>
        constexpr auto _priority_index = static_cast<uint8_t>(P);

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

            static uint8_t *_get_data_for_uid(const uint8_t uid, const uint8_t type) {
                for (const auto &index : _nfc_mem->uid_indices) {
                    if (index.uid == uid && index.frame_type == type) {
                        return index.data;
                    }
                }

                return nullptr;
            }      

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

            static void _clear_data_for_uid(const uint8_t uid, const uint8_t type){
                for (auto &index : _nfc_mem->uid_indices) {
                    if (index.uid == uid && index.frame_type == type) {
                        index.data = nullptr;
                        return;
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
                if (size <= 64) {
                    for (size_t i = 0; i < sizeof(_nfc_mem->small_buffers_in_use); i++) {
                        if (!(_nfc_mem->small_buffers_in_use[i])) {
                            _nfc_mem->small_buffers_in_use[i] = true;
                            return reinterpret_cast<uint8_t*>(_nfc_mem->small_buffers) + (i * _small_buffer_size);
                        }
                    }
                } else {
                    for (size_t i = 0; i < sizeof(_nfc_mem->large_buffers_in_use); i++) {
                        if (!(_nfc_mem->large_buffers_in_use[i])) {
                            _nfc_mem->large_buffers_in_use[i] = true;
                            return reinterpret_cast<uint8_t*>(_nfc_mem->large_buffers) + (i * _large_buffer_size);
                        }
                    }
                }

                return nullptr;
            }
        };

        /**
         * Base class for compile time priority
         * assignments. Specializations of this class
         * will offer tx and rx compile time constants
         * that represent mailboxes to be used with the given
         * priority level.
         *
         * @tparam Priority
         */
        template<priority Priority>
        struct _mailbox_assignment_s;

        template<>
        struct _mailbox_assignment_s<priority::HIGH> {
            constexpr static uint8_t tx = 0;
            constexpr static uint8_t rx = 1;
        };

        template<>
        struct _mailbox_assignment_s<priority::NORMAL> {
            constexpr static uint8_t tx = 2;
            constexpr static uint8_t rx = 3;
        };

        template<>
        struct _mailbox_assignment_s<priority::LOW> {
            constexpr static uint8_t tx = 4;
            constexpr static uint8_t rx = 5;
        };

        template<>
        struct _mailbox_assignment_s<priority::DATA_STREAM> {
            constexpr static uint8_t tx = 6;
            constexpr static uint8_t rx = 7;
        };
    }

    /**
     * A channel represents a send and receive mailbox
     * with a set priority.
     *
     * @tparam Bus
     * @tparam Priority
     */
    template<typename Bus, priority Priority>
    class channel_c {
    protected:
        using ids = detail::_mailbox_assignment_s<Priority>;

        /**
         * Volatile flag that is used to signify that there is space
         * in the tx_queue. The tx_queue is emptied in an interrupt and the
         * send_frame function will wait until there is space in the tx_queue before
         * it continues execution.
         *
         * However "using while(tx_queue.full()) {}" will cause the compiler to
         * optimize this to a while(true); loop. This is not the desired behaviour.
         * We can't mark the tx_queue itself als volatile, since that would require marking
         * all its member functions as volatile as well. So a flag that is cleared each time
         * the size of tx_queue is reduced is the best option.
         */
        inline volatile static bool space_in_tx_queue = true;

        /**
         * Place a frame into the tx_queue, accounting for
         * a full transfer queue and waiting for the interrupt
         * to resolve.
         *
         * @param frame
         */
        static void safely_push_frame(const detail::_can_frame_s &frame) {
            // For convenience; get a ref to the tx_queue
            auto &tx_queue = detail::_get_tx_queue_for_channel<Priority>();

            if (tx_queue.full()) {
                space_in_tx_queue = false;
            }

            // Wait for space, space is created in the interrupt
            while (!space_in_tx_queue) {
                // ERRP = Error Passive Mode
                bool errp = (port<Bus>->CAN_SR >> 18) & 1;

                // Are we in error mode?
                if (errp) {
                    // We are in error mode, so we'll want to update the tx_queue
                    // with more recent information. In the event that we are connected
                    // again (e.g. this is a temporary problem), we'll send recent information.
                    tx_queue.pop();
                    break;
                }
            }

            tx_queue.push(frame);
        }

    public:
        /**
         * Initialize the channel mailboxes.
         */
        static void init() {
            /*
             * On the Due, there is a separate memory area of just over 4 kilobytes
             * that is normally used for NFC. If NFC is not used, the core is free
             * to repurpose this memory.
             *
             * In this case, this memory area is used to store send and
             * receive buffers for the different channels.
             */
            detail::_init_nfc_memory_area();

            // Set mailbox mode
            constexpr uint32_t accept_mask = 0x07 << 26;

            // Tx
            detail::_set_mailbox_mode<Bus>(ids::tx, mailbox_mode::TX);
            detail::_set_mailbox_accept_mask<Bus>(ids::tx, accept_mask);

            // Rx
            port<Bus>->CAN_MB[ids::rx].CAN_MID = (ids::rx << 26) | CAN_MID_MIDE;
            detail::_set_mailbox_mode<Bus>(ids::rx, mailbox_mode::RX);
            detail::_set_mailbox_accept_mask<Bus>(ids::rx, accept_mask);

            // Rx interrupt
            port<Bus>->CAN_IER = 1U << ids::rx;
        }

        /**
         * Request the given packet on
         * the bus.
         * 
         * @param type 
         */
        static void request_frame(const frame_type &type) {
            detail::_can_frame_s frame{};

            frame.mode = detail::_can_frame_mode::READ;
            frame.length = 0;
            frame.frame_type = type;

            safely_push_frame(frame);

            // Enabling the interrupt on the CAN mailbox with the TX id
            // will cause a interrupt when the CAN controller is ready.
            // At that point will the frame be removed from the tx_queue
            // and put on the bus.
            port<Bus>->CAN_IER = (0x01 << ids::tx);
        }

        /**
         * Send a frame on this channel.
         *
         * @param frame
         */
        static void send_frame(const frame_type &type, const uint8_t *data, const size_t length) {
            /*
             * If the frame is "simple" (less than or equal to 8 bytes), it will
             * fit on the transport in one frame. Otherwise, a sequence
             * is created.
             */
            if (length <= 8) {
                detail::_can_frame_s frame{};

                for (size_t i = 0; i < length; i++) {
                    frame.data.bytes[i] = data[i];
                }

                frame.length = length;
                frame.frame_type = type;

                safely_push_frame(frame);
            } else {
                const uint_fast8_t total = length / 8;
                const uint_fast8_t remainder = length % 8;
                const int rem = remainder > 0;
                const uint16_t timer = port<Bus>->CAN_TIM & 0xF; // get the current timer register

                const uint8_t *ptr = data;

                // First, create the bulk of the frame.
                for (uint_fast8_t i = 0; i < total; i++) {
                    detail::_can_frame_s frame;

                    // Has to be 8 bytes; frame.length is copied in a lower layer
                    for (uint_fast8_t j = 0; j < 8; j++) {
                        frame.data.bytes[j] = *(ptr++);
                    }

                    frame.length = 8;
                    frame.frame_type = type;
                    frame.sequence_id = i;
                    frame.sequence_total = total + rem - 1;

                    // set uid for current transfer
                    frame.sequence_uid = timer;

                    safely_push_frame(frame);
                }

                // Handle any remaining bytes
                if (remainder > 0) {
                    detail::_can_frame_s frame;

                    for (uint_fast8_t i = 0; i < remainder; i++) {
                        frame.data.bytes[i] = *(ptr++);
                    }

                    frame.length = remainder;
                    frame.frame_type = type;
                    frame.sequence_id = total;
                    frame.sequence_total = total; // -1 and rem cancel each other out

                    // set uid for current transfer
                    frame.sequence_uid = timer;

                    safely_push_frame(frame);
                }
            }

            // Enabling the interrupt on the CAN mailbox with the TX id
            // will cause a interrupt when the CAN controller is ready.
            // At that point will the frame be removed from the tx_queue
            // and put on the bus.
            port<Bus>->CAN_IER = (0x01 << ids::tx);
        }

        /**
         * Handle a interrupt meant for this channel.
         */
        static void handle_interrupt(const uint8_t index) {
            // Is the mailbox ready?
            if (!(port<Bus>->CAN_MB[index].CAN_MSR & CAN_MSR_MRDY)) {
                return;
            }

            // Get the MOT (Mailbox Object Type) from the Message Mode (MMR) register
            const uint32_t mmr = (port<Bus>->CAN_MB[index].CAN_MMR >> 24) & 7;

            // if the MOT is 5 (MB_PRODUCER) or 6 (reserved), ignore
            if (mmr > 4) {
                return;
            }

            // For convenience; get a ref to the tx_queue
            auto &tx_queue = detail::_get_tx_queue_for_channel<Priority>();

            // Transmit
            if (mmr == 3) {
                if (!tx_queue.empty()) {
                    // Put the data on the bus
                    const auto frame = tx_queue.copy_and_pop();
                    detail::_write_tx_registers<Bus>(frame, ids::tx);
                    space_in_tx_queue = true;
                } else {
                    // Nothing left to send, disable interrupt on the tx mailbox
                    port<Bus>->CAN_IDR = (0x01 << ids::tx);
                }

                return;
            }

            // Receive
            detail::_can_frame_s can_frame;
            detail::_read_mailbox<Bus>(index, can_frame);

            frame_s frame{};

            frame.type = static_cast<frame_type>(can_frame.frame_type);
            frame.request = can_frame.mode == detail::_can_frame_mode::READ;

            if (can_frame.sequence_total > 0) {
                if (!(can_frame.sequence_id)) {                   
                    // allocate memory for the frame
                    auto * t = detail::_memory_manager_s::alloc(can_frame.sequence_total * 8);

                    if (!t) {
                        // panic with location if we can't handle the data 
                        hwlib::cout << "Error with data(didn't get a memory location)\n";
                        return;
                    }

                    // set the data pointer to the new data location
                    frame.data = t;

                    // save the pointer for the rest of the data
                    detail::_memory_manager_s::_set_data_for_uid(t, can_frame.sequence_uid, can_frame.frame_type);

                } else {
                    // get ptr for data
                    auto * t = detail::_memory_manager_s::_get_data_for_uid(can_frame.sequence_uid, can_frame.frame_type);

                    if (!t) {
                        // no valid pointer in uid set so return
                        hwlib::cout << "Error with data(nowhere to put the data)\n";
                        return;
                    }

                    frame.data = t;
                }

                // copy can frame to frame.data
                for(uint_fast8_t i = 0; i < can_frame.length; i++) {
                    frame.data[i + (can_frame.sequence_id * 8)] = can_frame.data.bytes[i];
                }

                if (can_frame.sequence_id != can_frame.sequence_total) {
                    return;
                }

                // set the amount of bytes the frame uses
                frame.length = ((can_frame.sequence_total + 1) * 8);

            } else {

                // copy can frame to frame.data
                for(uint_fast8_t i = 0; i < can_frame.length; i++) {
                    frame.data[i] = can_frame.data.bytes[i];
                }
            }

            using regs = comm_module_register_s;

            for (uint8_t i = 0; i < regs::count; i++) {
                if (regs::reg[i]->accepts_frame(frame.type)) {
                    regs::reg[i]->accept_frame(frame);
                }
            }

            // clear uid from list if we have a sequence_total
            if (can_frame.sequence_total) {
                detail::_memory_manager_s::_clear_data_for_uid(can_frame.sequence_uid, can_frame.frame_type);
            }            
        }
    };

    namespace detail {
        /**
         * With the given status gained from
         * ANDing the Status Register (SR) with the
         * Interrupt Mask Register (IMR), call the required
         * interrupt handlers.
         *
         * It's possible that multiple mailboxes are included in
         * the interrupt, so every one has to be checked.
         *
         * @internal
         * @tparam Bus
         * @param status
         */
        template<typename Bus>
        void _route_isr(const uint32_t status) {
            if ((status & 1) != 0) {
                channel_c<Bus, priority::HIGH>::handle_interrupt(0);
            }

            if ((status & (1 << 1)) != 0) {
                channel_c<Bus, priority::HIGH>::handle_interrupt(1);
            }

            if ((status & (1 << 2)) != 0) {
                channel_c<Bus, priority::NORMAL>::handle_interrupt(2);
            }

            if ((status & (1 << 3)) != 0) {
                channel_c<Bus, priority::NORMAL>::handle_interrupt(3);
            }

            if ((status & (1 << 4)) != 0) {
                channel_c<Bus, priority::LOW>::handle_interrupt(4);
            }

            if ((status & (1 << 5)) != 0) {
                channel_c<Bus, priority::LOW>::handle_interrupt(5);
            }

            if ((status & (1 << 6)) != 0) {
                channel_c<Bus, priority::DATA_STREAM>::handle_interrupt(6);
            }

            if ((status & (1 << 7)) != 0) {
                channel_c<Bus, priority::DATA_STREAM>::handle_interrupt(7);
            }
        }
    }
};

extern "C" {
void __CAN0_Handler() {
    r2d2::can_bus::detail::_route_isr<r2d2::can_bus::can0>(
        CAN0->CAN_SR & CAN0->CAN_IMR
    );
}

void __CAN1_Handler() {
    r2d2::can_bus::detail::_route_isr<r2d2::can_bus::can1>(
        CAN1->CAN_SR & CAN1->CAN_IMR
    );
}
}