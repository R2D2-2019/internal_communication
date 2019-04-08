#pragma once

#include <array>
#include <cstdint>
#include <cstring>

#include "can.hpp"
#include <queue.hpp>
#include <ringbuffer.hpp>
#include <cmath>
#include "base_comm.hpp"

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

        // Transfer queue for this channel, is processed in the
        // send interrupt.
        inline static queue_c<detail::_can_frame_s, 32, queue_optimization::READ> tx_queue;

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

                // First, create the bulk of the frame.
                for (uint_fast8_t i = 0; i < total; i++) {
                    detail::_can_frame_s frame;

                    // Has to be 8 bytes; frame.length is copied in a lower layer
                    for (uint_fast8_t j = 0; j < 8; j++) {
                        frame.data.bytes[j] = data[j + i];
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
                        frame.data.bytes[i] = data[i + total];
                    }

                    frame.length = remainder;
                    frame.frame_type = type;
                    frame.sequence_id = total + 1;
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

            for(uint_fast8_t i = 0; i < can_frame.length; i++){
                frame.bytes[i] = can_frame.data.bytes[i];
            }

            using regs = comm_module_register_s;

            for (uint8_t i = 0; i < regs::count; i++) {
                if (regs::reg[i]->accepts_frame(frame.type)) {
                    regs::reg[i]->accept_frame(frame);
                }
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