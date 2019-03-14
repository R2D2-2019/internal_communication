#pragma once

#include <array>
#include <cstdint>

#include "can.hpp"
#include "queue.hpp"
#include "ringbuffer.hpp"

namespace r2d2::can_bus {
    /**
     * All priority levels that can be assigned
     * to packets.
     */
    enum class priority {
        // High priority packet
        HIGH = 0,

        // Normal priority packet, default
        NORMAL = 1,

        // Low priority packet, used when there is no hard time constraint on the delivery of the data
        LOW = 2,

        // Data stream packet. Used when a stream of packets (e.g. video data) is put on the bus.
        // Assigning this will given these packets the lowest priority, preventing the large data stream
        // from clogging up the bus.
        DATA_STREAM = 3
    };

    namespace detail {
        /**
         * Base class for compile time priority
         * assignments. Specializations of this class
         * will offer tx and rx compile time constants
         * that represent mailboxes to be used with the givne
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

        inline static queue_c<detail::_can_frame_s, 32> tx_queue;
        inline static ringbuffer_c<detail::_can_frame_s, 32> rx_buffer;

    public:
        /**
         * Initialize the channel mailboxes.
         */
        static void init() {
            // First, clear mailboxes a to known state
            detail::_init_mailbox<Bus>(ids::tx);
            detail::_init_mailbox<Bus>(ids::rx);

            // Set mailbox mode
            constexpr uint32_t accept_mask = 0x7FF << 18;

            // Tx
            port<Bus>->CAN_MB[ids::tx].CAN_MID = (ids::rx << 18) | CAN_MID_MIDE;
            detail::_set_mailbox_mode<Bus>(ids::tx, mailbox_mode::TX);
            detail::_set_mailbox_accept_mask<Bus>(ids::tx, accept_mask);

            // Rx
            port<Bus>->CAN_MB[ids::rx].CAN_MID = (ids::rx << 18) | CAN_MID_MIDE;
            detail::_set_mailbox_mode<Bus>(ids::rx, mailbox_mode::RX);
            detail::_set_mailbox_accept_mask<Bus>(ids::rx, accept_mask);
            
            // Rx interrupt
            port<Bus>->CAN_IER = 1U << ids::rx;

            // Set mailbox priority
            port<Bus>->CAN_MB[ids::tx].CAN_MMR = 
                (port<Bus>->CAN_MB[ids::tx].CAN_MMR & ~CAN_MMR_PRIOR_Msk) 
                | (static_cast<uint32_t>(Priority) << CAN_MMR_PRIOR_Pos);
        }

        /**
         * Send a frame on this channel.
         *
         * @param frame
         */
        static void send_frame(const detail::_can_frame_s &frame) {
            tx_queue.push(frame);
            port<Bus>->CAN_IER = (0x01 << ids::tx);
        }

        /**
         * Is there any received data on this channel
         * that needs processing?
         *
         * @return
         */
        static bool has_data() {
            return ! rx_buffer.empty();
        }

        /**
         * Get last received frame from the channel.
         * This will remove it from the channel receive buffer.
         * 
         * @return
         */
        static std::array<uint8_t, 8> last_frame_data() {
            const auto frame = rx_buffer.copy_and_pop();

            std::array<uint8_t, 8> buffer;

            // Copy data into the buffer
            for (uint8_t i = 0; i < frame.length; i++) {
                buffer[i] = frame.data.bytes[i];
            }

            // Set the rest of the buffer to 0
            for (uint8_t i = frame.length; i < buffer.max_size(); i++) {
                buffer[i] = 0;
            }

            return buffer;
        }

        /**
         * Handle a interrupt meant for this channel.
         */
        static void handle_interrupt(const uint8_t index) {
            if (!(port<Bus>->CAN_MB[index].CAN_MSR & CAN_MSR_MRDY)) {
                return;
            }

            const uint32_t mmr = (port<Bus>->CAN_MB[index].CAN_MMR >> 24) & 7;

            if (mmr > 4) {
                return;
            }

            // Transmit
            if (mmr == 3) {
                if (! tx_queue.empty()) {
                    const auto frame = tx_queue.copy_and_pop();
                    detail::_write_tx_registers<Bus>(frame, ids::tx);
                } else {
                    port<Bus>->CAN_IDR = (0x01 << ids::tx);
                }

                return;
            }

            // Receive
            detail::_can_frame_s frame;
            detail::_read_mailbox<Bus>(index, frame);

            rx_buffer.push(frame);
        }
    };

    namespace detail {
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
                channel_c<Bus, priority::NORMAL>::handle_interrupt(6);
            }

            if ((status & (1 << 7)) != 0) {
                channel_c<Bus, priority::NORMAL>::handle_interrupt(7);
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