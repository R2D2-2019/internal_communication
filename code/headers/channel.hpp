#pragma once

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
        HIGH,

        // Normal priority packet, default
        NORMAL,

        // Low priority packet, used when there is no hard time constraint on the delivery of the data
        LOW,

        // Data stream packet. Used when a stream of packets (e.g. video data) is put on the bus.
        // Assigning this will given these packets the lowest priority, preventing the large data stream
        // from clogging up the bus.
        DATA_STREAM
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

        static queue_c<detail::_can_frame_s, 32> tx_queue;
        static ringbuffer_c<detail::_can_frame_s, 32> rx_buffer;

    public:
        /**
         * Initialize the channel mailboxes.
         */
        static void init() {
            // First, clear mailboxes to known state
            detail::_init_mailbox(ids::tx);
            detail::_init_mailbox(ids::rx);

            // Set mailbox mode

            // Tx
            detail::_set_mailbox_mode<Bus>(ids::tx, mailbox_mode::TX);
            detail::_set_mailbox_priority<Bus>(ids::tx, 10);
            detail::_set_mailbox_accept_mask<Bus>(ids::tx, 0x7FF, false);

            // Rx
            detail::_set_mailbox_mode<Bus>(ids::rx, mailbox_mode::RX);
            detail::_set_mailbox_id<Bus>(ids::rx, 0x0, false);
            detail::_set_mailbox_accept_mask<Bus>(ids::rx, 0x7FF, false);
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
        static bool has_data() const {
            return ! rx_buffer.empty();
        }

        /**
         * Get last received frame from the channel.
         * This will remove it from the channel receive buffer.
         */
        static uint8_t[8] last_frame_data() {
            return rx_buffer.copy_and_pop();
        }

        /**
         * Handle a interrupt meant for this channel.
         */
        static void handle_interrupt() {
            if (!(port<Bus>->CAN_MB[ids::rx].CAN_MSR & CAN_MSR_MRDY)) {
                return;
            }

            const uint32_t mmr = (port<Bus>->CAN_MB[index].CAN_MMR >> 24) & 7;

            if (mmr > 4) {
                return
            }

            // Transmit
            if (mmr == 3) {
                if (! tx_queue.empty()) {
                    const auto frame = tx_queue.copy_and_pop();
                    detail::_write_tx_registers(frame, ids::tx);
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
    }
};