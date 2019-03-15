#pragma once

#include <array>
#include <cstdint>

#include "can.hpp"
#include "queue.hpp"
#include "ringbuffer.hpp"

namespace r2d2 {
    struct frame_s {
        packet_type type;
        uint8_t bytes[8];

        /**
         * Consider the data in the frame as
         * the given datatype. The returned data is
         * a reference and as such non-owning.
         *
         * @tparam T
         * @return
         */
        template<
            typename T,
            typename = std::enable_if_t<
                is_suitable_packet_v<T> && !is_extended_packet_v<T>
            >
        >
        T &as_type() {
            return *(reinterpret_cast<T *>(&bytes));
        }

        /**
         * Consider the data in the frame
         * as the datatype associated with the given
         * packet type. The returned data is
         * a reference and as such non-owning.
         *
         * @tparam P
         * @return
         */
        template<packet_type P>
        auto as_packet_type() -> packet_data_t<P> & {
            return *(reinterpret_cast<packet_data_t<P> *>(&bytes));
        }
    };

    /**
     * Base class for the communication module.
     * This is used to prevent circular dependencies and
     * allow for mocks.
     */
    class base_comm_c {
    protected:
        ringbuffer_c<frame_s, 32> rx_buffer;

    public:
        /**
         * Does this module accept the given packet
         * type?
         *
         * @param p
         * @return
         */
        virtual bool accepts_packet_type(const packet_type &p) const = 0;

        /**
         * Accept the given frame. This will
         * insert it into the rx buffer.
         *
         * @param frame
         */
        void accept_packet(const frame_s &frame) {
            rx_buffer.push(frame);
        }
    };
}

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
            for (size_t i = 0; i < reg.max_size(); i++) {
                if (reg[i] == nullptr) {
                    reg[i] = c;
                }
            }
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
            detail::_set_mailbox_mode<Bus>(ids::tx, mailbox_mode::TX);
            detail::_set_mailbox_accept_mask<Bus>(ids::tx, accept_mask);

            // Rx
            port<Bus>->CAN_MB[ids::rx].CAN_MID = (ids::rx << 18) | CAN_MID_MIDE;
            detail::_set_mailbox_mode<Bus>(ids::rx, mailbox_mode::RX);
            detail::_set_mailbox_accept_mask<Bus>(ids::rx, accept_mask);

            // Rx interrupt
            port<Bus>->CAN_IER = 1U << ids::rx;
        }

        /**
         * Send a frame on this channel.
         *
         * @param frame
         */
        static void send_frame(const detail::_can_frame_s &frame) {
            tx_queue.push(frame);

            // Enabling the interrupt on the CAN mailbox with the TX id
            // will cause a interrupt when the CAN controller is ready.
            // At that point will the frame be removed from the tx_queue
            // and put on the bus.
            port<Bus>->CAN_IER = (0x01 << ids::tx);
        }

        /**
         * Is there any received data on this channel
         * that needs processing?
         *
         * @return
         */
        static bool has_data() {
            return !rx_buffer.empty();
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
                if (tx_queue.empty()) {
                    return;
                }

                // Put the data on the bus
                const auto frame = tx_queue.copy_and_pop();
                detail::_write_tx_registers<Bus>(frame, ids::tx);

                if (tx_queue.empty()) {
                    // Nothing left to send, disable interrupt on the tx mailbox
                    port<Bus>->CAN_IDR = (0x01 << ids::tx);
                }

                return;
            }

            // Receive
            detail::_can_frame_s can_frame;
            detail::_read_mailbox<Bus>(index, can_frame);

            frame_s frame{};
            frame.type = static_cast<packet_type>(can_frame.packet_type);

            for (size_t i = 0; i < can_frame.length; i++) {
                frame.bytes[i] = can_frame.data.bytes[i];
            }

            for (auto *mod : comm_module_register_s::reg) {
                if (mod && mod->accepts_packet_type(frame.type)) {
                    mod->accept_packet(frame);
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