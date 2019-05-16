#pragma once

#include <array>
#include <ringbuffer.hpp>

#include "priority.hpp"
#include "frame_types.hpp"

namespace r2d2 {
    struct frame_s {
        uint8_t* data;
        size_t length;

        frame_type type;
        bool request;

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
                is_suitable_frame_v <T> && !is_extended_frame_v <T>
            >
        >

        T as_type() const {
            return *(
                reinterpret_cast<const T *>(*data)
            );
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
        template<frame_type P>
        auto as_frame_type() const -> frame_data_t <P> {
            return *(
                reinterpret_cast<const frame_data_t<P> *>(data)
            );
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

        /**
         * A list of packets that this module
         * will listen for on the network.
         */
        std::array<frame_id, 8> listen_for{};

        /**
         * Does this module accept all packets types?
         */
        bool accept_all = false;

        /**
         * Send the given data with the given priority.
         *
         * @internal
         * @param buffer
         * @param prio
         */
        virtual void send_impl(
            const frame_type &type, const uint8_t data[], size_t length, priority prio
        ) = 0;

        /**
         * Update the acceptance filter for some of the buses
         * 
         */
        virtual void update_filter() {}

    public:
        /**
         * Request the given packet on
         * the bus.
         * 
         * @param type 
         */
        virtual void request(const frame_type &type, const priority prio = priority::NORMAL) = 0;

        /**
         * Send the given data with the given priority
         * on the bus.
         *
         * @tparam T
         * @param data
         * @param prio
         */
        template<
            typename T,
            typename = std::enable_if_t<
                is_suitable_frame_v<T>
            >
        >
        void send(const T &data, const priority prio = priority::NORMAL) {
            size_t size = sizeof(T);

            // Calculate string length and only send the relevant part.
            if constexpr (supports_string_optimisation_v<T>) {
                constexpr size_t offset = string_member_offset_v<T>;

                auto *string = reinterpret_cast<const uint8_t *>(&data) + offset;

                size_t string_length = 0;
                while (*(string++)) {
                    string_length++;
                }

                // add 1 to the offset to get the amount of bytes used of 
                // the data before the string
                size = (offset + 1) + string_length;
            }

            send_impl(
                static_cast<frame_type>(frame_type_v<T>),
                reinterpret_cast<const uint8_t *>(&data),
                size, prio
            );
        }

        /**
         * Send the given data to an external system
         * with the given priority.
         *
         * @tparam T
         * @param id
         * @param data
         * @param prio
         */
        template<
            typename T,
            typename = std::enable_if_t<
                is_suitable_frame_v <T>
            >
        >

        void send_external(const external_id_s &id, const T &data, const priority prio = priority::NORMAL) {
            // No {} needed, since all fields are filled.
            // Adding it will cause a call to memset
            frame_external_s frame;

            for (size_t i = 0; i < sizeof(T); i++) {
                frame.data[i] = reinterpret_cast<const uint8_t *>(&data)[i];
            }

            frame.type = static_cast<frame_type>(frame_type_v<T>);
            frame.length = sizeof(T);
            frame.id = id;

            send_impl(
                frame_type::EXTERNAL,
                reinterpret_cast<const uint8_t *>(&frame),

                // NOTE: we rely on the fact that data is the last member
                // in the struct here!
                offsetof(frame_external_s, data) + sizeof(T),
                prio
            );
        }

        /**
         * Listen for the given list of packets.
         *
         * @param listen_for
         */
        void listen_for_frames(std::array<frame_id, 8> listen_for /* Copy to allow move */) {
            // Sort to enable binary search
            int i = 1;
            while (i != listen_for.size()) {
                if (i > 0 && listen_for[i] < listen_for[i - 1]) {
                    std::swap(listen_for[i], listen_for[i - 1]);
                    i -= 1;
                } else {
                    i += 1;
                }
            }

            this->listen_for = listen_for;

            if (accepts_frame(frame_type::ALL)) {
                accept_all = true;
            }

            update_filter();
        }

        /**
         * Accept the given frame. This will
         * insert it into the rx buffer.
         *
         * @param frame
         */
        void accept_frame(const frame_s &frame) {
            rx_buffer.push(frame);
        }

        /**
         * Is there any data ready for processing?
         *
         * @return
         */
        bool has_data() const {
            return !rx_buffer.empty();
        }

        /**
        * Get data that is awaiting processing.
        *
        * @tparam T
        * @return
        */
        frame_s get_data() {
            return rx_buffer.copy_and_pop_front();
        }

        /**
         * Get the list of packets that this module
         * listens for.
         * @return
         */
        std::array<frame_id, 8> const &get_accepted_frame_types() const {
            return listen_for;
        }

        /**
         * Does this module accept the given packet
         * type?
         *
         * @param p
         * @return
         */
        bool accepts_frame(const frame_type &p) const {
            if (accept_all) {
                return true;
            }

            const auto &frames = get_accepted_frame_types();

            // Binary search
            int lower = 0;
            int upper = frames.size();

            while (lower < upper) {
                int x = lower + (upper - lower) / 2;
                int val = frames[x];

                if (p == val) {
                    return true;
                } else if (p > val) {
                    if (lower == x) {
                        break;
                    }

                    lower = x;
                } else if (p < val) {
                    upper = x;
                }
            }

            return false;
        }
    };
}
