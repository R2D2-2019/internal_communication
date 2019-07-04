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
        template<typename T> requires (is_suitable_frame_v<T>)
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
         * The type of this module.
         */ 
        module mod;

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

        /**
         * This function calculates the length of the actual frame.
         * When the data doesn't support any optimisations, this is simply sizeof(T).
         * When it supports either string optimisation or array optimisation, the length
         * is calculated appropriately.
         * 
         * When string optimisation is used, the string has to be 0-terminated.
         * 
         * @tparam T
         */ 
        template<typename T> requires (is_suitable_frame_v<T>)
        constexpr size_t get_optimized_size(const T &data) const {
            if constexpr (supports_array_optimisation_v<T>) {
                // We assume the length of the array is given by
                // a uint8_t. This is currently checked at compile time as
                // well.
                uint8_t length = *(
                    reinterpret_cast<const uint8_t *>(&data) 
                        + array_member_offset<T>::length_offset
                );

                return array_member_offset<T>::array_offset +
                    length * sizeof(typename array_member_offset<T>::array_type);

            } else if (supports_string_optimisation_v<T>) {
                constexpr size_t offset = string_member_offset_v<T>;

                auto *string = reinterpret_cast<const uint8_t *>(&data) + offset;

                // The string has to be 0-terminated.
                size_t string_length = 0;
                while (*(string++)) {
                    string_length++;
                }

                // Add 1 to the offset to get the amount of bytes used of 
                // the data before the string
                return (offset + 1) + string_length;
            }

            return sizeof(T);
        }

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
        template<typename T> requires (is_suitable_frame_v<T>)
        void send(const T &data, const priority prio = priority::NORMAL) {
            // get the size of the data to send
            size_t size = get_optimized_size(data);

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

            // Look at the different optimisations that can be applied
            // to the frame (type T) and calculate the size (the amount of bytes)
            // actually send across the wire.
            size_t size = get_optimized_size(data);

            for (size_t i = 0; i < size; i++) {
                frame.data[i] = reinterpret_cast<const uint8_t *>(&data)[i];
            }

            frame.type = static_cast<frame_type>(frame_type_v<T>);
            frame.length = size;
            frame.id = id;

            send_impl(
                frame_type::EXTERNAL,
                reinterpret_cast<const uint8_t *>(&frame),

                // NOTE: we rely on the fact that data is the last member
                // in the struct here!
                offsetof(frame_external_s, data) + size,
                prio
            );
        }

        /**
         * Listen for the given list of packets.
         *
         * @param listen_for
         */
        void configure(module mod, std::array<frame_id, 8> listen_for /* Copy to allow move */) {
            this->mod = mod;

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
         * Deprecated
         * Re-added for backwards compatibility
         * Listen for given packets
         * This function now calls it's replacement "configure" with module::NONE
         * 
         * @param listen_for
         */
        void listen_for_frames(std::array<frame_id, 8> listen_for /* Copy to allow move */) {
           configure(r2d2::module::NONE , listen_for);
        }

        /**
         * Accept the given frame. This will
         * insert it into the rx buffer.
         *
         * @param frame
         */
        void accept_frame(const frame_s &frame) {
            // Identity frames are handled here explicitly,
            // as we want to have access to higher level sending
            // functionality but don't want the identity request
            // frame to get into the rx_buffer of the specific module.
            if (frame.request && frame.type == frame_type::IDENTITY) {
                frame_identity_s identity = { mod };
                send(identity);
            } else {
                rx_buffer.push(frame);
            }            
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
         * Get the type of this module.
         */ 
        module get_module_type() const {
            return mod;
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
