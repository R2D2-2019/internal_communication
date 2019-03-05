#pragma once

#include "sam.h"

namespace r2d2::can_bus {
    struct pioa {
        constexpr static uint32_t instance_id = ID_PIOA;
        constexpr static uint32_t irqn = static_cast<uint32_t>(PIOA_IRQn);
    };

    struct piob {
        constexpr static uint32_t instance_id = ID_PIOB;
        constexpr static uint32_t irqn = static_cast<uint32_t>(PIOB_IRQn);
    };

    struct pioc {
        constexpr static uint32_t instance_id = ID_PIOC;
        constexpr static uint32_t irqn = static_cast<uint32_t>(PIOC_IRQn);
    };

    struct piod {
        constexpr static uint32_t instance_id = ID_PIOD;
        constexpr static uint32_t irqn = static_cast<uint32_t>(PIOD_IRQn);
    };

    namespace pins {
        template<typename Pin>
        constexpr uint32_t mask = 1U << Pin::number;

        template<typename Pin>
        constexpr uint32_t adc_channel = 1U << Pin::adc_channel;

        template<typename Pin>
        constexpr uint32_t pwm_channel = 1U << Pin::pwm_channel;

        template<typename Pin>
        const Pio *port = nullptr;

        template<>
        Pio *const port<pioa> = PIOA;
    }

    struct pio_periph_a {};
    struct pio_periph_b {};

    /**
    * Enable the clock on the peripheral.
    *
    * @tparam P
    */
    template<typename P>
    void enable_clock() {
        if constexpr (P::instance_id < 32) {
            // PCER0
            PMC->PMC_PCER0 = 1U << P::instance_id;
        } else {
            // PCER1
            PMC->PMC_PCER1 = 1U << (P::instance_id - 32);
        }
    }

    /**
     * Enable the clock on multiple peripherals.
     *
     * @tparam P
     * @tparam Args
     */
    template<typename P, typename P2, typename ...Args>
    void enable_clock() {
        enable_clock<P>();
        enable_clock<P2, Args...>();
    }

    template<typename Pin>
    void set_peripheral() {
        // Disable interrupts on the pin
        pins::port<typename Pin::port>->PIO_IDR = pins::mask<Pin>;
        uint32_t t = pins::port<typename Pin::port>->PIO_ABSR;

        // Change pio multiplexer
        if constexpr (std::is_same_v<typename Pin::periph, pio_periph_a>) {
            pins::port<typename Pin::port>->PIO_ABSR &= (~pins::mask<Pin> & t);
        } else if constexpr (std::is_same_v<typename Pin::periph, pio_periph_b>) {
            pins::port<typename Pin::port>->PIO_ABSR = (pins::mask<Pin> | t);
        }

        // disable pull ups
        pins::port<typename Pin::port>->PIO_PUDR = pins::mask<Pin>;

        // remove pin from pio controller
        pins::port<typename Pin::port>->PIO_PDR = pins::mask<Pin>;
    }

    namespace pins {
        struct d53 {
            using port = piob;
            constexpr static uint32_t number = 14;
            constexpr static uint32_t pwm_channel = 2;
        };

        struct dac0 {
            using port = piob;
            using periph = pio_periph_a;
            constexpr static uint32_t number = 15;
            constexpr static uint32_t pwm_channel = 3;
            constexpr static uint32_t dacc_channel = 0;
        };

        struct canrx {
            using port = pioa;
            using periph = pio_periph_a;
            constexpr static uint32_t number = 1;
        };

        struct cantx {
            using port = pioa;
            using periph = pio_periph_a;
            constexpr static uint32_t number = 0;
        };
    }

    struct can0 {
        constexpr static uint32_t instance_id = ID_CAN0;
        constexpr static uint32_t irqn = static_cast<uint32_t>(CAN0_IRQn);

        using tx = pins::cantx;
        using rx = pins::canrx;
    };

    struct can1 {
        constexpr static uint32_t instance_id = ID_CAN1;
        constexpr static uint32_t irqn = static_cast<uint32_t>(CAN1_IRQn);

        // Some details covered here:
        // https://copperhilltech.com/blog/arduino-due-can-bus-controller-area-network-interfaces/
        using tx = pins::d53;
        using rx = pins::dac0;
    };

    template<typename CAN>
    Can *const port = nullptr;

    template<>
    Can *const port<can0> = CAN0;

    template<>
    Can *const port<can1> = CAN1;

    enum mailbox_mode {
        DISABLE = 0,
        RX = 1,
        RX_OVER_WR = 2,
        TX = 3,
        CONSUMER = 4,
        PRODUCER = 5
    };

    enum mailbox_status {
        TRANSFER_OK = 0,
        NOT_READY = 0x01,
        RS_OVER = 0x02,
        RX_NEED_RD_AGAIN = 0x03
    };

    enum baudrate : uint32_t {
        BPS_1000K = 1000000,
        BPS_800K = 800000,
        BPS_500K = 500000,
        BPS_250K = 250000,
        BPS_125K = 125000,
        BPS_50K = 50000,
        BPS_33333 = 33333,
        BPS_25K = 25000,
        BPS_10K = 10000,
        BPS_5K = 5000,
    };

    namespace detail {
        /**
         * Disable the given CAN bus and clear
         * the SR register.
         *
         * @internal
         * @tparam Bus
         */
        template<typename Bus>
        void _disable_bus() {
            // Disable CAN hardware if previously enabled
            port<Bus>->CAN_MR &= ~CAN_MR_CANEN;

            // Read status register to be sure it's cleaned out
            (void) port<Bus>->CAN_SR;
        }

        template<typename Bus>
        void _enable_bus() {
            // Enable CAN hardware
            port<Bus>->CAN_MR |= CAN_MR_CANEN;

            // TODO: enablePin stuff?
        }

        struct _can_bit_timing_t {
            uint8_t time_quantum;      //! CAN_BIT_SYNC + uc_prog + uc_phase1 + uc_phase2 = uc_tq, 8 <= uc_tq <= 25.
            uint8_t propagation;    //! Propagation segment, (3-bits + 1), 1~8;
            uint8_t phase1;  //! Phase segment 1, (3-bits + 1), 1~8;
            uint8_t phase2;  //! Phase segment 2, (3-bits + 1), 1~8, CAN_BIT_IPT <= uc_phase2;
            uint8_t sync_jump_width;     //! Resynchronization jump width, (2-bits + 1), min(uc_phase1, 4);
            uint8_t sample_point;      //! Sample point value, 0~100 in percent.
        };

        /**
         * Values of bit time register for different baudrates.
         * Sample point = ((1 + uc_prog + uc_phase1) / uc_tq) * 100%.
         *
         * Source: https://github.com/collin80/due_can/blob/master/src/due_can.h
         *
         * @internal
         */
        constexpr _can_bit_timing_t _can_bit_time[] = {
            //TQ PROG     PH1      PH2      SJW      SAMP
            {8,  (2 + 1), (1 + 1), (1 + 1), (2 + 1), 75},
            {9,  (1 + 1), (2 + 1), (2 + 1), (1 + 1), 67},
            {10, (2 + 1), (2 + 1), (2 + 1), (2 + 1), 70},
            {11, (3 + 1), (2 + 1), (2 + 1), (2 + 1), 72},
            {12, (2 + 1), (3 + 1), (3 + 1), (2 + 1), 67},
            {13, (3 + 1), (3 + 1), (3 + 1), (2 + 1), 77},
            {14, (3 + 1), (3 + 1), (4 + 1), (2 + 1), 64},
            {15, (3 + 1), (4 + 1), (4 + 1), (2 + 1), 67},
            {16, (4 + 1), (4 + 1), (4 + 1), (2 + 1), 69},
            {17, (5 + 1), (4 + 1), (4 + 1), (2 + 1), 71},
            {18, (4 + 1), (5 + 1), (5 + 1), (2 + 1), 67},
            {19, (5 + 1), (5 + 1), (5 + 1), (2 + 1), 68},
            {20, (6 + 1), (5 + 1), (5 + 1), (2 + 1), 70},
            {21, (7 + 1), (5 + 1), (5 + 1), (2 + 1), 71},
            {22, (6 + 1), (6 + 1), (6 + 1), (2 + 1), 68},
            {23, (7 + 1), (7 + 1), (6 + 1), (2 + 1), 70},
            {24, (6 + 1), (7 + 1), (7 + 1), (2 + 1), 67},
            {25, (7 + 1), (7 + 1), (7 + 1), (2 + 1), 68}
        };

        /**
         * Set the baudrate of the given CAN bus.
         * Returns true on success, false otherwise.
         *
         * @internal
         * @tparam Bus
         * @tparam Baud
         * @return
         */
        template<typename Bus, baudrate Baud>
        bool _set_baudrate() {
            // https://github.com/collin80/due_can/blob/master/src/due_can.h
            constexpr uint32_t min_tq = 8;
            constexpr uint32_t max_tq = 25;
            constexpr uint32_t baudrate_max_div = 128;

            constexpr uint32_t mck = CHIP_FREQ_CPU_MAX;

            uint32_t prescale = (mck + (Baud * max_tq - 1)) / (mck * max_tq);

            /*
             *  Is the baudrate prescale greater than
             *  the max divide value?
             */
            if (prescale > baudrate_max_div) {
                return false;
            }

            /*
             * Is the input MCK too small?
             */
            if (mck < Baud * min_tq) {
                return false;
            }

            uint32_t tq = min_tq;
            uint32_t mod = 0xFFFFFFFF;

            // Find approximate Time Quantum
            for (uint8_t i = min_tq; i <= max_tq; ++i) {
                if (mck / (Baud * i) > baudrate_max_div) {
                    continue;
                }

                const uint32_t cur_mod = mck % (Baud * i);

                if (cur_mod >= mod) {
                    continue;
                }

                mod = cur_mod;
                tq = i;

                if (!mod) {
                    break;
                }
            }

            // Calculate baudrate prescale
            prescale = mck / (Baud * tq);

            // Get the right CAN bit timing group
            const auto bit_time = _can_bit_time[tq - min_tq];

            // Before modifying CANBR, disable CAN controller.
            detail::_disable_bus<Bus>();

            // Write to CAN baudrate register
            port<Bus>->CAN_BR = CAN_BR_PHASE2(bit_time.phase2 - 1) |
                                CAN_BR_PHASE1(bit_time.phase1 - 1) |
                                CAN_BR_PROPAG(bit_time.propagation - 1) |
                                CAN_BR_SJW(bit_time.sync_jump_width - 1) |
                                CAN_BR_BRP(prescale - 1);

            return true;
        }

        /**
         * Initialize the mailbox to a default state.
         * 
         * @internal
         * @tparam Bus
         * @param index
         */ 
        template<typename Bus>
        void _init_mailbox(const uint8_t index) {
            port<Bus>->CAN_MB[index].CAN_MMR = 0;
            port<Bus>->CAN_MB[index].CAN_MAM = 0;
            port<Bus>->CAN_MB[index].CAN_MID = 0;
            port<Bus>->CAN_MB[index].CAN_MDL = 0;
            port<Bus>->CAN_MB[index].CAN_MDH = 0;
            port<Bus>->CAN_MB[index].CAN_MCR = 0;
        }

        /**
         * Reset all mail boxes for the given bus.
         * 
         * @internal
         * @tparam Bus
         */ 
        template<typename Bus>
        void _reset_mailboxes() {
            for (uint8_t i = 0; i < CANMB_NUMBER; i++) {
                _init_mailbox<Bus>(i);
            }
        }
    }

    /**
    * @tparam Bus 
    */
    template<typename Bus>
    class controller {
    public:
        template<baudrate Baud = BPS_1000K>
        static bool init() {
            constexpr uint32_t can_timeout = 100000;

            // Init bus clock
            enable_clock<Bus>();

            // Change multiplexers on tx/rx pins to CAN
            set_peripheral<typename Bus::tx>();
            set_peripheral<typename Bus::rx>();

            if (!detail::_set_baudrate<Bus, Baud>()) {
                return false;
            }

            // TODO: mailbox stuf?

            // Disable all CAN interrupts by default
            port<Bus>->CAN_IDR = 0xffffffff;

            detail::_enable_bus<Bus>();

            uint32_t flag = 0;
            uint32_t tick = 0;

            do {
                flag = port<Bus>->CAN_SR;
                ++tick;
            } while (!(flag & CAN_SR_WAKEUP) && (tick < can_timeout));

            // TODO: priority in nvic for interrupts

            return tick != can_timeout;
        }
    };
}