#pragma once

#include <base_comm.hpp>

namespace r2d2 {
    class base_module_c {
    protected:
        /**
        * The communication module
        * instance.
        */
        base_comm_c &comm;

    public:
        /**
         * @param comm
         */
        explicit module_c(base_comm_c &comm)
            : comm(comm) { }

        /**
         * Let the module process data.
         */
        virtual void process() = 0;
    };
}