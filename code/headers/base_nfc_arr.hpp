#pragma once

#include <hwlib.hpp>

typedef struct nfc_memory_area_s{
    __IO uint8_t bytes[4224];
} Cmem;

#if (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
    #define NFC_MEM (NFC_RAM_ADDR)
#else
    #define NFC_MEM ((Cmem*) NFC_RAM_ADDR)
#endif

class nand_flash_ram{
    public:
        static void init(){
            // set ram size to maximum (4096 + 128 bytes)
            SMC->SMC_CFG = SMC_CFG_PAGESIZE_PS4096;

            // disable nand flash controller
            SMC->SMC_CTRL = SMC_CTRL_NFCDIS;
            
            // enable peripheral clock
            PMC->PMC_PCER0 |= 1 << ID_SMC;

            // clear all memory for first use
            memset((void*)NFC_MEM->bytes, 0x00, 4224);
        }
};