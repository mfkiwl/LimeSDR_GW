//--------------------------------------------------------------------------------
// Auto-generated by LiteX (15cd55675) on 2024-10-30 14:51:49
//--------------------------------------------------------------------------------
#ifndef __GENERATED_MEM_H
#define __GENERATED_MEM_H

#ifndef PLIC_BASE
#define PLIC_BASE 0xf0c00000L
#define PLIC_SIZE 0x00400000
#endif

#ifndef CLINT_BASE
#define CLINT_BASE 0xf0010000L
#define CLINT_SIZE 0x00010000
#endif

#ifndef ROM_BASE
#define ROM_BASE 0x00000000L
#define ROM_SIZE 0x00008000
#endif

#ifndef SRAM_BASE
#define SRAM_BASE 0x10000000L
#define SRAM_SIZE 0x00002000
#endif

#ifndef MAIN_RAM_BASE
#define MAIN_RAM_BASE 0x40000000L
#define MAIN_RAM_SIZE 0x00004100
#endif

#ifndef LIME_TOP_MMAP_BASE
#define LIME_TOP_MMAP_BASE 0x04000000L
#define LIME_TOP_MMAP_SIZE 0x00001000
#endif

#ifndef CSR_BASE
#define CSR_BASE 0xf0000000L
#define CSR_SIZE 0x00010000
#endif

#ifndef MEM_REGIONS
#define MEM_REGIONS "PLIC           0xf0c00000 0x400000 \nCLINT          0xf0010000 0x10000 \nROM            0x00000000 0x8000 \nSRAM           0x10000000 0x2000 \nMAIN_RAM       0x40000000 0x4100 \nLIME_TOP_MMAP  0x04000000 0x1000 \nCSR            0xf0000000 0x10000 "
#endif
#endif
