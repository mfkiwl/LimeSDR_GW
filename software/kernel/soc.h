//--------------------------------------------------------------------------------
// Auto-generated by LiteX (15cd55675) on 2024-10-30 14:51:49
//--------------------------------------------------------------------------------
#ifndef __GENERATED_SOC_H
#define __GENERATED_SOC_H
#define CONFIG_PLATFORM_NAME "limesdr_xtrx_platform"
#define CONFIG_CLOCK_FREQUENCY 125000000
#define CONFIG_CPU_HAS_INTERRUPT
#define CONFIG_CPU_RESET_ADDR 0
#define CONFIG_CPU_COUNT 1
#define CONFIG_CPU_ISA "rv32i2p0_mac"
#define CONFIG_CPU_MMU "sv32"
#define CONFIG_CPU_DCACHE_SIZE 4096
#define CONFIG_CPU_DCACHE_WAYS 1
#define CONFIG_CPU_DCACHE_BLOCK_SIZE 64
#define CONFIG_CPU_ICACHE_SIZE 4096
#define CONFIG_CPU_ICACHE_WAYS 1
#define CONFIG_CPU_ICACHE_BLOCK_SIZE 64
#define CONFIG_CPU_DTLB_SIZE 4
#define CONFIG_CPU_DTLB_WAYS 4
#define CONFIG_CPU_ITLB_SIZE 4
#define CONFIG_CPU_ITLB_WAYS 4
#define CONFIG_CPU_TYPE_VEXRISCV_SMP
#define CONFIG_CPU_VARIANT_STANDARD
#define CONFIG_CPU_FAMILY "riscv"
#define CONFIG_CPU_NAME "vexriscv"
#define CONFIG_CPU_HUMAN_NAME "VexRiscv SMP-STANDARD"
#define CONFIG_CPU_NOP "nop"
#define CONFIG_MAIN_RAM_INIT 1
#define CONFIG_IDENTIFIER "LiteX SoC on Limesdr XTRX  2024-10-30 14:45:35"
#define ROM_BOOT_ADDRESS 1073741824
#define DMA_CHANNELS 1
#define DMA_ADDR_WIDTH 32
#define PCIE_DMA0_READER_INTERRUPT 0
#define PCIE_DMA0_WRITER_INTERRUPT 1
#define CONFIG_CSR_DATA_WIDTH 32
#define CONFIG_CSR_ALIGNMENT 32
#define CONFIG_BUS_STANDARD "wishbone"
#define CONFIG_BUS_DATA_WIDTH 32
#define CONFIG_BUS_ADDRESS_WIDTH 32
#define CONFIG_BUS_BURSTING 0
#define CONFIG_CPU_INTERRUPTS 5
#define CNTRL_INTERRUPT 3
#define LIME_TOP_INTERRUPT 4
#define TIMER0_INTERRUPT 2
#define UART_INTERRUPT 1
#define CONFIG_HAS_I2C

#endif
