//--------------------------------------------------------------------------------
// Auto-generated by LiteX (b279fc9fb) on 2024-08-21 12:14:04
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
// CSR Includes.
//--------------------------------------------------------------------------------

#ifndef __GENERATED_CSR_H
#define __GENERATED_CSR_H

#ifndef CSR_BASE
#define CSR_BASE 0xf0000000L
#endif /* ! CSR_BASE */

//--------------------------------------------------------------------------------
// CSR Registers/Fields Definition.
//--------------------------------------------------------------------------------

/* UART Registers */
#define CSR_UART_BASE (CSR_BASE + 0x0L)
#define CSR_UART_RXTX_ADDR (CSR_BASE + 0x0L)
#define CSR_UART_RXTX_SIZE 1
#define CSR_UART_TXFULL_ADDR (CSR_BASE + 0x4L)
#define CSR_UART_TXFULL_SIZE 1
#define CSR_UART_RXEMPTY_ADDR (CSR_BASE + 0x8L)
#define CSR_UART_RXEMPTY_SIZE 1
#define CSR_UART_EV_STATUS_ADDR (CSR_BASE + 0xcL)
#define CSR_UART_EV_STATUS_SIZE 1
#define CSR_UART_EV_PENDING_ADDR (CSR_BASE + 0x10L)
#define CSR_UART_EV_PENDING_SIZE 1
#define CSR_UART_EV_ENABLE_ADDR (CSR_BASE + 0x14L)
#define CSR_UART_EV_ENABLE_SIZE 1
#define CSR_UART_TXEMPTY_ADDR (CSR_BASE + 0x18L)
#define CSR_UART_TXEMPTY_SIZE 1
#define CSR_UART_RXFULL_ADDR (CSR_BASE + 0x1cL)
#define CSR_UART_RXFULL_SIZE 1

/* UART Fields */
#define CSR_UART_EV_STATUS_TX_OFFSET 0
#define CSR_UART_EV_STATUS_TX_SIZE 1
#define CSR_UART_EV_STATUS_RX_OFFSET 1
#define CSR_UART_EV_STATUS_RX_SIZE 1
#define CSR_UART_EV_PENDING_TX_OFFSET 0
#define CSR_UART_EV_PENDING_TX_SIZE 1
#define CSR_UART_EV_PENDING_RX_OFFSET 1
#define CSR_UART_EV_PENDING_RX_SIZE 1
#define CSR_UART_EV_ENABLE_TX_OFFSET 0
#define CSR_UART_EV_ENABLE_TX_SIZE 1
#define CSR_UART_EV_ENABLE_RX_OFFSET 1
#define CSR_UART_EV_ENABLE_RX_SIZE 1

/* ICAP Registers */
#define CSR_ICAP_BASE (CSR_BASE + 0x800L)
#define CSR_ICAP_ADDR_ADDR (CSR_BASE + 0x800L)
#define CSR_ICAP_ADDR_SIZE 1
#define CSR_ICAP_DATA_ADDR (CSR_BASE + 0x804L)
#define CSR_ICAP_DATA_SIZE 1
#define CSR_ICAP_WRITE_ADDR (CSR_BASE + 0x808L)
#define CSR_ICAP_WRITE_SIZE 1
#define CSR_ICAP_DONE_ADDR (CSR_BASE + 0x80cL)
#define CSR_ICAP_DONE_SIZE 1
#define CSR_ICAP_READ_ADDR (CSR_BASE + 0x810L)
#define CSR_ICAP_READ_SIZE 1

/* ICAP Fields */

/* PCIE_PHY Registers */
#define CSR_PCIE_PHY_BASE (CSR_BASE + 0x1000L)
#define CSR_PCIE_PHY_PHY_LINK_STATUS_ADDR (CSR_BASE + 0x1000L)
#define CSR_PCIE_PHY_PHY_LINK_STATUS_SIZE 1
#define CSR_PCIE_PHY_PHY_MSI_ENABLE_ADDR (CSR_BASE + 0x1004L)
#define CSR_PCIE_PHY_PHY_MSI_ENABLE_SIZE 1
#define CSR_PCIE_PHY_PHY_MSIX_ENABLE_ADDR (CSR_BASE + 0x1008L)
#define CSR_PCIE_PHY_PHY_MSIX_ENABLE_SIZE 1
#define CSR_PCIE_PHY_PHY_BUS_MASTER_ENABLE_ADDR (CSR_BASE + 0x100cL)
#define CSR_PCIE_PHY_PHY_BUS_MASTER_ENABLE_SIZE 1
#define CSR_PCIE_PHY_PHY_MAX_REQUEST_SIZE_ADDR (CSR_BASE + 0x1010L)
#define CSR_PCIE_PHY_PHY_MAX_REQUEST_SIZE_SIZE 1
#define CSR_PCIE_PHY_PHY_MAX_PAYLOAD_SIZE_ADDR (CSR_BASE + 0x1014L)
#define CSR_PCIE_PHY_PHY_MAX_PAYLOAD_SIZE_SIZE 1

/* PCIE_PHY Fields */
#define CSR_PCIE_PHY_PHY_LINK_STATUS_STATUS_OFFSET 0
#define CSR_PCIE_PHY_PHY_LINK_STATUS_STATUS_SIZE 1
#define CSR_PCIE_PHY_PHY_LINK_STATUS_RATE_OFFSET 1
#define CSR_PCIE_PHY_PHY_LINK_STATUS_RATE_SIZE 1
#define CSR_PCIE_PHY_PHY_LINK_STATUS_WIDTH_OFFSET 2
#define CSR_PCIE_PHY_PHY_LINK_STATUS_WIDTH_SIZE 2
#define CSR_PCIE_PHY_PHY_LINK_STATUS_LTSSM_OFFSET 4
#define CSR_PCIE_PHY_PHY_LINK_STATUS_LTSSM_SIZE 6

/* PCIE_MSI Registers */
#define CSR_PCIE_MSI_BASE (CSR_BASE + 0x1800L)
#define CSR_PCIE_MSI_ENABLE_ADDR (CSR_BASE + 0x1800L)
#define CSR_PCIE_MSI_ENABLE_SIZE 1
#define CSR_PCIE_MSI_CLEAR_ADDR (CSR_BASE + 0x1804L)
#define CSR_PCIE_MSI_CLEAR_SIZE 1
#define CSR_PCIE_MSI_VECTOR_ADDR (CSR_BASE + 0x1808L)
#define CSR_PCIE_MSI_VECTOR_SIZE 1

/* PCIE_MSI Fields */

/* ANALYZER_CNTRL Registers */
#define CSR_ANALYZER_CNTRL_BASE (CSR_BASE + 0x2000L)
#define CSR_ANALYZER_CNTRL_MUX_VALUE_ADDR (CSR_BASE + 0x2000L)
#define CSR_ANALYZER_CNTRL_MUX_VALUE_SIZE 1
#define CSR_ANALYZER_CNTRL_TRIGGER_ENABLE_ADDR (CSR_BASE + 0x2004L)
#define CSR_ANALYZER_CNTRL_TRIGGER_ENABLE_SIZE 1
#define CSR_ANALYZER_CNTRL_TRIGGER_DONE_ADDR (CSR_BASE + 0x2008L)
#define CSR_ANALYZER_CNTRL_TRIGGER_DONE_SIZE 1
#define CSR_ANALYZER_CNTRL_TRIGGER_MEM_WRITE_ADDR (CSR_BASE + 0x200cL)
#define CSR_ANALYZER_CNTRL_TRIGGER_MEM_WRITE_SIZE 1
#define CSR_ANALYZER_CNTRL_TRIGGER_MEM_MASK_ADDR (CSR_BASE + 0x2010L)
#define CSR_ANALYZER_CNTRL_TRIGGER_MEM_MASK_SIZE 1
#define CSR_ANALYZER_CNTRL_TRIGGER_MEM_VALUE_ADDR (CSR_BASE + 0x2014L)
#define CSR_ANALYZER_CNTRL_TRIGGER_MEM_VALUE_SIZE 1
#define CSR_ANALYZER_CNTRL_TRIGGER_MEM_FULL_ADDR (CSR_BASE + 0x2018L)
#define CSR_ANALYZER_CNTRL_TRIGGER_MEM_FULL_SIZE 1
#define CSR_ANALYZER_CNTRL_SUBSAMPLER_VALUE_ADDR (CSR_BASE + 0x201cL)
#define CSR_ANALYZER_CNTRL_SUBSAMPLER_VALUE_SIZE 1
#define CSR_ANALYZER_CNTRL_STORAGE_ENABLE_ADDR (CSR_BASE + 0x2020L)
#define CSR_ANALYZER_CNTRL_STORAGE_ENABLE_SIZE 1
#define CSR_ANALYZER_CNTRL_STORAGE_DONE_ADDR (CSR_BASE + 0x2024L)
#define CSR_ANALYZER_CNTRL_STORAGE_DONE_SIZE 1
#define CSR_ANALYZER_CNTRL_STORAGE_LENGTH_ADDR (CSR_BASE + 0x2028L)
#define CSR_ANALYZER_CNTRL_STORAGE_LENGTH_SIZE 1
#define CSR_ANALYZER_CNTRL_STORAGE_OFFSET_ADDR (CSR_BASE + 0x202cL)
#define CSR_ANALYZER_CNTRL_STORAGE_OFFSET_SIZE 1
#define CSR_ANALYZER_CNTRL_STORAGE_MEM_LEVEL_ADDR (CSR_BASE + 0x2030L)
#define CSR_ANALYZER_CNTRL_STORAGE_MEM_LEVEL_SIZE 1
#define CSR_ANALYZER_CNTRL_STORAGE_MEM_DATA_ADDR (CSR_BASE + 0x2034L)
#define CSR_ANALYZER_CNTRL_STORAGE_MEM_DATA_SIZE 1

/* ANALYZER_CNTRL Fields */

/* PCIE_DMA0 Registers */
#define CSR_PCIE_DMA0_BASE (CSR_BASE + 0x2800L)
#define CSR_PCIE_DMA0_WRITER_ENABLE_ADDR (CSR_BASE + 0x2800L)
#define CSR_PCIE_DMA0_WRITER_ENABLE_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_ADDR (CSR_BASE + 0x2804L)
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_SIZE 2
#define CSR_PCIE_DMA0_WRITER_TABLE_WE_ADDR (CSR_BASE + 0x280cL)
#define CSR_PCIE_DMA0_WRITER_TABLE_WE_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_PROG_N_ADDR (CSR_BASE + 0x2810L)
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_PROG_N_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_ADDR (CSR_BASE + 0x2814L)
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_LEVEL_ADDR (CSR_BASE + 0x2818L)
#define CSR_PCIE_DMA0_WRITER_TABLE_LEVEL_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_RESET_ADDR (CSR_BASE + 0x281cL)
#define CSR_PCIE_DMA0_WRITER_TABLE_RESET_SIZE 1
#define CSR_PCIE_DMA0_READER_ENABLE_ADDR (CSR_BASE + 0x2820L)
#define CSR_PCIE_DMA0_READER_ENABLE_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_ADDR (CSR_BASE + 0x2824L)
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_SIZE 2
#define CSR_PCIE_DMA0_READER_TABLE_WE_ADDR (CSR_BASE + 0x282cL)
#define CSR_PCIE_DMA0_READER_TABLE_WE_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_PROG_N_ADDR (CSR_BASE + 0x2830L)
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_PROG_N_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_ADDR (CSR_BASE + 0x2834L)
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_LEVEL_ADDR (CSR_BASE + 0x2838L)
#define CSR_PCIE_DMA0_READER_TABLE_LEVEL_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_RESET_ADDR (CSR_BASE + 0x283cL)
#define CSR_PCIE_DMA0_READER_TABLE_RESET_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_ADDR (CSR_BASE + 0x2840L)
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_STATUS_ADDR (CSR_BASE + 0x2844L)
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_STATUS_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_ADDR (CSR_BASE + 0x2848L)
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS_ADDR (CSR_BASE + 0x284cL)
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS_SIZE 1

/* PCIE_DMA0 Fields */
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_ADDRESS_LSB_OFFSET 0
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_ADDRESS_LSB_SIZE 32
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_LENGTH_OFFSET 32
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_LENGTH_SIZE 24
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_IRQ_DISABLE_OFFSET 56
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_IRQ_DISABLE_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_LAST_DISABLE_OFFSET 57
#define CSR_PCIE_DMA0_WRITER_TABLE_VALUE_LAST_DISABLE_SIZE 1
#define CSR_PCIE_DMA0_WRITER_TABLE_WE_ADDRESS_MSB_OFFSET 0
#define CSR_PCIE_DMA0_WRITER_TABLE_WE_ADDRESS_MSB_SIZE 32
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_INDEX_OFFSET 0
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_INDEX_SIZE 16
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_COUNT_OFFSET 16
#define CSR_PCIE_DMA0_WRITER_TABLE_LOOP_STATUS_COUNT_SIZE 16
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_ADDRESS_LSB_OFFSET 0
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_ADDRESS_LSB_SIZE 32
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_LENGTH_OFFSET 32
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_LENGTH_SIZE 24
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_IRQ_DISABLE_OFFSET 56
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_IRQ_DISABLE_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_LAST_DISABLE_OFFSET 57
#define CSR_PCIE_DMA0_READER_TABLE_VALUE_LAST_DISABLE_SIZE 1
#define CSR_PCIE_DMA0_READER_TABLE_WE_ADDRESS_MSB_OFFSET 0
#define CSR_PCIE_DMA0_READER_TABLE_WE_ADDRESS_MSB_SIZE 32
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_INDEX_OFFSET 0
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_INDEX_SIZE 16
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_COUNT_OFFSET 16
#define CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_COUNT_SIZE 16
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_DEPTH_OFFSET 0
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_DEPTH_SIZE 24
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_SCRATCH_OFFSET 24
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_SCRATCH_SIZE 4
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_LEVEL_MODE_OFFSET 31
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_CONTROL_LEVEL_MODE_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_STATUS_LEVEL_OFFSET 0
#define CSR_PCIE_DMA0_BUFFERING_READER_FIFO_STATUS_LEVEL_SIZE 24
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_DEPTH_OFFSET 0
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_DEPTH_SIZE 24
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_SCRATCH_OFFSET 24
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_SCRATCH_SIZE 4
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_LEVEL_MODE_OFFSET 31
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_CONTROL_LEVEL_MODE_SIZE 1
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS_LEVEL_OFFSET 0
#define CSR_PCIE_DMA0_BUFFERING_WRITER_FIFO_STATUS_LEVEL_SIZE 24

/* AUX Registers */
#define CSR_AUX_BASE (CSR_BASE + 0x3000L)
#define CSR_AUX_CONTROL_ADDR (CSR_BASE + 0x3000L)
#define CSR_AUX_CONTROL_SIZE 1

/* AUX Fields */
#define CSR_AUX_CONTROL_IOVCC_SEL_OFFSET 0
#define CSR_AUX_CONTROL_IOVCC_SEL_SIZE 1
#define CSR_AUX_CONTROL_EN_SMSIGIO_OFFSET 1
#define CSR_AUX_CONTROL_EN_SMSIGIO_SIZE 1
#define CSR_AUX_CONTROL_OPTION_OFFSET 2
#define CSR_AUX_CONTROL_OPTION_SIZE 1
#define CSR_AUX_CONTROL_GPIO13_OFFSET 3
#define CSR_AUX_CONTROL_GPIO13_SIZE 1

/* CTRL Registers */
#define CSR_CTRL_BASE (CSR_BASE + 0x3800L)
#define CSR_CTRL_RESET_ADDR (CSR_BASE + 0x3800L)
#define CSR_CTRL_RESET_SIZE 1
#define CSR_CTRL_SCRATCH_ADDR (CSR_BASE + 0x3804L)
#define CSR_CTRL_SCRATCH_SIZE 1
#define CSR_CTRL_BUS_ERRORS_ADDR (CSR_BASE + 0x3808L)
#define CSR_CTRL_BUS_ERRORS_SIZE 1

/* CTRL Fields */
#define CSR_CTRL_RESET_SOC_RST_OFFSET 0
#define CSR_CTRL_RESET_SOC_RST_SIZE 1
#define CSR_CTRL_RESET_CPU_RST_OFFSET 1
#define CSR_CTRL_RESET_CPU_RST_SIZE 1

/* FLASH_CS_N Registers */
#define CSR_FLASH_CS_N_BASE (CSR_BASE + 0x4000L)
#define CSR_FLASH_CS_N_OUT_ADDR (CSR_BASE + 0x4000L)
#define CSR_FLASH_CS_N_OUT_SIZE 1

/* FLASH_CS_N Fields */

/* FPGACFG Registers */
#define CSR_FPGACFG_BASE (CSR_BASE + 0x4800L)
#define CSR_FPGACFG_BOARD_ID_ADDR (CSR_BASE + 0x4800L)
#define CSR_FPGACFG_BOARD_ID_SIZE 1
#define CSR_FPGACFG_MAJOR_REV_ADDR (CSR_BASE + 0x4804L)
#define CSR_FPGACFG_MAJOR_REV_SIZE 1
#define CSR_FPGACFG_COMPILE_REV_ADDR (CSR_BASE + 0x4808L)
#define CSR_FPGACFG_COMPILE_REV_SIZE 1
#define CSR_FPGACFG_RESERVED_03_ADDR (CSR_BASE + 0x480cL)
#define CSR_FPGACFG_RESERVED_03_SIZE 1
#define CSR_FPGACFG_RESERVED_04_ADDR (CSR_BASE + 0x4810L)
#define CSR_FPGACFG_RESERVED_04_SIZE 1
#define CSR_FPGACFG_RESERVED_05_ADDR (CSR_BASE + 0x4814L)
#define CSR_FPGACFG_RESERVED_05_SIZE 1
#define CSR_FPGACFG_RESERVED_06_ADDR (CSR_BASE + 0x4818L)
#define CSR_FPGACFG_RESERVED_06_SIZE 1
#define CSR_FPGACFG_CHANNEL_CNTRL_ADDR (CSR_BASE + 0x481cL)
#define CSR_FPGACFG_CHANNEL_CNTRL_SIZE 1

/* FPGACFG Fields */
#define CSR_FPGACFG_CHANNEL_CNTRL_CH_EN_OFFSET 0
#define CSR_FPGACFG_CHANNEL_CNTRL_CH_EN_SIZE 2

/* FLASH Registers */
#define CSR_FLASH_BASE (CSR_BASE + 0x5000L)
#define CSR_FLASH_SPI_CONTROL_ADDR (CSR_BASE + 0x5000L)
#define CSR_FLASH_SPI_CONTROL_SIZE 1
#define CSR_FLASH_SPI_STATUS_ADDR (CSR_BASE + 0x5004L)
#define CSR_FLASH_SPI_STATUS_SIZE 1
#define CSR_FLASH_SPI_MOSI_ADDR (CSR_BASE + 0x5008L)
#define CSR_FLASH_SPI_MOSI_SIZE 2
#define CSR_FLASH_SPI_MISO_ADDR (CSR_BASE + 0x5010L)
#define CSR_FLASH_SPI_MISO_SIZE 2
#define CSR_FLASH_SPI_CS_ADDR (CSR_BASE + 0x5018L)
#define CSR_FLASH_SPI_CS_SIZE 1
#define CSR_FLASH_SPI_LOOPBACK_ADDR (CSR_BASE + 0x501cL)
#define CSR_FLASH_SPI_LOOPBACK_SIZE 1

/* FLASH Fields */
#define CSR_FLASH_SPI_CONTROL_START_OFFSET 0
#define CSR_FLASH_SPI_CONTROL_START_SIZE 1
#define CSR_FLASH_SPI_CONTROL_LENGTH_OFFSET 8
#define CSR_FLASH_SPI_CONTROL_LENGTH_SIZE 8
#define CSR_FLASH_SPI_STATUS_DONE_OFFSET 0
#define CSR_FLASH_SPI_STATUS_DONE_SIZE 1
#define CSR_FLASH_SPI_STATUS_MODE_OFFSET 1
#define CSR_FLASH_SPI_STATUS_MODE_SIZE 1
#define CSR_FLASH_SPI_CS_SEL_OFFSET 0
#define CSR_FLASH_SPI_CS_SEL_SIZE 1
#define CSR_FLASH_SPI_CS_MODE_OFFSET 16
#define CSR_FLASH_SPI_CS_MODE_SIZE 1
#define CSR_FLASH_SPI_LOOPBACK_MODE_OFFSET 0
#define CSR_FLASH_SPI_LOOPBACK_MODE_SIZE 1

/* XADC Registers */
#define CSR_XADC_BASE (CSR_BASE + 0x5800L)
#define CSR_XADC_TEMPERATURE_ADDR (CSR_BASE + 0x5800L)
#define CSR_XADC_TEMPERATURE_SIZE 1
#define CSR_XADC_VCCINT_ADDR (CSR_BASE + 0x5804L)
#define CSR_XADC_VCCINT_SIZE 1
#define CSR_XADC_VCCAUX_ADDR (CSR_BASE + 0x5808L)
#define CSR_XADC_VCCAUX_SIZE 1
#define CSR_XADC_VCCBRAM_ADDR (CSR_BASE + 0x580cL)
#define CSR_XADC_VCCBRAM_SIZE 1
#define CSR_XADC_EOC_ADDR (CSR_BASE + 0x5810L)
#define CSR_XADC_EOC_SIZE 1
#define CSR_XADC_EOS_ADDR (CSR_BASE + 0x5814L)
#define CSR_XADC_EOS_SIZE 1

/* XADC Fields */

/* DNA Registers */
#define CSR_DNA_BASE (CSR_BASE + 0x6000L)
#define CSR_DNA_ID_ADDR (CSR_BASE + 0x6000L)
#define CSR_DNA_ID_SIZE 2

/* DNA Fields */

/* IDENTIFIER_MEM Registers */
#define CSR_IDENTIFIER_MEM_BASE (CSR_BASE + 0x6800L)

/* IDENTIFIER_MEM Fields */

/* LEDS Registers */
#define CSR_LEDS_BASE (CSR_BASE + 0x7000L)
#define CSR_LEDS_OUT_ADDR (CSR_BASE + 0x7000L)
#define CSR_LEDS_OUT_SIZE 1

/* LEDS Fields */

/* LIME_TOP Registers */
#define CSR_LIME_TOP_BASE (CSR_BASE + 0x7800L)
#define CSR_LIME_TOP_SCRATCH_ADDR (CSR_BASE + 0x7800L)
#define CSR_LIME_TOP_SCRATCH_SIZE 1
#define CSR_LIME_TOP_LMS7002_CONTROL_ADDR (CSR_BASE + 0x7804L)
#define CSR_LIME_TOP_LMS7002_CONTROL_SIZE 1
#define CSR_LIME_TOP_LMS7002_TX_EN_ADDR (CSR_BASE + 0x7808L)
#define CSR_LIME_TOP_LMS7002_TX_EN_SIZE 1
#define CSR_LIME_TOP_LMS7002_RX_EN_ADDR (CSR_BASE + 0x780cL)
#define CSR_LIME_TOP_LMS7002_RX_EN_SIZE 1
#define CSR_LIME_TOP_LMS7002_TRXIQ_PULSE_ADDR (CSR_BASE + 0x7810L)
#define CSR_LIME_TOP_LMS7002_TRXIQ_PULSE_SIZE 1
#define CSR_LIME_TOP_LMS7002_DDR_EN_ADDR (CSR_BASE + 0x7814L)
#define CSR_LIME_TOP_LMS7002_DDR_EN_SIZE 1
#define CSR_LIME_TOP_LMS7002_MIMO_INT_EN_ADDR (CSR_BASE + 0x7818L)
#define CSR_LIME_TOP_LMS7002_MIMO_INT_EN_SIZE 1
#define CSR_LIME_TOP_LMS7002_CH_EN_ADDR (CSR_BASE + 0x781cL)
#define CSR_LIME_TOP_LMS7002_CH_EN_SIZE 1
#define CSR_LIME_TOP_LMS7002_LMS1_TXEN_ADDR (CSR_BASE + 0x7820L)
#define CSR_LIME_TOP_LMS7002_LMS1_TXEN_SIZE 1
#define CSR_LIME_TOP_LMS7002_LMS1_RXEN_ADDR (CSR_BASE + 0x7824L)
#define CSR_LIME_TOP_LMS7002_LMS1_RXEN_SIZE 1
#define CSR_LIME_TOP_LMS7002_LMS1_TXRXEN_MUX_SEL_ADDR (CSR_BASE + 0x7828L)
#define CSR_LIME_TOP_LMS7002_LMS1_TXRXEN_MUX_SEL_SIZE 1
#define CSR_LIME_TOP_LMS7002_LMS1_TXRXEN_INV_ADDR (CSR_BASE + 0x782cL)
#define CSR_LIME_TOP_LMS7002_LMS1_TXRXEN_INV_SIZE 1
#define CSR_LIME_TOP_LMS7002_LMS1_RESETN_ADDR (CSR_BASE + 0x7830L)
#define CSR_LIME_TOP_LMS7002_LMS1_RESETN_SIZE 1
#define CSR_LIME_TOP_LMS7002_LMS1_CORE_LDO_EN_ADDR (CSR_BASE + 0x7834L)
#define CSR_LIME_TOP_LMS7002_LMS1_CORE_LDO_EN_SIZE 1
#define CSR_LIME_TOP_LMS7002_LMS1_TXNRX1_ADDR (CSR_BASE + 0x7838L)
#define CSR_LIME_TOP_LMS7002_LMS1_TXNRX1_SIZE 1
#define CSR_LIME_TOP_LMS7002_LMS2_TXNRX2_ADDR (CSR_BASE + 0x783cL)
#define CSR_LIME_TOP_LMS7002_LMS2_TXNRX2_SIZE 1
#define CSR_LIME_TOP_LMS7002_CMP_START_ADDR (CSR_BASE + 0x7840L)
#define CSR_LIME_TOP_LMS7002_CMP_START_SIZE 1
#define CSR_LIME_TOP_LMS7002_CMP_LENGTH_ADDR (CSR_BASE + 0x7844L)
#define CSR_LIME_TOP_LMS7002_CMP_LENGTH_SIZE 1
#define CSR_LIME_TOP_LMS7002_CMP_DONE_ADDR (CSR_BASE + 0x7848L)
#define CSR_LIME_TOP_LMS7002_CMP_DONE_SIZE 1
#define CSR_LIME_TOP_LMS7002_CMP_ERROR_ADDR (CSR_BASE + 0x784cL)
#define CSR_LIME_TOP_LMS7002_CMP_ERROR_SIZE 1
#define CSR_LIME_TOP_LMS7002_TEST_PTRN_EN_ADDR (CSR_BASE + 0x7850L)
#define CSR_LIME_TOP_LMS7002_TEST_PTRN_EN_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PHCFG_MODE_ADDR (CSR_BASE + 0x7854L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PHCFG_MODE_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PHCFG_DONE_ADDR (CSR_BASE + 0x7858L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PHCFG_DONE_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PHCFG_ERR_ADDR (CSR_BASE + 0x785cL)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PHCFG_ERR_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLCFG_DONE_ADDR (CSR_BASE + 0x7860L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLCFG_DONE_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLCFG_BUSY_ADDR (CSR_BASE + 0x7864L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLCFG_BUSY_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLCFG_START_ADDR (CSR_BASE + 0x7868L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLCFG_START_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLRST_START_ADDR (CSR_BASE + 0x786cL)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLRST_START_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLL_IND_ADDR (CSR_BASE + 0x7870L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLL_IND_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PHCFG_START_ADDR (CSR_BASE + 0x7874L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PHCFG_START_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLCFG_ERROR_ADDR (CSR_BASE + 0x7878L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_PLLCFG_ERROR_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_VCO_MULT_BYP_ADDR (CSR_BASE + 0x787cL)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_VCO_MULT_BYP_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_VCO_DIV_BYP_ADDR (CSR_BASE + 0x7880L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_VCO_DIV_BYP_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C0_DIV_BYP_ADDR (CSR_BASE + 0x7884L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C0_DIV_BYP_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C1_DIV_BYP_ADDR (CSR_BASE + 0x7888L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C1_DIV_BYP_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_VCO_DIV_CNT_ADDR (CSR_BASE + 0x788cL)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_VCO_DIV_CNT_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_VCO_MULT_CNT_ADDR (CSR_BASE + 0x7890L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_VCO_MULT_CNT_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C0_DIV_CNT_ADDR (CSR_BASE + 0x7894L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C0_DIV_CNT_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C1_DIV_CNT_ADDR (CSR_BASE + 0x7898L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C1_DIV_CNT_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C1_PHASE_ADDR (CSR_BASE + 0x789cL)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_C1_PHASE_SIZE 1
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_AUTO_PHCFG_SMPLS_ADDR (CSR_BASE + 0x78a0L)
#define CSR_LIME_TOP_LMS7002_CLK_CTRL_AUTO_PHCFG_SMPLS_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_CSR_RESET_ADDR (CSR_BASE + 0x78a4L)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_CSR_RESET_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_RESET_ADDR (CSR_BASE + 0x78a8L)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_RESET_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_LOCKED_ADDR (CSR_BASE + 0x78acL)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_LOCKED_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_READ_ADDR (CSR_BASE + 0x78b0L)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_READ_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_WRITE_ADDR (CSR_BASE + 0x78b4L)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_WRITE_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_DRDY_ADDR (CSR_BASE + 0x78b8L)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_DRDY_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_ADR_ADDR (CSR_BASE + 0x78bcL)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_ADR_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_DAT_W_ADDR (CSR_BASE + 0x78c0L)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_DAT_W_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_DAT_R_ADDR (CSR_BASE + 0x78c4L)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_DRP_DAT_R_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_LATCHED_DRDY_ADDR (CSR_BASE + 0x78c8L)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_LATCHED_DRDY_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_LATCHED_DRDY_RESET_ADDR (CSR_BASE + 0x78ccL)
#define CSR_LIME_TOP_LMS7002_PLL0_TX_MMCM_LATCHED_DRDY_RESET_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_CSR_RESET_ADDR (CSR_BASE + 0x78d0L)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_CSR_RESET_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_RESET_ADDR (CSR_BASE + 0x78d4L)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_RESET_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_LOCKED_ADDR (CSR_BASE + 0x78d8L)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_LOCKED_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_READ_ADDR (CSR_BASE + 0x78dcL)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_READ_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_WRITE_ADDR (CSR_BASE + 0x78e0L)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_WRITE_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_DRDY_ADDR (CSR_BASE + 0x78e4L)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_DRDY_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_ADR_ADDR (CSR_BASE + 0x78e8L)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_ADR_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_DAT_W_ADDR (CSR_BASE + 0x78ecL)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_DAT_W_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_DAT_R_ADDR (CSR_BASE + 0x78f0L)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_DRP_DAT_R_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_LATCHED_DRDY_ADDR (CSR_BASE + 0x78f4L)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_LATCHED_DRDY_SIZE 1
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_LATCHED_DRDY_RESET_ADDR (CSR_BASE + 0x78f8L)
#define CSR_LIME_TOP_LMS7002_PLL1_RX_MMCM_LATCHED_DRDY_RESET_SIZE 1
#define CSR_LIME_TOP_GPIO_GPIO_OVERRIDE_ADDR (CSR_BASE + 0x78fcL)
#define CSR_LIME_TOP_GPIO_GPIO_OVERRIDE_SIZE 1
#define CSR_LIME_TOP_GPIO_GPIO_OVERRIDE_DIR_ADDR (CSR_BASE + 0x7900L)
#define CSR_LIME_TOP_GPIO_GPIO_OVERRIDE_DIR_SIZE 1
#define CSR_LIME_TOP_GPIO_GPIO_OVERRIDE_VAL_ADDR (CSR_BASE + 0x7904L)
#define CSR_LIME_TOP_GPIO_GPIO_OVERRIDE_VAL_SIZE 1
#define CSR_LIME_TOP_GPIO_GPIO_VAL_ADDR (CSR_BASE + 0x7908L)
#define CSR_LIME_TOP_GPIO_GPIO_VAL_SIZE 1
#define CSR_LIME_TOP_RX_PATH_CH_EN_ADDR (CSR_BASE + 0x790cL)
#define CSR_LIME_TOP_RX_PATH_CH_EN_SIZE 1
#define CSR_LIME_TOP_RX_PATH_SMPL_WIDTH_ADDR (CSR_BASE + 0x7910L)
#define CSR_LIME_TOP_RX_PATH_SMPL_WIDTH_SIZE 1
#define CSR_LIME_TOP_RX_PATH_PKT_SIZE_ADDR (CSR_BASE + 0x7914L)
#define CSR_LIME_TOP_RX_PATH_PKT_SIZE_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_MUX_VALUE_ADDR (CSR_BASE + 0x7918L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_MUX_VALUE_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_ENABLE_ADDR (CSR_BASE + 0x791cL)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_ENABLE_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_DONE_ADDR (CSR_BASE + 0x7920L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_DONE_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_MEM_WRITE_ADDR (CSR_BASE + 0x7924L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_MEM_WRITE_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_MEM_MASK_ADDR (CSR_BASE + 0x7928L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_MEM_MASK_SIZE 5
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_MEM_VALUE_ADDR (CSR_BASE + 0x793cL)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_MEM_VALUE_SIZE 5
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_MEM_FULL_ADDR (CSR_BASE + 0x7950L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_TRIGGER_MEM_FULL_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_SUBSAMPLER_VALUE_ADDR (CSR_BASE + 0x7954L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_SUBSAMPLER_VALUE_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_ENABLE_ADDR (CSR_BASE + 0x7958L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_ENABLE_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_DONE_ADDR (CSR_BASE + 0x795cL)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_DONE_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_LENGTH_ADDR (CSR_BASE + 0x7960L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_LENGTH_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_OFFSET_ADDR (CSR_BASE + 0x7964L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_OFFSET_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_MEM_LEVEL_ADDR (CSR_BASE + 0x7968L)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_MEM_LEVEL_SIZE 1
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_MEM_DATA_ADDR (CSR_BASE + 0x796cL)
#define CSR_LIME_TOP_RX_PATH_ANALYZER_STORAGE_MEM_DATA_SIZE 1
#define CSR_LIME_TOP_ANALYZER_MUX_VALUE_ADDR (CSR_BASE + 0x7970L)
#define CSR_LIME_TOP_ANALYZER_MUX_VALUE_SIZE 1
#define CSR_LIME_TOP_ANALYZER_TRIGGER_ENABLE_ADDR (CSR_BASE + 0x7974L)
#define CSR_LIME_TOP_ANALYZER_TRIGGER_ENABLE_SIZE 1
#define CSR_LIME_TOP_ANALYZER_TRIGGER_DONE_ADDR (CSR_BASE + 0x7978L)
#define CSR_LIME_TOP_ANALYZER_TRIGGER_DONE_SIZE 1
#define CSR_LIME_TOP_ANALYZER_TRIGGER_MEM_WRITE_ADDR (CSR_BASE + 0x797cL)
#define CSR_LIME_TOP_ANALYZER_TRIGGER_MEM_WRITE_SIZE 1
#define CSR_LIME_TOP_ANALYZER_TRIGGER_MEM_MASK_ADDR (CSR_BASE + 0x7980L)
#define CSR_LIME_TOP_ANALYZER_TRIGGER_MEM_MASK_SIZE 3
#define CSR_LIME_TOP_ANALYZER_TRIGGER_MEM_VALUE_ADDR (CSR_BASE + 0x798cL)
#define CSR_LIME_TOP_ANALYZER_TRIGGER_MEM_VALUE_SIZE 3
#define CSR_LIME_TOP_ANALYZER_TRIGGER_MEM_FULL_ADDR (CSR_BASE + 0x7998L)
#define CSR_LIME_TOP_ANALYZER_TRIGGER_MEM_FULL_SIZE 1
#define CSR_LIME_TOP_ANALYZER_SUBSAMPLER_VALUE_ADDR (CSR_BASE + 0x799cL)
#define CSR_LIME_TOP_ANALYZER_SUBSAMPLER_VALUE_SIZE 1
#define CSR_LIME_TOP_ANALYZER_STORAGE_ENABLE_ADDR (CSR_BASE + 0x79a0L)
#define CSR_LIME_TOP_ANALYZER_STORAGE_ENABLE_SIZE 1
#define CSR_LIME_TOP_ANALYZER_STORAGE_DONE_ADDR (CSR_BASE + 0x79a4L)
#define CSR_LIME_TOP_ANALYZER_STORAGE_DONE_SIZE 1
#define CSR_LIME_TOP_ANALYZER_STORAGE_LENGTH_ADDR (CSR_BASE + 0x79a8L)
#define CSR_LIME_TOP_ANALYZER_STORAGE_LENGTH_SIZE 1
#define CSR_LIME_TOP_ANALYZER_STORAGE_OFFSET_ADDR (CSR_BASE + 0x79acL)
#define CSR_LIME_TOP_ANALYZER_STORAGE_OFFSET_SIZE 1
#define CSR_LIME_TOP_ANALYZER_STORAGE_MEM_LEVEL_ADDR (CSR_BASE + 0x79b0L)
#define CSR_LIME_TOP_ANALYZER_STORAGE_MEM_LEVEL_SIZE 1
#define CSR_LIME_TOP_ANALYZER_STORAGE_MEM_DATA_ADDR (CSR_BASE + 0x79b4L)
#define CSR_LIME_TOP_ANALYZER_STORAGE_MEM_DATA_SIZE 1
#define CSR_LIME_TOP_EV_STATUS_ADDR (CSR_BASE + 0x79b8L)
#define CSR_LIME_TOP_EV_STATUS_SIZE 1
#define CSR_LIME_TOP_EV_PENDING_ADDR (CSR_BASE + 0x79bcL)
#define CSR_LIME_TOP_EV_PENDING_SIZE 1
#define CSR_LIME_TOP_EV_ENABLE_ADDR (CSR_BASE + 0x79c0L)
#define CSR_LIME_TOP_EV_ENABLE_SIZE 1

/* LIME_TOP Fields */
#define CSR_LIME_TOP_LMS7002_CONTROL_LMS1_TXNRX1_OFFSET 12
#define CSR_LIME_TOP_LMS7002_CONTROL_LMS1_TXNRX1_SIZE 1
#define CSR_LIME_TOP_LMS7002_CONTROL_LMS1_TXNRX2_OFFSET 13
#define CSR_LIME_TOP_LMS7002_CONTROL_LMS1_TXNRX2_SIZE 1
#define CSR_LIME_TOP_EV_STATUS_CLK_CTRL_IRQ_OFFSET 0
#define CSR_LIME_TOP_EV_STATUS_CLK_CTRL_IRQ_SIZE 1
#define CSR_LIME_TOP_EV_PENDING_CLK_CTRL_IRQ_OFFSET 0
#define CSR_LIME_TOP_EV_PENDING_CLK_CTRL_IRQ_SIZE 1
#define CSR_LIME_TOP_EV_ENABLE_CLK_CTRL_IRQ_OFFSET 0
#define CSR_LIME_TOP_EV_ENABLE_CLK_CTRL_IRQ_SIZE 1

/* LMS_SPI Registers */
#define CSR_LMS_SPI_BASE (CSR_BASE + 0x8000L)
#define CSR_LMS_SPI_CONTROL_ADDR (CSR_BASE + 0x8000L)
#define CSR_LMS_SPI_CONTROL_SIZE 1
#define CSR_LMS_SPI_STATUS_ADDR (CSR_BASE + 0x8004L)
#define CSR_LMS_SPI_STATUS_SIZE 1
#define CSR_LMS_SPI_MOSI_ADDR (CSR_BASE + 0x8008L)
#define CSR_LMS_SPI_MOSI_SIZE 1
#define CSR_LMS_SPI_MISO_ADDR (CSR_BASE + 0x800cL)
#define CSR_LMS_SPI_MISO_SIZE 1
#define CSR_LMS_SPI_CS_ADDR (CSR_BASE + 0x8010L)
#define CSR_LMS_SPI_CS_SIZE 1
#define CSR_LMS_SPI_LOOPBACK_ADDR (CSR_BASE + 0x8014L)
#define CSR_LMS_SPI_LOOPBACK_SIZE 1

/* LMS_SPI Fields */
#define CSR_LMS_SPI_CONTROL_START_OFFSET 0
#define CSR_LMS_SPI_CONTROL_START_SIZE 1
#define CSR_LMS_SPI_CONTROL_LENGTH_OFFSET 8
#define CSR_LMS_SPI_CONTROL_LENGTH_SIZE 8
#define CSR_LMS_SPI_STATUS_DONE_OFFSET 0
#define CSR_LMS_SPI_STATUS_DONE_SIZE 1
#define CSR_LMS_SPI_STATUS_MODE_OFFSET 1
#define CSR_LMS_SPI_STATUS_MODE_SIZE 1
#define CSR_LMS_SPI_CS_SEL_OFFSET 0
#define CSR_LMS_SPI_CS_SEL_SIZE 1
#define CSR_LMS_SPI_CS_MODE_OFFSET 16
#define CSR_LMS_SPI_CS_MODE_SIZE 1
#define CSR_LMS_SPI_LOOPBACK_MODE_OFFSET 0
#define CSR_LMS_SPI_LOOPBACK_MODE_SIZE 1

/* PCIE_ENDPOINT Registers */
#define CSR_PCIE_ENDPOINT_BASE (CSR_BASE + 0x8800L)
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_ADDR (CSR_BASE + 0x8800L)
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_MSI_ENABLE_ADDR (CSR_BASE + 0x8804L)
#define CSR_PCIE_ENDPOINT_PHY_MSI_ENABLE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_MSIX_ENABLE_ADDR (CSR_BASE + 0x8808L)
#define CSR_PCIE_ENDPOINT_PHY_MSIX_ENABLE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_BUS_MASTER_ENABLE_ADDR (CSR_BASE + 0x880cL)
#define CSR_PCIE_ENDPOINT_PHY_BUS_MASTER_ENABLE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_MAX_REQUEST_SIZE_ADDR (CSR_BASE + 0x8810L)
#define CSR_PCIE_ENDPOINT_PHY_MAX_REQUEST_SIZE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_MAX_PAYLOAD_SIZE_ADDR (CSR_BASE + 0x8814L)
#define CSR_PCIE_ENDPOINT_PHY_MAX_PAYLOAD_SIZE_SIZE 1

/* PCIE_ENDPOINT Fields */
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_STATUS_OFFSET 0
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_STATUS_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_RATE_OFFSET 1
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_RATE_SIZE 1
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_WIDTH_OFFSET 2
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_WIDTH_SIZE 2
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_LTSSM_OFFSET 4
#define CSR_PCIE_ENDPOINT_PHY_LINK_STATUS_LTSSM_SIZE 6

/* TIMER0 Registers */
#define CSR_TIMER0_BASE (CSR_BASE + 0x9000L)
#define CSR_TIMER0_LOAD_ADDR (CSR_BASE + 0x9000L)
#define CSR_TIMER0_LOAD_SIZE 1
#define CSR_TIMER0_RELOAD_ADDR (CSR_BASE + 0x9004L)
#define CSR_TIMER0_RELOAD_SIZE 1
#define CSR_TIMER0_EN_ADDR (CSR_BASE + 0x9008L)
#define CSR_TIMER0_EN_SIZE 1
#define CSR_TIMER0_UPDATE_VALUE_ADDR (CSR_BASE + 0x900cL)
#define CSR_TIMER0_UPDATE_VALUE_SIZE 1
#define CSR_TIMER0_VALUE_ADDR (CSR_BASE + 0x9010L)
#define CSR_TIMER0_VALUE_SIZE 1
#define CSR_TIMER0_EV_STATUS_ADDR (CSR_BASE + 0x9014L)
#define CSR_TIMER0_EV_STATUS_SIZE 1
#define CSR_TIMER0_EV_PENDING_ADDR (CSR_BASE + 0x9018L)
#define CSR_TIMER0_EV_PENDING_SIZE 1
#define CSR_TIMER0_EV_ENABLE_ADDR (CSR_BASE + 0x901cL)
#define CSR_TIMER0_EV_ENABLE_SIZE 1

/* TIMER0 Fields */
#define CSR_TIMER0_EV_STATUS_ZERO_OFFSET 0
#define CSR_TIMER0_EV_STATUS_ZERO_SIZE 1
#define CSR_TIMER0_EV_PENDING_ZERO_OFFSET 0
#define CSR_TIMER0_EV_PENDING_ZERO_SIZE 1
#define CSR_TIMER0_EV_ENABLE_ZERO_OFFSET 0
#define CSR_TIMER0_EV_ENABLE_ZERO_SIZE 1

/* I2C0 Registers */
#define CSR_I2C0_BASE (CSR_BASE + 0xa000L)
#define CSR_I2C0_W_ADDR (CSR_BASE + 0xa000L)
#define CSR_I2C0_W_SIZE 1
#define CSR_I2C0_R_ADDR (CSR_BASE + 0xa004L)
#define CSR_I2C0_R_SIZE 1

/* I2C0 Fields */
#define CSR_I2C0_W_SCL_OFFSET 0
#define CSR_I2C0_W_SCL_SIZE 1
#define CSR_I2C0_W_OE_OFFSET 1
#define CSR_I2C0_W_OE_SIZE 1
#define CSR_I2C0_W_SDA_OFFSET 2
#define CSR_I2C0_W_SDA_SIZE 1
#define CSR_I2C0_R_SDA_OFFSET 0
#define CSR_I2C0_R_SDA_SIZE 1

/* I2C1 Registers */
#define CSR_I2C1_BASE (CSR_BASE + 0xa800L)
#define CSR_I2C1_W_ADDR (CSR_BASE + 0xa800L)
#define CSR_I2C1_W_SIZE 1
#define CSR_I2C1_R_ADDR (CSR_BASE + 0xa804L)
#define CSR_I2C1_R_SIZE 1

/* I2C1 Fields */
#define CSR_I2C1_W_SCL_OFFSET 0
#define CSR_I2C1_W_SCL_SIZE 1
#define CSR_I2C1_W_OE_OFFSET 1
#define CSR_I2C1_W_OE_SIZE 1
#define CSR_I2C1_W_SDA_OFFSET 2
#define CSR_I2C1_W_SDA_SIZE 1
#define CSR_I2C1_R_SDA_OFFSET 0
#define CSR_I2C1_R_SDA_SIZE 1

/* CNTRL Registers */
#define CSR_CNTRL_BASE (CSR_BASE + 0xd000L)
#define CSR_CNTRL_CNTRL_ADDR (CSR_BASE + 0xd000L)
#define CSR_CNTRL_CNTRL_SIZE 16
#define CSR_CNTRL_ENABLE_ADDR (CSR_BASE + 0xd040L)
#define CSR_CNTRL_ENABLE_SIZE 1
#define CSR_CNTRL_TEST_ADDR (CSR_BASE + 0xd044L)
#define CSR_CNTRL_TEST_SIZE 1
#define CSR_CNTRL_NDMA_ADDR (CSR_BASE + 0xd048L)
#define CSR_CNTRL_NDMA_SIZE 1
#define CSR_CNTRL_ENABLE_BOTH_ADDR (CSR_BASE + 0xd04cL)
#define CSR_CNTRL_ENABLE_BOTH_SIZE 1
#define CSR_CNTRL_EV_STATUS_ADDR (CSR_BASE + 0xd050L)
#define CSR_CNTRL_EV_STATUS_SIZE 1
#define CSR_CNTRL_EV_PENDING_ADDR (CSR_BASE + 0xd054L)
#define CSR_CNTRL_EV_PENDING_SIZE 1
#define CSR_CNTRL_EV_ENABLE_ADDR (CSR_BASE + 0xd058L)
#define CSR_CNTRL_EV_ENABLE_SIZE 1

/* CNTRL Fields */
#define CSR_CNTRL_EV_STATUS_CNTRL_ISR_OFFSET 0
#define CSR_CNTRL_EV_STATUS_CNTRL_ISR_SIZE 1
#define CSR_CNTRL_EV_PENDING_CNTRL_ISR_OFFSET 0
#define CSR_CNTRL_EV_PENDING_CNTRL_ISR_SIZE 1
#define CSR_CNTRL_EV_ENABLE_CNTRL_ISR_OFFSET 0
#define CSR_CNTRL_EV_ENABLE_CNTRL_ISR_SIZE 1

#endif /* ! __GENERATED_CSR_H */
