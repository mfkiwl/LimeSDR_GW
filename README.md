# LimeSDR-XTRX LiteX Gateware infrastructure

The objective of this project is to provide the necessary infrastructure to enable Lime Microsystems
to efficiently develop and maintain LiteX-based gateware for their LimeSDR XTRX product.

This repository is based on the [LimeSDR-XTRX LiteX Gateware](https://github.com/myriadrf/LimeSDR-XTRX_LiteX_GW)
repository with the following added/tested features:
- VexRiscv-SMP SoftCore CPU with debug capabilities over JTAG and interrupts.
- PCIe core with MMAP and DMA interfaces (AXI-MMAP and AXI-ST).
- Gateware loading/flashing to SRAM and SPI Flash.
- Firmware loading from PCIe.
- Firmware flashing/loading to/from SPI Flash.
- Firmware debug through GDB over JTAG.
- Firmware interrupt handling example.
- CSR registers integration example.
- VHDL integration example.
- LiteScope logic analyzer example


## Cloning the Repository

To clone this repository and initialize the submodules, run the following commands:

```bash
git clone https://github.com/enjoy-digital/LimeSDR-XTRX_LiteX_GW.git
git checkout litex_infrastructure_setup
git submodule init 
git submodule update
```

## Building the Gateware

To build the gateware, use the following command with the appropriate board option:

```bash
./limesdr_xtrx.py --build --board=fairwaves_pro [--load] [--flash]
```

This command builds the gateware for a *fairwaves_pro* XTRX board by default. To select another
variant, use the `--board` option with one of the following values:

- `fairwaves_pro` for the professional version of the *XTRX* with an *XC7A35T FPGA* (default).
- `fairwaves_cs` for the commercial version of the *XTRX* with an *XC7A50T FPGA* (default).
- `limesdr` for the *LimeMicroSystems/MyriadRF* latest variant of the  *XTRX*.

The `--load` and `--flash` options update the FPGA and/or Flash as described below.

Additional options include:

- `--with-bscan` to add JTAG access to the *vexriscv-smp* softcore for debugging.
- `--flash-boot` to flash the CPU Softcore's firmware in SPI (requires `--flash`).

**Note:** `--load`, `--flash`, and `--flash-boot` use **JTAG** to communicate with the FPGA. These
  actions require an external probe, with a *digilent_hs2* cable used by default. Use `--cable`
  followed by the cable name to change this (see `openFPGALoader --list-cables` for supported
  cables).

## VexRiscv-SMP SoftCore CPU with Debug Capabilities

VexRiscv-SMP SoftCore has been integrated and configured in the `limesdr_xtrx.py` target design. The
configuration has IRQ enabled (which have been recently added to upstream LiteX) and debug
capabilities over JTAG, which were the two requirements for the CPU. VexRiscv-SMP IRQs are enabled
by default. To enable debug capabilities in the target, the additional command lines are added:

```python
if with_bscan:
    from litex.soc.cores.cpu.vexriscv_smp import VexRiscvSMP
    VexRiscvSMP.privileged_debug     = True
    VexRiscvSMP.hardware_breakpoints = 4
    VexRiscvSMP.with_rvc             = True
```

IRQ and Debug capabilities are demonstrated in the following sections.

## PCIe Core with MMAP and DMA Interfaces

The `limesdr_xtrx.py` integrates LitePCIe core that provides:
- MMAP interface to access registers of the design and configure the cores.
- DMA interfaces to stream data to and from the Host.

MMAP and DMA interfaces are demonstrated by LitePCIe utilities:

### MMAP

```bash
cd software/user
./litepcie_util info
[> FPGA/SoC Information:
------------------------
FPGA Identifier:  LiteX SoC on Fairwaves_pro XTRX  2024-06-10 16:34:09.
FPGA DNA:         0x010a45867a084854
FPGA Temperature: 80.1 °C
FPGA VCC-INT:     0.93 V
FPGA VCC-AUX:     1.77 V
FPGA VCC-BRAM:    0.93 V
```

All the information retrieved by this command are directly retrieved from the board from CSR registers
through the MMAP interface.

### DMA

```bash
cd software/user
./litepcie_util dma_test -e
[> DMA loopback test:
---------------------
DMA_SPEED(Gbps)   TX_BUFFERS  RX_BUFFERS  DIFF  ERRORS
          2.93       8993        8865   128       0
          2.91      17921       17793   128       0
          2.91      26849       26721   128       0
          2.91      35777       35649   128       0
          2.91      44705       44577   128       0
```

Exercise the DMA interfaces of the LitePCIe cores that are also the one going to/from `LimeTop` module.

`litepcie_test -play/-record` utilities can also be used exercise the TX/RX DMA interfaces individually
and play/record directly from/to files.

## Gateware loading/flashing to SRAM and SPI Flash.

### Loading Gateware through JTAG (volatile and non-volatile memory)

[openFPGALoader](https://github.com/trabucayre/openFPGALoader) is used to load and/or flash
bitstreams and CPU firmware.

By default, a **digilent_hs2** *USB-JTAG* cable will be used. To change this behavior and select an
alternate cable, append the command line with `--cable xxx`, where `xxx` is the cable's name
(see `openFPGALoader --list-cables` for a complete list of supported cables).

### Loading Gateware to SRAM (Volatile Memory)

In this mode, the gateware will be lost after a power cycle.

```bash python3 limesdr_xtrx.py --load [--cable XXX] ```

### Flashing Gateware to SPI Flash (Non-Volatile Memory)

In this mode, the gateware will be automatically loaded after flashing and power cycles.

```bash python3 limesdr_xtrx.py --flash [--cable XXX] ```

### PCIe Rescan
After bitstream loaded/flashed, computer must be rebooted or a PCIe Bus rescan must be performed:

```bash
# Get PCIe location
lspci | grep -i RF_controller
# Remove device (replace X with actual value, see previous command)
echo 1 | sudo tee /sys/bus/pci/devices/0000\:0X\:00.0/remove
# Rescan PCIe Bus after flashing/loading new bitstream
echo 1 | sudo tee /sys/bus/pci/rescan
```

### Gateware loading/flashing to SRAM and SPI Flash.

## Firmware Loading from PCIe

First, load the *litepcie* driver:

```bash
cd software/kernel
make clean all
sudo ./init.sh
```

Then load the firmware over PCIe:

```bash
litex_term /dev/ttyLXU0 --kernel firmware/firmware.bin
```

## Firmware Flashing/Loading to/from SPI Flash

By default, the CPU's firmware is included in the gateware as ROM in a BlockRAM, but it is also
possible to have an external firmware written to the SPI flash. This allows firmware updates
without rebuilding the full gateware. To enable this option, use the following command:

```bash
python limesdr_xtrx.py --build --flash-boot --flash [--firmware-flash-offset 0xXXXXX]
```

The default offset for the CPU's firmware is 0x220000. Use `--firmware-flash-offset` to specify a
different offset.

## Firmware Debug through GDB over JTAG

To build and load a gateware with a debug interface:

```bash
./limesdr_xtrx.py --with bscan --build --load --flash

# Load firmware through serial:
litex_term /dev/ttyLXU0 --kernel firmware/firmware.bin

# Run OpenOCD with the specified configurations:
openocd -f ./digilent_hs2.cfg -c "set TAP_NAME xc7.tap" -f ./riscv_jtag_tunneled.tcl

# Connect GDB for debugging:
gdb-multiarch -q firmware/firmware.elf -ex "target extended-remote localhost:3333"
```

Note that instead of using GDB directly, Eclipse IDE can be configured to debug code in a more
user-friendly way. Follow this guide to configure Eclipse IDE:
[Using Eclipse to run and debug the software](https://github.com/SpinalHDL/VexRiscv?tab=readme-ov-file#using-eclipse-to-run-and-debug-the-software)


## Firmware Interrupt Handling Example

This example demonstrates how to handle firmware interrupts using the VexRiscv-SMP SoftCore CPU in
the LimeSDR-XTRX LiteX gateware infrastructure.

A simple IRQ generation code has been set up in the `LimeTop` gateware module and does the
following:
- Two interrupt sources, `irq0` and `irq1`, are defined and integrated into the `EventManager`.
- `irq0` is configured to trigger every 1 second.
- `irq1` is configured to trigger every 2 seconds.

The firmware implementation handles the interrupts triggered by the gateware. The example firmware
code includes:
- An Interrupt Service Routine (ISR) that checks and clears the interrupt status.
- An initialization function that sets up the interrupt by attaching the ISR and enabling the
  interrupts in the mask register.

After loading the firmware, the interrupts will be triggered as per the configured timers (1 second
for `irq0` and 2 seconds for `irq1`). The messages `IRQ0!` and `IRQ1!` will be printed to the
console, indicating that the interrupts have been handled.

## CSR Registers Integration Example

To demonstrate the CSR Registers integration with LiteX, a scratch register has been added to `LimeTop`
module.

First step is to start a server configured for JTAG access:
```bash
litex_server --jtag --jtag-config openocd_xc7_ft232.cfg
```

`--jtag-config` must be adapted according to the JTAG interface

Now it's possible to read/write the scratch CSR register:

```bash
litex_cli --write lime_top_scratch 0x12345678
litex_cli --read  lime_top_scratch
litex_cli --write lime_top_scratch 0x5aa55aa5
litex_cli --read  lime_top_scratch
```

## VHDL Integration Example

To demonstrate the VHDL integration with LiteX, the GPIO VHDL module is reused from LimeIP_HDL and
integrated in the design through a LiteX wrapper. This wrapper is then integrated in the target
design and connected to the GPIO of the board, here a LED. A test is provided to demonstrate
integration and control from JTAGBone, but control could  also be done over PCIeBone or directly
from C code running on the firmware or the Host.

### Target integration

The code is integrated in the `LimeTop` module in `VHDL GPIO Example` section.

### CPU Firmware Test

The GPIO integration can be tested with `gpioled` command of the firmware. The command will blink
the leds of the board.

### JTAGBone Test

First step is to start a server configured for JTAG access:
```bash
litex_server --jtag --jtag-config openocd_xc7_ft232.cfg
```

`--jtag-config` must be adapted according to the JTAG interface

Now it's possible to read/write `gpioTop` registers:

Value read:
```bash
litex_cli --read lime_top_gpio_gpio_val
```

HOST access (direction/level)
```bash
# control overriden for all pins
litex_cli --write lime_top_gpio_gpio_override 0x07
# set all pins as output
litex_cli --write lime_top_gpio_gpio_override_dir 0x00
# pins 0 & 2 low, pin 1 high
litex_cli --write lime_top_gpio_gpio_override_val 0x05
```

## LiteScope Logic Analyzer Example

First step is to start a server configured for JTAG access:
```bash
litex_server --jtag --jtag-config openocd_xc7_ft232.cfg
```
or PCIe access:
```bash
litex_server --pcie --pcie-bar=04:00.0 (Find pcie-bar with lspci)
```

Trigger LiteScope on a MMAP access to LiteTop's SRAM memory:
```bash
litescope_cli --csv=lime_top_analyzer.csv -r main_lime_top_mmap_ar_valid
```

Run `mmap` command from firmware to write/read to MMAP SRAM in `LimeTop` module.

```bash
[running]...
[uploading]...
[====================>] 100%
[writing to dump.vcd]...
```

The generated dump.vcd can then be opened with a VCD viewer like GTKWave and AXI-Lite accesses from
mmap firmware test observed.

