#!/usr/bin/env python3

#
# This file is part of LimeSDR-XTRX_LiteX_GW.
#
# Copyright (c) 2021 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Build/Use ----------------------------------------------------------------------------------------
# Build/Load bitstream:
# ./LimeSDR_XTRX.py --integrated-main-ram-size 0x8000 --build --load --uart-name=jtag_uart --cpu-type=vexriscv_smp --with-rvc  --with-privileged-debug --hardware-breakpoints 4
#
# Build firmware:
# cd firmware && make clean all && cd ../
#
# Load CPU firmware:
# litex_term jtag --jtag-config=openocd_xc7_ft2232.cfg --kernel firmware/demo.bin

import os

from migen import *

from litex.gen import *

import LimeSDR_XTRX_platform as limesdr_xtrx

from migen.genlib.cdc import MultiReg

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *
from litex.soc.interconnect import stream
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.soc.cores.led import LedChaser
from litex.soc.cores.clock import *
from litex.soc.cores.bitbang import I2CMaster

#from litepcie.phy.s7pciephy import S7PCIEPHY
# Temporary using modified version of S7PCIEPHY
from gateware.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software

from litex.soc.cores.jtag import XilinxJTAG

from gateware.GpioTop import GpioTop

# CRG ----------------------------------------------------------------------------------------------

class CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_pcie=False):
        self.rst    = Signal()
        self.cd_sys = ClockDomain()
        self.cd_fpga = ClockDomain()

        self.vctcxo_pads = platform.request("vctcxo")
        self.comb += self.vctcxo_pads.EN_TCXO.eq(1)
        self.comb += self.vctcxo_pads.EXT_CLK.eq(0)

        # # #

        fpga_clk = platform.request("FPGA_CLK")
        if with_pcie:
            assert sys_clk_freq == int(125e6)
            self.comb += [
                self.cd_sys.clk.eq(ClockSignal("pcie")),
                self.cd_sys.rst.eq(ResetSignal("pcie")),
            ]

        self.pll = pll = S7PLL(speedgrade=-2)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(fpga_clk, 26e6)
        pll.create_clkout(self.cd_fpga, sys_clk_freq)

# BaseSoC -----------------------------------------------------------------------------------------


# Create a led blinker module
class Blink(Module):
    def __init__(self, led):
        counter = Signal(26)
        # combinatorial assignment
        self.comb += led.eq(counter[25])

        # synchronous assignment
        self.sync += counter.eq(counter + 1)


# LMS Control CSR----------------------------------------------------------------------------------------
class CNTRL_CSR(Module, AutoCSR):
    def __init__(self, ndmas):
        self.cntrl          = CSRStorage(512, 0)
        self.enable         = CSRStorage()
        self.test           = CSRStorage(32)
        self.ndma           = CSRStatus(4, reset=ndmas)
        self.enable_both    = CSRStorage()

        # Create event manager for interrupt
        self.submodules.ev    = EventManager()
        self.ev.cntrl_isr = EventSourceProcess(edge="rising")
        self.ev.finalize()

        # Trigger interrupt when cntrl register is written
        self.comb += self.ev.cntrl_isr.trigger.eq(self.cntrl.re)


class BaseSoC(SoCCore):
    SoCCore.interrupt_map = {
        "CNTRL" : 3,
    }
    SoCCore.mem_map["csr"] = 0x00000000
    SoCCore.csr_map = {
        "ctrl":           0,
        "crg" :           1,
        "pcie_phy":       2,
        "pcie_msi":       3,
        "pcie_msi_table": 4,
        "CNTRL":         26,

    }
    SoCCore.interrupt_map.update(SoCCore.interrupt_map)
    def __init__(self, sys_clk_freq=125e6, with_pcie=False, with_led_chaser=True, **kwargs):
        platform = limesdr_xtrx.Platform()

        # CRG --------------------------------------------------------------------------------------
        self.crg = CRG(platform, sys_clk_freq, with_pcie)

        # SoCCore ----------------------------------------------------------------------------------
        #if kwargs["uart_name"] != "jtag_uart" and kwargs["uart_name"] != "gpio_uart":
        #    kwargs["uart_name"]     = "crossover"
        #    kwargs["with_jtagbone"] = True
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on LimeSDR-XTRX", **kwargs)

        # PCIe -------------------------------------------------------------------------------------
        if with_pcie:
            self.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x2"),
                data_width = 64,
                bar0_size  = 0x20000)
            self.add_pcie(phy=self.pcie_phy, ndmas=1)

            # ICAP (For FPGA reload over PCIe).
            from litex.soc.cores.icap import ICAP
            self.icap = ICAP()
            self.icap.add_reload()
            self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

            # Flash (For SPIFlash update over PCIe).
            from litex.soc.cores.gpio import GPIOOut
            from litex.soc.cores.spi_flash import S7SPIFlash
            self.flash_cs_n = GPIOOut(platform.request("FPGA_CFG_CS"))
            self.flash      = S7SPIFlash(platform.request("flash"), sys_clk_freq, 25e6)

            self.submodules.CNTRL = CNTRL = CNTRL_CSR(1)


        # Leds -------------------------------------------------------------------------------------
        if with_led_chaser:
            self.leds = LedChaser(
                pads         = platform.request_all("FPGA_LED1"),
                sys_clk_freq = sys_clk_freq)

        # JTAG instance for GDB
        self.jtag = jtag = XilinxJTAG(XilinxJTAG.get_primitive("xc7a"), chain=4)

        #self.comb += [
        #    self.cpu.jtag_reset.eq(jtag.reset),
        #    self.cpu.jtag_capture.eq(jtag.capture),
        #    self.cpu.jtag_shift.eq(jtag.shift),
        #    self.cpu.jtag_update.eq(jtag.update),
        #    self.cpu.jtag_clk.eq(jtag.tck),
        #    self.cpu.jtag_tdi.eq(jtag.tdi),
        #    self.cpu.jtag_enable.eq(True),
        #    jtag.tdo.eq(self.cpu.jtag_tdo),
        #]

        # GPIO instance
        self.gpio = GpioTop(platform, platform.request("gpio"))
        # Set all gpio to inputs
        self.comb += self.gpio.GPIO_DIR.eq(0b1111)
        self.comb += self.gpio.GPIO_OUT_VAL.eq(0b1010)

        # Alive signal
        self.alive = Signal()
        self.blinker = ClockDomainsRenamer("fpga")(Blink(led=self.alive))

        # Alive LED - it also can be overridden from CPU
        self.gpio_led = GpioTop(platform, platform.request("FPGA_LED2"))
        # Set all gpio to inputs
        self.comb += self.gpio_led.GPIO_DIR.eq(0b0)
        self.comb += self.gpio_led.GPIO_OUT_VAL.eq(self.alive)

        # FPGA_I2C1 Bus:
        # - Temperature Sensor (TMP107  @ 0x4B).
        # - VCTCXO DAC         (AD5693  @ 0x4C).
        # - I2C EEPROM (NF)    (M24128  @ 0x50).
        # - PMIC (IC22)        (LP8758  @ 0x60).
        self.submodules.i2c0 = I2CMaster(pads=platform.request("i2c", 0))

        # FPGA_I2C2 Bus:
        # - PMIC (IC31)        (LP8758  @ 0x60).
        self.submodules.i2c1 = I2CMaster(pads=platform.request("i2c", 1))







# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=limesdr_xtrx.Platform, description="LiteX SoC on LimeSDR-XTRX.")
    parser.add_target_argument("--flash",           action="store_true",       help="Flash bitstream.")
    parser.add_target_argument("--sys-clk-freq",    default=125e6, type=float, help="System clock frequency.")
    parser.add_target_argument("--with-pcie",       action="store_true",       help="Enable PCIe support.")
    parser.add_target_argument("--driver",          action="store_true",       help="Generate PCIe driver.")
    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq = args.sys_clk_freq,
        with_pcie    = args.with_pcie,
        **parser.soc_argdict
    )
    builder  = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash"))

if __name__ == "__main__":
    main()
