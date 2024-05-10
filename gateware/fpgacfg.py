#!/usr/bin/env python3

from migen import *
from litex.soc.interconnect.csr import *

class fpgacfg_csr(Module, AutoCSR):
    def __init__(self):
        self.board_id       = CSRStatus(16, reset=27)
        self.major_rev      = CSRStatus(16, reset=0)
        self.compile_rev    = CSRStatus(16, reset=1)
        self.reserved_03    = CSRStorage(16, reset=0)
        self.reserved_04    = CSRStorage(16, reset=0)
        self.reserved_05    = CSRStorage(16, reset=0)
        self.reserved_06    = CSRStorage(16, reset=0)
        self.channel_cntrl  = CSRStorage(fields=[
            CSRField("ch_en", size=2, offset=0, values=[
                ("``2b01", "Channel A"),
                ("``2b10", "Channel B"),
                ("``2b11", "Channels A and B")
            ], reset=0)
        ])

