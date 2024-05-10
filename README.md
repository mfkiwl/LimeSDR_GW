# LimeSDR-XTRX Gateware 

**NOTE:** vexriscv cpu is used for this example

Cloning repo:
```
git clone https://github.com/myriadrf/LimeSDR-XTRX_LiteX_GW.git
git submodule init 
git submodule update
```

## Build gateware, load firmware trough GPIO UART and launch CPU debug trough JTAG

Required hardware:
1. LimeSDR-XTRX v1.2 board
2. Mini PCIe to PCIe adapter
3. FT2232H mini module connected to PCIe adapter JTAG:
    ```
    JTAG       FT2232H
    ------------------
    TMS   ->   CN2_12 
    TDI   ->   CN2_10
    TDO   ->   CN2_9
    TCK   ->   CN2_7
    GND   ->   CN2_2
    VIO   ->   CN2_11
    ``` 
4. USB Serial converter connected to PCIe adapter GPIO
   ```
   GPIO              Serial 
   ------------------------
   GPIO3N(RX)   <-   TX
   GPIO3P(TX)  ->    RX
   ```

Steps to load build/load gateware, load firmware:
```
# Build/Load gateware bitstream:
./LimeSDR_XTRX.py --integrated-main-ram-size 0x8000 --uart-name=gpio_serial --cpu-type=vexriscv  --csr-csv=csr.csv --with-pcie --build --driver --load

# Build firmware:
cd firmware && make clean all && cd ../

# Load firmware trough serial
litex_term --csr-csv csr.csv /dev/ttyUSB0 --kernel firmware/demo.bin
```




