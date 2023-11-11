# Amaranth-SDRAM-controller
Templated Amaranth SDRAM controller

- Currently tested for the Sipeed Tang Primer with Anlogic EG4S20BG256 FPGA chip and Sipeed Tang Nano 20K with GoWin GW2AR-LV18QN88C8/I7 FPGA chip
- Tested interface includes 3 and 4 bytes wide data-interface
- Sample Verilog code for a 120MHz SDRAM clock interface and demo top template for interconecting the controller to the FPGA internal SDRAM

# Known Bugs
- It will not work for 1 and 2 bytes wide data-interface from a 32-bit wide data SDRAM DQ line
