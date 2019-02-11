# CAN-bus Bus Functional Model (BFM)

This repository contains a basic BFM for CAN bus for VHDL, VHDL UVVM, and SystemVerilog. It also includes testbenches that test the functionality of the BFMs, and which also serve as an example of how the BFMs can be used.

# UVVM BFM/Testbench

The UVVM BFM and testbench was developed and tested for UVVM 1.4.0. Which by now is a rather old version of UVVM, and should be updated for the latest version.

Only the basic BFM for UVVM is provided, a VVC (Virtual Verification Component) has not been developed yet.


# Repo structure:
* doc/				- Documentation
* script/ 			- TCL scripts for simulating the code (modelsim)
* src/				- Sources
* src/bfm			- BFM sources
* src/bfm/can_bfm_base 		- CAN bus BFM base procedures (raw VHDL, no UVVM). Used by UVVM BFM
* src/bfm/can_bfm_uvvm		- CAN bus UVVM BFM (uses can_bfm_base)
* src/bfm/can_bfm_sv		- CAN bus SystemVerilog BFM
* src/tb			- Testbench sources


# Scripts and testbenches

Several testbenches are included in the src/tb directory:
* can_bfm_tb.vhd			- Testbench for base VHDL CAN BFM
* can_uvvm_bfm_tb.vhd			- Testbench for UVVM CAN BFM
* can_bfm_tb.sv				- Testbench for SystemVerilog CAN BFM

Corresponding scripts for running the testbenches:
* /script/run_can_bfm_base_tb.do	- Run base VHDL CAN BFM testbench
* /script/run_can_uvvm_bfm_tb.do	- Run UVVM CAN BFM testbench
* /script/run_can_bfm_sv_tb.do		- Run SystemVerilog CAN BFM testbench

To simulate the testbenches:

```bash
cd script
vsim -do run_can_bfm_base_tb.do
vsim -do run_can_bfm_sv_tb.do
vsim -do run_can_bfm_uvvm_tb.do
```

## Note: UVVM library path
The scripts for testbenches that use UVVM expects to find the library in src/bitvis_uvvm/, relative to the top-level directory of this repository. You will have to edit the paths in the scripts if you have it located somewhere else.
UVVM 1.4.0 was used for the development of this BFM. 

## Note: Bitvis wishbone BFM
Several of these testbench uses a bitvis wishbone BFM that has not been publicly released. It has not been included in this repo because it is not my work. Contact bitvis (or me) if you need it to run the testbench.

## Note: OpenCores CAN controller
The CAN controller from OpenCores is not included in this repo, and must be downloaded from here:

https://opencores.org/projects/can

It is used in several of the testbenches. Download and unzip contents of /can/trunk/rtl/verilog to the empty src/can_ctrl directory in this repo.

And then uncomment the line "// `define   CAN_WISHBONE_IF" in can_defines.v, so the controller uses the wishbone interface.


# Author
Simon Voigt Nesbo <svn@hvl.no>
