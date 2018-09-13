# Comet
RISC-V ISA based 32-bit processor written in C++ for High Level Synthesis (HLS).

## Pre-requisites
1. [RISC-V toolchain](https://github.com/riscv/riscv-tools)
2. [Catapult HLS](https://www.mentor.com/hls-lp/catapult-high-level-synthesis/)

## Components of the project
Vivado HLS is used for FPGA IP synthesis and Catapult HLS is used for ASIC synthesis.

* Run `make` to generate the simulator `comet.sim`.
* To build it as an FPGA IP, run `script.tcl` in Vivado HLS. (not maintained)
* To synthesize it to rtl for ASIC, run [genCore.py](catapult/genCore.py) in `catapult` folder. See `genCore.py -h` for argument list.

## Documentation 
The primary design goal behind using High Level Synthesis (HLS) is that for simulating the functionality on a host machine,
FPGA synthesis of ASIC synthesis the same C / C++ code can be used, with minimal changes. However since the target platforms,
and tools involved are different in nature, these differences have to be accounted for without having to re-write the major
functionality.

## Architecture
The 5 stage pipelined architecture currently has 64MB of instruction memory and 64MB of data memory. 32 bit
Arithmetic and logical integer operations are supported. We support M extension but the division and the modulo operation are external to the core because they take more than one cycle to execute at the synthesis. 

## Simulator
In order to execute an elf file compiled by `riscv32-unknown-elf-gcc`, it is necessary to have a process which reads the 
different sections of the binary file, extracts the program instructions and data, and stores it in the instruction and data 
memory. The functions to handle the the elf file are in [elfFile.cpp](src/elfFile.cpp), [elf.h](include/elf.h) and 
[elfFile.h](include/elfFile.h). The process which calls these functions, fills the memory and starts the execution of the 
processor, is defined in [reformeddm_sim.cpp](src/reformeddm_sim.cpp). 
To verify the simulator behavior, you can run a benchmark through it and check that the output is coherent with the [check.py](check.py) python script.

## Benchmarks
```
git clone --recursive https://github.com/riscv/riscv-tools 
export RISCV=/where/you/want/the/compilation/to/install
cd riscv-tools
./build-rv32ima.sh
export PATH=$PATH:$RISCV/bin
```
Now that you have compiled the compiler, you can compile the benchmarks:
```
cd benchmarks
make
```
After this, you can `./comet.sim -f benchmarks/build/matmul_int_4.riscv32` and this will run the benchmarks through the simulator.
