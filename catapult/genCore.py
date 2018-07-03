#!/usr/bin/python3

import argparse, os, sys
from math import log2
import subprocess

genMem = """flow package require MemGen

flow run /MemGen/MemoryGenerator_BuildLib {{
VENDOR           STMicroelectronics
RTLTOOL          DesignCompiler
TECHNOLOGY       {{28nm FDSOI}}
LIBRARY          ST_singleport_{memsize}x{bitwidth}
MODULE           ST_SPHD_BB_8192x32m16_aTdol_wrapper
OUTPUT_DIR       ../memories/
FILES {{
  {{ FILENAME /opt/DesignKit/cmos28fdsoi_29/memcut_28nm/C28SOI_SPHD_BB_170612/4.3-00.00/behaviour/verilog/SPHD_BB_170612.v         FILETYPE Verilog MODELTYPE generic PARSE 1 PATHTYPE abs STATICFILE 1 VHDL_LIB_MAPS work }}
  {{ FILENAME /opt/DesignKit/cmos28fdsoi_29/memcut_28nm/C28SOI_SPHD_BB_170612/4.3-00.00/behaviour/verilog/SPHD_BB_170612_wrapper.v FILETYPE Verilog MODELTYPE generic PARSE 1 PATHTYPE abs STATICFILE 1 VHDL_LIB_MAPS work }}
}}
VHDLARRAYPATH    {{}}
WRITEDELAY       0.194
INITDELAY        1
READDELAY        0.940
VERILOGARRAYPATH {{}}
INPUTDELAY       0.146
WIDTH            {bitwidth}
AREA             {area}
WRITELATENCY     1
RDWRRESOLUTION   UNKNOWN
READLATENCY      1
DEPTH            {memsize}
PARAMETERS {{
  {{ PARAMETER Words                TYPE hdl IGNORE 1 MIN {{}} MAX {{}} DEFAULT 0 }}
  {{ PARAMETER Bits                 TYPE hdl IGNORE 1 MIN {{}} MAX {{}} DEFAULT 0 }}
  {{ PARAMETER mux                  TYPE hdl IGNORE 1 MIN {{}} MAX {{}} DEFAULT 0 }}
  {{ PARAMETER Bits_Func            TYPE hdl IGNORE 0 MIN {{}} MAX {{}} DEFAULT 0 }}
  {{ PARAMETER mask_bits            TYPE hdl IGNORE 1 MIN {{}} MAX {{}} DEFAULT 0 }}
  {{ PARAMETER repair_address_width TYPE hdl IGNORE 1 MIN {{}} MAX {{}} DEFAULT 0 }}
  {{ PARAMETER Addr                 TYPE hdl IGNORE 0 MIN {{}} MAX {{}} DEFAULT 0 }}
  {{ PARAMETER read_margin_size     TYPE hdl IGNORE 1 MIN {{}} MAX {{}} DEFAULT 0 }}
  {{ PARAMETER write_margin_size    TYPE hdl IGNORE 1 MIN {{}} MAX {{}} DEFAULT 0 }}
}}
PORTS {{
  {{ NAME port_0 MODE ReadWrite }}
}}
PINMAPS {{
  {{ PHYPIN A     LOGPIN ADDRESS      DIRECTION in  WIDTH Addr      PHASE {{}} DEFAULT {{}} PORTS port_0 }}
  {{ PHYPIN CK    LOGPIN CLOCK        DIRECTION in  WIDTH 1.0       PHASE 1  DEFAULT {{}} PORTS port_0 }}
  {{ PHYPIN CSN   LOGPIN CHIP_SELECT  DIRECTION in  WIDTH 1.0       PHASE 1  DEFAULT {{}} PORTS {{}}     }}
  {{ PHYPIN D     LOGPIN DATA_IN      DIRECTION in  WIDTH Bits_Func PHASE {{}} DEFAULT {{}} PORTS port_0 }}
  {{ PHYPIN INITN LOGPIN UNCONNECTED  DIRECTION in  WIDTH 1.0       PHASE {{}} DEFAULT 1  PORTS {{}}     }}
  {{ PHYPIN Q     LOGPIN DATA_OUT     DIRECTION out WIDTH Bits_Func PHASE {{}} DEFAULT {{}} PORTS port_0 }}
  {{ PHYPIN WEN   LOGPIN WRITE_ENABLE DIRECTION in  WIDTH 1.0       PHASE 1  DEFAULT {{}} PORTS port_0 }}
}}

}}"""

header = """logfile close
logfile open {cachesize}x32cachecore.log
project set -incr_directory {cachesize}x32cachecore
solution new -state initial
solution options set /Input/CompilerFlags {{-D __CATAPULT__=1}}
solution options set /Input/SearchPath ../include
solution options set ComponentLibs/SearchPath ../memories -append
flow package require /SCVerify
solution file add ../src/core.cpp -type C++
solution file add ../src/reformeddm_sim.cpp -type C++
solution file add ../src/cache.cpp -type C++
solution file add ../src/syscalls.cpp -type C++
solution file add ../src/elfFile.cpp -type C++
solution file add ../src/portability.cpp -type C++
go new
directive set -DESIGN_GOAL area
directive set -OLD_SCHED false
directive set -SPECULATE true
directive set -MERGEABLE true
directive set -REGISTER_THRESHOLD 4096 
directive set -MEM_MAP_THRESHOLD 64
directive set -FSM_ENCODING none
directive set -REG_MAX_FANOUT 0
directive set -NO_X_ASSIGNMENTS true
directive set -SAFE_FSM false
directive set -ASSIGN_OVERHEAD 0
directive set -UNROLL no
directive set -IO_MODE super
directive set -REGISTER_IDLE_SIGNAL false
directive set -IDLE_SIGNAL {{}}
directive set -STALL_FLAG false
directive set -TRANSACTION_DONE_SIGNAL true
directive set -DONE_FLAG {{}}
directive set -READY_FLAG {{}}
directive set -START_FLAG {{}}
directive set -BLOCK_SYNC none
directive set -TRANSACTION_SYNC ready
directive set -DATA_SYNC none
directive set -RESET_CLEARS_ALL_REGS true
directive set -CLOCK_OVERHEAD 0.000000
directive set -OPT_CONST_MULTS use_library
directive set -CHARACTERIZE_ROM false
directive set -PROTOTYPE_ROM true
directive set -ROM_THRESHOLD 64
directive set -CLUSTER_ADDTREE_IN_COUNT_THRESHOLD 0
directive set -CLUSTER_OPT_CONSTANT_INPUTS true
directive set -CLUSTER_RTL_SYN false
directive set -CLUSTER_FAST_MODE false
directive set -CLUSTER_TYPE combinational
directive set -COMPGRADE fast
directive set -DESIGN_HIERARCHY doStep
"""

cacheparameters = """solution file set ../src/core.cpp -args {{-DSize={cachesize} -DAssociativity={associativity} -DBlocksize={blocksize}}}
//solution file set ../src/cache.cpp -args {{-DSize={cachesize} -DAssociativity={associativity} -DBlocksize={blocksize}}}
"""

libraries = """go analyze
solution library remove *
solution library add C28SOI_SC_12_CORE_LL_ccs -file {{$MGC_HOME/pkgs/siflibs/designcompiler/CORE65LPHVT_ccs.lib}} -- -rtlsyntool DesignCompiler -vendor STMicroelectronics -technology {{28nm FDSOI}}
solution library add ST_singleport_16777216x32
solution library add ST_singleport_{datasize}x{datawidth}
solution library add ST_singleport_{interleaveddatasize}x{datawidth}
solution library add ST_singleport_{sets}x{ctrlwidth}
go libraries
"""

genCore = """directive set -CLOCKS {{clk {{-CLOCK_PERIOD {period:.2f} -CLOCK_EDGE rising -CLOCK_HIGH_TIME {halfperiod:.2f} -CLOCK_OFFSET 0.000000 -CLOCK_UNCERTAINTY 0.0 -RESET_KIND sync -RESET_SYNC_NAME rst -RESET_SYNC_ACTIVE high -RESET_ASYNC_NAME arst_n -RESET_ASYNC_ACTIVE low -ENABLE_NAME {{}} -ENABLE_ACTIVE high}}}}
go assembly
directive set /doStep/core/REG:rsc -MAP_TO_MODULE {{[Register]}}
directive set /doStep/core/main -PIPELINE_INIT_INTERVAL 1
directive set /doStep/core -CLOCK_OVERHEAD 0.0
directive set /doStep/core/idata:rsc -MAP_TO_MODULE ST_singleport_{datasize}x{datawidth}.ST_SPHD_BB_8192x32m16_aTdol_wrapper
directive set /doStep/core/idata:rsc -INTERLEAVE {associativity}
directive set /doStep/core/ictrl.tag:rsc -PACKING_MODE sidebyside
directive set /doStep/core/ictrl.tag:rsc -MAP_TO_MODULE ST_singleport_{sets}x{ctrlwidth}.ST_SPHD_BB_8192x32m16_aTdol_wrapper
directive set /doStep/core/ictrl.tag -WORD_WIDTH {tagbits} 
directive set /doStep/core/ictrl.valid -RESOURCE ictrl.tag:rsc
directive set /doStep/core/ictrl.valid -WORD_WIDTH {associativity}
directive set /doStep/core/ictrl.policy -RESOURCE ictrl.tag:rsc
directive set /doStep/core/ddata:rsc -INTERLEAVE {associativity}
directive set /doStep/core/ddata:rsc -MAP_TO_MODULE ST_singleport_{datasize}x{datawidth}.ST_SPHD_BB_8192x32m16_aTdol_wrapper
directive set /doStep/core/dctrl.tag:rsc -PACKING_MODE sidebyside
directive set /doStep/core/dctrl.tag:rsc -MAP_TO_MODULE ST_singleport_{sets}x{ctrlwidth}.ST_SPHD_BB_8192x32m16_aTdol_wrapper
directive set /doStep/core/dctrl.tag -WORD_WIDTH {tagbits} 
directive set /doStep/core/dctrl.dirty -RESOURCE dctrl.tag:rsc
directive set /doStep/core/dctrl.dirty -WORD_WIDTH {associativity}
directive set /doStep/core/dctrl.valid -RESOURCE dctrl.tag:rsc
directive set /doStep/core/dctrl.valid -WORD_WIDTH {associativity}
directive set /doStep/core/dctrl.policy -RESOURCE dctrl.tag:rsc

go architect
cycle add /doStep/core/core:rlp/main/icache:case-0:if#1:read_mem(idata:rsc(0)(0).@) -from /doStep/core/core:rlp/main/loadiset:read_mem(ictrl.tag:rsc.@) -equal 0
cycle add /doStep/core/core:rlp/main/dcache:case-0:if:if:if:read_mem(ddata:rsc(0)(0).@) -from /doStep/core/core:rlp/main/loaddset:read_mem(dctrl.tag:rsc.@) -equal 0
cycle add /doStep/core/core:rlp/main/loaddset:read_mem(dctrl.tag:rsc.@) -from /doStep/core/core:rlp/main/loadiset:read_mem(ictrl.tag:rsc.@) -equal 0
go schedule
go extract
project save {cachesize}x32cachecore.ccs
"""

exploreCore = """dofile func.tcl
set maxf 0
set co 0
for {set overhead 0.0} {$overhead < 20} {set overhead [expr $overhead+1]} {
	if { [catch {
		for {set f 700} {$f < 1000} {set f [expr $f+5]} {
			if { [catch {
				set period [string range [expr 1000./$f] 0 4]
				set solutionname "${f}MHz_${overhead}CO"
				go new
				solution new
				solution rename ${solutionname}
				directive set -CLOCK_OVERHEAD $overhead
				go libraries
				directive set -CLOCKS "clk {-CLOCK_PERIOD ${period} -CLOCK_EDGE rising -CLOCK_HIGH_TIME [expr ${period}/2] -CLOCK_OFFSET 0.000000 -CLOCK_UNCERTAINTY 0.0 -RESET_KIND sync -RESET_SYNC_NAME rst -RESET_SYNC_ACTIVE high -RESET_ASYNC_NAME arst_n -RESET_ASYNC_ACTIVE low -ENABLE_NAME {} -ENABLE_ACTIVE high}"
				go architect
				go schedule
				go extract
				if {$maxf < $f } {
					set maxf [max $f $maxf]
					set co $overhead
					logfile message "New maximum frequency achieved : $maxf\n" info
				}
			}
		]} {
			logfile message "Failed to synthesize for $f MHz with $overhead\% clock overhead\n" info
			puts "Failed to synthesize for $f MHz with $overhead\% clock overhead\n"
			break
		} }
	}]} {
		logfile message "Unexpected error\n" error
		puts "Unexpected error\n"
	}
}
logfile message "Maximum frequency : $maxf for $co\%\n" comment
puts "Maximum frequency : $maxf for $co\%\n"
project save
"""

def is_powerof2(num):
	return (num & (num - 1)) == 0 and num > 0

def nextpowerof2(num):
	assert(num > 0)
	return 2**(num-1).bit_length()
	
def doMem(memsize, bitwidth, name=None):
	area = int(1.4624*memsize*(bitwidth/8)+4915)
	memorypath = "../memories/ST_singleport_{}x{}.lib".format(memsize, bitwidth)
	if not os.path.isfile(memorypath):
		mem = genMem.format(**locals())
		if name is not None:
			print("Generating {} memory {}x{} with area {}".format(name, memsize, bitwidth, area))
		else:
			print("Generating memory {}x{} with area {}".format(memsize, bitwidth, area))
		with open("../memories/generateCacheMemories.tcl", "w") as f:
			f.write(mem)
		subprocess.check_call(["./catapult.sh", "-product lb -shell -f ../memories/generateCacheMemories.tcl"])
	
def doCore(cachesize, associativity, blocksize, explore=False):
	sets = int(cachesize/(blocksize)/associativity)
	tagbits = int(32 - log2(blocksize/4) - log2(sets) - 2)
	bits = int(associativity*(tagbits+1+1)+associativity*(associativity-1)/2)

	print("Generating cached core with cachesize ", cachesize, " bytes and associativity ", associativity, " and block size of ", blocksize, " bytes.")
	print("Tagbits {}\nBitwidth of control {}({})\nSets {}".format(tagbits, bits, nextpowerof2(bits), sets))
	bitwidth = nextpowerof2(bits)
	sets = nextpowerof2(sets)
	area = int((1.4624*sets*bitwidth)/8+4915)
	tagbits = 4*tagbits

	doMem(2**24, 32, "main")
	doMem(sets, bitwidth, "control")
	doMem(int(8*cachesize/32), 32, "data")
	bitwidth = 32

	#core
	datasize = int(8*cachesize/bitwidth)
	interleaveddatasize = int(8*cachesize/bitwidth/associativity)
	blocksize = int(blocksize/4)
	datawidth = 32
	ctrlwidth = nextpowerof2(bits)
	period = 1.5
	halfperiod = period/2
	core = (header + cacheparameters + libraries + genCore).format(**locals())
	
	if explore:
		core += exploreCore
		
	with open("genCore_{}x32.tcl".format(cachesize), "w") as f:
		f.write(core)

	if args.shell:
		subprocess.check_call(["./catapult.sh", "-shell -f genCore_{}x32.tcl".format(cachesize)])
	else:
		subprocess.check_call(["./catapult.sh", "-f genCore_{}x32.tcl".format(cachesize)])

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("-s", "--shell", help="Launch catapult in shell", action="store_true")
	parser.add_argument("-n", "--no-cache", help="Synthesize without cache", action="store_true")
	parser.add_argument("-c", "--cache-size", help="Cache size in bytes", type=int)
	parser.add_argument("-a", "--associativity", help="Cache associativity", type=int)
	parser.add_argument("-b", "--blocksize", help="Cache blocksize in bytes", type=int)
	parser.add_argument("-p", "--policy", help="Replacement policy")
	parser.add_argument("-e", "--explore", help="Do some exploration in the solution space", action="store_true")

	args = parser.parse_args()
	try:
		cachesize = args.cache_size
		assert is_powerof2(cachesize), "cachesize is not a power of 2"
	except TypeError:
		cachesize = 1024
		print("Setting default cachesize to {}".format(cachesize))

	try:
		associativity = args.associativity
		assert is_powerof2(associativity), "associativity is not a power of 2"
	except TypeError:
		associativity = 4
		print("Setting default associativity to {}".format(associativity))

	try:
		blocksize = int(args.blocksize/1)
		assert is_powerof2(blocksize), "blocksize is not a power of 2"
	except TypeError:
		blocksize = 32
		print("Setting default blocksize to {}".format(blocksize))

	try:
		policy = args.policy + ""
	except TypeError:
		if associativity == 1:
			policy = "NONE"
		else:
			policy = "LRU"
		print("Setting default policy to {}".format(policy))

	policy = policy.upper()
	assert (associativity == 1 and policy == "NONE") or (associativity != 1 and policy != "NONE")
	assert policy in ["LRU", "RANDOM", "NONE", "FIFO"]

	# ~ defines = "-D{}={} "
	# ~ defines = defines.format("Size", cachesize) + defines.format("Associativity", associativity) + defines.format("Blocksize", int(blocksize/4)) + defines.format("Policy", policy)
	# ~ subprocess.check_call(["make", "DEFINES="+defines])
	# ~ with open("output.log", "w") as output:
		# ~ subprocess.check_call(["./testbench.sim"], stdout=output)

	doCore(cachesize, associativity, blocksize, args.explore)
	
