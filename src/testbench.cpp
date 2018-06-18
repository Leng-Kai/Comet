#include "elfFile.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdio.h>
#include <map>
#include <iostream>
#include <string.h>
#include "core.h"
#include "portability.h"
#include "mc_scverify.h"

#ifdef __VIVADO__
#include "DataMemory.h"
#else
#include "DataMemoryCatapult.h"
#endif

using namespace std;

class Simulator
{

private:
    //counters
    map<ac_int<32, false>, ac_int<8, false> > ins_memorymap;
    map<ac_int<32, false>, ac_int<8, false> > data_memorymap;
    ac_int<32, false> nbcycle;
    ac_int<32, false> ins_addr;

    ac_int<32, false> pc;

public:
    ac_int<32, true>* ins_memory;
    ac_int<32, true>* data_memory;

    Simulator()
    {
        ins_memory = (ac_int<32, true> *)malloc(8192 * sizeof(ac_int<32, true>));
        data_memory = (ac_int<32, true> *)malloc(8192 * sizeof(ac_int<32, true>));
        for(int i =0; i<8192; i++)
        {
            ins_memory[i]=0;
            data_memory[i]=0;
        }
    }

    ~Simulator()
    {
        free(ins_memory);
        free(data_memory);
    }

    void fillMemory()
    {
        //Check whether data memory and instruction memory from program will actually fit in processor.
        cout << ins_memorymap.size()<<endl;

        if(ins_memorymap.size() / 4 > 8192)
        {
            printf("Error! Instruction memory size exceeded");
            exit(-1);
        }
        if(data_memorymap.size() / 4 > 8192)
        {
            printf("Error! Data memory size exceeded");
            exit(-1);
        }

        //fill instruction memory
        for(map<ac_int<32, false>, ac_int<8, false> >::iterator it = ins_memorymap.begin(); it!=ins_memorymap.end(); ++it)
        {
            ins_memory[(it->first/4) % 8192].set_slc(((it->first)%4)*8,it->second);
        }

        //fill data memory
        for(map<ac_int<32, false>, ac_int<8, false> >::iterator it = data_memorymap.begin(); it!=data_memorymap.end(); ++it)
        {
            //data_memory.set_byte((it->first/4)%8192,it->second,it->first%4);
            data_memory[(it->first/4) % 8192].set_slc((it->first%4)*8,it->second);
        }
    }

    void setInstructionMemory(ac_int<32, false> addr, ac_int<8, true> value)
    {
        ins_memorymap[addr] = value;
    }

    void setDataMemory(ac_int<32, false> addr, ac_int<8, true> value)
    {
        data_memorymap[addr] = value;
    }

    ac_int<32, true>* getInstructionMemory()
    {

        return ins_memory;
    }


    ac_int<32, true>* getDataMemory()
    {
        return data_memory;
    }

    void setPC(ac_int<32, false> pc)
    {
        this->pc = pc;
    }

    ac_int<32, false> getPC()
    {
        return pc;
    }

};


CCS_MAIN(int argv, char **argc)
{
    char* binaryFile = "../benchmarks/build/matmul4_4.out";
    ElfFile elfFile(binaryFile);
    Simulator sim;
    int counter = 0;
    for (unsigned int sectionCounter = 0; sectionCounter<elfFile.sectionTable->size(); sectionCounter++)
    {
        ElfSection *oneSection = elfFile.sectionTable->at(sectionCounter);
        if(oneSection->address != 0 && oneSection->getName().compare(".text"))
        {
            //If the address is not null we place its content into memory
            unsigned char* sectionContent = oneSection->getSectionCode();
            for (unsigned int byteNumber = 0; byteNumber<oneSection->size; byteNumber++)
            {
                counter++;
                sim.setDataMemory(oneSection->address + byteNumber, sectionContent[byteNumber]);
            }
        }

        if (!oneSection->getName().compare(".text"))
        {
            unsigned char* sectionContent = oneSection->getSectionCode();
            for (unsigned int byteNumber = 0; byteNumber<oneSection->size; byteNumber++)
            {
                sim.setInstructionMemory((oneSection->address + byteNumber) & 0x0FFFF, sectionContent[byteNumber]);
            }
        }
    }

    for (int oneSymbol = 0; oneSymbol < elfFile.symbols->size(); oneSymbol++)
    {
        ElfSymbol *symbol = elfFile.symbols->at(oneSymbol);
        const char* name = (const char*) &(elfFile.sectionTable->at(elfFile.indexOfSymbolNameSection)->getSectionCode()[symbol->name]);
        if (strcmp(name, "_start") == 0)
        {
            fprintf(stderr, "%s\n", name);
            sim.setPC(symbol->offset);
        }
    }


    sim.fillMemory();
//    ac_int<32, true>* dm_in;
//    dm_in = sim.getDataMemory();
    ac_int<32, true>* dm_out;
    ac_int<32, true>* debug_out;
    dm_out = (ac_int<32, true> *)malloc(8192 * sizeof(ac_int<32, true>));
    debug_out = (ac_int<32, true> *)malloc(200 * sizeof(ac_int<32, true>));

    int ins = 123456789;
    //std::cin >> ins;
    CCS_DESIGN(doStep(sim.getPC(),ins,sim.getInstructionMemory(),sim.getDataMemory(),dm_out));//,debug_out));
    for(int i = 0; i<32; i++)
    {
        std::cout << std::dec << i << " : ";
        std::cout << std::hex << debug_out[i] << std::endl;
    }
    std::cout << "dm" <<std::endl;
    /*for(int i = 0;i<8192;i++){
        	std::cout << std::dec << i << " : ";
        	std::cout << std::dec << dm_out[i] << std::endl;
    }*/
    free(dm_out);
    free(debug_out);
    CCS_RETURN(0);
}
