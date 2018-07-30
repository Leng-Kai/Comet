#include "cache.h"

bool find(ICacheControl& ictrl, ac_int<32, false> address)
{
    bool found = false;
    bool valid = false;

    #pragma hls_unroll yes
    findiloop:for(int i = 0; i < Associativity; ++i)
    {
        if((ictrl.setctrl.tag[i] == getTag(address)) && ictrl.setctrl.valid[i])
        {
            found = true;
            valid = true;
            ictrl.currentway = i;
        }
    }

    return found && valid;
}

bool find(DCacheControl& dctrl, ac_int<32, false> address)
{
    bool found = false;
    bool valid = false;

    #pragma hls_unroll yes
    finddloop:for(int i = 0; i < Associativity; ++i)
    {
        if((dctrl.setctrl.tag[i] == getTag(address)) && dctrl.setctrl.valid[i])
        {
            found = true;
            valid = true;
            dctrl.currentway = i;
        }
    }

    return found && valid;
}

void select(ICacheControl& ictrl)
{
#if Associativity > 1
  #if Policy == RP_FIFO
    ictrl.currentway = ictrl.setctrl.policy++;
  #elif Policy == RP_LRU
    #if Associativity == 2
        ictrl.currentway = !ictrl.setctrl.policy;
    #elif Associativity == 4
        if(ictrl.setctrl.policy.slc<3>(3) == 0)
        {
            ictrl.currentway = 3;
        }
        else if(ictrl.setctrl.policy.slc<2>(1) == 0)
        {
            ictrl.currentway = 2;
        }
        else if(ictrl.setctrl.policy.slc<1>(0) == 0)
        {
            ictrl.currentway = 1;
        }
        else
        {
            ictrl.currentway = 0;
        }
    #else
        #error "RP_LRU with DRAM_SIZE >= 8 ways associativity is not implemented"
    #endif
  #elif Policy == RP_RANDOM
    ictrl.currentway = ictrl.policy.slc<ac::log2_ceil<Associativity>::val>(0);     // ictrl.policy & (Associativity - 1)
    ictrl.policy = (ictrl.policy.slc<1>(31) ^ ictrl.policy.slc<1>(21) ^ ictrl.policy.slc<1>(1) ^ ictrl.policy.slc<1>(0)) | (ictrl.policy << 1);
  #else   // None
    ictrl.currentway = 0;
  #endif
#endif
}

void select(DCacheControl& dctrl)
{
#if Associativity > 1
  #if Policy == RP_FIFO
    dctrl.currentway = dctrl.setctrl.policy++;
  #elif Policy == RP_LRU
    #if Associativity == 2
        dctrl.currentway = !dctrl.setctrl.policy;
    #elif Associativity == 4
        if(dctrl.setctrl.policy.slc<3>(3) == 0)
        {
            dctrl.currentway = 3;
        }
        else if(dctrl.setctrl.policy.slc<2>(1) == 0)
        {
            dctrl.currentway = 2;
        }
        else if(dctrl.setctrl.policy.slc<1>(0) == 0)
        {
            dctrl.currentway = 1;
        }
        else
        {
            dctrl.currentway = 0;
        }
    #else
        #error "RP_LRU with DRAM_SIZE >= 8 ways associativity is not implemented"
    #endif
  #elif Policy == RP_RANDOM
    dctrl.currentway = dctrl.policy.slc<ac::log2_ceil<Associativity>::val>(0);     // dctrl.policy & (Associativity - 1)
    dctrl.policy = (dctrl.policy.slc<1>(31) ^ dctrl.policy.slc<1>(21) ^ dctrl.policy.slc<1>(1) ^ dctrl.policy.slc<1>(0)) | (dctrl.policy << 1);
  #else   // None
    dctrl.currentway = 0;
  #endif
#endif
}

void update_policy(ICacheControl& ictrl)
{
#if Associativity > 1
  #if Policy == RP_FIFO
    // no promotion
  #elif Policy == RP_LRU
    #if Associativity == 2
        ictrl.setctrl.policy = ictrl.currentway;
    #elif Associativity == 4
        switch (ictrl.currentway) {
        case 0:
            ictrl.setctrl.policy.set_slc(0, (ac_int<1, false>)0);
            ictrl.setctrl.policy.set_slc(1, (ac_int<1, false>)0);
            ictrl.setctrl.policy.set_slc(3, (ac_int<1, false>)0);
            break;
        case 1:
            ictrl.setctrl.policy.set_slc(0, (ac_int<1, false>)1);
            ictrl.setctrl.policy.set_slc(2, (ac_int<1, false>)0);
            ictrl.setctrl.policy.set_slc(4, (ac_int<1, false>)0);
            break;
        case 2:
            ictrl.setctrl.policy.set_slc(1, (ac_int<1, false>)1);
            ictrl.setctrl.policy.set_slc(2, (ac_int<1, false>)1);
            ictrl.setctrl.policy.set_slc(5, (ac_int<1, false>)0);
            break;
        case 3:
            ictrl.setctrl.policy.set_slc(3, (ac_int<1, false>)1);
            ictrl.setctrl.policy.set_slc(4, (ac_int<1, false>)1);
            ictrl.setctrl.policy.set_slc(5, (ac_int<1, false>)1);
            break;
        default:
            break;
        }
    #else
        #error "RP_LRU with DRAM_SIZE >= 8 ways associativity is not implemented"
    #endif
  #elif Policy == RP_RANDOM
    // no promotion
  #else   // None

  #endif
#endif
}

void update_policy(DCacheControl& dctrl)
{
#if Associativity > 1
  #if Policy == RP_FIFO
    // no promotion
  #elif Policy == RP_LRU
    #if Associativity == 2
        dctrl.setctrl.policy = dctrl.currentway;
    #elif Associativity == 4
        switch (dctrl.currentway) {
        case 0:
            dctrl.setctrl.policy.set_slc(0, (ac_int<1, false>)0);
            dctrl.setctrl.policy.set_slc(1, (ac_int<1, false>)0);
            dctrl.setctrl.policy.set_slc(3, (ac_int<1, false>)0);
            break;
        case 1:
            dctrl.setctrl.policy.set_slc(0, (ac_int<1, false>)1);
            dctrl.setctrl.policy.set_slc(2, (ac_int<1, false>)0);
            dctrl.setctrl.policy.set_slc(4, (ac_int<1, false>)0);
            break;
        case 2:
            dctrl.setctrl.policy.set_slc(1, (ac_int<1, false>)1);
            dctrl.setctrl.policy.set_slc(2, (ac_int<1, false>)1);
            dctrl.setctrl.policy.set_slc(5, (ac_int<1, false>)0);
            break;
        case 3:
            dctrl.setctrl.policy.set_slc(3, (ac_int<1, false>)1);
            dctrl.setctrl.policy.set_slc(4, (ac_int<1, false>)1);
            dctrl.setctrl.policy.set_slc(5, (ac_int<1, false>)1);
            break;
        default:
            break;
        }
    #else
        #error "RP_LRU with DRAM_SIZE >= 8 ways associativity is not implemented"
    #endif
  #elif Policy == RP_RANDOM
    // no promotion
  #else   // None

  #endif
#endif
}

void icache(ICacheControl& ictrl, ac_int<128, false> memictrl[Sets],                            // control
            unsigned int imem[DRAM_SIZE], unsigned int data[Sets][Blocksize][Associativity],    // memory and cachedata
            ac_int<32, false> address,                                                          // from cpu
            ac_int<32, false>& cachepc, int& instruction, bool& insvalid                        // to cpu
#ifndef __HLS__
           , ac_int<64, false>& cycles
#endif
           )
{
    if(ictrl.state != IState::Fetch && ictrl.currentset != getSet(address))  // different way but same set keeps same control, except for data......
    {
        ictrl.state = IState::StoreControl;
        gdebug("address %06x storecontrol\n", (ictrl.setctrl.tag[ictrl.currentway].to_int() << tagshift) | (ictrl.currentset.to_int() << setshift));
    }

    switch(ictrl.state)
    {
    case IState::Idle:
        {
            ictrl.currentset = getSet(address);
            ictrl.i = getOffset(address);

            if(!ictrl.ctrlLoaded)
            {
                ac_int<128, false> setctrl = memictrl[ictrl.currentset];
                ictrl.setctrl.bourrage = setctrl.slc<ibourrage>(ICacheControlWidth);
                #pragma hls_unroll yes
                loadiset:for(int i = 0; i < Associativity; ++i)
                {
                    ictrl.setctrl.tag[i] = setctrl.slc<32-tagshift>(i*(32-tagshift));
                    ictrl.setctrl.valid[i] = setctrl.slc<1>(Associativity*(32-tagshift) + i);
                }
            #if Associativity > 1 && (Policy == RP_FIFO || Policy == RP_LRU)
                ictrl.setctrl.policy = setctrl.slc<IPolicyWidth>(Associativity*(32-tagshift+1));
            #endif
            }

            #pragma hls_unroll yes
            loadidata:for(int i = 0; i < Associativity; ++i)    // force reload because offset may have changed
            {
                ictrl.setctrl.data[i] = data[ictrl.currentset][ictrl.i][i];
            }

            ictrl.workAddress = address;
            ictrl.ctrlLoaded = true;

            if(find(ictrl, address))
            {
                instruction = ictrl.setctrl.data[ictrl.currentway];

                ictrl.state = IState::Idle;
            }
            else    // not found or invalid
            {
                select(ictrl);
                coredebug("cim  @%06x   not found or invalid   ", address.to_int());
                ictrl.setctrl.tag[ictrl.currentway] = getTag(address);

                ictrl.state = IState::Fetch;
                ictrl.setctrl.valid[ictrl.currentway] = false;
                ictrl.i = getOffset(address);
                ac_int<32, false> wordad = 0;
                wordad.set_slc(0, address.slc<30>(2));
                wordad.set_slc(30, (ac_int<2, false>)0);
                coredebug("starting fetching to %d %d from %06x to %06x (%06x to %06x)\n", ictrl.currentset.to_int(), ictrl.currentway.to_int(), (wordad.to_int() << 2)&(tagmask+setmask),
                      (((int)(wordad.to_int()+Blocksize) << 2)&(tagmask+setmask))-1, (address >> 2).to_int() & (~(blockmask >> 2)), (((address >> 2).to_int() + Blocksize) & (~(blockmask >> 2)))-1);
                ictrl.valuetowrite = imem[wordad];
                simul(cycles += MEMORY_READ_LATENCY);
                // critical word first
                instruction = ictrl.valuetowrite;
            }

            update_policy(ictrl);
            insvalid = true;
            cachepc = address;
        }
        break;
    case IState::StoreControl:
        #pragma hls_unroll yes
        if(ictrl.ctrlLoaded)        // this prevent storing false control when we jump to another jump instruction
        {
            gdebug("StoreControl for %d %d  %06x to %06x\n", ictrl.currentset.to_int(), ictrl.currentway.to_int(),
                        (ictrl.setctrl.tag[ictrl.currentway].to_int() << tagshift) | (ictrl.currentset.to_int() << setshift),
                        ((ictrl.setctrl.tag[ictrl.currentway].to_int() << tagshift) | (ictrl.currentset.to_int() << setshift))+Blocksize*4-1);

            ac_int<128, false> setctrl = 0;
            setctrl.set_slc(ICacheControlWidth, ictrl.setctrl.bourrage);
            #pragma hls_unroll yes
            storeicontrol:for(int i = 0; i < Associativity; ++i)
            {
                setctrl.set_slc(i*(32-tagshift), ictrl.setctrl.tag[i]);
                setctrl.set_slc(Associativity*(32-tagshift) + i, (ac_int<1, false>)ictrl.setctrl.valid[i]);
                gdebug("tag : %6x      valid : %s\n", (ictrl.setctrl.tag[i].to_int() << tagshift) | (ictrl.currentset.to_int() << setshift), ictrl.setctrl.valid[i]?"true":"false");
            }
        #if Associativity > 1 && (Policy == RP_FIFO || Policy == RP_LRU)
            setctrl.set_slc(Associativity*(32-tagshift+1), ictrl.setctrl.policy);
        #endif
            memictrl[ictrl.currentset] = setctrl;
        }
        else
        {
            gdebug("StoreControl but control not loaded\n");
        }

        ictrl.state = IState::Idle;
        ictrl.currentset = getSet(address);  //use workaddress?
        ictrl.workAddress = address;
        ictrl.ctrlLoaded = false;
        insvalid = false;
        break;
    case IState::Fetch:
        data[ictrl.currentset][ictrl.i][ictrl.currentway] = ictrl.valuetowrite;

        if(++ictrl.i != getOffset(ictrl.workAddress))
        {
            ac_int<32, false> bytead = 0;
            setTag(bytead, ictrl.setctrl.tag[ictrl.currentway]);
            setSet(bytead, ictrl.currentset);
            setOffset(bytead, ictrl.i);

            ictrl.valuetowrite = imem[bytead >> 2];
            simul(cycles += MEMORY_READ_LATENCY);
            instruction = ictrl.valuetowrite;
            cachepc = ictrl.workAddress;
            cachepc.set_slc(2, ictrl.i);
            insvalid = true;
        }
        else
        {
            ictrl.state = IState::Idle;
            ictrl.setctrl.valid[ictrl.currentway] = true;
            ictrl.ctrlLoaded = true;
            //ictrl.currentset = getSet(address);  //use workaddress?
            //ictrl.workAddress = address;
            insvalid = false;
        }
        break;
    default:
        insvalid = false;
        ictrl.state = IState::Idle;
        ictrl.ctrlLoaded = false;
        break;
    }

    simul(if(insvalid)
    {
        coredebug("i    @%06x   %08x    %d %d\n", cachepc.to_int(), instruction, ictrl.currentset.to_int(), ictrl.currentway.to_int());
    })

}

void dcache(DCacheControl& dctrl, ac_int<128, false> memdctrl[Sets],                            // control
            unsigned int dmem[DRAM_SIZE], unsigned int data[Sets][Blocksize][Associativity],    // memory and cachedata
            ac_int<32, false> address, ac_int<2, false> datasize, bool signenable, bool dcacheenable, bool writeenable, int writevalue,    // from cpu
            int& read, bool& datavalid                                                          // to cpu
#ifndef __HLS__
           , ac_int<64, false>& cycles
#endif
           )
{
    /*if(dcacheenable && datavalid)   // we can avoid storing control if we hit same set multiple times in a row
    {
        if(dctrl.currentset != getSet(address))
        {
            dctrl.state == DState::StoreControl;
        }
        else
        {
            dctrl.state == DState::Idle;
        }
    }
    else if(!dcacheenable && datavalid)
    {
        dctrl.state == DState::StoreControl;
    }*/

    switch(dctrl.state)
    {
    case DState::Idle:
        if(dcacheenable)
        {
            dctrl.currentset = getSet(address);
            dctrl.i = getOffset(address);

            ac_int<128, false> setctrl = memdctrl[dctrl.currentset];
            dctrl.setctrl.bourrage = setctrl.slc<dbourrage>(DCacheControlWidth);
            #pragma hls_unroll yes
            loaddset:for(int i = 0; i < Associativity; ++i)
            {
                dctrl.setctrl.data[i] = data[dctrl.currentset][dctrl.i][i];

                dctrl.setctrl.tag[i] = setctrl.slc<32-tagshift>(i*(32-tagshift));
                dctrl.setctrl.valid[i] = setctrl.slc<1>(Associativity*(32-tagshift) + i);
                dctrl.setctrl.dirty[i] = setctrl.slc<1>(Associativity*(32-tagshift+1) + i);
            }
        #if Associativity > 1 && (Policy == RP_FIFO || Policy == RP_LRU)
            dctrl.setctrl.policy = setctrl.slc<DPolicyWidth>(Associativity*(32-tagshift+2));
        #endif

            if(find(dctrl, address))
            {
                if(writeenable)
                {
                    dctrl.valuetowrite = dctrl.setctrl.data[dctrl.currentway];
                    formatwrite(address, datasize, dctrl.valuetowrite, writevalue);
                    dctrl.workAddress = address;
                    dctrl.setctrl.dirty[dctrl.currentway] = true;

                    dctrl.state = DState::StoreData;
                }
                else
                {
                    ac_int<32, false> r = dctrl.setctrl.data[dctrl.currentway];
                    formatread(address, datasize, signenable, r);

                    read = r;

                    dctrl.state = DState::StoreControl;
                }
                update_policy(dctrl);
                datavalid = true;
            }
            else    // not found or invalid
            {
                select(dctrl);
                gdebug("cdm  @%06x   not found or invalid   ", address.to_int());
                if(dctrl.setctrl.dirty[dctrl.currentway] && dctrl.setctrl.valid[dctrl.currentway])
                {
                    dctrl.state = DState::FirstWriteBack;
                    dctrl.i = 0;
                    dctrl.workAddress = 0;
                    setTag(dctrl.workAddress, dctrl.setctrl.tag[dctrl.currentway]);
                    setSet(dctrl.workAddress, dctrl.currentset);
                    //dctrl.valuetowrite = dctrl.setctrl.data[dctrl.currentway];    // only if same offset than requested address
                    datavalid = false;
                    coredebug("starting writeback from %d %d from %06x to %06x\n", dctrl.currentset.to_int(), dctrl.currentway.to_int(), dctrl.workAddress.to_int(), dctrl.workAddress.to_int() + 4*Blocksize-1);
                }
                else
                {
                    dctrl.setctrl.tag[dctrl.currentway] = getTag(address);
                    dctrl.workAddress = address;
                    dctrl.state = DState::Fetch;
                    dctrl.setctrl.valid[dctrl.currentway] = false;
                    dctrl.i = getOffset(address);
                    ac_int<32, false> wordad = 0;
                    wordad.set_slc(0, address.slc<30>(2));
                    wordad.set_slc(30, (ac_int<2, false>)0);
                    coredebug("starting fetching to %d %d for %s from %06x to %06x (%06x to %06x)\n", dctrl.currentset.to_int(), dctrl.currentway.to_int(), writeenable?"W":"R", (wordad.to_int() << 2)&(tagmask+setmask),
                          (((int)(wordad.to_int()+Blocksize) << 2)&(tagmask+setmask))-1, (address >> 2).to_int() & (~(blockmask >> 2)), (((address >> 2).to_int() + Blocksize) & (~(blockmask >> 2)))-1 );
                    dctrl.valuetowrite = dmem[wordad];
                    simul(cycles += MEMORY_READ_LATENCY);
                    // critical word first
                    if(writeenable)
                    {
                        formatwrite(address, datasize, dctrl.valuetowrite, writevalue);
                        dctrl.setctrl.dirty[dctrl.currentway] = true;
                    }
                    else
                    {
                        ac_int<32, false> r = dctrl.valuetowrite;
                        dctrl.setctrl.dirty[dctrl.currentway] = false;
                        formatread(address, datasize, signenable, r);
                        read = r;
                    }

                    datavalid = true;
                }
            }
        }
        else
            datavalid = false;
        break;
    case DState::StoreControl:
    {
        ac_int<128, false> setctrl = 0;
        setctrl.set_slc(DCacheControlWidth, dctrl.setctrl.bourrage);
        #pragma hls_unroll yes
        storedcontrol:for(int i = 0; i < Associativity; ++i)
        {
            setctrl.set_slc(i*(32-tagshift), dctrl.setctrl.tag[i]);
            setctrl.set_slc(Associativity*(32-tagshift) + i, (ac_int<1, false>)dctrl.setctrl.valid[i]);
            setctrl.set_slc(Associativity*(32-tagshift+1) + i, (ac_int<1, false>)dctrl.setctrl.dirty[i]);
        }
    #if Associativity > 1 && (Policy == RP_FIFO || Policy == RP_LRU)
        setctrl.set_slc(Associativity*(32-tagshift+2), dctrl.setctrl.policy);
    #endif

        memdctrl[dctrl.currentset] = setctrl;
        dctrl.state = DState::Idle;
        datavalid = false;
        break;
    }
    case DState::StoreData:
    {
        ac_int<128, false> setctrl = 0;
        setctrl.set_slc(DCacheControlWidth, dctrl.setctrl.bourrage);
        #pragma hls_unroll yes
        storedata:for(int i = 0; i < Associativity; ++i)
        {
            setctrl.set_slc(i*(32-tagshift), dctrl.setctrl.tag[i]);
            setctrl.set_slc(Associativity*(32-tagshift) + i, (ac_int<1, false>)dctrl.setctrl.valid[i]);
            setctrl.set_slc(Associativity*(32-tagshift+1) + i, (ac_int<1, false>)dctrl.setctrl.dirty[i]);
        }
    #if Associativity > 1 && (Policy == RP_FIFO || Policy == RP_LRU)
        setctrl.set_slc(Associativity*(32-tagshift+2), dctrl.setctrl.policy);
    #endif
        memdctrl[dctrl.currentset] = setctrl;

        data[dctrl.currentset][getOffset(dctrl.workAddress)][dctrl.currentway] = dctrl.valuetowrite;

        dctrl.state = DState::Idle;
        datavalid = false;
        break;
    }
    case DState::FirstWriteBack:
    {   //bracket for scope and allow compilation
        dctrl.i = 0;
        ac_int<32, false> bytead = 0;
        setTag(bytead, dctrl.setctrl.tag[dctrl.currentway]);
        setSet(bytead, dctrl.currentset);
        setOffset(bytead, dctrl.i);

        dctrl.valuetowrite = data[dctrl.currentset][dctrl.i][dctrl.currentway];
        dctrl.state = DState::WriteBack;
        datavalid = false;
        break;
    }
    case DState::WriteBack:
    {   //bracket for scope and allow compilation
        ac_int<32, false> bytead = 0;
        setTag(bytead, dctrl.setctrl.tag[dctrl.currentway]);
        setSet(bytead, dctrl.currentset);
        setOffset(bytead, dctrl.i);
        dmem[bytead >> 2] = dctrl.valuetowrite;
        simul(cycles += MEMORY_WRITE_LATENCY);

        if(++dctrl.i)
            dctrl.valuetowrite = data[dctrl.currentset][dctrl.i][dctrl.currentway];
        else
        {
            dctrl.state = DState::StoreControl;
            dctrl.setctrl.dirty[dctrl.currentway] = false;
            //gdebug("end of writeback\n");
        }

        datavalid = false;
        break;
    }
    case DState::Fetch:
        data[dctrl.currentset][dctrl.i][dctrl.currentway] = dctrl.valuetowrite;

        if(++dctrl.i != getOffset(dctrl.workAddress))
        {
            ac_int<32, false> bytead = 0;
            setTag(bytead, dctrl.setctrl.tag[dctrl.currentway]);
            setSet(bytead, dctrl.currentset);
            setOffset(bytead, dctrl.i);

            dctrl.valuetowrite = dmem[bytead >> 2];
            simul(cycles += MEMORY_READ_LATENCY);
        }
        else
        {
            dctrl.state = DState::StoreControl;
            dctrl.setctrl.valid[dctrl.currentway] = true;
            update_policy(dctrl);
            //gdebug("end of fetch to %d %d\n", dctrl.currentset.to_int(), dctrl.currentway.to_int());
        }

        datavalid = false;
        break;
    default:
        datavalid = false;
        dctrl.state = DState::Idle;
        break;
    }

    simul(if(datavalid)
    {
        if(writeenable)
            coredebug("dW%d  @%06x   %08x   %08x   %08x   %d %d\n", datasize.to_int(), address.to_int(), dctrl.state == DState::Fetch?dmem[address/4]:data[dctrl.currentset][dctrl.i][dctrl.currentway],
                                                                      writevalue, dctrl.valuetowrite.to_int(), dctrl.currentset.to_int(), dctrl.currentway.to_int());
        else
            coredebug("dR%d  @%06x   %08x   %08x   %5s   %d %d\n", datasize.to_int(), address.to_int(), dctrl.state == DState::Fetch?dmem[address/4]:data[dctrl.currentset][dctrl.i][dctrl.currentway],
                                                                    read, signenable?"true":"false", dctrl.currentset.to_int(), dctrl.currentway.to_int());
    })
}

