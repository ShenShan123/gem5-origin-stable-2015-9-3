/*
 * Copyright (c) 2012-2013 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005,2014 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Erik Hallnor
 */

/**
 * @file
 * Definitions of a LRU tag store.
 */

#include "debug/CacheRepl.hh"
#include "mem/cache/tags/lru.hh"
#include "mem/cache/base.hh"
#include <iostream>

LRU::LRU(const Params *p)
    : BaseSetAssoc(p)
{
}

CacheBlk*
LRU::accessBlock(Addr addr, bool is_secure, Cycles &lat, int master_id)
{
    CacheBlk *blk = BaseSetAssoc::accessBlock(addr, is_secure, lat, master_id);

    if (blk != NULL) {
        // move this block to head of the MRU list
        sets[blk->set].moveToHead(blk);
        DPRINTF(CacheRepl, "set %x: moving blk %x (%s) to MRU\n",
                blk->set, regenerateBlkAddr(blk->tag, blk->set),
                is_secure ? "s" : "ns");
    }

    return blk;
}

CacheBlk*
LRU::findVictim(Addr addr)
{
    int set = extractSet(addr);
    // changed by shen
    BlkType *blk = NULL;

    if (!(name() == "system.cpu.dcache.tags" || name() == "system.cpu.icache.tags" || name() == "system.l2.tags")) {
        blk = sets[set].blks[assoc - 1];
        blk->faultyMatch = true;
        
        if (blk->isValid())
            DPRINTF(CacheRepl, "set %x: selecting blk %x for replacement\n",
                    set, regenerateBlkAddr(blk->tag, set));
        
        return blk;
    }

    int8_t i;
    // grab a replacement candidate
    for(i = assoc - 1; i >= 0; --i) {
        int numFaults = 0;

        for (uint8_t j = 0; j < blkSize / SUBBLOCKSIZE; ++j)
            // we need to find the physical way index of this block and count its faulty entries
            numFaults += !faultMap[(set * assoc + sets[set].blks[i]->physicalWay) * (blkSize / SUBBLOCKSIZE) + j];

        if (numFaults <= numNullSubblocks) {
            // we have found the LRU way that fits the allocation as a victim
            blk = sets[set].blks[i];
            blk->faultyMatch = true;
            // assign current CM to the global CM
            std::memcpy(&comprMap[(set * assoc + sets[set].blks[i]->physicalWay) * (blkSize / SUBBLOCKSIZE)], curBlockMC, sizeof(bool) * blkSize / SUBBLOCKSIZE);
            //for (uint8_t j = 0; j < blkSize / SUBBLOCKSIZE; ++j)
                //std::cout << " " << curBlockMC[i] << " " << comprMap[(set * assoc + sets[set].blks[i]->physicalWay) * (blkSize / SUBBLOCKSIZE) + i];
            DPRINTF(CacheRepl, "%s, num faults %d, num null blocks %d, physicalWay %d, way %d", name(), numFaults, numNullSubblocks, blk->physicalWay, i);
            break;
        }
    }

    // if we don't find the victim, invalidate the LRU way
    if (i == -1) {
        //blk = sets[set].blks[assoc - 1];
        //blk->faultyMatch = false;
        DPRINTF(CacheRepl, "address: %llx, no victim found, blk Ptr %llx", addr, blk);
    }
    // end, by shen
    return blk;
}

void
LRU::insertBlock(PacketPtr pkt, BlkType *blk)
{
    BaseSetAssoc::insertBlock(pkt, blk);

    int set = extractSet(pkt->getAddr());
    sets[set].moveToHead(blk);
}

void
LRU::invalidate(CacheBlk *blk)
{
    BaseSetAssoc::invalidate(blk);

    // should be evicted before valid blocks
    int set = blk->set;
    sets[set].moveToTail(blk);
}

LRU*
LRUParams::create()
{
    return new LRU(this);
}
