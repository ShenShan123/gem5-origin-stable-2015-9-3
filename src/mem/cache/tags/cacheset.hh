/*
 * Copyright (c) 2013 ARM Limited
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
 * Copyright (c) 2009 The Regents of The University of Michigan
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
 * Authors: Lisa Hsu
 */

/**
 * @file
 * Declaration of an associative set
 */

#ifndef __CACHESET_HH__
#define __CACHESET_HH__

#include <cassert>
#include <chrono>
#include <random>
#include <bitset>

#include "mem/cache/blk.hh" // base class
#include "base/statistics.hh"

/**
 * An associative set of cache blocks.
 */
template <class Blktype>
class CacheSet
{
  public:
    /** The associativity of this set. */
    int assoc;

    /** Cache blocks in this set, maintained in LRU order 0 = MRU. */
    Blktype **blks;

    /**
     * Find a block matching the tag in this set.
     * @param way_id The id of the way that matches the tag.
     * @param tag The Tag to find.
     * @param is_secure True if the target memory space is secure.
     * @return Pointer to the block if found. Set way_id to assoc if none found
     */
    // by shen
    Blktype* findBlk(Addr tag, bool is_secure, int& way_id, 
        Stats::SparseHistogram * diff = nullptr, Stats::SparseHistogram * hit = nullptr, Stats::SparseHistogram * mis = nullptr, 
        Stats::SparseHistogram * off = nullptr, Stats::SparseHistogram * faMis = nullptr, Stats::Scalar * faHit = nullptr);
    Blktype* findBlk(Addr tag, bool is_secure, 
        Stats::SparseHistogram * diff = nullptr, Stats::SparseHistogram * hit = nullptr, Stats::SparseHistogram * mis = nullptr, 
        Stats::SparseHistogram * off = nullptr, Stats::SparseHistogram * faMis = nullptr, Stats::Scalar * faHit = nullptr);
    // end

    /**
     * Move the given block to the head of the list.
     * @param blk The block to move.
     */
    void moveToHead(Blktype *blk);

    /**
     * Move the given block to the tail of the list.
     * @param blk The block to move
     */
    void moveToTail(Blktype *blk);

};

// by shen
inline int countBits(Addr t)
{
    int count = 0;
    while (t) {
        count += t & 1;
        t = t >> 1;
    }
    return count;
}
// end

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure, int& way_id,
    Stats::SparseHistogram * diff, Stats::SparseHistogram * hit, Stats::SparseHistogram * mis, 
    Stats::SparseHistogram * off, Stats::SparseHistogram * faMis, Stats::Scalar * faHit)
{
    // by shen
    // generate a number with uniform distribution
    int numTagBits = 28;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine e (seed);
    std::uniform_real_distribution<double> unif(0.0,1.0);
    double mispred = 0.01; // P(1 bit misprediction) = 0.0062
    std::bitset<28> errorMask;

    /**
     * Way_id returns the id of the way that matches the block
     * If no block is found way_id is set to assoc.
     */
    way_id = assoc;
    int hamDist = -1;
    bool findHit = false;
    Addr specTag;

    for (int i = 0; i < assoc; ++i) {
        // statistic the differency of tag bits
        if (diff != nullptr) {
            // do tag differency distribution
            Addr diffTag = blks[i]->tag ^ tag;
            int shifts = 0;
            while (diffTag) {
                diff->sample(shifts, diffTag & 1);
                ++shifts;
                diffTag = diffTag >> 1; // logical shift right
            }
            // done here

            for (int j = 0; j < numTagBits; ++j)
                errorMask[j] = mispred >= unif(e); // generate error bits
            
            Addr mask = errorMask.to_ullong();

            specTag = mask ^ blks[i]->tag;
            hamDist = countBits(specTag ^ tag);
            // do statistics
            // turn off the disimilar ways
            off->sample(0, hamDist > 0);
            off->sample(1, hamDist > 1);
            off->sample(2, hamDist > 2);
            off->sample(3, hamDist > 3);
            off->sample(4, hamDist > 4);
            off->sample(5, hamDist > 5);
            off->sample(6, hamDist > 6);
            off->sample(7, hamDist > 7);
            off->sample(8, hamDist > 8);
            off->sample(9, hamDist > 9);
            // actual miss but we speculate it as a hit
            //faHit += specTag == tag && way_id == assoc;
        }
        // find the matched tag
        if (blks[i]->tag == tag && blks[i]->isValid() && blks[i]->isSecure() == is_secure) {
            way_id = i;
            findHit = true;
        }

        if (findHit && faMis != nullptr) {
            // actual hit but we turn off the way
            faMis->sample(0, hamDist > 0);
            faMis->sample(1, hamDist > 1);
            faMis->sample(2, hamDist > 2);
            faMis->sample(3, hamDist > 3);
            faMis->sample(4, hamDist > 4);
            faMis->sample(5, hamDist > 5);
            faMis->sample(6, hamDist > 6);
            faMis->sample(7, hamDist > 7);
            faMis->sample(8, hamDist > 8);
            faMis->sample(9, hamDist > 9);
            //inform("origin tag %lx, spec tag %lx, req tag %lx, ham dist %d", blks[i]->tag, specTag, tag, hamDist);
            // hamming dist distribution when hits
            if (hit != nullptr)
                hit->sample(countBits(blks[i]->tag ^ tag));
        }
        // hamming dist distribution when miss
        else if (!findHit && mis != nullptr)
            mis->sample(countBits(blks[i]->tag ^ tag));

        findHit = false;
    }

    if (way_id != assoc) return blks[way_id];
    return nullptr;
    // end
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure, 
    Stats::SparseHistogram * diff, Stats::SparseHistogram * hit, Stats::SparseHistogram * mis, 
    Stats::SparseHistogram * off, Stats::SparseHistogram * faMis, Stats::Scalar * faHit)
{
    int ignored_way_id;
    return findBlk(tag, is_secure, ignored_way_id, diff, hit, mis, off, faMis, faHit);
}

template <class Blktype>
void
CacheSet<Blktype>::moveToHead(Blktype *blk)
{
    // nothing to do if blk is already head
    if (blks[0] == blk)
        return;

    // write 'next' block into blks[i], moving up from MRU toward LRU
    // until we overwrite the block we moved to head.

    // start by setting up to write 'blk' into blks[0]
    int i = 0;
    Blktype *next = blk;

    do {
        assert(i < assoc);
        // swap blks[i] and next
        Blktype *tmp = blks[i];
        blks[i] = next;
        next = tmp;
        ++i;
    } while (next != blk);
}

template <class Blktype>
void
CacheSet<Blktype>::moveToTail(Blktype *blk)
{
    // nothing to do if blk is already tail
    if (blks[assoc - 1] == blk)
        return;

    // write 'next' block into blks[i], moving from LRU to MRU
    // until we overwrite the block we moved to tail.

    // start by setting up to write 'blk' into tail
    int i = assoc - 1;
    Blktype *next = blk;

    do {
        assert(i >= 0);
        // swap blks[i] and next
        Blktype *tmp = blks[i];
        blks[i] = next;
        next = tmp;
        --i;
    } while (next != blk);
}

#endif
