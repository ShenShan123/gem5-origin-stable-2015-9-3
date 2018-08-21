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
// added by shen
#include "base/types.hh"
#include "base/statistics.hh"
// end
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
    Blktype* findBlk(Addr tag, bool is_secure, int& way_id) const ;
    Blktype* findBlk(Addr tag, bool is_secure) const ;

    // by shen
    Blktype* findBlk(Addr tag, bool is_secure, int& way_id, Stats::Vector & off, Stats::Vector & faMis, 
        bool & error, Stats::Scalar & tagMisSpec, Stats::Vector & hdWrong, Stats::Scalar & waypred, 
        Stats::Scalar & ptOffs, Stats::Vector & hit, Stats::Vector & mis);
    
    Blktype* findBlk(Addr tag, bool is_secure, Stats::Vector & off, Stats::Vector & faMis, 
        bool & error, Stats::Scalar & tagMisSpec, Stats::Vector & hdWrong, Stats::Scalar & waypred, 
        Stats::Scalar & ptOffs, Stats::Vector & hit, Stats::Vector & mis);
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

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure, int& way_id) const
{
    /**
     * Way_id returns the id of the way that matches the block
     * If no block is found way_id is set to assoc.
     */
    way_id = assoc;
    for (int i = 0; i < assoc; ++i) {
        if (blks[i]->tag == tag && blks[i]->isValid() &&
            blks[i]->isSecure() == is_secure) {
            way_id = i;
            return blks[i];
        }
    }
    return NULL;
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure) const
{
    int ignored_way_id;
    return findBlk(tag, is_secure, ignored_way_id);
}

// by shen
inline int countBits(Addr n)
{
    unsigned int c = 0;
    
    for (c = 0; n; ++c)
        n &= (n - 1);

    return c;
}


template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure, int& way_id, Stats::Vector & off, Stats::Vector & faMis, 
    bool & error, Stats::Scalar & tagMisSpec, Stats::Vector & hdWrong, Stats::Scalar & waypred, 
    Stats::Scalar & ptOffs, Stats::Vector & hit, Stats::Vector & mis)
{
    /**
     * Way_id returns the id of the way that matches the block
     * If no block is found way_id is set to assoc.
     */
    way_id = assoc;

    // by shen
    // generate a number with uniform distribution
    int numTagBits = 28;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine e (seed);
    std::uniform_real_distribution<double> unif(0.0,1.0);
    double bitError = 0.01; // P(1 bit bit error) = 0.0102
    double tagErrorRate = 0.0088; // the tag mis-speculation in double sensing, P(tag error) = 0.0088
    double dataErrorRate = 0.02; // default data is 64 bits, P(data error) = 0.0199
    int numTagErrors = 0;
    std::bitset<28> errorMask;
    // tag timing speculations
    int hamDist = -1;
    Addr specTag;

    for (int i = 0; i < assoc; ++i) {
        // find the matched tag
        if (blks[i]->tag == tag && blks[i]->isValid() && blks[i]->isSecure() == is_secure)
            way_id = i;

        for (int j = 0; j < numTagBits; ++j)
            errorMask[j] = bitError >= unif(e); // generate error bits
            
        Addr mask = errorMask.to_ullong();
        specTag = mask ^ blks[i]->tag;
        hamDist = countBits(specTag ^ tag);
        // do statistics
        // turn off the disimilar ways
        off[0] += (hamDist > 0);
        off[1] += (hamDist > 1);
        off[2] += (hamDist > 2);
        off[3] += (hamDist > 3);
        off[4] += (hamDist > 4);
        off[5] += (hamDist > 5);
        off[6] += (hamDist > 6);
        off[7] += (hamDist > 7);
        off[8] += (hamDist > 8);
        off[9] += (hamDist > 9);

        if (way_id == i) {
            // actual hit but we turn off the way
            faMis[0] += (hamDist > 0);
            faMis[1] += (hamDist > 1);
            faMis[2] += (hamDist > 2);
            faMis[3] += (hamDist > 3);
            faMis[4] += (hamDist > 4);
            faMis[5] += (hamDist > 5);
            faMis[6] += (hamDist > 6);
            faMis[7] += (hamDist > 7);
            faMis[8] += (hamDist > 8);
            faMis[9] += (hamDist > 9);
            //inform("origin tag %lx, spec tag %lx, req tag %lx, ham dist %d", blks[i]->tag, specTag, tag, hamDist);
        }

        double ter = unif(e);
        double der = unif(e);
        // tags will be mis-speculative when the ways are active
        numTagErrors += (ter < tagErrorRate);

        if (way_id == i) { // data mis-speculation will occur when the way is active and the tag is correct.
            hdWrong[0] += !(hamDist > 0) && (der < dataErrorRate);
            hdWrong[1] += !(hamDist > 1) && (der < dataErrorRate);
            hdWrong[2] += !(hamDist > 2) && (der < dataErrorRate);
            hdWrong[3] += !(hamDist > 3) && (der < dataErrorRate);
            hdWrong[4] += !(hamDist > 4) && (der < dataErrorRate);
            hdWrong[5] += !(hamDist > 5) && (der < dataErrorRate);
            hdWrong[6] += !(hamDist > 6) && (der < dataErrorRate);
            hdWrong[7] += !(hamDist > 7) && (der < dataErrorRate);
            hdWrong[8] += !(hamDist > 8) && (der < dataErrorRate);
            hdWrong[9] += !(hamDist > 9) && (der < dataErrorRate);
        }

        ptOffs += ((blks[i]->tag ^ tag) & 0x7) != 0;
    }

    tagMisSpec += (numTagErrors > 0); //totTagMisSpec += numTagErrors;

    for (int i = 0; i < assoc; ++i) {
        // hamming dist distribution when 
        // a hit in this set
        hit[countBits(blks[i]->tag ^ tag)] += (way_id != assoc);
        // a miss in this set
        mis[countBits(blks[i]->tag ^ tag)] += (way_id == assoc);
    }

    // way prediction always predicts the MRU way
    waypred += (way_id == 0);

    if (way_id != assoc) return blks[way_id];
    return nullptr;
    // end
}

template <class Blktype>
Blktype*
CacheSet<Blktype>::findBlk(Addr tag, bool is_secure, Stats::Vector & off, Stats::Vector & faMis, 
    bool & error, Stats::Scalar & tagMisSpec, Stats::Vector & hdWrong, Stats::Scalar & waypred, 
    Stats::Scalar & ptOffs, Stats::Vector & hit, Stats::Vector & mis)
{
    int ignored_way_id;
    return findBlk(tag, is_secure, ignored_way_id, off, faMis, error, tagMisSpec, hdWrong, waypred, ptOffs, hit, mis);
}
// end, by shen


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