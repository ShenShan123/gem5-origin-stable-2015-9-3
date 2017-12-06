/*
 * Copyright (c) 2014 The Regents of The University of Michigan
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
 * Authors: Anthony Gutierrez
 */

/* @file
 * Implementation of a bi-mode branch predictor
 */

#include "cpu/pred/gshare.hh"
#include "base/bitfield.hh"
#include "base/intmath.hh"

#define CoLBIT 4

GShareBP::GShareBP(const GShareBPParams *params)
    : BPredUnit(params),
      globalHistoryReg(params->numThreads, 0),
      globalHistoryBits(ceilLog2(params->globalPredictorSize)),
      globalPredictorSize(params->globalPredictorSize),
      globalCtrBits(params->globalCtrBits)
{
    if (!isPowerOf2(globalPredictorSize))
        fatal("Invalid global history predictor size.\n");

    patternTalbe.resize(globalPredictorSize);

    for (int i = 0; i < globalPredictorSize; ++i) {
        patternTalbe[i].setBits(globalCtrBits);
    }

    historyRegisterMask = mask(globalHistoryBits);
    globalHistoryMask = globalPredictorSize - 1;
    takenThreshold = (ULL(1) << (globalCtrBits - 1)) - 1;
}

/*
 * For an unconditional branch we set its history such that
 * everything is set to taken. I.e., its choice predictor
 * chooses the taken array and the taken array predicts taken.
 */
void
GShareBP::uncondBranch(ThreadID tid, Addr pc, void * &bpHistory)
{
    // a dynamic instance of new predictor history item
    BPHistory *history = new BPHistory;
    // ghrs are separated by threads
    history->globalHistoryReg = globalHistoryReg[tid];
    history->finalPred = true;
    bpHistory = static_cast<void*>(history);
    updateGlobalHistReg(tid, true);
}

void
GShareBP::squash(ThreadID tid, void *bpHistory)
{
    // if this predictor squashed, u need to recovery GHR!!
    BPHistory *history = static_cast<BPHistory*>(bpHistory);
    globalHistoryReg[tid] = history->globalHistoryReg;
    // release the dynamic instance!!
    delete history;
}

/*
 * Here we lookup the actual branch prediction. We use the PC to
 * identify the bias of a particular branch, which is based on the
 * prediction in the choice array. A hash of the global history
 * register and a branch's PC is used to index into both the taken
 * and not-taken predictors, which both present a prediction. The
 * choice array's prediction is used to select between the two
 * direction predictors for the final branch prediction.
 */
bool
GShareBP::lookup(ThreadID tid, Addr branchAddr, void * &bpHistory)
{
    // indices to these pattern tables
    // this is for selecting which row
    unsigned globalHistoryIdx = (branchAddr >> (instShiftAmt + CoLBIT)) ^ globalHistoryReg[tid];
    unsigned colMask = (unsigned)(1 << CoLBIT) - 1;
    unsigned column = (branchAddr >> instShiftAmt) & colMask;
    //std::cout << std::hex << "colMask: " << colMask << " globalHistoryIdx: " << globalHistoryIdx << " column: " << column << std::endl;
    globalHistoryIdx = ((globalHistoryIdx << CoLBIT) | column) & globalHistoryMask;
    //std::cout << "globalHistoryIdx: " << globalHistoryIdx << std::endl;

    assert(globalHistoryIdx < globalPredictorSize);

    // get the prediction from two pattern tables
    bool finalPrediction = patternTalbe[globalHistoryIdx].read()
                              > takenThreshold;

    // update the predictor history, it has been set up in GShareBP::uncondBranch()
    BPHistory *history = new BPHistory;
    history->globalHistoryReg = globalHistoryReg[tid];
    history->finalPred = finalPrediction;
    bpHistory = static_cast<void*>(history);
    // first update the GHR with this prediction.
    updateGlobalHistReg(tid, finalPrediction);

    return finalPrediction;
}

void
GShareBP::btbUpdate(ThreadID tid, Addr branchAddr, void * &bpHistory)
{
    // no entry matched in BTB, so lest significant bit in GHR is 0, as not taken
    globalHistoryReg[tid] &= (historyRegisterMask & ~ULL(1));
}

/* Only the selected direction predictor will be updated with the final
 * outcome; the status of the unselected one will not be altered. The choice
 * predictor is always updated with the branch outcome, except when the
 * choice is opposite to the branch outcome but the selected counter of
 * the direction predictors makes a correct final prediction.
 */
void
GShareBP::update(ThreadID tid, Addr branchAddr, bool taken, void *bpHistory,
                 bool squashed)
{
    assert(bpHistory);

    BPHistory *history = static_cast<BPHistory*>(bpHistory);

    // We do not update the counters speculatively on a squash.
    // We just restore the global history register.
    if (squashed) {
        globalHistoryReg[tid] = (history->globalHistoryReg << 1) | taken;
        return;
    }

    unsigned globalHistoryIdx = (((branchAddr >> instShiftAmt)
                                ^ history->globalHistoryReg)
                                & globalHistoryMask);

    assert(globalHistoryIdx < globalPredictorSize);

    // if the taken array's prediction was used, update it
    if (taken) {
        patternTalbe[globalHistoryIdx].increment();
    } else {
        patternTalbe[globalHistoryIdx].decrement();
    }

    delete history;
}

unsigned
GShareBP::getGHR(ThreadID tid, void *bp_history) const
{
    return static_cast<BPHistory*>(bp_history)->globalHistoryReg;
}

void
GShareBP::updateGlobalHistReg(ThreadID tid, bool taken)
{
    // 0 for not taken, 1 for taken. The bit is set at lest significant bit.
    globalHistoryReg[tid] = taken ? (globalHistoryReg[tid] << 1) | 1 :
                               (globalHistoryReg[tid] << 1);
    globalHistoryReg[tid] &= historyRegisterMask;
}

GShareBP*
GShareBPParams::create()
{
    return new GShareBP(this);
}
