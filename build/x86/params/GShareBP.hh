#ifndef __PARAMS__GShareBP__
#define __PARAMS__GShareBP__

class GShareBP;

#include <cstddef>
#include "base/types.hh"
#include <cstddef>
#include "base/types.hh"

#include "params/BranchPredictor.hh"

struct GShareBPParams
    : public BranchPredictorParams
{
    GShareBP * create();
    unsigned globalCtrBits;
    unsigned globalPredictorSize;
};

#endif // __PARAMS__GShareBP__
