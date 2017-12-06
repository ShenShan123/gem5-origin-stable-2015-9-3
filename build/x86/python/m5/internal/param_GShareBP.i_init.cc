#include "sim/init.hh"

extern "C" {
    void init_param_GShareBP();
}

EmbeddedSwig embed_swig_param_GShareBP(init_param_GShareBP);
