#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_CheckerCPU[] = {
    120,156,181,82,77,139,219,48,16,29,197,73,118,147,109,97,
    217,158,11,186,213,148,58,238,87,160,133,82,74,66,10,133,
    210,132,100,247,208,92,140,214,158,196,110,109,217,72,50,155,
    156,183,255,187,157,81,54,108,254,64,37,123,120,122,204,151,
    222,40,133,135,21,208,255,69,2,216,247,4,50,250,4,148,
    0,215,140,58,80,10,168,4,172,5,136,44,0,20,176,17,
    144,117,225,15,192,61,192,207,117,7,178,30,172,194,62,5,
    22,127,105,133,130,144,99,243,242,0,207,200,76,148,197,233,
    226,198,13,9,79,115,76,127,163,161,99,122,90,126,194,229,
    23,4,16,184,22,149,166,212,24,192,186,203,221,172,123,128,
    125,248,117,6,120,206,45,221,119,96,61,56,97,2,207,12,
    143,76,192,13,50,115,1,203,85,216,163,164,203,14,25,123,
    69,38,109,218,56,61,180,16,19,30,229,185,189,36,122,182,
    43,156,172,181,84,90,162,49,181,177,31,136,188,105,50,229,
    80,186,28,229,67,136,188,43,92,238,137,74,21,90,210,37,
    94,88,105,29,123,157,6,79,41,248,219,70,42,89,214,42,
    147,6,109,91,58,89,88,89,232,180,54,6,83,247,138,220,
    203,189,108,76,161,29,185,221,41,163,11,189,165,4,153,204,
    106,169,107,39,145,26,10,159,176,128,231,100,146,68,171,10,
    147,196,43,152,36,85,157,181,37,31,187,236,176,111,208,131,
    107,211,162,247,86,183,214,25,149,58,239,157,238,118,73,142,
    42,67,227,88,137,133,50,170,242,238,147,186,46,61,245,85,
    149,22,221,5,107,79,69,231,122,198,119,112,79,233,220,122,
    1,142,204,51,98,184,211,57,181,62,215,223,233,106,158,15,
    121,132,143,198,206,200,196,149,118,113,190,221,216,120,149,43,
    131,171,28,117,188,197,106,28,213,166,216,22,58,34,201,110,
    75,140,222,190,126,51,142,62,70,239,98,107,82,158,70,252,
    248,56,70,205,222,15,77,114,74,158,80,95,28,246,149,24,
    248,253,35,228,103,227,6,100,170,241,168,225,107,217,165,56,
    206,250,191,53,229,5,254,116,24,192,231,231,156,154,181,28,
    138,203,206,63,237,78,200,148,
};

EmbeddedPython embedded_m5_objects_CheckerCPU(
    "m5/objects/CheckerCPU.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/cpu/CheckerCPU.py",
    "m5.objects.CheckerCPU",
    data_m5_objects_CheckerCPU,
    488,
    846);

} // anonymous namespace
