#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_I8259[] = {
    120,156,181,84,223,107,219,64,12,214,57,137,211,164,237,90,
    54,40,108,48,8,219,139,25,75,189,118,36,180,101,140,173,
    237,96,131,182,20,187,131,213,47,193,181,47,241,117,254,17,
    124,151,172,129,190,117,255,247,38,201,73,127,176,215,213,137,
    133,78,150,117,250,244,125,231,8,230,87,13,239,79,29,0,
    125,137,78,140,127,1,41,192,217,220,19,149,103,65,106,65,
    86,131,160,6,130,214,53,72,235,144,53,32,104,64,102,67,
    96,99,180,14,178,9,67,1,113,3,126,3,220,0,156,7,
    75,16,219,32,107,28,109,222,70,91,16,47,129,239,180,112,
    59,245,7,47,71,160,103,200,188,169,220,39,104,246,67,173,
    162,83,85,28,202,169,138,164,99,81,124,13,205,143,157,254,
    183,220,248,197,164,140,228,169,202,205,234,93,76,229,63,41,
    242,108,30,217,217,238,237,30,132,58,10,99,121,92,196,50,
    186,143,119,159,240,190,70,71,2,4,2,84,109,14,188,143,
    64,17,91,31,193,32,140,126,96,129,231,59,54,53,74,111,
    154,101,52,92,247,56,212,70,150,138,187,109,47,130,126,26,
    78,165,178,30,36,98,83,163,84,58,180,167,89,66,51,24,
    228,97,38,7,3,126,107,48,200,138,120,146,210,146,18,178,
    112,236,208,54,119,70,31,162,113,179,220,184,201,104,168,93,
    63,9,75,233,39,50,119,71,50,235,117,139,82,141,84,222,
    213,38,188,72,101,119,251,221,86,175,187,219,125,239,234,50,
    114,99,57,117,175,118,250,46,247,176,57,158,121,117,44,244,
    138,42,18,24,91,180,69,83,152,198,162,201,127,71,115,126,
    59,26,156,11,78,1,71,131,204,227,112,2,100,185,1,151,
    200,170,77,100,226,172,110,48,216,228,32,178,93,135,27,204,
    108,241,178,13,114,153,166,72,9,43,36,4,228,63,88,165,
    129,54,177,186,71,35,210,11,246,252,207,123,123,220,138,94,
    39,17,206,187,87,220,125,146,232,231,24,60,75,100,103,172,
    242,142,73,148,238,112,110,39,46,213,84,106,253,2,159,126,
    45,126,221,127,130,78,84,81,31,107,210,3,83,83,61,123,
    219,81,195,78,152,207,34,34,138,110,26,195,1,129,166,68,
    195,50,184,102,197,95,11,132,104,249,76,222,137,177,171,198,
    80,140,236,230,147,236,66,150,172,87,143,18,42,133,210,152,
    181,76,135,236,164,42,151,143,192,37,107,5,39,177,69,69,
    105,87,16,14,157,24,143,244,229,181,22,109,152,217,88,26,
    90,69,87,87,131,40,13,181,102,205,209,42,145,56,152,146,
    249,63,13,203,48,243,184,119,66,85,76,204,120,98,88,45,
    92,4,245,41,153,41,94,157,124,63,58,226,183,52,141,211,
    91,127,160,213,255,44,216,229,121,73,22,132,45,170,223,138,
    104,137,53,241,212,58,113,8,10,163,203,122,155,99,194,160,
    249,120,209,170,44,174,102,12,166,250,116,120,98,145,90,125,
    38,240,19,193,120,153,53,134,245,5,185,100,200,183,155,62,
    6,34,238,239,67,117,224,63,190,164,202,132,161,141,71,113,
    93,108,88,27,246,95,62,121,31,91,
};

EmbeddedPython embedded_m5_objects_I8259(
    "m5/objects/I8259.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/dev/x86/I8259.py",
    "m5.objects.I8259",
    data_m5_objects_I8259,
    698,
    1424);

} // anonymous namespace
