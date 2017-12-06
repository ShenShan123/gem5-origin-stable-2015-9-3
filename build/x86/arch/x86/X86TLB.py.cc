#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_X86TLB[] = {
    120,156,181,84,91,111,211,80,12,118,210,245,186,13,198,4,
    60,241,112,196,3,170,16,93,184,168,211,152,16,218,246,128,
    132,196,198,148,14,1,125,137,78,19,183,73,201,165,228,156,
    106,45,175,227,127,131,237,116,163,8,137,183,165,202,145,237,
    56,246,247,217,95,26,194,234,170,209,125,164,0,76,68,6,
    221,145,3,41,192,197,202,114,42,203,133,212,133,204,133,161,
    11,14,251,53,72,107,144,213,96,88,35,127,3,176,6,99,
    7,162,58,252,4,184,2,248,58,220,128,168,1,232,74,180,
    121,19,173,67,212,130,65,183,77,141,146,95,116,117,29,178,
    44,31,79,43,179,73,199,137,54,120,241,225,164,10,112,234,
    41,102,31,71,83,12,173,221,37,239,203,193,254,185,158,160,
    213,163,20,63,235,244,27,150,225,58,147,19,102,114,74,6,
    2,12,29,230,67,144,137,8,1,37,18,132,11,235,140,254,
    138,34,4,176,9,211,22,96,27,166,29,32,22,87,148,176,
    41,193,45,230,66,12,56,178,13,254,160,203,192,124,174,111,
    238,84,24,222,15,142,15,15,171,254,230,17,133,116,25,198,
    222,226,96,223,155,93,131,11,46,229,233,94,28,155,199,148,
    112,94,148,86,141,139,82,217,24,85,172,203,232,82,151,168,
    36,83,85,153,102,155,235,47,141,197,76,21,66,56,217,224,
    200,33,29,103,243,108,132,165,42,198,170,152,91,99,117,30,
    37,249,68,222,51,84,80,91,21,234,92,141,80,153,239,115,
    109,98,140,212,140,178,195,101,152,98,151,1,219,22,29,65,
    144,235,12,131,192,118,196,201,138,104,158,178,203,77,236,114,
    134,50,236,112,177,8,194,84,27,35,89,236,197,168,35,44,
    197,61,213,4,174,100,38,242,210,140,141,58,115,211,165,206,
    108,131,172,129,192,23,147,130,152,91,203,67,211,249,82,66,
    21,57,1,243,41,55,201,36,199,200,222,39,39,159,103,65,
    133,60,32,220,65,133,155,55,250,231,48,239,232,240,178,220,
    122,241,100,108,188,1,77,16,7,49,230,222,4,179,126,175,
    40,147,73,146,247,140,140,179,247,242,249,139,126,239,117,239,
    149,103,202,208,187,89,12,45,141,100,181,55,91,202,30,159,
    93,239,178,225,84,191,45,231,129,211,118,4,102,149,249,175,
    172,142,255,47,171,105,131,149,181,210,81,83,34,36,174,22,
    75,159,212,196,193,54,75,137,59,248,172,100,179,185,38,37,
    106,104,238,174,235,200,166,35,146,78,114,196,137,60,48,74,
    80,38,249,129,230,30,79,158,68,246,151,118,186,188,30,159,
    151,226,243,66,124,105,34,162,229,119,125,238,228,239,240,158,
    69,81,84,70,134,32,108,87,5,110,111,220,204,181,207,53,
    183,214,198,189,75,247,153,204,66,100,151,245,247,102,44,34,
    35,226,96,175,44,22,75,159,255,3,124,247,250,219,147,74,
    183,7,84,90,191,169,62,139,183,79,184,54,15,181,227,116,
    220,29,103,199,125,216,254,13,74,28,33,51,
};

EmbeddedPython embedded_m5_objects_X86TLB(
    "m5/objects/X86TLB.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/arch/x86/X86TLB.py",
    "m5.objects.X86TLB",
    data_m5_objects_X86TLB,
    668,
    1330);

} // anonymous namespace
