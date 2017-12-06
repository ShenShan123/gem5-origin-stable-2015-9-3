#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_IntrControl[] = {
    120,156,181,81,61,79,195,48,16,61,167,105,41,133,1,137,
    137,1,41,99,64,74,195,135,58,32,161,10,129,132,196,2,
    168,97,33,75,228,38,110,147,42,78,34,219,21,237,92,254,
    55,220,57,45,240,7,176,157,231,119,167,248,221,59,59,133,
    237,232,224,119,231,1,232,49,146,12,23,131,18,64,50,136,
    25,48,138,29,40,29,120,219,178,14,178,14,8,6,51,6,
    153,11,159,0,27,128,247,216,133,172,11,145,223,67,137,226,
    11,135,207,144,153,125,132,168,144,47,211,133,72,77,155,34,
    56,55,7,136,79,149,81,15,53,66,93,166,127,189,220,147,
    151,51,36,2,200,2,214,141,29,50,21,99,89,23,22,93,
    16,61,88,236,145,153,141,3,113,31,38,145,79,199,38,14,
    130,62,70,72,155,101,88,160,110,146,182,234,195,60,215,39,
    84,59,23,158,94,107,35,164,247,33,60,174,132,215,112,101,
    188,122,230,147,79,211,71,72,146,138,75,145,36,102,96,3,
    89,103,203,146,66,151,126,88,55,194,230,211,213,42,201,5,
    207,132,50,93,12,95,185,226,210,80,231,145,21,183,20,147,
    162,50,134,140,241,106,109,119,44,237,83,143,191,160,31,17,
    66,89,153,48,159,207,116,24,229,120,40,202,69,21,206,133,
    28,5,181,42,230,69,21,104,195,167,165,8,174,46,46,71,
    193,77,112,29,106,149,134,212,225,159,235,27,54,107,219,189,
    71,154,84,189,199,104,62,251,100,207,28,34,200,209,240,231,
    29,38,176,123,26,204,54,100,94,219,222,41,82,245,170,149,
    250,63,167,182,214,109,123,179,227,211,157,227,35,54,192,249,
    13,242,171,147,125,
};

EmbeddedPython embedded_m5_objects_IntrControl(
    "m5/objects/IntrControl.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/cpu/IntrControl.py",
    "m5.objects.IntrControl",
    data_m5_objects_IntrControl,
    373,
    660);

} // anonymous namespace
