#include "sim/init.hh"

namespace {

const uint8_t data_m5_objects_Terminal[] = {
    120,156,181,146,77,111,211,64,16,134,103,157,175,54,109,105,
    85,193,5,9,201,220,44,36,199,80,148,3,18,170,80,43,
    14,92,160,138,115,193,23,203,177,55,177,35,239,218,218,29,
    163,228,86,169,252,111,152,89,39,192,31,192,31,163,153,119,
    189,51,207,140,55,135,195,53,160,247,147,15,96,111,201,41,
    232,17,80,3,40,1,137,0,193,177,7,181,7,203,131,55,
    32,111,0,82,192,90,64,49,132,159,0,79,0,223,147,33,
    20,35,136,131,49,165,168,126,209,21,8,242,240,148,76,92,
    169,111,171,173,204,177,151,216,188,193,19,178,75,105,84,165,
    179,58,255,23,228,142,65,82,114,36,112,125,42,154,120,76,
    148,80,205,33,108,71,32,199,176,157,48,201,147,7,201,137,
    19,79,129,136,136,133,149,169,83,206,152,166,24,59,229,220,
    41,23,32,159,65,49,113,202,37,44,226,128,9,22,30,25,
    123,201,109,203,31,17,30,120,102,101,105,159,115,31,154,20,
    211,181,232,231,141,70,211,212,181,52,213,227,5,237,56,163,
    213,186,178,40,181,223,54,6,43,56,166,57,166,240,117,167,
    86,210,216,151,164,125,214,217,170,150,126,211,97,219,161,95,
    116,170,245,177,241,215,85,45,131,43,222,194,32,105,170,51,
    37,211,20,167,46,80,77,209,213,28,14,249,131,125,43,157,
    158,239,118,105,41,179,66,26,28,81,248,144,153,76,33,163,
    124,33,186,251,30,17,199,253,138,212,136,60,206,76,239,241,
    188,239,197,164,135,54,112,194,195,207,219,7,66,119,21,184,
    7,247,53,37,114,9,122,122,183,118,215,208,6,118,150,166,
    147,110,177,111,36,224,158,255,26,123,79,38,82,26,163,114,
    179,182,81,92,18,66,92,74,29,109,164,154,135,141,169,54,
    149,14,45,242,36,194,155,183,239,230,225,135,240,125,100,77,
    30,241,224,143,7,97,214,238,221,31,121,205,9,153,122,44,
    248,126,33,174,233,254,26,112,211,174,25,53,159,253,57,84,
    11,56,158,51,82,91,30,137,117,19,229,200,52,187,62,225,
    127,130,117,133,62,246,63,235,246,21,39,230,249,92,137,169,
    152,122,191,1,233,213,195,161,
};

EmbeddedPython embedded_m5_objects_Terminal(
    "m5/objects/Terminal.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/dev/Terminal.py",
    "m5.objects.Terminal",
    data_m5_objects_Terminal,
    488,
    862);

} // anonymous namespace
