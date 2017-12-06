#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_LocalBP[] = {
    120,156,197,88,235,114,219,198,21,62,11,128,148,72,81,214,
    93,178,37,217,98,47,110,88,79,37,38,78,20,167,99,215,
    77,148,73,103,154,233,40,46,152,140,29,38,45,10,1,75,
    18,20,9,112,128,149,101,102,164,63,149,167,237,11,244,17,
    250,163,111,211,55,106,207,57,11,128,208,197,83,207,180,67,
    203,196,122,177,216,61,123,46,223,185,236,122,144,254,149,240,
    249,180,14,144,196,2,192,199,159,128,1,192,80,64,91,128,
    144,2,252,85,56,46,65,252,17,248,37,120,13,208,54,64,
    26,112,129,29,19,190,51,32,172,241,154,50,12,76,30,17,
    48,174,130,180,160,93,130,231,225,18,88,178,12,199,85,136,
    255,4,66,136,80,192,11,127,6,252,89,120,141,212,177,83,
    97,130,179,64,131,85,30,172,128,63,199,131,85,240,107,220,
    153,131,241,34,200,26,180,231,105,90,251,22,146,125,128,100,
    23,152,236,191,136,172,143,95,214,192,191,69,211,145,175,111,
    105,166,69,51,121,191,5,166,178,152,113,185,4,237,229,172,
    191,82,232,175,22,250,107,220,199,93,151,161,191,14,253,13,
    232,223,134,14,42,98,41,223,225,14,72,19,250,155,208,222,
    4,137,191,59,112,129,186,242,151,11,43,182,120,197,74,190,
    98,155,87,220,133,246,93,144,248,219,214,43,202,208,106,172,
    163,254,131,127,227,95,3,245,15,170,134,205,75,25,39,65,
    20,58,65,216,137,2,131,190,151,169,33,107,121,212,204,164,
    102,251,156,204,246,79,96,155,249,70,106,182,115,64,194,130,
    100,25,24,112,206,157,115,3,198,13,56,19,208,183,192,55,
    225,12,183,41,17,3,93,1,23,6,124,111,210,132,115,108,
    45,84,238,61,176,148,182,89,159,149,171,41,205,192,121,9,
    206,74,208,122,113,102,208,192,113,5,226,127,192,15,219,76,
    116,150,137,26,112,134,173,5,23,22,156,151,225,57,78,194,
    161,126,133,196,23,47,206,80,82,28,105,53,44,228,246,176,
    32,46,137,226,7,113,232,14,165,186,133,125,103,228,198,238,
    208,249,93,228,185,131,131,103,141,106,54,39,74,246,70,174,
    234,217,188,200,36,109,12,71,138,137,69,161,84,115,216,233,
    4,161,239,12,35,255,100,32,213,44,81,114,58,193,64,58,
    14,127,252,237,112,20,197,234,139,56,142,98,155,20,202,131,
    131,200,205,87,144,58,189,65,148,200,6,237,198,219,216,68,
    94,209,236,206,136,41,18,3,204,40,45,246,101,226,197,193,
    72,161,157,52,69,154,77,212,26,100,33,110,146,63,96,211,
    28,134,170,217,235,118,146,102,171,231,198,178,213,147,97,179,
    43,135,251,187,81,28,116,131,112,55,81,238,209,64,238,62,
    124,255,131,253,221,95,238,126,216,60,58,9,6,126,243,213,
    39,31,55,71,99,213,139,194,230,112,191,25,132,74,162,138,
    6,205,75,202,217,195,9,203,180,205,105,208,117,2,22,208,
    233,201,193,72,198,243,52,186,73,44,136,69,81,19,101,97,
    138,134,152,199,94,9,31,83,108,27,115,226,48,32,17,61,
    18,155,176,101,21,209,68,38,22,112,108,64,188,77,88,233,
    227,79,144,113,17,49,45,250,102,240,183,223,147,110,244,104,
    223,36,4,232,193,51,198,23,2,13,103,62,33,147,135,192,
    32,41,65,191,12,26,60,136,57,141,166,120,76,45,78,39,
    50,6,18,183,32,249,59,160,174,17,54,103,144,66,234,194,
    4,17,46,130,170,146,87,227,232,58,110,248,103,70,101,171,
    65,236,31,50,62,84,47,72,162,211,144,173,64,125,246,163,
    22,106,230,217,248,171,163,190,244,84,178,131,3,223,70,39,
    117,207,13,195,72,213,93,223,175,187,74,197,193,209,137,146,
    73,93,69,245,251,73,131,12,107,47,101,16,203,233,141,71,
    25,164,200,252,8,41,253,226,7,158,194,151,21,126,97,43,
    36,82,33,60,122,145,159,224,56,145,232,74,101,19,147,138,
    148,28,49,35,140,30,135,166,210,246,56,143,96,255,89,198,
    9,67,180,81,206,0,149,200,65,71,85,25,155,110,146,56,
    204,9,141,51,12,137,240,75,119,112,34,153,58,2,73,33,
    67,212,213,60,76,21,136,183,73,168,76,7,44,88,24,133,
    254,24,249,12,188,247,136,133,219,12,199,26,3,114,13,193,
    56,131,109,25,255,47,139,117,195,179,82,8,150,51,24,82,
    80,84,192,32,16,41,14,16,146,23,24,128,26,6,71,16,
    150,141,189,244,199,212,163,197,246,54,53,119,169,185,71,205,
    78,38,254,180,116,48,127,85,7,143,104,95,131,5,103,17,
    201,96,102,38,162,127,201,211,238,76,60,13,3,102,139,60,
    198,32,191,154,120,140,69,193,53,126,74,45,78,101,95,52,
    33,249,154,66,57,121,22,19,35,39,66,119,160,222,196,73,
    88,97,246,34,41,98,54,195,183,77,160,45,34,183,91,64,
    174,77,182,98,216,218,119,178,88,233,208,12,13,88,123,139,
    72,149,110,208,120,157,154,31,77,91,237,19,232,117,175,65,
    239,49,177,176,152,66,111,158,33,87,197,103,209,240,204,212,
    22,121,30,93,185,2,57,194,155,117,3,222,126,70,61,243,
    186,244,239,8,106,169,204,191,41,64,141,216,52,138,162,29,
    98,103,188,65,18,21,65,182,129,197,193,243,112,3,243,189,
    193,249,254,125,206,247,92,51,112,229,164,3,185,201,177,92,
    119,74,164,154,142,9,235,105,30,79,42,216,142,226,232,213,
    184,30,117,234,138,101,167,184,251,228,126,178,119,63,121,140,
    17,181,254,148,99,153,142,169,58,106,198,114,68,81,143,150,
    126,241,202,147,156,65,249,205,113,116,144,115,56,224,57,105,
    102,70,188,173,145,98,141,76,227,28,238,19,21,83,148,159,
    170,206,171,185,206,73,132,47,105,211,42,43,220,20,27,136,
    173,170,96,206,28,29,229,185,78,227,175,248,28,144,17,72,
    122,9,84,85,219,45,205,55,139,68,194,217,191,184,132,159,
    41,9,100,55,113,135,111,50,220,148,39,184,161,199,204,92,
    226,175,192,149,172,128,191,0,33,3,1,144,186,68,238,65,
    4,133,21,154,254,71,96,223,185,161,118,224,120,212,162,122,
    129,103,96,152,74,30,241,84,93,74,124,9,127,43,56,94,
    150,240,205,180,86,45,38,124,43,143,101,12,169,183,74,234,
    214,229,160,71,70,234,185,9,77,211,145,108,226,203,147,180,
    145,151,153,24,201,167,133,175,89,189,157,67,156,125,63,65,
    23,165,204,45,177,98,20,48,243,1,53,15,115,184,136,108,
    108,10,76,238,192,155,243,188,163,51,200,119,196,137,197,188,
    47,204,176,186,211,245,185,79,148,50,159,120,152,251,132,228,
    60,247,154,143,50,212,26,100,252,11,67,224,153,18,203,62,
    58,194,89,32,75,208,46,147,247,112,177,46,82,231,18,89,
    148,163,152,120,41,137,178,98,14,181,202,114,251,107,211,82,
    243,106,170,209,131,172,251,100,224,14,143,124,247,233,49,109,
    73,251,122,153,187,25,153,16,139,69,33,200,85,196,155,228,
    224,215,253,76,152,151,83,141,28,31,227,14,185,16,236,39,
    126,228,113,184,248,186,39,235,67,57,60,194,19,108,47,24,
    213,59,3,183,203,150,50,83,33,191,202,132,84,108,234,171,
    101,74,242,128,218,168,238,69,33,134,248,19,79,69,113,221,
    151,120,180,147,126,125,183,206,249,161,30,36,117,247,8,191,
    186,158,210,208,191,236,197,92,31,187,113,55,225,82,248,248,
    148,186,83,183,180,131,103,247,0,15,7,67,200,211,178,62,
    88,230,225,94,159,118,217,147,48,131,226,161,77,141,117,92,
    163,146,197,222,163,230,231,240,46,178,194,71,192,240,131,132,
    212,87,22,91,70,197,224,114,35,157,242,140,230,39,215,93,
    249,244,109,92,89,95,12,165,14,93,166,153,114,134,238,15,
    168,173,80,90,104,87,179,193,57,110,107,60,56,159,221,61,
    221,226,193,5,190,207,41,243,200,18,197,131,153,255,53,30,
    176,31,77,221,131,212,255,53,12,216,143,222,169,12,246,39,
    144,86,12,111,10,1,162,40,224,188,94,219,23,217,97,166,
    40,29,223,167,172,94,197,156,227,197,210,85,82,27,108,123,
    202,194,114,48,209,12,156,78,124,250,122,169,253,89,46,215,
    5,215,75,227,85,182,163,62,203,177,29,197,243,112,19,107,
    110,139,107,238,39,84,115,159,177,18,28,67,151,221,19,160,
    150,114,93,144,185,67,121,234,92,210,135,46,170,137,47,119,
    52,146,161,111,63,128,98,157,204,159,167,138,135,199,26,180,
    147,194,197,20,171,88,24,95,247,76,138,208,5,57,217,160,
    165,220,23,167,109,90,198,241,69,134,227,6,93,169,77,194,
    180,253,132,26,14,204,121,76,182,127,157,27,230,222,53,144,
    14,232,237,115,21,31,4,42,161,195,217,127,155,130,101,20,
    223,85,21,7,213,79,111,94,243,44,150,116,231,20,197,173,
    224,7,201,196,223,106,34,109,177,156,109,113,233,19,155,140,
    61,205,151,3,169,228,101,128,41,18,59,189,31,240,37,102,
    220,104,140,135,51,62,228,224,251,192,113,166,159,156,126,149,
    154,137,3,36,38,39,81,198,244,180,38,214,140,74,185,34,
    184,6,184,114,95,174,185,35,27,232,114,126,156,216,28,173,
    22,114,19,242,205,110,150,129,201,218,124,24,61,116,135,250,
    46,142,239,151,236,159,64,122,242,183,223,203,161,64,119,35,
    124,134,210,167,87,244,79,174,79,184,28,177,63,164,113,186,
    10,25,238,239,101,226,236,105,113,14,98,55,244,122,185,29,
    248,134,121,184,175,182,110,156,220,10,134,250,254,146,99,64,
    241,187,31,187,216,95,187,50,154,200,56,112,7,104,90,150,
    43,27,230,105,55,239,78,188,31,208,235,55,88,176,112,242,
    47,70,94,54,126,44,187,65,130,132,152,202,149,245,105,40,
    34,187,168,205,107,96,44,174,158,58,88,116,201,173,111,17,
    158,210,29,86,242,41,54,116,255,88,89,168,32,112,40,62,
    153,120,108,159,23,150,89,91,172,88,181,185,138,85,153,49,
    249,138,104,30,15,92,85,171,50,87,19,250,223,14,194,171,
    106,236,44,85,196,127,0,88,117,7,223,
};

EmbeddedPython embedded_m5_internal_param_LocalBP(
    "m5/internal/param_LocalBP.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_LocalBP.py",
    "m5.internal.param_LocalBP",
    data_m5_internal_param_LocalBP,
    2267,
    6968);

} // anonymous namespace
