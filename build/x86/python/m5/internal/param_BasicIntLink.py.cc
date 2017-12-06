#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BasicIntLink[] = {
    120,156,197,88,253,114,219,198,17,223,3,64,74,164,40,75,
    178,44,201,182,100,11,77,199,13,235,169,196,196,137,226,116,
    226,186,77,210,116,38,153,140,146,130,233,216,97,50,69,65,
    226,72,130,34,1,14,112,178,76,143,52,157,169,60,105,95,
    160,125,131,254,209,183,233,27,165,187,123,0,8,125,53,153,
    105,71,148,137,243,225,176,183,183,31,191,221,219,187,14,164,
    127,37,124,126,99,3,36,255,16,0,62,254,4,12,1,70,
    2,90,2,132,20,224,223,130,131,18,196,239,130,95,130,215,
    0,45,3,164,1,167,216,49,225,27,3,194,26,207,41,195,
    208,228,17,1,147,42,72,11,90,37,120,22,174,128,37,203,
    112,80,133,248,79,32,132,8,5,60,247,231,192,159,135,215,
    200,29,59,21,102,56,15,52,88,229,193,10,248,11,60,88,
    5,191,198,157,5,152,44,131,172,65,107,145,200,90,55,144,
    237,67,100,187,196,108,255,77,108,125,252,178,6,254,13,34,
    71,185,190,38,74,139,40,121,189,37,230,178,156,73,185,2,
    173,155,89,127,181,208,191,85,232,175,21,250,235,133,254,70,
    161,127,155,251,40,217,77,24,220,129,193,93,24,108,66,23,
    141,181,146,75,177,5,210,132,193,61,104,221,3,137,191,45,
    56,69,123,250,55,11,51,238,243,140,213,124,198,54,207,176,
    161,101,131,196,223,182,158,81,134,102,125,29,125,20,124,143,
    127,117,244,17,168,26,54,47,100,156,4,81,232,6,97,55,
    10,12,250,94,166,134,60,218,161,102,46,117,237,199,228,218,
    127,1,251,213,55,82,215,158,0,50,22,164,203,208,128,19,
    238,156,24,48,169,195,177,128,129,5,190,9,199,184,76,137,
    4,232,9,56,53,224,91,147,8,78,176,181,208,1,247,193,
    82,218,175,3,118,128,230,52,7,39,37,56,46,65,243,249,
    177,65,3,7,21,136,255,9,175,182,152,233,60,51,53,224,
    24,91,11,78,45,56,41,195,51,36,194,161,65,133,212,23,
    207,143,81,83,28,105,214,45,148,118,191,160,46,169,226,7,
    113,232,141,164,90,197,190,59,246,98,111,228,126,228,37,65,
    231,211,80,125,30,132,7,245,106,70,24,37,187,99,79,245,
    29,158,105,146,73,70,99,197,28,163,80,170,5,236,116,131,
    208,119,71,145,127,56,148,106,158,216,185,221,96,40,93,151,
    63,126,58,26,71,177,250,36,142,163,216,33,171,242,224,48,
    242,242,25,100,211,206,48,74,100,157,86,227,101,28,98,175,
    136,186,59,102,142,36,0,75,75,147,125,153,116,226,96,172,
    208,89,154,35,81,19,183,58,185,137,155,164,141,77,99,20,
    170,70,191,215,77,26,205,190,23,203,102,95,134,141,158,28,
    237,237,68,113,208,11,194,157,68,121,237,161,220,121,244,214,
    219,123,59,191,220,121,167,209,62,12,134,126,227,229,251,239,
    53,198,19,213,143,194,198,104,175,17,132,74,162,157,134,141,
    139,22,218,69,170,155,180,214,81,208,115,3,214,210,237,203,
    225,88,198,139,52,122,151,228,16,203,162,38,202,194,20,117,
    177,136,189,18,62,166,216,50,22,196,126,64,122,118,72,119,
    66,153,85,196,21,57,91,192,129,1,241,22,161,102,128,63,
    65,110,70,236,52,233,155,193,223,126,79,6,210,163,3,147,
    176,160,7,143,25,105,8,57,164,124,66,206,15,129,225,82,
    130,65,25,52,140,16,125,26,87,241,132,90,36,39,54,6,
    50,183,32,249,59,160,193,17,64,199,144,130,235,212,4,17,
    46,131,170,82,14,192,209,117,92,240,47,140,207,102,157,196,
    223,103,144,168,126,144,68,71,33,187,130,250,28,81,77,180,
    204,151,147,47,218,3,217,81,201,54,14,124,29,29,218,29,
    47,12,35,101,123,190,111,123,74,197,65,251,80,201,196,86,
    145,253,32,169,147,119,157,149,12,103,57,191,201,56,195,21,
    97,0,113,165,95,252,160,163,240,133,1,236,178,23,18,169,
    16,35,253,200,79,112,156,88,244,164,114,72,72,69,70,142,
    88,16,134,144,75,164,180,60,210,221,192,247,15,51,73,24,
    167,245,114,134,170,68,14,187,170,202,0,245,146,196,101,73,
    104,156,177,72,140,95,120,195,67,201,220,17,77,10,5,162,
    174,150,225,250,209,120,155,52,203,12,193,218,133,81,232,79,
    80,216,160,243,38,201,113,155,49,89,99,84,174,33,34,231,
    176,45,227,255,101,177,110,116,172,20,135,229,12,139,148,35,
    21,48,18,68,10,6,196,229,41,230,163,186,193,9,133,21,
    228,120,125,131,122,52,217,217,162,230,30,53,247,169,217,206,
    108,112,173,134,88,60,111,136,199,180,184,193,218,179,158,228,
    58,51,211,211,63,19,115,119,166,49,135,73,180,73,177,99,
    80,132,77,99,199,162,132,27,63,165,22,73,57,42,77,72,
    190,162,244,78,49,198,204,40,156,48,48,168,55,13,23,182,
    154,179,76,214,152,207,144,238,16,124,139,24,238,21,48,236,
    144,195,24,192,206,157,44,117,186,68,161,161,235,108,18,171,
    210,37,102,183,169,249,201,76,108,63,5,97,239,2,8,63,
    32,57,150,83,16,46,50,248,170,248,44,27,29,51,117,72,
    190,193,174,158,3,31,33,207,186,4,121,63,163,158,121,209,
    4,179,4,93,170,248,239,10,160,35,89,141,162,126,251,216,
    153,108,144,90,69,184,109,96,233,240,44,220,192,106,192,224,
    106,224,45,174,6,184,162,224,218,75,39,119,147,243,187,238,
    148,200,62,93,19,214,211,93,62,169,96,59,142,163,151,19,
    59,234,218,138,13,64,185,248,201,131,100,247,65,242,1,102,
    89,251,41,231,55,157,103,117,38,141,229,152,50,33,77,253,
    228,101,71,242,214,202,111,174,171,19,159,203,73,208,77,183,
    108,68,222,26,89,215,200,204,206,91,64,162,98,202,252,215,
    111,248,106,110,120,210,227,51,90,185,202,86,55,197,6,162,
    172,42,88,60,87,167,127,46,229,248,43,62,31,145,39,200,
    4,18,168,56,119,154,90,120,214,139,52,116,126,113,6,73,
    215,169,149,211,192,101,254,144,33,168,60,69,16,61,102,22,
    33,127,5,174,120,5,124,7,132,17,132,66,26,33,121,64,
    17,40,86,137,252,143,192,161,116,73,101,193,57,170,73,213,
    4,83,96,234,74,30,51,169,46,52,62,131,191,21,226,48,
    43,7,204,180,166,45,150,3,86,158,223,24,92,63,106,203,
    183,206,38,66,242,84,223,75,136,76,103,183,105,104,79,247,
    147,188,18,197,236,126,173,72,155,215,107,186,36,222,183,83,
    156,209,134,186,41,86,141,2,122,222,166,230,81,14,28,145,
    141,93,151,164,219,112,117,41,224,234,253,229,27,18,199,98,
    5,150,230,184,94,43,50,201,227,164,148,197,201,163,60,78,
    36,111,133,175,249,4,68,173,65,88,56,53,4,30,87,177,
    70,164,211,161,5,178,4,173,50,69,20,151,247,34,13,56,
    145,165,63,74,150,103,246,89,54,209,190,54,94,14,7,237,
    105,106,94,94,127,90,33,103,63,25,122,163,182,239,61,13,
    105,93,90,188,147,133,160,145,105,178,92,212,132,194,71,92,
    165,12,191,238,101,26,189,184,254,148,242,30,46,147,107,194,
    1,228,71,29,206,35,95,245,165,61,146,163,54,30,129,251,
    193,216,238,14,189,30,251,204,76,53,253,34,211,84,177,211,
    207,215,52,201,67,106,35,187,19,133,184,11,28,118,84,20,
    219,190,196,99,161,244,237,29,155,183,16,59,72,108,175,141,
    95,189,142,210,225,112,54,188,185,172,246,226,94,194,21,244,
    193,17,117,103,227,115,215,13,194,0,15,22,99,200,183,111,
    125,50,205,119,4,62,50,232,232,194,157,22,15,124,106,162,
    179,30,213,55,206,46,53,63,135,153,109,28,239,226,50,35,
    90,143,12,89,22,155,70,197,224,115,106,145,238,75,154,153,
    92,140,241,163,31,19,227,250,50,42,141,244,50,81,202,57,
    186,143,160,182,66,219,71,171,154,13,46,112,91,227,193,197,
    236,190,235,6,15,46,241,29,82,153,71,86,40,81,204,253,
    175,137,130,99,107,54,81,117,244,127,205,15,206,227,217,43,
    226,188,15,105,141,113,85,110,16,69,45,23,117,110,24,136,
    236,72,84,84,145,47,105,238,92,10,65,183,19,75,79,73,
    237,191,173,89,168,205,249,70,75,241,106,26,241,23,11,246,
    15,115,13,79,185,214,154,220,98,183,234,179,33,187,85,60,
    11,239,98,229,110,113,229,254,132,42,247,99,54,135,107,232,
    226,125,10,222,82,110,21,58,99,135,242,200,189,104,25,93,
    159,147,112,222,120,44,67,223,121,8,197,146,155,63,95,63,
    70,40,195,253,25,10,149,143,41,110,97,141,125,49,110,41,
    157,23,52,102,255,150,242,72,157,137,167,25,224,223,101,0,
    175,83,78,156,230,116,231,9,53,156,197,243,4,238,252,58,
    247,211,189,203,209,27,70,190,116,61,58,240,253,0,5,86,
    97,236,75,253,250,95,137,219,63,200,174,125,150,93,155,189,
    194,1,230,203,161,84,242,18,52,41,210,44,189,97,240,37,
    110,195,209,4,15,117,124,46,194,247,161,235,206,104,179,250,
    21,46,243,2,210,52,137,155,149,40,227,118,181,38,214,140,
    74,185,34,184,58,56,119,21,175,69,164,171,13,125,2,152,
    36,14,167,171,165,220,85,124,95,156,109,203,228,85,62,201,
    238,123,35,125,185,199,119,85,206,79,33,189,59,112,222,204,
    93,78,6,228,99,151,62,250,98,88,114,229,194,133,138,243,
    78,134,130,209,222,110,166,211,110,65,39,39,194,67,77,204,
    119,214,163,61,46,191,47,18,126,60,140,58,7,210,215,23,
    162,87,48,99,154,223,70,35,15,199,55,47,165,104,6,163,
    148,195,202,185,239,126,76,179,214,206,141,38,50,14,188,97,
    240,74,94,193,143,197,39,127,176,229,178,143,138,12,122,158,
    160,146,129,146,222,184,208,184,144,214,25,98,177,236,5,9,
    153,99,169,56,33,205,107,228,114,54,208,37,240,46,78,158,
    13,34,117,217,175,239,57,158,210,125,91,66,215,56,116,97,
    90,89,170,32,58,41,227,153,162,138,57,207,50,107,203,21,
    171,182,80,177,42,115,38,223,100,45,226,25,176,106,85,22,
    106,162,248,111,27,145,92,53,182,87,42,226,63,15,41,88,
    29,
};

EmbeddedPython embedded_m5_internal_param_BasicIntLink(
    "m5/internal/param_BasicIntLink.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_BasicIntLink.py",
    "m5.internal.param_BasicIntLink",
    data_m5_internal_param_BasicIntLink,
    2289,
    7202);

} // anonymous namespace
