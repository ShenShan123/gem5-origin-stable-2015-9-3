#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_SimpleNetwork[] = {
    120,156,205,89,109,115,219,198,17,222,3,64,74,164,36,235,
    221,242,139,98,209,73,29,51,174,37,58,78,20,167,19,215,
    173,227,186,51,201,52,74,10,166,181,195,100,138,66,196,145,
    4,69,2,28,224,100,153,25,169,211,169,60,73,39,211,15,
    253,210,233,47,232,135,252,155,252,163,118,119,15,128,32,137,
    156,113,166,29,179,18,121,90,28,246,246,110,119,159,221,219,
    59,53,33,249,41,224,247,151,21,128,248,247,6,128,135,31,
    1,61,128,190,128,134,0,33,5,120,43,176,87,128,232,93,
    240,10,240,2,160,97,128,52,224,24,9,19,190,52,32,152,
    229,49,69,232,153,220,35,96,88,6,105,65,163,0,79,130,
    69,176,100,17,246,202,16,253,17,132,16,129,128,167,222,20,
    120,211,240,2,165,35,81,98,129,211,64,157,101,238,44,129,
    55,195,157,101,240,102,153,152,129,225,2,200,89,104,204,17,
    91,227,2,138,189,133,98,231,89,236,15,36,214,195,55,171,
    224,93,32,118,92,215,23,196,105,17,39,207,55,207,82,22,
    210,85,46,66,99,41,165,151,115,244,74,142,94,205,209,23,
    115,244,90,142,190,148,163,47,231,232,43,57,250,106,142,94,
    207,209,175,229,232,107,57,122,35,71,87,114,244,245,28,253,
    122,142,126,35,71,255,36,71,223,200,209,111,230,232,155,57,
    186,202,52,90,118,9,186,111,65,247,22,116,127,10,45,116,
    246,98,102,197,219,32,77,232,110,66,99,19,36,126,110,195,
    49,226,193,91,202,141,216,226,17,203,217,136,26,143,184,3,
    141,59,32,241,83,211,35,138,80,175,94,68,140,249,255,198,
    159,170,64,74,205,98,243,76,70,177,31,6,142,31,180,66,
    223,160,247,69,106,8,145,77,106,166,18,104,62,34,104,126,
    15,140,75,207,72,160,121,4,40,88,144,46,61,3,142,152,
    56,50,96,88,133,67,1,93,11,60,19,14,113,154,2,45,
    160,45,224,216,128,175,76,98,56,194,214,66,0,93,3,75,
    105,92,118,25,64,90,210,20,28,21,224,176,0,245,167,135,
    6,117,236,149,32,250,23,124,189,206,66,167,89,168,1,135,
    216,90,112,108,193,81,17,158,32,19,118,117,75,164,190,120,
    122,136,154,98,79,189,106,225,106,119,114,234,146,42,158,31,
    5,110,95,170,21,164,157,129,27,185,125,167,238,247,7,61,
    185,35,213,65,24,237,85,203,41,103,24,111,13,92,213,177,
    121,168,73,54,233,15,20,139,12,3,169,102,144,104,249,129,
    231,244,67,111,191,39,213,52,201,115,90,126,79,58,14,191,
    252,168,63,8,35,245,56,138,194,200,38,179,114,103,47,116,
    179,17,100,212,102,47,140,101,149,102,227,105,108,18,175,136,
    187,53,96,137,180,0,94,46,13,246,100,220,140,252,129,66,
    111,105,137,196,77,210,170,228,39,110,98,114,89,173,31,168,
    90,167,221,138,107,245,142,27,201,122,71,6,181,182,236,111,
    111,134,145,223,246,131,205,88,185,187,61,185,121,247,206,219,
    219,155,63,219,124,167,182,187,239,247,188,218,243,247,223,171,
    13,134,170,19,6,181,254,118,205,15,148,68,67,245,106,35,
    76,180,133,108,75,52,217,129,223,118,124,86,211,233,200,222,
    64,70,115,212,123,133,22,34,22,196,172,40,10,83,84,197,
    28,82,5,252,154,98,221,152,17,59,62,41,218,36,229,9,
    103,86,30,89,228,110,1,123,6,68,235,132,155,46,126,4,
    57,26,209,83,167,119,6,191,251,45,89,72,247,118,77,66,
    131,238,60,100,172,33,232,144,243,62,185,63,0,6,76,1,
    186,69,208,64,66,252,105,100,69,67,106,145,157,196,24,40,
    220,130,248,31,128,22,71,8,29,66,2,175,99,19,68,176,
    0,170,76,89,12,123,47,226,132,127,97,132,214,171,180,252,
    29,70,137,234,248,113,120,16,176,47,136,230,152,170,163,101,
    62,27,126,186,219,149,77,21,111,96,199,23,225,126,165,233,
    6,65,168,42,174,231,85,92,165,34,127,119,95,201,184,162,
    194,202,141,184,74,238,181,23,83,160,101,242,134,131,20,88,
    4,2,4,150,126,240,252,166,194,135,101,126,96,47,196,82,
    33,72,58,161,23,99,63,137,104,75,101,211,34,21,25,57,
    228,133,48,134,28,98,165,233,145,239,2,62,63,76,87,194,
    64,173,22,83,88,197,178,215,82,101,70,168,27,199,14,175,
    132,250,25,140,36,248,153,219,219,151,44,29,225,164,112,65,
    68,234,53,76,0,142,151,72,181,212,18,172,94,16,6,222,
    16,87,235,55,111,210,66,46,49,40,103,25,150,171,8,201,
    41,108,139,248,183,40,46,26,77,43,1,98,49,5,35,165,
    73,5,12,5,145,160,1,129,121,140,41,169,106,112,78,97,
    13,57,98,95,39,138,6,219,235,212,188,70,205,53,106,54,
    82,35,188,90,75,204,157,181,196,61,154,221,96,245,89,81,
    114,158,153,42,234,157,138,186,203,39,81,135,137,180,78,209,
    99,80,140,157,68,143,69,73,55,122,64,45,178,114,92,154,
    16,127,78,41,158,162,140,133,81,64,97,104,16,117,18,48,
    108,54,123,129,204,49,157,98,221,38,0,231,81,220,206,161,
    216,38,143,49,132,237,203,105,246,116,136,67,131,215,190,74,
    162,10,35,236,94,161,230,250,100,140,127,2,195,246,57,24,
    126,64,11,89,72,96,56,199,240,43,227,119,193,104,154,137,
    71,178,93,118,249,12,252,8,123,214,8,236,189,73,148,121,
    222,6,19,133,93,162,249,175,115,176,163,197,26,121,5,119,
    144,24,174,145,94,121,192,173,97,1,241,36,88,195,154,192,
    224,154,224,14,215,4,92,87,112,5,169,19,188,201,57,94,
    19,5,50,80,203,132,139,201,94,31,151,176,29,68,225,243,
    97,37,108,85,20,91,128,242,241,253,27,241,214,141,248,3,
    204,180,149,7,156,227,116,174,213,217,52,146,3,202,134,52,
    244,241,243,166,228,253,149,159,28,71,39,63,135,19,161,147,
    236,219,136,189,85,50,175,145,218,157,183,129,88,69,148,253,
    39,96,249,114,102,121,82,228,99,154,186,204,102,55,197,26,
    226,172,44,120,125,142,222,3,184,162,227,183,248,253,144,92,
    65,54,144,64,103,12,187,174,87,207,138,145,138,246,237,83,
    88,122,165,106,217,53,156,231,119,41,134,138,39,24,162,175,
    153,6,201,183,192,149,175,128,111,128,80,130,96,72,130,36,
    139,41,130,197,50,177,255,1,56,154,70,212,23,156,167,234,
    84,83,48,7,166,175,248,30,179,234,114,227,99,248,107,46,
    20,211,162,192,76,106,219,124,81,96,101,57,142,225,245,82,
    27,191,117,58,25,146,171,58,110,76,108,58,195,157,68,247,
    201,166,146,21,164,152,225,95,45,214,166,245,164,14,173,239,
    171,19,164,209,182,122,85,44,27,57,252,188,77,205,221,12,
    58,34,237,123,101,75,221,128,241,21,129,163,119,153,47,105,
    61,22,107,48,63,197,25,236,148,148,44,86,10,105,172,220,
    205,98,69,242,142,248,130,15,67,212,26,4,135,99,67,224,
    201,27,139,69,58,232,90,32,11,208,40,82,84,113,161,47,
    146,160,19,105,14,164,249,78,109,183,108,164,29,109,190,12,
    17,218,217,212,60,159,64,110,33,127,223,239,185,253,93,207,
    125,240,39,154,152,102,111,166,97,104,164,170,44,228,85,161,
    16,18,227,180,225,199,237,84,165,103,19,200,43,239,225,60,
    153,42,28,69,94,216,228,100,242,121,71,86,250,178,191,139,
    231,225,142,63,168,180,122,110,155,189,102,38,170,126,154,170,
    170,216,237,103,139,155,248,22,181,97,165,25,6,184,25,236,
    55,85,24,85,60,137,71,68,233,85,54,43,188,147,84,252,
    184,226,238,226,91,183,169,116,72,156,142,113,174,176,221,168,
    29,115,49,189,119,64,228,132,188,238,56,126,224,227,33,227,
    207,144,109,227,250,152,154,109,12,124,124,208,17,134,59,46,
    30,254,212,80,231,62,42,116,236,45,106,222,130,201,237,31,
    239,226,60,71,52,33,153,178,40,174,26,37,131,75,205,83,
    140,159,209,216,248,124,160,255,237,101,2,93,95,174,37,225,
    94,36,78,57,69,247,19,212,150,104,27,105,148,211,206,25,
    110,103,185,115,46,237,188,192,237,60,119,46,164,151,122,139,
    220,185,4,141,101,186,29,162,158,21,74,33,83,255,109,10,
    225,160,155,80,184,125,251,63,205,28,246,189,255,3,77,236,
    247,33,41,65,198,101,13,145,87,115,78,103,141,174,72,79,
    77,121,29,249,42,231,202,104,104,58,205,72,186,74,106,23,
    174,79,68,113,206,69,122,25,223,157,36,131,243,53,253,195,
    76,199,99,46,198,134,43,236,89,125,128,100,207,138,39,193,
    21,44,238,45,46,238,239,83,113,127,200,6,113,12,93,223,
    159,0,184,144,217,101,13,155,64,30,56,35,108,163,107,120,
    90,157,59,24,200,192,179,111,65,190,44,231,215,19,192,9,
    165,191,191,67,174,54,50,197,10,214,225,231,163,151,146,125,
    78,103,118,113,33,139,215,201,56,155,81,254,207,20,229,85,
    78,246,89,198,183,239,83,195,57,62,75,239,246,47,50,87,
    85,199,64,216,245,92,60,83,61,147,78,20,238,43,63,224,
    19,226,203,51,99,245,166,104,202,179,47,212,27,99,36,236,
    238,183,90,50,114,98,255,107,201,51,189,12,31,77,66,23,
    12,185,62,117,107,204,56,68,218,32,68,51,58,187,110,224,
    29,248,158,234,240,52,63,130,157,102,163,251,211,243,175,24,
    60,156,11,60,217,147,74,142,130,61,91,35,185,48,241,36,
    22,19,225,16,79,168,124,198,195,231,158,227,76,106,199,253,
    57,206,243,13,77,72,186,225,142,43,138,184,231,174,10,252,
    53,74,197,146,224,50,231,204,63,24,244,42,233,92,160,207,
    51,195,216,230,236,58,159,129,138,47,193,211,242,130,240,199,
    39,243,29,183,175,47,44,249,250,205,38,23,243,101,136,125,
    51,3,39,93,26,241,33,82,31,229,49,135,112,9,198,21,
    151,253,14,245,211,117,73,127,123,43,85,107,75,171,101,239,
    239,14,19,165,248,34,190,191,205,71,137,60,227,135,110,236,
    55,31,63,87,191,241,131,61,231,153,164,74,79,93,27,41,
    44,207,169,42,99,231,123,20,6,232,198,94,15,149,92,31,
    203,84,31,198,74,246,199,76,164,157,241,137,236,135,209,112,
    204,68,15,147,194,51,97,186,58,146,9,95,234,91,235,115,
    74,235,247,143,122,97,115,79,122,9,207,104,11,50,207,175,
    194,190,139,253,163,103,193,213,38,18,22,207,188,247,34,26,
    181,122,166,55,150,145,239,246,40,40,47,141,150,167,77,115,
    118,50,25,236,179,70,168,238,39,161,39,207,217,246,161,231,
    69,182,27,180,101,234,196,235,103,25,78,153,44,229,26,173,
    53,187,218,198,220,132,78,28,173,53,115,48,18,70,2,234,
    163,224,101,1,149,112,158,243,243,41,231,140,147,147,91,103,
    202,66,49,150,190,103,143,156,143,4,74,47,9,205,149,245,
    249,122,133,19,82,36,219,126,76,22,224,186,237,100,124,178,
    97,83,134,224,85,143,74,148,249,241,19,74,97,250,188,171,
    175,249,30,208,126,16,183,176,161,127,25,148,230,75,152,206,
    104,47,55,69,25,119,115,203,156,93,40,89,179,51,37,171,
    52,101,242,77,238,156,88,54,202,86,105,102,86,252,152,223,
    13,76,140,101,99,99,169,36,254,3,86,30,146,249,
};

EmbeddedPython embedded_m5_internal_param_SimpleNetwork(
    "m5/internal/param_SimpleNetwork.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_SimpleNetwork.py",
    "m5.internal.param_SimpleNetwork",
    data_m5_internal_param_SimpleNetwork,
    2542,
    8199);

} // anonymous namespace
