#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BaseSimpleCPU[] = {
    120,156,205,88,109,111,27,199,17,158,189,59,82,34,37,217,
    178,101,89,126,145,45,198,137,99,198,141,68,219,137,108,23,
    113,141,202,78,10,56,72,20,245,232,192,182,18,244,122,186,
    91,145,71,29,239,136,187,149,109,6,18,90,68,70,155,126,
    232,183,244,39,244,67,255,77,255,81,59,51,247,194,35,69,
    1,46,90,152,149,201,245,112,119,118,118,103,230,153,217,217,
    117,32,253,43,225,247,215,53,128,56,208,0,92,252,8,240,
    1,186,2,182,5,8,41,192,61,7,123,37,136,62,5,183,
    4,111,0,182,53,144,26,28,33,161,195,119,26,4,179,60,
    167,12,190,206,61,2,250,85,144,6,108,151,224,89,112,6,
    12,89,134,189,42,68,191,7,33,68,32,224,185,59,5,238,
    52,188,65,233,72,84,88,224,52,80,103,149,59,43,224,206,
    112,103,21,220,89,38,102,160,63,15,114,22,182,231,136,109,
    251,20,138,189,137,98,79,179,216,127,146,88,23,71,22,193,
    61,69,236,184,175,23,196,105,16,39,175,119,154,165,204,103,
    187,60,3,219,103,51,122,161,64,159,43,208,139,5,250,124,
    129,94,42,208,23,10,244,197,2,125,169,64,95,46,208,203,
    5,250,74,129,190,90,160,87,10,116,173,64,191,87,160,175,
    21,232,247,11,244,7,5,250,122,129,254,176,64,223,40,208,
    245,2,253,81,129,190,201,52,90,252,44,116,126,1,157,143,
    161,179,10,187,8,130,51,185,117,215,64,234,208,105,192,118,
    3,36,126,214,224,8,113,226,158,45,204,184,197,51,22,242,
    25,183,121,198,29,216,190,3,18,63,183,147,25,101,104,214,
    207,35,246,188,127,225,95,93,32,165,102,177,121,41,163,216,
    11,3,203,11,118,67,79,163,241,50,53,132,84,135,154,169,
    20,178,143,9,178,255,0,198,171,171,165,144,61,4,20,44,
    72,23,95,131,67,38,14,53,232,215,225,64,64,199,0,87,
    135,3,92,166,68,27,104,9,56,210,224,123,157,24,14,177,
    53,16,88,87,193,80,9,94,59,12,172,68,210,20,28,150,
    224,160,4,205,231,7,26,117,236,85,32,250,59,252,176,204,
    66,167,89,168,6,7,216,26,112,100,192,97,25,158,33,19,
    118,117,42,164,190,120,126,128,154,98,79,179,110,224,110,55,
    11,234,146,42,174,23,5,118,87,170,115,72,91,61,59,178,
    187,214,35,59,150,77,175,219,243,229,227,173,111,235,213,140,
    51,140,215,122,182,106,155,60,85,39,155,116,123,138,69,134,
    129,84,51,72,236,122,129,107,117,67,119,223,151,106,154,228,
    89,187,158,47,45,139,7,159,116,123,97,164,190,136,162,48,
    50,201,172,220,233,135,118,62,131,140,234,248,97,44,235,180,
    26,47,99,146,120,69,220,187,61,150,72,27,224,237,210,100,
    87,198,78,228,245,20,122,43,145,72,220,36,173,78,126,226,
    38,38,151,53,186,129,106,180,91,187,113,163,217,182,35,217,
    108,203,160,209,146,221,245,213,48,242,90,94,176,26,43,123,
    199,151,171,119,110,221,94,95,253,229,234,39,141,157,125,207,
    119,27,175,239,223,109,244,250,170,29,6,141,238,122,195,11,
    148,68,67,249,141,49,38,90,67,182,179,180,216,43,175,101,
    121,172,166,213,150,126,79,70,115,212,123,137,54,34,230,197,
    172,40,11,93,212,197,28,82,37,252,234,98,89,155,17,155,
    30,41,234,144,242,132,51,163,136,44,114,183,128,61,13,162,
    101,194,77,7,63,130,28,141,232,105,210,152,198,99,191,37,
    11,37,189,29,157,208,144,116,30,48,214,16,116,200,249,128,
    220,31,0,3,166,4,157,50,36,64,66,252,37,200,138,250,
    212,34,59,137,209,80,184,1,241,223,0,45,142,16,58,128,
    20,94,71,58,136,96,30,84,149,178,27,246,158,199,5,127,
    100,132,54,235,180,253,77,70,137,106,123,113,248,42,96,95,
    16,205,49,213,68,203,108,245,191,217,233,72,71,197,43,216,
    241,34,220,175,57,118,16,132,170,102,187,110,205,86,42,242,
    118,246,149,140,107,42,172,93,143,235,228,94,243,76,6,180,
    92,94,191,151,1,139,64,128,192,74,126,184,158,163,240,199,
    2,255,96,47,196,82,33,72,218,161,27,99,63,137,104,73,
    101,210,38,21,25,57,228,141,48,134,44,98,165,229,145,239,
    20,254,222,200,118,194,64,173,151,51,88,197,210,223,85,85,
    70,168,29,199,22,239,132,250,25,140,36,248,165,237,239,75,
    150,142,112,82,184,33,34,147,61,76,0,142,23,72,181,204,
    18,172,94,16,6,110,31,119,235,57,55,104,35,23,24,148,
    179,12,203,69,132,228,20,182,101,252,191,44,206,107,142,145,
    2,177,156,129,145,210,164,2,134,130,72,209,128,192,60,194,
    148,84,215,56,167,176,134,28,177,215,136,162,201,230,50,53,
    87,168,185,74,205,74,102,132,119,107,137,185,81,75,220,163,
    213,53,86,159,21,37,231,233,153,162,238,80,212,93,28,68,
    29,38,210,38,69,143,70,49,54,136,30,131,146,110,244,144,
    90,100,229,184,212,33,126,74,41,158,162,140,133,81,64,97,
    104,16,53,8,24,54,155,57,79,230,152,206,176,110,18,128,
    139,40,110,21,80,108,146,199,24,194,230,197,44,123,90,196,
    145,128,215,188,76,162,74,99,236,94,163,230,189,201,24,127,
    0,195,214,49,24,126,70,27,153,79,97,56,199,240,171,226,
    119,94,115,244,212,35,249,41,187,48,2,63,194,158,49,6,
    123,31,18,165,31,183,193,68,97,151,106,254,155,2,236,104,
    179,90,81,193,77,36,250,75,164,87,17,112,75,88,64,60,
    11,150,176,38,208,184,38,184,197,53,1,215,21,92,89,38,
    9,94,231,28,159,16,37,50,208,174,14,231,211,179,62,174,
    96,219,139,194,215,253,90,184,91,83,108,1,202,199,15,174,
    199,107,215,227,207,48,211,214,30,114,142,75,114,109,146,77,
    35,217,163,108,72,83,191,120,237,72,62,95,249,151,101,37,
    201,207,226,68,104,165,231,54,98,111,145,204,171,101,118,231,
    99,32,86,17,101,255,9,88,190,154,91,158,20,249,146,150,
    174,178,217,117,177,132,56,171,10,222,159,149,156,1,92,209,
    241,40,126,31,145,43,200,6,18,232,238,97,54,147,221,179,
    98,164,162,249,241,16,150,222,169,90,102,3,215,249,54,195,
    80,121,128,33,250,234,89,144,252,25,184,242,21,240,39,32,
    148,32,24,210,32,201,99,138,96,177,64,236,191,3,142,166,
    49,245,5,231,169,38,213,20,204,129,233,43,190,199,172,73,
    185,241,37,252,84,8,197,172,40,208,211,218,182,88,20,24,
    121,142,99,120,189,213,193,111,12,39,67,114,85,219,142,137,
    45,201,112,131,232,30,28,42,121,65,138,25,254,221,98,109,
    58,89,212,162,253,125,63,64,26,29,171,151,197,130,86,192,
    207,109,106,238,228,208,17,89,223,59,219,234,10,156,92,17,
    88,201,41,243,29,237,199,96,13,78,79,113,6,27,146,146,
    199,74,41,139,149,59,121,172,72,62,17,223,240,101,136,90,
    141,224,112,164,9,188,145,99,177,72,23,96,3,100,9,182,
    203,20,85,92,232,139,52,232,68,150,3,105,189,161,227,150,
    141,180,153,152,47,71,68,226,108,106,94,79,32,183,144,191,
    31,248,118,119,199,181,31,254,145,22,166,213,157,44,12,181,
    76,149,249,162,42,20,66,226,36,109,248,231,122,166,210,203,
    9,228,149,187,184,78,174,10,71,145,27,58,156,76,158,182,
    101,173,43,187,59,120,31,110,123,189,218,174,111,183,216,107,
    122,170,234,55,153,170,138,221,62,90,220,196,55,169,13,107,
    78,24,224,97,176,239,168,48,170,185,18,175,136,210,173,173,
    214,248,36,169,121,113,205,222,193,81,219,81,73,72,12,199,
    56,87,216,118,212,138,185,152,222,123,69,228,132,188,110,89,
    94,224,225,37,227,71,200,143,241,228,154,154,31,12,124,125,
    72,34,12,79,92,188,252,169,126,146,251,168,208,49,215,168,
    249,8,38,119,126,124,138,235,252,129,22,36,83,150,197,101,
    173,162,113,169,57,196,184,69,115,227,227,129,254,226,109,2,
    61,121,116,67,6,89,134,206,20,183,211,116,122,208,155,154,
    145,190,169,81,231,12,108,207,102,111,121,115,148,15,202,255,
    109,62,224,8,154,80,236,252,229,127,154,6,204,123,255,7,
    154,152,247,33,173,39,78,74,1,67,197,235,70,146,2,18,
    47,97,213,209,63,199,90,39,55,37,214,90,60,11,46,97,
    21,107,112,21,251,128,170,216,3,174,116,45,45,41,100,7,
    206,229,11,12,191,229,44,97,19,200,87,214,24,124,38,197,
    42,161,198,238,245,100,224,154,55,161,88,127,242,240,4,108,
    72,113,254,87,40,20,1,186,56,135,5,231,113,100,83,86,
    43,232,204,8,46,229,88,94,158,28,2,126,206,16,80,63,
    53,148,218,204,7,212,204,15,229,177,196,75,215,198,103,16,
    107,39,178,3,167,189,21,73,151,238,63,111,195,134,85,9,
    151,238,131,46,54,168,162,215,51,87,250,82,201,113,80,80,
    180,169,244,182,236,74,60,73,194,62,94,79,184,192,199,223,
    190,101,77,42,221,254,10,215,249,9,210,235,22,166,91,188,
    217,46,106,149,114,69,240,233,54,242,174,156,236,239,19,200,
    202,216,126,108,114,28,158,206,237,204,111,159,217,169,66,222,
    224,11,217,166,221,77,222,169,248,213,197,124,31,210,59,176,
    121,35,119,21,189,21,240,221,33,185,193,97,68,241,201,203,
    7,173,201,43,210,211,64,119,125,45,83,104,45,85,40,119,
    131,71,39,55,191,193,118,215,213,229,177,204,168,121,242,182,
    167,206,140,140,187,145,141,244,226,72,111,44,35,207,246,189,
    31,164,186,56,126,113,180,38,218,145,159,15,142,143,62,191,
    127,247,233,87,143,212,7,39,13,110,217,116,229,70,15,62,
    179,253,61,52,216,120,41,205,126,172,100,247,152,66,50,216,
    239,90,95,203,110,24,245,191,14,93,169,150,71,198,55,92,
    55,50,237,160,37,173,151,146,13,51,106,190,141,180,158,73,
    100,100,92,181,177,123,24,230,61,193,184,56,152,26,119,101,
    236,248,99,63,116,246,164,155,242,92,57,153,231,243,176,75,
    206,56,217,228,100,213,171,39,89,245,171,208,177,253,141,30,
    162,97,188,46,40,192,115,182,188,240,115,249,210,115,228,9,
    186,12,198,71,119,129,43,60,105,110,100,230,58,209,241,200,
    195,25,161,56,248,56,220,39,50,155,59,234,177,100,238,19,
    44,66,159,162,173,17,15,163,243,183,162,208,145,113,156,205,
    31,111,159,148,137,3,49,27,226,43,210,48,98,167,210,68,
    135,52,87,92,188,216,80,114,224,92,21,201,150,135,240,139,
    242,75,214,224,108,163,196,193,38,30,151,47,139,115,39,148,
    217,146,59,80,242,244,243,144,30,33,99,172,111,128,158,145,
    43,167,43,162,172,209,177,167,139,42,30,124,134,62,59,95,
    49,102,103,42,70,101,74,231,215,189,57,188,19,87,141,202,
    204,172,248,79,255,173,96,230,172,106,43,115,21,241,111,246,
    197,242,76,
};

EmbeddedPython embedded_m5_internal_param_BaseSimpleCPU(
    "m5/internal/param_BaseSimpleCPU.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_BaseSimpleCPU.py",
    "m5.internal.param_BaseSimpleCPU",
    data_m5_internal_param_BaseSimpleCPU,
    2467,
    7735);

} // anonymous namespace
