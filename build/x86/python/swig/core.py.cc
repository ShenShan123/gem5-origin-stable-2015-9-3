#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_core[] = {
    120,156,189,89,235,111,219,200,17,31,146,146,108,201,118,108,
    199,177,157,135,19,43,201,37,81,114,113,124,175,220,181,184,
    52,109,236,248,210,92,47,78,142,202,33,57,221,161,44,205,
    93,73,180,41,82,71,174,18,235,96,23,69,29,180,69,81,
    244,67,129,246,67,63,244,67,129,22,104,255,154,254,71,237,
    204,172,72,201,121,20,87,24,172,109,173,135,195,229,236,60,
    126,51,179,75,121,48,248,41,226,231,71,85,128,68,154,0,
    2,255,12,8,0,58,6,52,12,48,164,1,226,28,236,20,
    33,254,0,68,17,94,0,52,76,192,137,7,72,88,240,149,
    9,225,36,63,83,130,192,98,142,1,253,10,200,2,52,138,
    240,36,156,133,130,44,193,78,5,226,159,129,97,24,161,1,
    79,197,24,136,113,120,129,210,145,40,179,192,113,32,102,133,
    153,101,16,19,204,172,128,152,100,98,2,250,51,32,39,161,
    49,69,211,26,199,80,236,53,20,59,205,98,255,69,98,5,
    222,89,6,113,140,166,163,94,95,210,204,2,205,228,245,166,
    89,202,12,8,150,210,68,123,102,179,137,179,32,45,216,62,
    14,141,227,32,241,111,22,14,200,100,100,205,65,99,14,228,
    28,108,159,128,198,9,16,199,89,198,60,207,158,39,66,204,
    49,103,129,57,11,68,136,19,204,89,100,206,98,74,156,164,
    49,21,122,10,26,167,152,123,122,148,123,6,26,103,64,204,
    243,211,75,124,123,137,8,177,192,156,179,204,57,75,132,88,
    100,206,57,230,156,35,66,156,100,206,50,115,150,137,16,167,
    152,83,101,78,149,8,113,154,57,231,153,115,158,8,113,134,
    57,23,152,115,129,8,177,196,156,139,204,185,72,132,56,203,
    156,183,152,243,22,19,232,140,75,208,184,196,196,101,104,92,
    102,226,10,52,174,48,81,131,70,141,137,171,208,184,202,196,
    53,104,92,99,226,109,104,188,205,196,117,104,92,39,164,212,
    107,136,54,240,255,141,63,53,3,41,53,137,195,51,25,39,
    126,20,58,126,216,140,124,147,238,151,104,32,128,122,52,140,
    13,144,186,78,72,253,7,48,76,133,57,64,234,62,128,65,
    215,0,129,9,251,76,236,155,208,175,193,158,1,219,5,16,
    22,236,225,50,69,242,121,203,128,3,19,190,182,104,194,62,
    142,5,196,211,57,40,40,13,211,109,198,147,150,52,6,251,
    69,216,43,66,253,233,158,73,140,157,50,196,127,131,111,151,
    88,232,56,11,53,97,15,199,2,28,20,96,191,4,79,112,
    18,178,182,203,4,46,227,233,30,90,138,156,122,173,128,218,
    110,142,152,75,166,8,63,14,221,142,84,100,146,227,69,177,
    172,85,210,91,81,114,163,235,170,182,205,115,45,114,66,167,
    171,88,70,20,74,53,129,68,211,15,133,211,137,68,47,144,
    106,156,4,56,77,63,144,142,195,55,239,119,186,81,172,54,
    226,56,138,109,242,35,51,131,200,205,158,160,37,189,32,74,
    100,141,86,227,101,108,18,175,104,118,179,203,18,73,1,214,
    143,30,22,50,241,98,191,171,48,60,90,34,205,38,105,53,
    10,12,15,201,3,28,86,59,161,90,109,183,154,201,106,189,
    237,198,178,222,150,225,106,75,118,110,174,68,177,223,242,195,
    149,68,185,91,129,92,121,239,157,119,111,174,124,127,229,253,
    213,173,158,31,136,213,221,239,125,184,218,237,171,118,20,174,
    38,207,253,214,42,57,227,6,50,142,147,88,100,56,62,27,
    228,180,101,208,149,241,20,113,79,211,146,198,140,49,105,148,
    12,203,168,25,83,72,21,241,99,25,75,230,132,177,233,147,
    73,30,153,73,16,42,140,130,134,34,105,192,142,9,241,18,
    65,98,27,255,12,138,33,2,163,78,247,76,190,247,57,249,
    66,115,183,45,10,180,102,238,49,140,16,79,56,243,22,69,
    54,4,198,66,17,182,75,160,49,130,208,210,160,137,251,52,
    226,116,18,99,162,240,2,36,127,4,244,45,162,99,15,6,
    200,57,176,192,8,103,64,85,168,94,33,119,1,23,252,37,
    131,175,94,35,245,55,25,15,170,237,39,209,243,144,189,78,
    52,167,75,29,61,243,168,255,112,107,91,122,42,89,70,198,
    151,81,175,234,185,97,24,169,170,43,68,213,85,42,246,183,
    122,74,38,85,21,85,47,37,53,10,164,61,155,66,42,147,
    215,239,166,16,162,112,35,132,244,133,240,61,133,23,115,124,
    193,81,72,164,66,56,180,35,145,32,159,68,180,164,178,73,
    73,69,78,142,88,17,70,139,67,83,105,121,156,119,12,175,
    239,164,154,48,36,107,165,20,64,137,12,154,170,194,88,116,
    147,196,97,77,136,159,165,197,51,55,232,73,150,142,192,81,
    168,16,145,90,135,92,129,119,146,140,72,109,102,67,194,40,
    20,125,212,203,247,174,208,146,39,25,126,147,12,192,121,4,
    223,24,142,37,252,95,50,22,76,175,48,128,92,41,133,221,
    2,25,12,28,116,99,16,119,132,224,1,214,149,154,201,133,
    129,109,225,44,188,64,20,61,108,47,209,112,150,134,115,52,
    44,167,230,230,101,243,212,203,54,127,68,235,152,108,40,155,
    68,1,177,82,147,196,161,76,58,53,204,36,172,123,117,202,
    8,147,242,102,152,17,5,170,145,241,109,26,113,42,231,154,
    5,201,99,170,200,148,57,44,140,146,4,225,78,212,48,9,
    216,65,246,12,25,62,158,226,215,38,80,142,34,179,53,130,
    76,155,98,195,176,180,79,165,181,207,161,25,26,144,246,25,
    18,85,124,141,135,171,52,156,207,219,205,67,104,181,94,129,
    214,199,180,228,204,0,90,83,12,169,10,126,102,76,207,26,
    248,62,107,127,115,47,65,138,240,84,120,13,158,46,19,101,
    189,106,237,255,9,74,3,27,63,25,129,18,169,101,142,154,
    178,137,68,127,145,44,24,5,209,34,246,240,39,225,34,182,
    101,147,219,242,59,220,150,185,181,243,158,78,23,98,139,107,
    177,38,138,228,138,166,5,11,131,118,155,148,113,236,198,209,
    110,191,26,53,171,138,109,165,186,121,235,82,114,227,82,242,
    49,86,196,234,109,174,69,186,38,234,170,23,203,46,85,45,
    122,116,99,215,147,220,241,248,202,113,116,145,114,184,96,57,
    131,78,138,120,154,39,71,154,169,135,185,92,39,42,166,42,
    157,171,143,43,153,143,73,229,79,105,145,10,59,216,50,22,
    17,59,21,131,53,113,116,85,230,237,19,223,197,207,26,57,
    157,172,149,64,251,123,187,174,245,100,19,200,24,251,250,33,
    124,228,100,128,189,138,18,191,72,113,81,26,226,130,62,86,
    10,241,95,3,111,215,13,248,21,80,228,49,192,3,136,103,
    25,65,161,158,163,233,63,5,206,133,215,244,118,174,39,117,
    234,231,60,3,203,76,242,17,79,213,173,254,83,248,205,72,
    34,165,13,217,26,108,25,71,27,114,33,171,69,12,153,239,
    212,116,11,135,139,22,5,165,237,38,52,77,87,162,97,110,
    14,203,124,182,237,195,74,156,23,126,198,181,120,135,52,249,
    122,136,30,106,105,103,140,57,115,4,19,239,210,240,94,6,
    7,35,229,229,160,212,50,188,185,239,58,186,194,127,69,43,
    23,88,215,233,49,222,15,172,247,189,0,183,167,41,196,139,
    41,196,239,102,16,151,220,134,94,240,129,129,70,147,98,123,
    96,26,120,88,213,199,84,220,123,209,89,167,4,114,140,246,
    245,116,4,45,14,142,160,152,30,180,8,195,51,253,112,217,
    162,34,119,168,235,177,103,54,181,207,178,128,235,88,210,176,
    155,107,57,160,112,222,10,220,206,150,112,111,183,104,9,90,
    199,75,243,201,76,149,158,25,85,154,114,193,120,147,222,124,
    121,51,85,254,89,174,165,224,67,148,152,41,205,192,23,145,
    199,249,255,184,45,171,29,217,217,194,147,97,219,239,86,155,
    129,219,226,72,164,189,240,94,26,104,149,90,244,59,125,238,
    59,193,102,166,155,16,58,104,63,9,79,99,51,41,112,51,
    249,33,53,147,61,46,8,142,169,251,201,48,128,188,55,224,
    67,14,165,68,40,159,59,26,97,186,77,16,24,220,110,87,
    134,194,190,118,40,127,121,231,234,198,45,61,47,247,96,59,
    120,76,246,113,131,190,61,204,221,105,172,253,39,176,246,191,
    138,85,82,109,196,66,78,225,98,150,204,75,57,235,203,241,
    253,38,141,175,62,134,100,141,134,15,8,58,231,177,87,227,
    241,78,245,117,133,189,65,195,85,26,110,101,1,161,156,19,
    18,15,176,114,16,19,53,3,217,38,16,207,168,42,142,250,
    216,161,185,243,225,117,224,56,249,247,177,15,80,98,51,13,
    66,9,139,103,201,44,151,202,195,32,100,185,55,53,132,41,
    188,146,120,25,222,226,168,243,192,247,226,232,209,250,176,13,
    244,186,94,174,38,220,161,242,4,135,183,104,255,163,234,52,
    35,140,226,142,27,140,106,111,175,229,237,253,117,93,208,142,
    162,58,237,9,253,196,62,236,247,252,53,167,170,245,139,163,
    107,142,141,242,97,79,117,123,234,174,31,15,17,35,240,34,
    79,229,55,80,226,111,71,148,135,193,79,86,113,42,153,242,
    47,183,198,97,34,71,27,187,190,90,15,164,27,246,186,249,
    231,233,61,148,248,251,35,168,60,199,110,77,104,201,59,65,
    240,153,159,40,25,98,91,202,95,241,31,163,196,63,28,13,
    40,21,6,138,20,182,27,138,168,163,97,82,24,240,114,213,
    253,62,74,252,211,209,116,159,213,32,95,15,34,111,231,147,
    88,126,211,147,161,215,215,38,80,235,80,190,183,147,60,146,
    113,93,122,184,85,204,213,152,159,160,196,63,31,1,65,212,
    150,188,94,252,24,53,206,31,53,36,241,47,71,243,188,46,
    47,177,239,6,254,183,132,249,225,171,107,175,171,156,188,75,
    12,189,22,248,235,209,12,160,25,45,132,78,91,122,59,221,
    200,15,149,46,237,15,243,246,253,35,148,248,247,163,169,78,
    175,190,123,97,230,253,123,65,180,229,6,137,142,128,201,17,
    200,213,130,207,81,226,63,135,22,240,46,247,165,175,100,244,
    250,43,144,30,85,251,137,77,28,123,58,179,129,191,69,72,
    247,116,63,32,14,189,72,217,116,59,250,61,48,191,235,180,
    47,194,224,45,149,77,47,86,245,174,143,222,219,241,251,1,
    253,230,5,183,224,252,238,133,15,204,246,251,52,208,166,139,
    43,178,222,6,242,6,48,150,45,42,202,49,87,54,239,153,
    27,51,84,31,184,187,148,112,188,199,225,221,2,55,94,62,
    105,174,249,173,141,80,248,110,184,214,87,242,97,44,100,204,
    114,176,142,160,179,66,197,49,120,117,142,34,181,63,243,149,
    10,228,127,125,122,254,77,211,184,121,114,59,226,210,206,53,
    146,107,11,231,44,227,158,17,196,65,96,12,15,54,38,184,
    69,89,67,129,3,171,238,8,17,43,114,246,253,240,25,98,
    68,60,110,199,210,21,247,239,242,19,3,222,163,40,86,200,
    161,29,183,23,117,186,126,32,239,186,74,114,59,160,163,148,
    115,119,99,237,139,123,250,251,36,186,220,212,215,211,233,245,
    99,251,206,250,253,205,123,206,195,205,220,235,149,62,192,234,
    151,106,183,73,161,228,231,56,208,235,245,242,116,25,183,212,
    244,205,142,101,84,240,116,83,176,38,103,202,133,201,137,114,
    161,60,102,241,27,210,41,99,206,172,20,202,19,11,147,101,
    188,42,27,101,171,108,14,127,167,12,253,249,78,191,198,232,
    239,127,0,233,143,134,101,
};

EmbeddedPython embedded_m5_internal_core(
    "m5/internal/core.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/swig/core.py",
    "m5.internal.core",
    data_m5_internal_core,
    2535,
    7967);

} // anonymous namespace
