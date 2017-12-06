#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_BaseCache[] = {
    120,156,197,89,109,115,219,198,17,222,3,95,100,82,146,37,
    91,150,252,38,91,244,139,108,198,141,36,191,201,118,106,199,
    137,237,184,51,205,76,148,20,76,199,137,146,41,10,1,71,
    18,50,9,176,192,73,54,51,210,76,167,242,180,253,3,237,
    183,126,236,135,254,155,254,128,78,255,74,187,187,199,131,64,
    138,140,53,105,71,178,164,245,97,111,177,119,187,247,236,222,
    222,193,131,222,191,2,254,125,90,1,72,254,45,0,124,252,
    21,208,2,104,11,88,23,32,164,0,255,12,188,46,64,124,
    31,252,2,188,3,88,183,64,90,176,135,141,28,124,103,65,
    56,193,239,20,161,149,99,142,128,110,25,100,30,214,11,240,
    42,60,5,121,89,132,215,101,136,127,11,66,136,80,192,55,
    254,24,248,39,224,29,106,199,70,137,21,158,0,98,150,153,
    89,2,127,156,153,101,240,39,184,49,14,221,105,144,19,176,
    62,73,98,235,39,81,237,45,84,59,197,106,255,73,106,125,
    236,153,5,255,36,137,227,188,190,37,201,60,73,242,120,83,
    172,101,218,204,242,20,172,159,54,237,153,76,251,76,166,61,
    155,105,207,101,218,103,51,237,115,153,246,249,76,251,66,166,
    125,49,211,158,207,180,47,101,218,151,185,141,22,158,134,205,
    5,216,172,192,230,21,168,163,211,79,165,214,92,5,153,131,
    205,107,176,126,13,36,254,94,133,61,92,23,255,116,230,141,
    235,252,198,76,250,198,34,191,113,3,214,111,128,196,223,69,
    253,70,17,106,213,57,92,235,224,63,248,175,138,107,13,106,
    2,201,182,140,147,32,10,157,32,172,71,129,69,253,69,34,
    132,12,143,200,88,15,34,47,8,34,255,0,198,135,111,245,
    32,178,11,168,88,144,45,45,11,118,185,177,107,65,183,10,
    59,2,54,243,224,231,96,7,135,41,208,4,26,2,246,44,
    248,62,71,2,187,72,243,184,144,151,33,175,52,62,54,121,
    33,181,166,49,216,45,192,78,1,106,223,236,88,196,120,93,
    130,248,239,240,195,60,43,61,193,74,45,216,65,154,135,189,
    60,236,22,225,21,10,33,107,179,68,230,139,111,118,208,82,
    228,212,170,121,156,237,90,198,92,50,197,15,226,208,109,75,
    53,141,109,167,227,198,110,219,121,238,38,242,133,235,53,101,
    181,108,164,162,100,185,227,170,166,205,175,229,200,31,237,142,
    98,117,81,40,213,56,54,234,65,232,59,237,200,223,106,73,
    117,130,116,57,245,160,37,29,135,59,127,217,238,68,177,122,
    25,199,81,108,147,75,153,217,138,220,244,13,114,168,215,138,
    18,89,165,209,120,24,155,212,43,146,174,119,88,35,77,128,
    167,74,47,251,50,241,226,160,163,112,165,180,70,146,38,109,
    85,90,35,38,137,131,100,165,29,170,149,102,163,158,172,212,
    154,110,44,107,77,25,174,52,100,123,117,41,138,131,70,16,
    46,37,202,221,104,201,165,187,183,239,172,46,125,180,116,111,
    101,99,43,104,249,43,111,31,61,88,233,116,85,51,10,87,
    218,171,43,65,168,36,58,169,181,50,224,158,101,20,57,77,
    3,189,9,26,78,192,38,58,77,217,234,200,120,146,184,23,
    104,18,98,90,76,136,162,200,137,170,152,196,86,1,255,114,
    98,222,26,23,107,1,25,233,145,225,132,175,124,22,81,180,
    204,2,94,91,16,207,19,94,54,241,87,208,2,35,106,106,
    212,103,113,223,175,200,59,154,187,153,35,20,104,230,14,99,
    12,193,134,146,79,104,217,67,96,160,20,96,179,8,26,64,
    136,59,141,168,184,75,20,197,73,141,133,202,243,144,252,5,
    208,219,8,157,29,232,193,106,47,7,34,156,6,85,166,44,
    130,220,57,28,240,15,140,204,90,149,166,191,198,8,81,205,
    32,137,222,132,188,14,212,230,88,170,161,103,190,234,126,185,
    177,41,61,149,44,32,227,219,104,171,226,185,97,24,169,138,
    235,251,21,87,169,56,216,216,82,50,169,168,168,178,152,84,
    105,105,237,83,6,100,169,190,110,199,128,138,0,128,160,210,
    15,126,224,41,124,152,225,7,94,133,68,42,4,72,51,242,
    19,228,147,138,134,84,54,77,82,145,147,35,158,8,227,199,
    33,81,26,30,229,78,226,243,51,51,19,6,105,181,104,32,
    149,200,86,93,149,25,157,110,146,56,60,19,226,51,16,73,
    241,182,219,218,146,172,29,161,164,112,66,212,212,115,56,98,
    40,158,35,179,140,23,216,180,48,10,253,46,206,52,240,110,
    210,36,206,49,32,39,24,146,179,8,199,49,164,69,252,191,
    40,230,44,47,223,3,97,209,0,145,82,163,2,134,129,232,
    33,1,65,185,135,105,168,106,113,30,97,235,56,82,175,82,
    139,94,182,231,137,92,34,114,153,200,130,113,192,209,121,97,
    114,208,11,15,105,100,139,77,103,35,105,209,114,198,72,191,
    47,218,206,239,71,27,38,206,26,69,141,69,177,181,31,53,
    121,74,178,241,83,162,40,202,241,152,131,228,107,74,233,20,
    93,172,140,2,9,67,130,90,251,129,194,46,179,41,201,86,
    79,24,140,219,4,220,44,122,27,25,244,218,180,90,12,93,
    251,188,201,152,14,73,104,208,218,23,73,85,97,136,207,43,
    68,174,28,189,227,247,225,215,56,0,191,199,52,137,233,30,
    252,38,25,118,101,252,155,182,188,92,111,53,210,29,117,102,
    0,118,132,185,252,16,204,221,160,86,238,160,253,199,6,183,
    158,213,191,200,192,141,38,106,101,141,91,195,70,247,44,217,
    148,5,218,89,44,20,94,133,103,113,239,183,120,239,191,205,
    123,63,215,15,92,177,233,132,158,227,156,174,27,5,114,78,
    61,7,115,189,61,61,41,33,237,196,209,219,110,37,170,87,
    20,91,79,249,247,201,98,178,188,152,60,198,204,90,121,202,
    57,77,231,86,157,61,99,217,161,236,71,175,190,124,235,73,
    222,75,249,201,113,116,178,115,56,241,57,189,61,26,49,55,
    75,174,181,140,207,57,237,39,42,166,108,127,196,94,47,167,
    94,39,35,62,167,97,203,236,242,156,56,139,248,42,11,158,
    155,163,243,61,87,109,220,139,127,207,105,25,200,126,9,84,
    207,219,53,61,115,54,138,204,179,63,236,195,208,145,153,100,
    175,224,24,191,54,216,41,238,99,135,254,114,38,48,254,4,
    92,217,10,248,35,16,58,16,4,189,192,72,227,136,224,48,
    67,226,191,1,142,160,33,117,4,231,165,26,213,14,44,129,
    233,42,121,200,162,186,172,248,28,254,156,9,63,179,249,231,
    122,181,107,118,243,207,167,57,141,97,117,168,13,62,223,159,
    252,104,153,154,110,66,98,58,163,237,71,244,254,6,146,22,
    157,152,209,143,14,99,39,244,128,14,205,237,251,125,132,209,
    246,121,81,204,88,25,220,220,33,114,55,133,140,48,188,35,
    153,230,2,140,222,245,29,189,155,124,71,115,201,243,236,167,
    198,56,194,83,13,105,108,20,76,108,220,77,99,67,242,174,
    247,142,15,56,68,45,130,192,158,37,240,84,139,133,32,29,
    34,243,32,11,176,94,164,40,226,2,94,244,130,76,152,124,
    71,217,177,111,75,101,231,172,105,183,165,40,208,11,76,228,
    237,17,231,17,90,227,39,45,183,189,225,187,79,41,107,38,
    52,178,103,194,206,50,102,76,103,205,160,144,17,163,44,225,
    199,85,99,206,246,17,231,144,7,192,123,167,54,131,35,198,
    143,60,78,28,95,55,101,165,45,219,27,120,182,109,6,157,
    74,189,229,54,120,181,114,61,51,191,52,102,42,94,238,193,
    194,37,185,69,52,170,120,81,136,9,127,203,83,81,92,241,
    37,30,249,164,95,89,170,240,110,81,9,146,138,187,129,189,
    174,167,116,8,244,199,51,87,205,110,220,72,184,64,126,253,
    134,154,199,176,218,14,158,235,3,60,52,108,67,186,77,235,
    35,103,154,252,249,56,160,35,10,119,84,60,204,169,174,206,
    113,84,196,216,203,68,62,128,227,217,35,238,247,214,54,33,
    23,22,197,69,171,100,169,169,108,56,127,69,239,36,7,131,
    250,95,226,16,65,173,47,169,122,161,93,36,73,57,70,247,
    11,68,75,180,77,172,151,13,115,156,233,4,51,39,13,243,
    36,211,41,102,78,27,230,41,166,167,153,57,99,152,103,152,
    206,50,115,206,48,207,50,61,199,204,243,134,121,129,233,69,
    102,206,27,230,37,166,151,153,185,96,152,21,166,87,152,121,
    213,48,175,49,189,206,204,69,195,188,193,244,38,51,171,134,
    249,1,211,91,204,252,153,97,126,200,116,137,153,203,134,185,
    194,244,54,51,239,24,230,93,166,247,152,121,223,48,87,153,
    62,96,230,67,195,124,196,244,35,102,254,220,48,31,51,125,
    194,204,143,13,243,41,211,79,152,249,169,185,108,124,198,204,
    231,176,254,130,110,203,136,243,25,165,223,177,255,53,253,114,
    210,58,134,116,181,251,127,205,186,246,195,99,182,194,126,4,
    189,82,109,84,198,21,89,19,39,117,198,221,20,230,52,153,
    181,143,175,181,230,14,134,183,227,197,210,85,82,47,219,252,
    145,27,204,249,91,79,225,247,251,73,244,224,89,231,89,106,
    219,30,23,171,221,51,188,154,250,64,205,171,41,94,133,23,
    240,208,147,231,67,207,19,58,244,236,176,35,28,75,159,123,
    246,1,91,72,253,65,174,13,229,27,103,192,39,250,92,67,
    51,115,59,29,25,250,246,45,200,30,85,184,251,136,113,65,
    219,197,95,33,83,55,230,196,25,60,155,28,140,82,218,24,
    51,182,242,178,22,210,184,60,250,5,102,68,255,205,32,186,
    250,18,178,187,163,253,132,8,239,135,233,86,104,127,146,46,
    207,194,16,184,226,57,32,118,98,55,108,200,132,14,201,239,
    149,193,58,150,111,59,50,60,117,126,216,59,73,18,121,172,
    113,116,47,233,34,95,242,147,186,57,68,206,151,109,151,110,
    174,147,38,14,38,19,25,111,75,214,121,88,89,26,129,64,
    57,164,79,93,29,162,163,30,197,88,251,248,78,18,70,81,
    71,59,228,16,98,52,10,213,37,253,236,161,174,108,98,121,
    211,194,224,12,189,238,72,119,103,101,140,187,51,60,85,25,
    242,78,144,56,42,234,56,45,185,45,91,172,248,189,66,164,
    153,110,33,178,204,161,198,182,221,183,78,59,72,48,179,69,
    91,161,26,233,147,1,49,227,147,126,246,80,48,208,170,36,
    35,161,162,123,13,84,248,73,221,24,34,215,137,101,93,42,
    175,233,68,161,227,122,158,76,180,202,67,138,146,126,250,90,
    112,176,75,93,254,17,13,50,230,65,222,35,66,202,203,25,
    229,88,181,94,31,242,6,2,179,131,37,188,236,3,200,161,
    4,105,0,138,250,193,14,181,56,228,237,68,254,110,75,134,
    42,112,91,89,63,29,78,146,6,226,219,132,193,30,190,96,
    60,240,126,240,131,14,215,145,157,164,143,47,242,241,65,93,
    24,38,213,77,148,108,179,146,31,233,38,53,124,201,207,143,
    67,135,83,110,35,25,57,23,238,52,115,161,7,117,101,152,
    84,67,225,178,226,122,114,26,33,93,239,151,34,165,92,71,
    100,185,67,223,123,19,7,74,58,27,91,245,186,236,69,195,
    251,165,140,246,62,174,186,59,12,141,244,249,171,77,174,10,
    124,137,177,24,134,210,163,43,197,76,80,255,132,215,146,30,
    246,127,92,108,180,102,175,179,245,83,38,52,250,181,190,9,
    141,20,227,221,159,139,55,95,182,36,122,110,96,20,253,185,
    85,159,50,125,220,223,226,168,235,56,250,210,18,159,91,142,
    115,28,199,203,143,129,11,110,125,7,132,199,75,81,196,3,
    230,172,56,196,143,85,42,150,4,159,245,7,190,152,107,11,
    104,47,213,23,120,221,196,38,142,61,149,86,12,252,101,215,
    156,179,169,184,224,11,170,53,183,173,191,196,241,183,37,251,
    26,244,110,252,237,155,105,229,65,153,156,111,77,245,157,53,
    22,133,124,15,193,215,14,246,61,226,83,233,212,94,93,54,
    38,47,63,195,130,194,166,122,194,217,150,116,133,193,31,151,
    219,171,188,145,101,229,246,93,243,213,126,58,61,55,84,168,
    166,211,193,197,129,78,25,110,181,157,47,100,59,138,187,95,
    68,190,228,56,235,155,73,239,162,68,139,152,233,12,159,71,
    191,236,129,161,180,16,118,234,175,172,188,221,31,236,127,209,
    138,188,215,210,239,201,92,26,45,243,89,212,118,145,63,124,
    148,90,96,70,57,53,208,239,199,244,214,236,0,23,171,32,
    204,223,38,241,14,119,241,215,148,11,9,4,166,143,111,52,
    6,173,162,37,78,159,248,30,164,255,100,196,145,20,203,70,
    128,203,17,179,134,84,186,119,64,248,216,32,226,64,130,207,
    188,121,12,81,167,239,32,245,39,150,167,84,135,113,132,208,
    39,218,210,84,9,35,144,206,13,57,81,198,147,67,62,55,
    49,93,202,79,140,151,242,165,177,28,127,65,155,20,51,86,
    57,95,26,159,16,163,126,22,48,46,203,214,194,213,146,248,
    47,23,165,14,98,
};

EmbeddedPython embedded_m5_internal_param_BaseCache(
    "m5/internal/param_BaseCache.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_BaseCache.py",
    "m5.internal.param_BaseCache",
    data_m5_internal_param_BaseCache,
    2869,
    9439);

} // anonymous namespace
