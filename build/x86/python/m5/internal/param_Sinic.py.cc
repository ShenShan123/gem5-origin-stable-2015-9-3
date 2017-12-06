#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_Sinic[] = {
    120,156,197,89,109,115,219,198,17,222,3,72,74,164,36,235,
    205,146,108,73,182,104,59,182,105,57,18,109,39,142,211,137,
    235,214,177,221,153,100,26,217,5,211,177,163,100,130,66,192,
    145,4,69,2,28,224,36,139,174,244,161,81,250,242,7,242,
    173,95,251,161,255,166,255,168,221,93,16,32,68,138,146,103,
    218,50,150,180,62,238,237,237,221,238,62,187,247,66,27,186,
    255,178,248,247,235,34,64,184,172,1,56,248,43,160,9,208,
    18,176,45,64,72,1,206,69,216,205,66,240,49,56,89,248,
    17,96,91,3,169,193,49,54,116,248,86,3,111,146,199,228,
    160,169,51,71,64,167,0,50,3,219,89,120,237,205,66,70,
    230,96,183,0,193,31,64,8,225,9,120,227,140,129,51,14,
    63,162,118,108,228,89,225,56,16,179,192,204,60,56,19,204,
    44,128,51,201,141,9,232,204,128,156,132,237,41,18,219,190,
    128,106,215,81,237,52,171,253,23,169,117,176,103,1,156,11,
    36,142,235,250,134,36,51,36,201,243,77,179,150,153,120,149,
    179,176,61,23,183,231,83,237,139,169,246,66,170,189,152,106,
    47,165,218,151,82,237,203,169,246,114,170,189,146,106,175,166,
    218,87,82,237,171,169,246,90,170,93,76,181,175,165,218,215,
    83,237,27,220,70,239,204,65,227,3,104,220,132,198,45,168,
    98,192,102,19,79,220,6,169,67,163,4,219,37,144,248,123,
    27,142,49,166,206,92,106,196,29,30,49,159,140,88,231,17,
    119,97,251,46,72,252,93,143,70,228,160,82,90,68,156,184,
    255,198,127,37,129,45,53,137,100,95,6,161,235,123,166,235,
    85,125,87,163,254,28,17,66,149,77,100,172,11,175,103,4,
    175,127,2,99,203,209,186,240,58,2,84,44,200,150,166,6,
    71,220,56,210,160,83,130,67,1,141,12,56,58,28,226,52,
    89,90,64,77,192,177,6,223,233,36,112,132,52,131,32,184,
    10,25,21,97,171,193,32,136,52,141,193,81,22,14,179,80,
    121,115,168,17,99,55,15,193,63,224,221,42,43,29,103,165,
    26,28,34,205,192,113,6,142,114,240,26,133,144,213,200,147,
    249,226,205,33,90,138,156,74,41,131,171,221,74,153,75,166,
    56,110,224,89,45,201,166,155,109,43,176,90,102,197,245,92,
    187,84,136,37,252,112,179,109,169,186,193,67,116,242,69,171,
    173,88,149,239,73,53,129,141,170,235,57,102,203,119,246,154,
    82,141,147,30,179,234,54,165,105,114,231,23,173,182,31,168,
    23,65,224,7,6,185,147,153,77,223,74,70,144,51,237,166,
    31,202,18,205,198,211,24,164,94,145,116,181,205,26,105,1,
    188,76,26,236,200,208,14,220,182,194,40,69,26,73,154,180,
    149,40,62,76,194,111,145,148,91,158,42,215,107,213,176,92,
    169,91,129,172,212,165,87,174,201,214,195,13,63,112,107,174,
    183,17,42,107,167,41,55,30,220,187,255,112,227,23,27,31,
    149,119,246,220,166,83,62,248,244,147,114,187,163,234,190,87,
    110,61,44,187,158,146,232,160,102,57,229,154,77,236,158,163,
    73,222,186,53,211,101,243,204,186,108,182,101,48,69,220,101,
    90,128,152,17,147,34,39,116,81,18,83,216,202,226,159,46,
    86,181,9,177,229,146,129,54,25,77,184,202,164,145,68,225,
    21,176,171,65,176,74,56,105,224,175,160,192,34,90,42,212,
    167,113,223,239,200,51,17,183,161,83,244,35,230,33,99,11,
    65,134,146,143,41,220,30,48,64,178,208,200,65,4,28,196,
    91,132,164,160,67,20,197,73,141,134,202,51,16,254,4,232,
    105,132,204,33,116,225,116,172,131,240,102,64,21,168,242,32,
    119,17,39,252,129,17,89,41,209,242,183,24,29,170,238,134,
    254,91,143,99,64,109,6,82,5,61,243,170,243,114,167,33,
    109,21,174,33,227,27,127,175,104,91,158,231,171,162,229,56,
    69,75,169,192,221,217,83,50,44,42,191,120,51,44,81,88,
    141,217,24,96,137,190,78,59,6,20,5,31,1,21,125,112,
    92,91,225,135,121,254,192,81,8,165,66,112,212,125,39,68,
    62,169,168,73,101,208,34,21,57,217,231,133,48,118,76,18,
    165,233,81,238,2,126,126,26,175,132,1,90,202,197,112,10,
    101,179,170,10,140,76,43,12,77,94,9,241,25,132,164,120,
    223,106,238,73,214,142,48,82,184,32,106,70,107,24,33,12,
    47,145,73,177,7,216,44,207,247,156,14,174,210,181,111,211,
    2,46,49,24,39,25,142,11,8,197,49,164,57,252,63,39,
    22,53,59,211,5,96,46,6,33,149,67,5,12,1,209,69,
    1,2,242,24,75,79,73,227,218,193,150,113,134,94,167,22,
    13,54,86,137,92,33,114,149,200,90,108,252,104,60,48,213,
    239,129,71,52,171,198,102,179,129,20,44,61,54,208,57,145,
    101,151,123,89,134,133,178,66,217,162,81,78,245,178,37,67,
    69,53,120,66,20,69,57,15,117,8,191,166,18,78,89,197,
    202,40,129,48,21,168,213,75,16,118,151,49,67,110,24,143,
    177,109,16,96,211,168,173,165,80,107,80,164,24,178,198,229,
    184,74,154,36,17,129,213,88,33,85,217,83,252,93,36,114,
    109,180,78,239,193,174,54,0,187,207,104,1,51,93,216,77,
    49,220,10,248,55,163,217,122,55,18,201,238,57,223,7,55,
    194,90,230,20,172,221,162,150,62,104,251,207,2,179,174,197,
    191,73,193,140,22,169,165,13,219,194,70,103,137,236,73,3,
    108,9,15,4,175,189,37,220,227,53,222,227,239,241,30,207,
    231,4,62,213,69,5,92,231,26,30,53,178,228,152,170,14,
    139,221,189,59,204,35,109,7,254,65,167,232,87,139,138,45,
    167,122,251,248,102,184,121,51,252,12,43,105,241,9,215,176,
    168,150,70,213,50,144,109,170,118,52,244,197,129,45,121,223,
    228,79,166,25,21,55,147,11,157,217,221,143,17,107,11,228,
    86,45,246,55,151,249,80,5,84,221,71,232,241,66,226,113,
    50,224,75,154,178,192,238,214,197,18,226,170,32,120,93,102,
    84,219,249,100,198,189,248,247,57,133,128,108,151,64,231,125,
    163,18,173,154,13,34,211,140,15,79,96,103,36,230,24,101,
    212,255,251,24,51,185,30,102,232,79,143,147,225,175,192,39,
    87,1,127,1,66,5,6,191,155,12,73,238,16,12,230,73,
    252,123,224,172,57,229,188,192,117,168,66,103,4,150,192,242,
    20,62,98,209,232,248,240,37,252,45,149,114,241,38,175,119,
    207,166,233,77,62,147,212,48,134,211,123,109,228,153,147,197,
    142,66,84,183,66,18,139,42,88,47,139,123,155,69,114,176,
    196,10,62,26,108,141,71,147,153,180,174,239,122,200,162,109,
    114,69,204,107,41,188,220,39,242,32,129,138,136,121,255,247,
    37,174,193,240,157,221,140,118,13,90,66,152,225,149,79,143,
    241,190,194,163,205,103,47,127,251,114,171,98,62,151,251,174,
    45,147,204,200,198,153,241,32,201,12,201,251,220,143,124,133,
    33,170,17,8,142,53,129,119,94,60,242,209,21,51,3,50,
    11,219,57,202,33,62,166,139,110,138,137,184,210,81,93,60,
    177,137,178,139,182,34,231,37,56,136,66,76,228,96,132,21,
    132,162,252,184,105,181,118,28,235,73,135,38,164,89,237,56,
    233,180,216,132,153,180,9,148,48,98,152,21,252,241,97,108,
    202,254,8,171,199,39,168,63,49,129,115,197,241,109,46,25,
    95,215,101,177,37,91,59,120,107,173,187,237,98,181,105,213,
    56,74,122,215,196,151,177,137,138,195,220,127,68,9,215,137,
    250,69,219,247,176,196,239,217,202,15,138,142,196,11,157,116,
    138,27,69,222,31,138,110,88,180,118,176,215,178,85,148,0,
    39,51,153,207,197,86,80,11,249,8,188,251,150,154,35,142,
    178,137,183,117,23,175,4,127,132,100,83,142,46,147,73,185,
    231,195,126,148,79,184,127,226,85,77,117,162,202,70,199,21,
    99,147,200,29,24,253,174,240,49,165,4,77,68,174,203,137,
    21,45,175,241,201,143,5,94,145,108,56,152,192,223,139,247,
    72,224,232,185,170,155,198,57,146,148,99,244,90,64,52,79,
    155,194,118,33,102,78,48,157,100,230,84,204,188,192,116,154,
    153,51,49,115,150,233,28,51,231,99,230,69,166,11,204,92,
    140,153,75,76,47,49,243,114,204,92,102,186,194,204,213,152,
    121,133,233,85,102,174,197,204,34,211,107,204,188,30,51,111,
    48,253,128,153,55,99,230,45,166,183,153,89,138,153,119,152,
    174,51,243,110,204,252,144,233,6,51,55,227,135,191,50,51,
    239,193,246,125,122,125,34,206,3,42,118,99,255,109,177,227,
    50,49,226,2,241,195,255,180,198,25,143,126,70,11,140,79,
    161,123,36,26,86,223,68,218,188,169,168,190,53,68,124,75,
    75,219,198,79,68,115,39,147,202,180,3,105,41,25,133,106,
    117,164,134,114,149,140,166,255,115,175,92,13,222,33,158,38,
    54,29,243,97,176,115,145,35,24,93,80,57,130,226,181,183,
    140,151,137,12,95,38,30,211,101,226,144,29,96,106,209,125,
    162,7,208,108,226,135,105,36,158,124,107,166,124,17,221,21,
    104,85,86,187,45,61,199,88,135,244,241,159,187,71,136,3,
    42,200,63,65,234,76,166,139,139,120,222,31,204,70,218,118,
    82,54,114,40,179,73,254,141,54,168,140,222,191,199,232,45,
    125,4,233,189,199,120,76,132,119,155,100,163,49,126,149,132,
    100,185,15,154,142,108,90,29,211,246,219,29,186,107,158,213,
    141,39,67,190,38,245,88,170,216,39,29,28,152,85,183,234,
    155,77,255,173,217,178,130,93,86,121,174,16,41,166,245,246,
    119,168,107,67,70,170,122,32,195,186,223,116,88,255,249,82,
    52,193,108,106,130,164,71,173,12,142,109,89,7,61,119,156,
    213,79,90,39,34,173,49,111,152,60,198,49,56,75,31,247,
    247,233,35,222,128,109,170,107,65,221,173,213,123,30,62,95,
    42,246,192,64,207,208,177,103,251,120,80,170,127,134,225,62,
    86,231,248,88,157,226,227,20,79,173,246,201,239,187,129,218,
    179,154,38,222,16,35,39,159,41,64,26,249,139,151,20,83,
    93,25,50,194,246,247,60,197,58,207,150,32,165,83,41,165,
    204,85,151,251,198,188,147,129,223,51,123,120,47,105,163,23,
    147,132,163,174,14,151,117,223,73,86,119,142,8,233,188,144,
    214,201,108,117,99,232,168,147,241,127,31,57,154,98,254,196,
    20,73,31,151,89,222,21,177,120,72,37,211,27,2,167,126,
    247,137,210,145,120,240,247,59,166,25,189,182,80,245,49,205,
    81,159,146,127,137,250,255,68,19,209,67,32,158,146,69,14,
    207,201,11,98,200,143,150,207,229,5,95,79,250,190,190,139,
    86,204,183,1,126,109,232,132,6,113,140,233,164,20,243,87,
    77,241,245,128,170,54,71,125,203,106,69,95,15,240,163,183,
    65,158,231,39,73,227,118,82,210,9,58,252,196,19,61,172,
    225,14,203,87,39,190,41,25,180,27,48,26,90,15,55,99,
    19,55,35,19,95,168,186,12,240,162,254,185,133,177,210,88,
    132,129,61,92,18,175,244,156,159,131,18,175,108,183,219,191,
    124,122,127,211,82,85,63,104,13,153,224,11,172,110,207,124,
    36,126,147,223,119,7,37,42,157,80,201,214,192,236,210,219,
    107,153,95,201,150,31,116,190,242,29,201,201,158,238,127,138,
    249,108,88,94,77,154,251,146,238,151,92,186,78,8,116,47,
    151,145,142,88,170,120,234,26,78,202,14,241,4,118,70,223,
    61,241,43,202,96,255,179,166,111,239,74,167,43,115,186,59,
    88,230,185,223,178,144,127,250,44,21,55,158,101,182,175,223,
    9,104,212,66,31,55,148,129,107,53,41,199,79,215,247,188,
    101,157,29,95,215,239,246,19,76,227,78,206,225,83,192,68,
    21,53,205,224,171,102,111,231,63,249,72,196,185,30,200,154,
    139,1,14,88,99,122,104,247,152,72,89,200,200,72,151,157,
    244,192,17,151,133,232,125,39,122,180,126,66,91,82,72,79,
    121,244,69,87,126,58,143,37,130,78,142,186,40,224,217,49,
    163,79,206,228,51,147,19,249,76,126,76,231,239,35,166,196,
    188,86,200,228,39,38,197,249,63,107,88,78,10,218,218,106,
    94,252,7,229,148,55,37,
};

EmbeddedPython embedded_m5_internal_param_Sinic(
    "m5/internal/param_Sinic.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_Sinic.py",
    "m5.internal.param_Sinic",
    data_m5_internal_param_Sinic,
    2727,
    8799);

} // anonymous namespace
