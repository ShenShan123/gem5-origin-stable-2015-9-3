#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_SrcClockDomain[] = {
    120,156,205,89,221,115,219,198,17,223,3,64,74,164,36,139,
    250,182,45,217,98,210,113,205,122,42,49,113,162,56,157,184,
    110,211,52,157,105,30,148,20,76,107,135,201,20,133,128,35,
    9,10,4,56,192,73,50,61,210,67,43,79,155,127,160,125,
    239,67,31,250,223,228,63,106,119,247,8,8,250,154,104,38,
    29,179,146,120,62,46,22,123,183,187,191,253,184,179,7,227,
    159,18,126,126,89,7,72,255,33,0,124,252,19,16,2,12,
    4,180,5,8,41,192,95,134,253,18,36,239,131,95,130,215,
    0,109,3,164,1,167,56,49,225,107,3,162,89,126,167,12,
    161,201,20,1,163,42,72,11,218,37,120,30,45,128,37,203,
    176,95,133,228,79,32,132,136,4,188,240,167,192,159,134,215,
    40,29,39,21,22,56,13,68,172,50,177,2,254,12,19,171,
    224,207,242,100,6,70,53,144,179,208,158,35,182,246,45,20,
    251,8,197,206,179,216,239,72,172,143,79,86,192,191,69,236,
    184,175,175,136,211,34,78,94,111,158,165,212,178,93,46,64,
    123,49,155,47,21,230,203,133,249,74,97,190,90,152,175,21,
    230,183,121,142,59,91,132,254,29,232,223,133,254,58,116,208,
    88,11,249,46,54,64,154,208,191,7,237,123,32,241,111,3,
    78,209,158,254,98,225,141,251,252,198,82,254,198,38,191,81,
    135,118,29,36,254,109,234,55,202,208,106,172,162,143,130,255,
    224,79,3,125,4,106,22,135,67,153,164,65,28,57,65,212,
    137,3,131,158,151,105,32,143,122,52,76,141,93,251,9,185,
    246,223,192,126,245,141,177,107,79,0,5,11,210,37,52,224,
    132,39,39,6,140,26,112,44,160,111,129,111,194,49,46,83,
    162,13,116,5,156,26,240,141,73,12,39,56,90,232,128,251,
    96,41,237,215,62,59,64,75,154,130,147,18,28,151,160,245,
    226,216,32,194,126,5,146,127,193,171,13,22,58,205,66,13,
    56,198,209,130,83,11,78,202,240,28,153,144,212,175,144,250,
    226,197,49,106,138,148,86,195,194,221,238,22,212,37,85,252,
    32,137,220,129,84,43,56,119,134,110,226,14,156,86,226,125,
    18,198,222,254,175,227,129,27,68,141,106,198,26,167,219,67,
    87,245,108,126,215,36,163,12,134,138,101,198,145,84,51,56,
    233,4,145,239,12,98,255,32,148,106,154,4,58,157,32,148,
    142,195,15,127,59,24,198,137,250,52,73,226,196,38,187,50,
    49,140,221,252,13,178,170,23,198,169,108,208,106,188,140,77,
    226,21,113,119,134,44,145,54,192,251,165,151,125,153,122,73,
    48,84,232,46,45,145,184,73,90,131,28,197,67,138,138,67,
    115,16,169,102,175,219,73,155,173,158,155,200,86,79,70,205,
    174,28,236,108,197,73,208,13,162,173,84,185,123,161,220,122,
    252,206,187,59,91,63,219,122,175,185,119,16,132,126,243,229,
    135,31,52,135,35,213,139,163,230,96,167,25,68,74,162,165,
    194,230,85,54,218,70,190,69,90,237,40,232,58,1,235,233,
    244,100,56,148,201,28,81,239,210,78,68,77,204,138,178,48,
    69,67,204,225,172,132,31,83,108,24,51,98,55,32,77,61,
    210,158,144,102,21,177,69,14,23,176,111,64,178,65,200,233,
    227,159,32,87,35,126,90,244,204,224,103,191,35,19,105,106,
    223,36,60,104,226,49,163,13,97,135,156,79,9,0,17,48,
    100,74,208,47,131,134,18,34,80,99,43,25,209,136,236,36,
    198,64,225,22,164,127,7,52,57,130,232,24,198,0,59,53,
    65,68,53,80,85,202,3,72,93,197,5,255,194,24,109,53,
    104,251,187,12,19,213,11,210,248,40,98,103,208,156,163,170,
    133,150,249,98,244,249,94,95,122,42,221,68,194,87,241,65,
    221,115,163,40,86,117,215,247,235,174,82,73,176,119,160,100,
    90,87,113,253,65,218,32,255,218,11,25,210,114,121,163,97,
    134,44,66,1,34,75,127,241,3,79,225,151,37,254,194,94,
    72,165,66,148,244,98,63,69,58,137,232,74,101,211,38,21,
    25,57,230,141,48,136,28,98,165,229,145,239,22,126,255,56,
    219,9,35,181,81,206,112,149,202,176,163,170,12,81,55,77,
    29,222,9,209,25,141,36,248,208,13,15,36,75,71,60,41,
    220,16,77,245,30,38,129,199,219,164,91,102,10,214,47,138,
    35,127,132,219,13,188,135,180,147,219,140,202,89,198,229,10,
    98,114,10,199,50,254,91,22,171,134,103,141,145,88,206,208,
    72,153,82,1,99,65,140,225,128,200,60,197,172,212,48,56,
    173,176,138,28,179,111,211,140,94,182,55,104,184,71,195,125,
    26,54,51,43,188,97,83,204,93,52,197,19,90,222,96,253,
    89,83,114,159,153,105,234,159,139,187,59,103,113,135,201,180,
    69,241,99,80,148,157,197,143,69,137,55,121,70,35,178,114,
    100,154,144,126,73,105,158,226,140,133,81,72,97,112,208,236,
    44,100,216,110,118,141,236,49,157,161,221,38,8,23,113,220,
    45,224,216,38,151,49,136,237,59,89,2,117,136,67,195,215,
    94,39,81,165,43,12,95,167,225,173,9,89,255,12,136,221,
    75,64,252,136,118,82,27,3,113,142,1,88,197,79,205,240,
    204,177,75,242,82,187,116,1,128,132,62,235,10,244,253,152,
    102,230,101,35,76,22,120,99,213,127,83,0,30,237,214,40,
    106,184,139,147,209,26,41,86,132,220,26,182,17,207,163,53,
    236,12,12,238,12,222,225,206,128,187,11,238,195,116,146,55,
    57,207,235,73,137,44,212,49,97,117,92,241,211,10,142,195,
    36,126,57,170,199,157,186,98,19,80,78,126,250,32,221,126,
    144,126,132,217,182,254,140,243,156,206,183,58,163,38,114,72,
    25,145,94,253,244,165,39,185,200,242,55,199,209,9,208,225,
    100,232,140,139,55,162,143,58,8,246,6,27,158,75,65,170,
    18,170,0,147,48,125,53,55,61,105,242,25,173,93,101,187,
    155,98,13,145,86,21,188,65,71,23,2,110,236,248,41,126,
    126,69,190,32,35,72,160,86,221,110,233,237,179,102,164,163,
    253,211,115,104,122,179,122,217,77,92,232,247,25,138,202,103,
    40,162,143,153,197,201,223,128,59,96,1,127,5,194,9,194,
    97,28,39,121,88,17,48,150,136,253,143,192,1,117,69,151,
    193,185,170,69,157,5,115,96,10,75,159,48,171,110,58,62,
    131,111,11,209,152,181,6,230,184,199,45,182,6,86,158,231,
    24,96,55,42,255,214,249,132,72,190,234,185,41,177,233,44,
    119,22,224,103,149,37,239,75,49,203,191,97,180,77,235,85,
    29,218,224,55,103,88,163,226,186,46,150,140,2,130,222,165,
    225,113,14,30,145,209,222,220,94,55,225,250,198,192,209,181,
    230,107,218,144,197,42,204,79,113,87,116,94,76,30,47,165,
    44,94,30,231,241,34,185,48,190,230,115,17,141,6,33,226,
    212,16,120,136,197,174,145,206,140,22,200,18,180,203,20,89,
    220,242,139,113,224,137,44,17,82,218,60,87,117,217,76,187,
    218,128,57,40,180,191,105,120,57,137,4,67,46,127,26,186,
    131,61,223,125,22,209,202,180,188,151,133,162,145,233,82,43,
    234,66,97,36,174,83,135,191,238,100,58,29,78,34,185,124,
    128,11,229,186,112,40,249,177,199,25,229,203,158,172,15,228,
    96,15,15,199,189,96,88,239,132,110,151,253,102,142,117,253,
    60,211,85,177,227,47,118,57,233,35,26,227,186,23,71,88,
    19,14,60,21,39,117,95,226,113,81,250,245,173,58,23,148,
    122,144,214,221,61,124,234,122,74,135,197,249,64,231,102,219,
    77,186,41,247,213,251,71,52,157,148,223,29,39,136,2,60,
    112,12,33,47,231,250,204,154,215,7,14,26,29,101,88,121,
    241,32,168,70,58,3,82,199,99,111,211,240,19,152,96,25,
    121,31,23,26,208,138,100,204,178,88,55,42,134,90,190,20,
    231,95,208,219,233,229,104,255,231,77,162,93,95,86,141,99,
    190,76,156,114,138,238,43,104,172,80,57,105,87,51,226,12,
    143,179,76,156,203,136,183,120,156,103,98,45,35,46,240,184,
    200,196,165,236,230,108,153,137,43,208,94,165,43,36,162,172,
    81,114,153,250,161,201,133,163,113,82,113,120,244,63,205,41,
    246,147,255,7,85,236,15,97,220,161,92,151,79,68,81,207,
    57,157,79,250,34,59,88,21,149,228,11,159,245,107,32,235,
    120,137,116,149,212,94,220,152,140,234,156,167,244,62,94,157,
    229,137,203,109,255,199,185,150,167,220,173,141,150,217,185,250,
    148,201,206,21,207,163,187,216,255,91,220,255,63,165,254,255,
    152,77,226,24,250,8,112,6,226,82,110,25,58,117,69,242,
    200,185,202,58,186,207,167,237,185,195,161,140,124,251,17,20,
    91,119,126,60,9,172,80,110,252,51,20,186,39,83,44,99,
    175,126,57,134,169,20,20,180,102,47,151,242,168,157,144,191,
    25,234,223,102,80,111,240,89,61,175,7,246,83,26,184,2,
    228,201,223,254,69,238,173,251,215,226,152,40,116,122,252,62,
    22,236,229,178,11,84,111,95,189,125,29,179,207,95,156,192,
    103,153,55,96,35,185,116,236,203,41,220,90,95,249,18,215,
    68,44,117,29,39,148,135,50,228,21,110,204,76,235,204,35,
    243,5,186,122,120,157,128,195,56,84,110,87,102,91,165,197,
    110,202,75,107,81,125,62,79,102,248,113,70,241,101,40,149,
    188,50,116,20,185,112,124,53,227,75,236,86,226,17,158,132,
    249,40,137,223,67,199,153,88,69,255,57,41,68,43,82,25,
    199,138,46,202,88,211,87,4,255,26,149,114,69,112,43,117,
    225,127,52,244,70,233,102,72,31,156,70,169,205,121,122,62,
    71,38,95,186,103,29,12,129,152,209,176,235,14,244,253,40,
    95,246,217,63,130,241,197,139,253,48,71,56,221,80,241,105,
    85,223,26,96,46,226,54,143,187,58,251,189,44,69,13,118,
    182,51,205,182,89,33,231,80,82,119,200,119,254,131,29,62,
    172,20,121,180,246,127,208,142,211,202,171,187,23,120,198,79,
    51,73,235,87,138,104,5,3,125,27,173,22,46,60,247,19,
    18,186,114,129,154,202,36,112,195,224,149,84,247,174,148,87,
    240,6,219,44,123,204,226,47,179,208,213,93,225,59,247,99,
    87,84,51,6,90,34,187,65,138,210,88,212,229,100,78,142,
    87,111,93,7,253,162,128,73,97,83,159,149,244,69,209,51,
    210,60,165,123,48,186,119,174,204,87,16,167,148,232,77,81,
    197,84,111,153,179,181,138,53,59,83,177,42,83,38,95,6,
    206,225,241,185,106,85,102,102,69,241,119,19,241,92,53,54,
    151,42,226,191,199,189,203,61,
};

EmbeddedPython embedded_m5_internal_param_SrcClockDomain(
    "m5/internal/param_SrcClockDomain.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_SrcClockDomain.py",
    "m5.internal.param_SrcClockDomain",
    data_m5_internal_param_SrcClockDomain,
    2360,
    7535);

} // anonymous namespace
