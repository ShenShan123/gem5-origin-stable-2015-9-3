#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_param_SeriesRequestGenerator[] = {
    120,156,205,89,91,115,219,198,21,62,11,128,148,72,73,214,
    93,178,45,217,162,157,168,97,220,74,76,28,43,78,39,174,
    91,231,210,78,51,19,197,5,211,177,195,100,138,66,196,138,
    4,69,2,44,176,178,204,84,154,206,84,238,229,33,175,253,
    9,125,232,191,233,228,15,181,231,156,5,64,232,102,37,109,
    167,180,37,174,151,123,57,123,46,223,185,236,170,9,201,191,
    2,126,126,86,1,136,191,21,0,30,254,10,232,2,244,4,
    52,4,8,41,192,91,128,189,2,68,247,192,43,192,11,128,
    134,1,210,128,99,236,152,240,165,1,193,36,239,41,66,215,
    228,17,1,131,50,72,11,26,5,120,18,204,130,37,139,176,
    87,134,232,183,32,132,8,4,60,245,198,192,27,135,23,72,
    29,59,37,38,56,14,52,88,230,193,18,120,19,60,88,6,
    111,146,59,19,48,152,1,57,9,141,41,90,214,184,130,100,
    239,32,217,105,38,251,79,34,235,225,204,34,120,87,104,57,
    242,245,5,173,180,104,37,159,55,205,84,102,82,46,103,161,
    49,151,246,231,115,253,133,92,127,49,215,95,202,245,151,115,
    253,171,185,254,181,92,255,122,174,191,146,235,175,230,250,55,
    184,143,82,205,65,231,38,116,214,160,83,129,93,84,244,108,
    38,193,45,144,38,116,110,67,227,54,72,252,189,5,199,104,
    11,111,46,183,227,53,222,49,159,237,120,157,119,172,67,99,
    29,36,254,190,174,119,20,161,94,93,66,251,250,255,194,127,
    85,180,47,168,73,108,158,201,40,246,195,192,241,131,221,208,
    55,104,190,72,13,161,161,73,205,88,2,139,15,9,22,255,
    0,198,132,103,36,176,56,2,36,44,72,150,174,1,71,220,
    57,50,96,80,133,67,1,29,11,60,19,14,241,152,2,49,
    208,18,112,108,192,87,38,45,56,194,214,66,227,221,4,75,
    105,76,116,216,120,154,210,24,28,21,224,176,0,245,167,135,
    6,13,236,149,32,250,59,124,189,202,68,199,153,168,1,135,
    216,90,112,108,193,81,17,158,224,34,28,234,148,72,124,241,
    244,16,37,197,145,122,213,66,110,183,115,226,146,40,158,31,
    5,110,79,170,27,216,119,250,110,228,246,156,186,140,124,25,
    219,242,119,251,50,86,191,144,129,140,92,21,70,213,114,186,
    37,140,55,251,174,106,219,76,195,36,229,244,250,138,105,135,
    129,84,19,216,217,245,3,207,233,133,222,126,87,170,113,34,
    236,236,250,93,233,56,60,249,203,94,63,140,212,199,81,20,
    70,54,233,151,7,187,161,155,237,32,237,54,187,97,44,171,
    116,26,31,99,19,121,69,171,119,251,76,145,24,96,190,105,
    179,39,227,102,228,247,21,154,77,83,164,213,68,173,74,6,
    227,38,70,251,64,173,23,168,90,187,181,27,215,234,109,55,
    146,245,182,12,106,45,217,219,218,8,35,191,229,7,27,177,
    114,119,186,114,227,238,91,111,111,109,252,120,227,157,218,206,
    190,223,245,106,207,223,123,183,214,31,168,118,24,212,122,91,
    53,63,80,18,53,214,173,189,76,87,155,184,126,142,78,61,
    240,91,142,207,242,58,109,217,237,203,104,138,70,175,19,71,
    98,70,76,138,162,48,69,85,76,97,175,128,31,83,172,26,
    19,98,219,39,137,155,164,5,66,158,149,199,26,1,64,192,
    158,1,209,42,33,169,131,191,130,76,143,120,170,211,156,193,
    115,191,34,85,233,209,142,73,248,208,131,135,140,62,132,33,
    174,124,64,128,8,128,33,84,128,78,17,52,180,16,145,26,
    107,209,128,90,92,78,100,12,36,110,65,252,55,64,213,35,
    168,14,33,1,220,177,9,34,152,1,85,166,152,130,163,75,
    120,224,31,25,179,245,42,177,191,205,112,81,109,63,14,15,
    2,54,10,245,217,203,234,168,153,199,131,207,118,58,178,169,
    226,53,28,248,34,220,175,52,221,32,8,85,197,245,188,138,
    171,84,228,239,236,43,25,87,84,88,89,143,171,100,103,123,
    54,69,92,70,111,208,79,17,70,104,64,132,233,47,158,223,
    84,248,101,158,191,176,21,98,169,16,45,237,208,139,113,156,
    72,180,164,178,137,73,69,74,14,153,17,6,147,67,75,233,
    120,92,119,5,191,63,74,57,97,196,86,139,41,190,98,217,
    221,85,101,134,170,27,199,14,115,66,227,140,74,34,252,204,
    237,238,75,166,142,184,82,200,16,117,53,15,163,196,229,85,
    146,49,85,9,203,25,132,129,55,64,182,253,230,27,196,209,
    85,70,231,36,227,115,17,177,57,134,109,17,255,47,138,37,
    163,105,37,136,44,166,168,164,8,170,128,49,33,18,88,32,
    66,143,49,90,85,13,14,55,44,42,251,240,109,234,209,102,
    123,149,26,138,56,246,77,106,214,82,109,140,72,37,83,167,
    85,114,159,216,48,88,15,44,49,153,211,76,37,246,78,248,
    225,181,161,31,98,176,173,147,63,25,228,117,67,127,178,40,
    48,71,15,169,197,165,236,169,38,196,159,83,26,32,191,99,
    98,228,98,232,44,212,27,186,16,235,207,158,33,189,140,167,
    232,183,9,210,121,92,183,114,184,182,201,116,12,106,251,90,
    26,88,29,90,161,225,108,175,16,169,194,57,6,168,80,115,
    107,196,86,24,2,179,117,6,152,239,19,71,51,9,48,167,
    24,144,101,252,204,24,77,51,49,77,150,146,231,79,1,146,
    208,104,157,131,198,31,80,207,60,171,140,87,3,136,137,10,
    126,158,3,34,113,109,228,37,221,198,206,96,153,4,204,67,
    112,25,203,142,39,193,50,86,18,6,87,18,111,113,37,193,
    213,8,215,124,58,9,152,156,7,116,167,64,154,218,53,97,
    41,169,16,226,18,182,253,40,124,62,168,132,187,21,197,170,
    160,152,253,96,61,222,92,143,223,199,104,92,121,200,113,80,
    199,99,29,113,35,217,167,136,73,91,63,126,222,148,156,140,
    249,155,227,232,0,233,112,176,116,146,36,143,104,92,36,61,
    27,169,1,56,85,196,42,162,12,49,74,19,148,51,19,144,
    68,159,16,15,101,214,191,41,150,17,121,101,193,140,58,58,
    97,112,65,200,179,248,249,128,108,66,202,144,64,215,3,187,
    174,197,96,9,73,86,251,71,39,208,53,26,249,236,26,30,
    248,235,20,85,197,33,170,232,99,166,254,243,23,224,10,90,
    192,159,129,112,131,240,72,252,39,115,55,2,202,60,45,255,
    13,176,163,157,83,149,112,44,171,83,37,194,43,48,196,197,
    247,121,169,46,82,62,129,191,230,188,52,45,37,204,164,70,
    206,151,18,86,22,7,25,112,223,169,92,176,78,6,76,178,
    89,219,141,105,153,142,130,67,199,31,102,160,172,158,197,44,
    48,34,244,141,235,211,29,98,244,171,33,246,40,25,175,136,
    121,35,135,168,183,169,185,155,129,73,164,99,255,127,158,215,
    224,226,130,194,209,185,233,75,98,204,98,81,166,199,20,149,
    12,231,147,203,252,169,144,250,211,221,204,159,36,39,212,23,
    124,223,162,214,32,164,28,27,2,47,214,88,125,210,61,214,
    2,89,128,70,145,60,143,175,16,34,113,76,145,6,76,10,
    175,39,178,53,171,109,91,43,52,3,139,198,1,53,207,71,
    25,136,8,10,15,186,110,111,199,115,31,198,196,1,177,209,
    76,93,213,72,101,154,201,203,68,110,38,46,18,139,191,110,
    165,178,61,27,101,16,122,55,145,135,101,98,151,243,194,38,
    71,158,207,219,178,210,147,189,29,188,132,183,253,126,101,183,
    235,182,216,142,102,34,243,103,169,204,138,129,112,186,90,138,
    239,80,27,86,154,97,128,185,100,191,137,231,85,60,137,215,
    81,233,85,54,42,156,136,42,126,92,113,119,112,214,109,42,
    237,54,39,3,2,23,241,110,212,138,185,94,223,59,160,238,
    168,113,224,56,126,224,227,133,102,31,178,114,64,223,141,179,
    188,194,87,21,237,133,152,185,241,162,169,6,58,98,82,229,
    100,111,82,243,38,188,2,233,231,30,30,24,209,201,164,220,
    162,88,49,74,134,90,189,48,30,60,38,106,241,217,168,240,
    205,119,137,10,250,161,45,137,13,69,90,41,199,232,189,132,
    218,18,165,163,70,57,29,156,224,118,146,7,167,210,193,43,
    220,78,243,224,76,250,192,55,203,131,115,208,152,167,215,42,
    26,89,160,120,51,246,223,198,27,118,204,81,187,228,225,255,
    52,204,216,247,95,37,145,236,247,32,41,110,46,10,49,34,
    47,239,148,14,49,29,145,222,217,242,194,242,27,211,107,151,
    160,214,105,70,210,85,82,91,119,117,180,170,224,80,166,249,
    249,195,48,132,156,189,81,60,202,164,62,230,194,111,176,192,
    70,215,23,90,54,186,120,18,92,199,171,133,197,87,139,7,
    116,181,56,100,21,57,134,190,93,12,65,94,200,52,133,148,
    33,144,7,23,176,167,181,165,175,18,196,166,219,239,203,192,
    179,239,64,254,118,192,211,163,196,18,133,209,63,65,174,32,
    51,197,2,94,7,206,250,60,101,143,156,22,216,250,133,204,
    203,71,140,3,118,137,111,82,151,168,114,246,200,82,136,253,
    128,26,78,26,89,190,176,127,154,89,241,222,101,120,199,58,
    60,194,60,133,48,235,201,64,57,177,255,181,164,107,236,127,
    182,17,75,73,126,229,56,103,78,253,240,50,130,193,126,15,
    9,208,2,102,224,251,172,167,115,201,196,195,33,85,187,108,
    59,102,219,38,49,119,16,249,42,57,242,251,238,161,99,41,
    131,159,28,102,212,113,160,241,100,87,42,249,82,15,82,100,
    185,228,81,200,195,233,40,28,224,157,155,47,171,248,189,235,
    56,35,207,253,63,193,3,127,79,39,211,211,56,230,126,81,
    196,236,191,40,240,199,40,21,75,130,75,176,83,127,113,209,
    236,210,83,141,190,152,13,98,155,131,249,116,6,75,254,99,
    64,90,241,16,130,249,209,97,219,237,233,247,90,126,116,180,
    73,133,252,224,99,191,145,193,155,94,200,248,54,172,95,41,
    48,64,113,121,200,213,160,253,78,170,248,222,214,102,42,223,
    166,150,239,35,63,194,155,191,244,50,209,248,207,18,189,45,
    126,191,58,187,188,62,136,149,236,169,149,83,147,146,0,246,
    169,236,133,209,224,211,208,147,92,3,229,231,31,33,238,109,
    55,104,73,231,153,164,42,86,221,58,189,32,41,97,53,141,
    116,85,229,92,30,78,174,61,195,139,94,132,147,250,57,158,
    47,116,103,231,63,236,134,205,61,233,37,107,110,92,188,230,
    163,176,231,226,248,249,167,212,253,244,148,217,83,243,94,68,
    187,22,79,141,146,15,186,93,242,122,50,116,58,172,150,201,
    234,23,152,131,8,159,25,229,194,243,37,57,155,253,38,146,
    45,31,173,21,49,249,51,36,146,68,69,40,86,111,94,230,
    223,121,114,163,118,60,125,149,212,239,110,15,233,77,56,126,
    140,13,61,239,151,166,75,232,132,148,204,76,81,198,116,102,
    153,147,51,37,107,114,162,100,149,198,76,126,99,157,18,243,
    70,217,42,77,76,138,243,127,214,208,113,203,198,218,92,73,
    252,27,49,201,126,79,
};

EmbeddedPython embedded_m5_internal_param_SeriesRequestGenerator(
    "m5/internal/param_SeriesRequestGenerator.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/m5/internal/param_SeriesRequestGenerator.py",
    "m5.internal.param_SeriesRequestGenerator",
    data_m5_internal_param_SeriesRequestGenerator,
    2438,
    7988);

} // anonymous namespace
