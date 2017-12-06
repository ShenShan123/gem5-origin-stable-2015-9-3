#include "sim/init.hh"

namespace {

const uint8_t data_m5_internal_drain[] = {
    120,156,205,89,235,111,28,87,21,63,119,102,118,237,93,219,
    241,219,110,210,208,46,160,208,165,194,113,95,105,138,18,162,
    180,73,64,173,192,45,179,65,73,221,194,48,222,185,187,30,
    123,119,102,153,25,39,89,100,131,192,17,84,8,36,36,4,
    159,248,192,23,62,240,223,240,31,193,249,157,121,236,172,237,
    72,161,233,90,120,119,175,239,220,185,115,238,121,252,206,227,
    222,105,83,246,87,225,223,237,6,81,124,95,17,121,252,85,
    212,35,234,43,218,86,164,180,34,111,141,246,43,20,189,67,
    94,133,158,18,109,27,164,13,58,230,142,73,159,25,20,204,
    202,51,85,234,153,50,162,104,88,39,109,209,118,133,30,4,
    139,100,233,42,237,215,41,250,57,41,165,2,69,15,189,41,
    242,166,233,41,83,231,78,77,8,78,19,6,235,50,88,35,
    111,70,6,235,228,205,74,103,134,134,11,164,103,105,123,14,
    211,182,47,48,217,215,153,236,188,144,253,55,200,122,124,103,
    157,188,11,152,206,124,125,138,153,22,102,202,122,243,66,101,
    129,60,161,210,97,121,22,139,137,139,164,77,218,91,162,237,
    37,210,252,93,164,99,22,217,91,202,39,46,23,19,151,101,
    226,10,109,175,144,230,239,114,58,113,69,72,175,202,189,85,
    116,188,85,25,89,147,145,53,116,88,49,173,230,75,172,97,
    255,63,252,215,100,13,83,50,203,205,35,29,197,126,24,56,
    126,208,9,125,3,247,171,104,96,143,54,154,169,204,48,119,
    96,152,127,145,88,197,51,50,195,28,17,47,174,160,247,158,
    65,71,210,57,50,104,216,164,67,69,123,22,121,38,29,242,
    50,21,48,217,85,116,108,208,231,38,38,28,113,107,177,250,
    94,33,43,73,173,178,39,234,75,41,77,209,81,133,14,43,
    212,122,120,104,96,96,191,70,209,63,233,151,151,133,232,180,
    16,53,232,144,91,139,142,45,58,170,210,3,158,196,67,123,
    53,168,72,61,60,100,73,121,164,213,180,152,219,173,146,184,
    16,197,243,163,192,237,235,4,66,58,94,228,250,65,179,158,
    223,11,227,171,3,55,217,181,101,178,9,45,244,7,137,16,
    9,3,157,204,112,167,227,7,158,211,15,189,131,158,78,166,
    65,193,233,248,61,237,56,114,243,195,254,32,140,146,123,81,
    20,70,54,20,41,131,189,208,45,158,128,26,219,189,48,214,
    77,172,38,203,216,32,159,96,118,103,32,20,193,128,48,136,
    135,61,29,183,35,127,144,176,125,82,138,152,13,106,77,88,
    70,154,120,139,155,205,126,144,108,238,118,59,241,102,107,215,
    141,116,107,87,7,155,93,221,191,182,17,70,126,215,15,54,
    226,196,221,233,233,141,183,222,120,243,218,198,119,55,222,222,
    220,57,240,123,222,230,147,247,222,221,28,12,147,221,48,216,
    140,31,251,221,77,209,198,85,30,89,2,93,30,113,124,145,
    200,217,213,189,129,142,230,48,122,9,107,170,5,53,171,170,
    202,84,77,53,199,189,10,255,76,117,217,152,81,91,62,100,
    106,67,78,232,215,42,195,6,182,84,180,111,80,116,25,160,
    216,227,175,130,21,25,26,45,220,51,228,222,143,161,140,116,
    116,207,132,169,211,193,67,1,18,35,138,103,222,132,109,3,
    18,52,84,104,175,74,41,74,24,92,41,108,162,33,90,158,
    14,50,6,19,183,40,254,43,177,114,25,31,135,148,97,231,
    216,36,21,44,80,82,135,131,242,232,26,47,248,91,129,95,
    171,9,246,183,4,16,201,174,31,135,143,3,81,59,250,226,
    48,45,214,204,39,195,143,119,246,116,59,137,95,229,129,79,
    195,131,70,219,13,130,48,105,184,158,215,112,147,36,242,119,
    14,18,29,55,146,176,113,37,110,194,146,246,98,142,169,130,
    222,112,144,99,8,246,102,12,165,23,158,223,78,248,98,89,
    46,196,10,177,78,24,15,187,161,23,243,56,72,116,117,98,
    131,73,1,113,40,140,8,92,28,76,197,242,60,239,2,95,
    191,159,115,34,152,108,86,115,4,197,186,215,73,234,2,70,
    55,142,29,225,4,227,130,59,16,126,228,246,14,82,23,97,
    228,36,204,16,186,41,15,147,69,30,226,83,33,180,72,18,
    132,129,55,100,198,252,246,107,88,243,37,193,223,172,32,112,
    149,209,55,197,109,149,255,87,213,154,209,182,50,204,85,115,
    220,173,65,98,18,171,171,204,240,140,193,99,14,45,77,67,
    98,131,8,35,126,248,13,244,240,176,125,25,205,215,208,188,
    130,230,213,92,222,137,9,61,119,82,232,235,88,200,16,73,
    69,38,152,196,204,101,242,198,124,233,226,200,151,56,246,181,
    224,19,6,60,103,228,19,22,226,100,116,11,45,79,21,111,
    51,57,195,34,42,195,119,132,24,220,132,1,143,222,200,13,
    68,67,246,2,36,159,206,17,108,3,150,101,108,118,75,216,
    180,97,28,1,166,125,49,15,127,14,102,164,144,180,95,6,
    169,202,25,42,110,160,249,250,196,245,60,2,87,247,20,184,
    110,96,205,133,12,92,115,2,170,58,255,22,140,182,153,41,
    191,200,129,203,39,64,5,68,89,103,32,234,91,232,153,167,
    197,61,47,48,101,66,126,191,4,38,240,101,148,101,1,3,
    195,117,136,80,134,209,58,103,242,7,193,58,39,103,67,146,
    243,27,146,156,37,193,75,33,147,6,99,83,226,113,218,169,
    64,23,29,147,214,178,164,27,215,184,29,68,225,147,97,35,
    236,52,18,17,22,177,243,230,149,248,234,149,248,6,71,197,
    198,45,137,71,105,92,76,35,95,164,7,136,92,120,244,222,
    147,182,150,180,39,87,142,147,6,42,71,130,150,147,165,83,
    70,212,42,52,105,228,42,150,144,29,39,17,34,245,100,149,
    92,47,148,12,158,63,194,42,117,209,176,169,214,25,61,117,
    37,172,56,105,104,150,42,74,238,242,239,3,104,29,226,106,
    66,85,107,183,82,70,69,6,72,99,127,103,12,33,147,146,
    192,222,100,146,63,201,145,81,29,33,3,63,51,71,249,239,
    73,138,79,69,191,35,216,158,77,156,161,188,112,10,24,123,
    25,211,127,70,226,14,103,100,120,137,41,45,100,117,153,193,
    161,38,190,46,83,211,132,255,17,125,81,242,165,60,45,155,
    89,233,88,78,203,86,17,143,4,52,207,149,122,173,241,192,
    5,171,236,186,49,166,165,209,104,228,158,163,88,95,84,127,
    28,141,39,134,160,233,148,190,3,86,62,31,225,7,137,237,
    101,181,108,148,80,241,38,154,183,10,64,168,124,108,18,92,
    189,74,207,78,191,78,26,231,63,195,210,150,48,59,63,37,
    245,208,93,60,253,35,55,112,187,58,42,176,94,201,177,238,
    20,88,215,146,146,158,202,6,2,173,1,27,31,27,138,247,
    106,233,46,141,43,49,236,89,170,164,167,80,231,99,7,86,
    201,119,96,213,108,7,198,27,47,116,102,101,235,70,216,138,
    177,11,33,70,8,130,243,159,196,54,68,194,177,228,40,186,
    219,74,181,90,96,34,53,55,154,39,147,141,25,176,248,205,
    158,219,223,241,220,91,93,172,129,133,218,185,207,25,57,215,
    11,101,174,225,47,234,89,140,203,229,181,156,251,71,147,141,
    23,239,50,201,130,107,241,14,47,108,75,144,184,191,171,27,
    125,221,223,225,109,228,174,63,104,116,122,110,247,116,158,121,
    159,36,103,166,134,224,80,50,92,17,33,211,82,69,132,84,
    15,130,75,156,112,44,73,56,55,145,112,14,37,41,57,70,
    154,115,70,246,147,10,66,118,67,208,85,160,31,59,101,4,
    166,9,5,169,197,29,12,116,224,217,175,83,57,71,200,237,
    201,27,218,225,237,180,207,101,252,94,217,181,77,181,194,201,
    225,52,80,225,77,37,249,196,195,43,133,175,95,158,52,195,
    98,219,95,20,136,204,89,43,16,57,87,216,46,171,31,203,
    112,20,59,172,158,136,2,40,61,238,132,7,65,114,78,18,
    136,202,243,53,33,69,94,239,20,25,173,236,93,133,44,105,
    54,122,14,113,226,92,156,81,169,33,91,121,25,155,184,96,
    249,234,143,74,130,125,9,35,93,62,37,149,223,13,220,158,
    12,221,13,3,125,94,182,154,7,221,241,165,135,35,201,154,
    216,183,142,42,33,217,198,166,25,137,171,201,129,142,146,97,
    90,1,92,69,243,109,52,55,10,9,17,141,60,221,211,137,
    30,139,8,18,38,178,237,138,167,185,62,12,135,92,73,74,
    129,198,215,61,46,35,191,7,18,183,209,124,64,231,82,125,
    189,195,36,59,148,225,177,202,25,191,106,212,170,53,85,51,
    106,102,205,148,146,87,4,0,205,211,25,245,143,244,28,25,
    53,203,165,22,233,202,88,70,69,167,142,68,138,142,28,101,
    162,115,129,182,231,243,100,187,144,39,219,197,60,217,46,229,
    201,118,57,63,39,93,145,67,82,57,113,228,244,59,67,47,
    152,126,37,139,77,62,198,253,154,190,202,172,107,95,63,95,
    166,237,247,40,43,203,159,149,113,205,76,162,143,75,145,206,
    83,167,182,244,49,50,226,86,216,104,135,1,59,195,65,59,
    9,163,134,167,59,126,160,189,198,70,67,118,90,13,63,110,
    184,59,124,215,109,103,49,124,188,90,150,211,34,55,234,198,
    146,104,247,31,163,59,89,85,192,203,127,115,70,0,252,191,
    74,162,95,156,2,216,255,150,120,230,203,142,159,29,78,143,
    82,14,106,109,175,20,215,38,27,166,43,249,106,127,56,67,
    235,95,166,54,24,201,100,235,248,160,127,110,249,102,38,23,
    36,93,246,79,47,38,206,218,152,56,236,133,15,34,63,209,
    59,110,123,255,188,228,1,12,202,235,254,249,197,4,90,63,
    41,208,135,193,35,183,231,123,110,114,110,22,154,75,37,26,
    45,252,151,175,82,36,174,11,229,162,149,156,179,72,99,11,
    255,109,36,82,19,24,26,149,56,246,29,52,82,212,72,41,
    51,170,108,68,152,197,49,97,236,131,32,240,131,174,84,47,
    121,127,105,108,134,244,48,60,157,15,227,98,241,244,28,237,
    9,149,188,15,6,202,197,19,38,218,247,176,12,14,16,237,
    31,162,129,162,236,79,208,216,104,238,211,185,20,75,63,96,
    146,191,162,236,221,19,23,75,70,205,168,170,90,254,145,146,
    9,159,177,99,54,65,74,157,74,155,206,211,48,129,222,218,
    145,102,227,220,61,51,170,78,76,32,156,247,252,253,197,32,
    142,50,160,221,211,110,112,48,24,99,94,14,51,240,184,44,
    230,244,207,33,83,216,63,101,146,255,40,193,91,94,156,142,
    191,72,78,25,128,95,166,7,107,195,24,0,34,123,190,16,
    72,94,125,230,21,62,60,66,170,224,45,183,159,190,187,146,
    215,51,246,55,41,59,86,183,95,43,60,5,111,26,228,52,
    51,61,41,14,244,99,169,96,164,96,177,223,70,131,98,59,
    185,72,39,119,63,44,67,164,187,126,156,232,72,32,118,34,
    180,143,221,135,197,68,206,201,163,35,61,39,74,79,184,111,
    33,119,197,183,185,193,235,174,218,124,141,247,9,56,70,48,
    85,93,205,41,203,156,93,168,89,179,51,53,171,54,101,202,
    251,138,57,181,108,212,173,218,204,218,106,77,213,141,181,75,
    220,154,236,32,252,249,47,206,32,32,190,
};

EmbeddedPython embedded_m5_internal_drain(
    "m5/internal/drain.py",
    "/mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/build/x86/python/swig/drain.py",
    "m5.internal.drain",
    data_m5_internal_drain,
    2475,
    8547);

} // anonymous namespace
